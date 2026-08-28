#include "stuck_recovery_controller/stuck_recovery_controller.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <string>

#include <cmath>
#include <functional>
#include <limits>
#include <memory>

namespace stuck_recovery_controller
{
using nav_msgs::msg::Odometry;
using v2x_msgs::msg::V2XVehiclePositionArray;
namespace
{

constexpr float kStuckSpeedThreshold = 0.1;
// 壁に刺さってから復帰を始めるまでの待ち。短いほど、pure_pursuit が
// 壁に向かって舵を切っている時間が短くなる。速度がほぼ0で、かつ
// 「前へ出ろ」と指令されているのに動いていない、という条件つきなので
// 0.6 秒でも誤検出はしにくい。
constexpr double kStuckDurationSec = 0.6;
// 入口の停滞判定。この秒数のあいだに この距離[m] も進めていなければ、
// 速度が出ていても「進んでいない」とみなす。
// 押し合いは 0.2-0.5m/s で動き続けるので、瞬間速度では捉えられない。
constexpr double kEntryProgressSec = 2.5;
constexpr double kEntryProgressDist = 0.8;
// 通常側が「前車が近いので停止」を出し続ける玉突き状態も復帰対象にする。
// 実戦bagでは前方車1.5〜1.9mに挟まれ、速度指令0のまま121秒停止した。
constexpr double kBlockedVehicleDurationSec = 3.0;
constexpr double kBlockedVehicleMinLongitudinal = 0.3;
constexpr double kBlockedVehicleMaxLongitudinal = 4.0;
constexpr double kBlockedVehicleMaxLateral = 1.6;
constexpr float kStoppedCommandThreshold = 0.1;
constexpr float kCommandSpeedThreshold = 1.0;
constexpr float kCommandAccelerationThreshold = 0.3;
constexpr float kMovingSpeedThreshold = 0.5;
constexpr double kRearClearDist = 6.0;      // 後方この距離[m]以内に車がいたら下がらない
constexpr double kRearKeep = 2.0;           // 後方の他車との間にこれだけ[m]残す
// 復帰の経路計画で避ける他車を拾う範囲[m]。遠くまで入れると経路が見つからない。
constexpr double kCarObstacleRange = 12.0;
// 前進中に他車がこれ[m]より近づいたら、停滞を待たずに引き直す。
constexpr double kFwdAbortCarDist = 0.25;
// 「閾値を割った」だけでは降りず、区間開始よりこれだけ[m]悪化したときに降りる。
// これが無いと、もともと狭い所で出だしに必ず降りて同じ計画を引き直し続ける
// (ユーザー報告「何もないのに何度も切り返す」の一因)。
constexpr double kFwdAbortWorsen = 0.08;
// 前後左右を問わず、この距離[m]以内に他車がいれば「近接」とみなす。
// 横並びで押し合っている接触を拾うために要る(前方の箱だけでは取りこぼす)。
constexpr double kBlockedVehicleAnyDirRange = 2.5;
constexpr double kRearKeepTight = 0.5;      // 手詰まりのときはここまで詰めてよい
constexpr double kRearWaitMax = 2.0;        // 後方が退くのをこの時間[s]しか待たない
constexpr double kRearClearWidth = 1.6;     // 後方判定の横幅[m]
constexpr double kWheelBase = 2.14;         // ホイールベース[m]
constexpr double kMaxSteerRad = 0.6109;     // 最大舵角 35度(parameter.md の実装値)
constexpr double kHalfWidth = 0.73;         // 車体半幅[m]
constexpr double kAvoidMargin = 0.3;        // 回避時の余裕[m]
constexpr double kAvoidCheckRange = 8.0;    // 前方この距離[m]までを判定対象にする
constexpr double kFrontObstacleHalf = 1.5;  // 前方障害物とみなす横幅の半分[m]

// --- 計画追従の定数 ---
constexpr double kRecoveryMaxSec = 30.0;   // 復帰全体の上限[s]。超えたら通常制御へ返す
constexpr double kReplanMax = 4;           // 計画のやり直し上限
// 動けているかの判定は「ギアが入ってから」「舵を入れ終わってから」数える。
// 1.2s では、停止 -> ギア切替 -> 舵を入れる の途中で毎回「動けない」と判定され、
// 実測では69回の判定すべてが計画を1つも実行しないまま向きを反転させていた。
constexpr double kStallSec = 3.0;          // この時間[s]動けなければ計画をやり直す
constexpr double kPhaseMinSec = 1.2;       // 区間を始めてこの時間[s]は停滞判定をしない
// 実ギア・実舵角が指令に追いつくのを待つ上限[s]。
// 実測(gearlat2.py): 実ギアの切替 0.012〜0.099s、舵は角速度上限 2.0rad/s なので
// フルロック 0.61rad で 0.31s。両方を見て、届いたら即動く。
constexpr double kActuatorWaitMax = 0.45;
constexpr double kSteerReadyRad = 0.05;    // 舵が目標に届いたとみなす差[rad](2.9deg)
constexpr double kStallDist = 0.15;        // 動けているとみなす距離[m]
constexpr double kPhaseDoneSlack = 0.10;   // 区間の走破判定の余裕[m]
// 20deg で返していたところ、実測4件のうち3件が「ちょうど20deg」で
// 制御を返していた。計画の終端は方位差 0〜2deg まで直せているので、
// 途中で切り上げず、走り出せる向きになるまで握り続ける。
constexpr double kAlignYaw = 0.22;         // コース方位とのずれ[rad](12.6deg)
// 通常制御に返す前に、この距離[m]ぶん先まで追従経路を検査する。
// カーブ1つを跨げる長さが要る(半径5mのヘアピンで四分円が約8m)。
constexpr double kHandbackAhead = 12.0;
constexpr double kHandbackLookahead = 3.5;      // pure_pursuit の lookahead_min_distance
constexpr double kHandbackLookaheadGain = 0.20; // pure_pursuit の lookahead_gain
constexpr double kHandbackRelaxSec = 12.0;      // これを過ぎたら検査距離を半分にする
constexpr double kTrajStillSec = 2.0;           // 経路で渡して動かない時間[s]の上限
constexpr double kPlanMinEscape = 2.5;     // 計画の終端は詰まった場所からこれだけ[m]離れること
// 地図の上では余裕があるのに動けなかったとき、後退の下限をこれだけ[m]ずつ伸ばす。
// 自己位置推定が実際とずれていると、地図では前方が空いているのに壁に当たる。
// 実測で「前方 1.85m 空き」なのに前進できない姿勢があり、そこで同じ計画を
// 4回出して30秒を使い切っていた。地図を信じ続けても抜けられない。
constexpr double kBlockedReverseStep = 1.75;
constexpr double kBlockedReverseMax = 7.0;
constexpr double kExplainedClearance = 0.30;   // これだけ[m]余裕があれば「当たっていないはず」
// 前進中に壁までの実測余裕がこれを割ったら、停滞を待たず後退から引き直す。
// wall_margin(0.12)より少し大きくして、食い込む前に反応させる。
constexpr double kFwdAbortClearance = 0.20;
// 区間の出だしは姿勢が定まらないので、少し走ってから見る。
constexpr double kFwdAbortMinTravel = 0.05;
// 前後を切り替える前に止まりきる。実測で、後退区間が終わった時点でまだ
// -0.8m/s あり、0.3m 行き過ぎてから前進に入って、後退した円弧をそのまま
// 戻って同じ壁に当たっていた。
constexpr float kBrakeDoneSpeed = 0.25f;    // これ以下[m/s]なら止まったとみなす
constexpr double kBrakeMaxSec = 1.5;        // 止まりきらなくてもこの時間[s]で進む
constexpr float kBrakeAccel = -1.37f;       // 減速指令
constexpr double kEscapeDist = 2.5;        // 復帰開始地点からこれだけ[m]離れたら脱出とみなす
constexpr double kHandbackMargin = 0.60;   // 壁からこれだけ[m]離れて初めて通常制御へ返す
// 復帰中の目標速度[m/s]。計画どおり走らせるため低速で一定。
//
// 【後退が遅く見える理由 — 実測して分かったこと】
// AWSIM のバイナリに `ReverseSpeedLimit` などの文字列があるので
// 「5km/h で頭打ち」と一度断定したが**誤り**。実測では後退でも
// **最大 2.29〜2.90 m/s** 出ている(復帰追跡ログの速度を全ビルドで集計)。
// 遅く見えるのは速度上限ではなく、**区間を始めるたびに
// 「ギアが入り、舵が目標へ届くまで速度0で保持」する待ちが入る**ため。
// 計画を引き直すほどこの待ちが増える。速度定数を上げても解決しない。
// 対策は「引き直しを減らす」こと。
constexpr float kRecoverySpeed = 2.5f;
constexpr float kRecoveryAccel = 1.0f;     // 加速度上限(parameter.md MaxAccelerationInput)
constexpr double kCooldownSec = 2.0;       // 復帰完了後、再突入を禁止する時間[s]
// --- 最終手段 ---
// 計画をすべて試しても動けないときだけ使う。壁へ押し当てて車体の向きを変えたり、
// 詰まった相手を押しのけたりする。ふさわしい動きではないので、他に手が無いときに限る。
constexpr double kDesperateSwingSec = 1.6;    // 前後を切り替える間隔[s]
constexpr float kDesperateSpeed = 4.0f;       // 押し当てる速度指令[m/s]
constexpr float kDesperateAccel = 1.37f;      // AWSIM の入力上限いっぱい
constexpr double kDesperateFreeDist = 1.2;    // これだけ[m]動けたら通常の復帰へ戻す
constexpr float kForwardAssistSpeed = 30.0f;  // 速度で制限しない(通常制御と同じだけ出す)
constexpr float kForwardAssistAccel = 1.0f;   // 加速度上限(parameter.md MaxAccelerationInput)
constexpr double kDriveSettleDurationSec = 0.5;
// --- 動作確認用の壁当て ---
constexpr double kCrashDriveSec = 2.5;   // 全舵で走る時間[s]
constexpr float kCrashSpeed = 8.0f;      // 目標速度[m/s]
constexpr float kCrashAccel = 1.37f;

}  // namespace

StuckRecoveryController::StuckRecoveryController() : Node("stuck_recovery_controller")
{
  control_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
  gear_pub_ = create_publisher<GearCommand>("/control/command/gear_cmd", 1);

  // 動作確認用の強制発動。壁に当たらなくなると復帰が動く場面に出会えないため。
  // 軌道の経路にも入る。通常は素通しし、復帰の前進区間だけ差し替える。
  // 軌道のトピックは全ノードが best_effort + volatile + KeepLast(1) で
  // そろえてある。既定QoS(reliable)にすると DDS が「QoS 不一致」として
  // メッセージを1件も配送せず、経路がここで切れて pure_pursuit に何も
  // 届かなくなる(実測: rviz に軌道が出ず、車が動かなくなった)。
  const auto traj_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  traj_pub_ = create_publisher<Trajectory>("output/trajectory", traj_qos);
  traj_sub_ = create_subscription<Trajectory>(
    "input/trajectory", traj_qos,
    [this](const Trajectory::ConstSharedPtr msg) {
      latest_traj_ = msg;
      traj_following_ = !traj_giveup_ && publishRecoveryTrajectory();
      if (!traj_following_) { traj_pub_->publish(*msg); }
    });

  gear_report_sub_ = create_subscription<GearReport>(
    "/vehicle/status/gear_status", 1,
    [this](const GearReport::ConstSharedPtr msg) {
      gear_report_ = msg->report;
      gear_report_seen_ = true;
    });
  steer_report_sub_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", 1,
    [this](const SteeringReport::ConstSharedPtr msg) {
      steer_report_ = msg->steering_tire_angle;
    });

  // 動作確認用の壁当て。data=true で左へ、false で右へ全舵を切って突っ込む。
  crash_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/debug/crash_me", 1,
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      crash_me_ = true;
      crash_since_ = this->now();
      crash_steer_ = msg->data ? 1.0 : -1.0;
      RCLCPP_WARN(get_logger(), "壁当て(動作確認用) 開始 %s",
                  msg->data ? "左" : "右");
    });

  force_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/debug/force_recovery", 1,
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      if (msg->data) {
        force_recovery_ = true;
        RCLCPP_WARN(get_logger(), "復帰 強制発動の指示を受けた(動作確認用)");
      }
    });

  nominal_sub_ = create_subscription<AckermannControlCommand>(
    "/control/command/nominal_control_cmd", 1,
    std::bind(&StuckRecoveryController::onNominalCommand, this, std::placeholders::_1));
  // 復帰経路の計算に使う情報:
  //  - 自車の位置と向き(kinematic_state)
  //  - コース境界(lanelet2 から抽出した corridor CSV: raceline 各点の左右可動域)
  //  - 他車の位置(V2X)
  // これらから「後退したときに最も空いている方向」を選ぶ。
  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state",
    rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
    [this](const Odometry::ConstSharedPtr msg) { odom_ = msg; });
  v2x_sub_ = create_subscription<V2XVehiclePositionArray>(
    "/v2x/vehicle_positions", rclcpp::QoS(10),
    [this](const V2XVehiclePositionArray::ConstSharedPtr msg) { v2x_ = msg; });

  const auto raceline = declare_parameter<std::string>("raceline_csv", "");
  const auto corridor = declare_parameter<std::string>("corridor_csv", "");
  const auto grid = declare_parameter<std::string>("occupancy_grid_yaml", "");
  loadRaceline(raceline, corridor);
  if (!grid.empty() && obstacles_.load(grid)) {
    RCLCPP_INFO(get_logger(), "復帰用の占有格子を読んだ: %s", grid.c_str());
  } else if (!grid.empty()) {
    RCLCPP_WARN(get_logger(),
                "復帰用の占有格子を読めない: %s (コリドアで代用する)", grid.c_str());
  }

  velocity_sub_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1,
    [this](const VelocityReport::ConstSharedPtr msg) {
      latest_velocity_ = msg->longitudinal_velocity;
    });
}

void StuckRecoveryController::loadRaceline(
  const std::string & raceline_csv, const std::string & corridor_csv)
{
  if (raceline_csv.empty()) {
    return;
  }
  std::ifstream f(raceline_csv);
  if (!f.is_open()) {
    RCLCPP_WARN(get_logger(), "raceline を読めない: %s", raceline_csv.c_str());
    return;
  }
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string a, b;
    if (std::getline(ss, a, ',') && std::getline(ss, b, ',')) {
      line_x_.push_back(std::stod(a));
      line_y_.push_back(std::stod(b));
    }
  }
  if (!corridor_csv.empty()) {
    std::ifstream g(corridor_csv);
    if (g.is_open()) {
      std::getline(g, line);
      while (std::getline(g, line)) {
        if (line.empty()) {
          continue;
        }
        std::stringstream ss(line);
        std::string i, lo, hi;
        if (std::getline(ss, i, ',') && std::getline(ss, lo, ',') && std::getline(ss, hi, ',')) {
          corr_lo_.push_back(std::stod(lo));
          corr_hi_.push_back(std::stod(hi));
        }
      }
    }
  }
  // 経路計算にも同じものを渡す。
  // 領域が無いと計画を立てられないので、そのときは警告を出す
  // (黙って旧来のもがき方に落ちるより、原因が見えるほうがよい)。
  corridor_.x = line_x_;
  corridor_.y = line_y_;
  corridor_.lo = corr_lo_;
  corridor_.hi = corr_hi_;
  veh_.wheel_base = kWheelBase;
  veh_.max_steer = kMaxSteerRad;
  veh_.half_width = kHalfWidth;
  RCLCPP_INFO(get_logger(), "復帰用の経路 %zu 点 / 可動域 %zu 点 / 経路計算 %s",
              line_x_.size(), corr_lo_.size(),
              corridor_.valid() ? "可" : "不可");
}

// 舵角を最大まで切ったときに、前方の障害物を避けきれるかを判定する。
//
// 距離で待つ方式(「前方5mが空くまで」)だと、相手が動かない場合に下がりすぎる。
// 幾何で判定すれば「今の位置から最大舵角で回れば当たらない」時点で発進できるので、
// 必要最小限の後退で済む。
//
// 最大舵角での旋回半径 R = wheelBase / tan(maxSteer)。
// 車両はその円弧を通るので、円の中心から相手までの距離が
// (R ± 自車半幅 + 相手半幅) の帯に入らなければ避けられる。
bool StuckRecoveryController::canSteerAround()
{
  if (!odom_ || !v2x_) {
    return true;   // 情報が無ければ従来どおり進む
  }
  const double ex = odom_->pose.pose.position.x;
  const double ey = odom_->pose.pose.position.y;
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double fx = std::cos(yaw), fy = std::sin(yaw);
  const double lx = -fy, ly = fx;                    // 左向き

  const double R = kWheelBase / std::tan(kMaxSteerRad);
  // 車体が掃く帯の幅(自車半幅 + 相手半幅 + 余裕)
  const double band = kHalfWidth * 2.0 + kAvoidMargin;

  for (const auto & v : v2x_->vehicles) {
    const double dx = v.position.x - ex;
    const double dy = v.position.y - ey;
    const double fwd = dx * fx + dy * fy;
    if (fwd <= 0.0 || fwd > kAvoidCheckRange) {
      continue;                                       // 後ろ、または十分遠い
    }
    const double side = -dx * fy + dy * fx;           // 左が正

    // 左右どちらかに最大舵角で回れば避けられるかを調べる
    bool ok = false;
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      // 旋回中心は自車の真横 R の位置
      const double cx = ex + sgn * R * lx;
      const double cy = ey + sgn * R * ly;
      const double d = std::hypot(v.position.x - cx, v.position.y - cy);
      if (std::abs(d - R) > band * 0.5) {
        ok = true;                                    // その円弧上に相手がいない
        break;
      }
    }
    if (!ok) {
      (void)side;
      return false;                                   // どちらへ切っても当たる
    }
  }
  return true;
}

// 後退した先が壁かどうかを調べる。
// isRearClear() は他車しか見ておらず、壁は一切見ていなかった。
// そのため後退を伸ばすと後ろの壁へ刺さり、前にも後ろにも行けなくなる。
// 自車の横位置と、その地点のコリドア境界を返す。
// isRearWallClose() と同じ基準(左向き法線)で測る。
bool StuckRecoveryController::lateralNow(double & lat, double & lo, double & hi)
{
  const size_t n = line_x_.size();
  if (!odom_ || n < 3 || corr_lo_.size() != n) {
    return false;
  }
  const double px = odom_->pose.pose.position.x;
  const double py = odom_->pose.pose.position.y;
  size_t best = 0;
  double bd = 1e18;
  for (size_t i = 0; i < n; ++i) {
    const double d = (line_x_[i] - px) * (line_x_[i] - px) +
                     (line_y_[i] - py) * (line_y_[i] - py);
    if (d < bd) { bd = d; best = i; }
  }
  const size_t nx = (best + 1) % n;
  const size_t pv = (best + n - 1) % n;
  double tx = line_x_[nx] - line_x_[pv];
  double ty = line_y_[nx] - line_y_[pv];
  const double tl = std::hypot(tx, ty);
  if (tl < 1e-9) { return false; }
  tx /= tl; ty /= tl;
  const double nlx = -ty, nly = tx;
  lat = (px - line_x_[best]) * nlx + (py - line_y_[best]) * nly;
  lo = corr_lo_[best];
  hi = corr_hi_[best];
  return true;
}

// コース方位に対する車体の向きのずれを返す。
// 実測(idx=87)で yaw=88.4deg、コース方位はほぼ 0deg = 外壁へ真正面から刺さっていた。
// この状態で舵を切って下がってもタイヤが横に擦るだけで動けない。
// 通常制御に返したあと、pure_pursuit が実際にたどる経路が壁に当たらないか。
//
// 以前は「今の向きのまま真っ直ぐ進めるか」だけを見ていた。しかし
// pure_pursuit はカーブの先の目標点へ向けて曲がるので、目の前が直線でも
// 曲がった先で壁に当たる。カーブ手前で復帰すると再接触するのはこれが原因。
// ここでは追従則をそのまま模擬して経路を作り、その全体を検査する。
bool StuckRecoveryController::handbackPathClear(double dist) const
{
  if (!obstacles_.valid() || line_x_.size() < 3) { return true; }
  recovery::Pose p;
  if (!currentPose(p)) { return false; }
  const std::size_t n = line_x_.size();
  const double ld = std::max(kHandbackLookahead,
                             kHandbackLookaheadGain * std::abs(latest_velocity_));
  const double step = 0.25;
  double travelled = 0.0;
  while (travelled < dist) {
    // いちばん近い経路点から先へ進み、ld[m] 離れた点を目標にする
    std::size_t best = 0;
    double bd = 1e18;
    for (std::size_t i = 0; i < n; ++i) {
      const double d = (line_x_[i] - p.x) * (line_x_[i] - p.x) +
                       (line_y_[i] - p.y) * (line_y_[i] - p.y);
      if (d < bd) { bd = d; best = i; }
    }
    std::size_t tgt = best;
    for (std::size_t k = 0; k < n; ++k) {
      const std::size_t i = (best + k) % n;
      if (std::hypot(line_x_[i] - p.x, line_y_[i] - p.y) >= ld) { tgt = i; break; }
    }
    // pure_pursuit の操舵則: delta = atan(2 L sin(alpha) / ld)
    const double alpha = std::atan2(line_y_[tgt] - p.y, line_x_[tgt] - p.x) - p.yaw;
    const double ld_now = std::max(std::hypot(line_x_[tgt] - p.x, line_y_[tgt] - p.y), 0.5);
    double delta = std::atan2(2.0 * kWheelBase * std::sin(alpha), ld_now);
    delta = std::clamp(delta, -kMaxSteerRad, kMaxSteerRad);
    // 1歩進める
    const double dyaw = step / kWheelBase * std::tan(delta);
    const double mid = p.yaw + dyaw * 0.5;
    p.x += step * std::cos(mid);
    p.y += step * std::sin(mid);
    p.yaw += dyaw;
    travelled += step;
    if (recovery::wallClearanceAt(obstacles_, veh_, p) < veh_.wall_margin) {
      return false;
    }
  }
  return true;
}

bool StuckRecoveryController::headingErrorToTrack(double & err)
{
  const size_t n = line_x_.size();
  if (!odom_ || n < 3) {
    return false;
  }
  const double px = odom_->pose.pose.position.x;
  const double py = odom_->pose.pose.position.y;
  size_t best = 0;
  double bd = 1e18;
  for (size_t i = 0; i < n; ++i) {
    const double d = (line_x_[i] - px) * (line_x_[i] - px) +
                     (line_y_[i] - py) * (line_y_[i] - py);
    if (d < bd) { bd = d; best = i; }
  }
  const size_t nx = (best + 1) % n;
  const size_t pv = (best + n - 1) % n;
  const double track = std::atan2(line_y_[nx] - line_y_[pv], line_x_[nx] - line_x_[pv]);
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double e = yaw - track;
  while (e > M_PI) { e -= 2.0 * M_PI; }
  while (e < -M_PI) { e += 2.0 * M_PI; }
  err = std::abs(e);
  return true;
}

// 後退したときに他車へぶつからないか調べる。
// 後方 rear_clear_dist_ 以内に車がいたら後退しない(その場で待つ)。
// これを見ないと、下がった先の相手に当たって Crash(10秒 5km/h)を繰り返す。
bool StuckRecoveryController::isRearClear()
{
  if (!odom_ || !v2x_) {
    return true;   // 情報が無ければ従来どおり後退する
  }
  const double ex = odom_->pose.pose.position.x;
  const double ey = odom_->pose.pose.position.y;
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double bx = -std::cos(yaw), by = -std::sin(yaw);   // 後ろ向き
  for (const auto & v : v2x_->vehicles) {
    const double dx = v.position.x - ex;
    const double dy = v.position.y - ey;
    const double back = dx * bx + dy * by;                 // 後方成分
    const double side = std::abs(-dx * by + dy * bx);      // 横方向のずれ
    if (back > 0.0 && back < kRearClearDist && side < kRearClearWidth) {
      return false;                                        // 後ろが塞がっている
    }
  }
  return true;
}

// 復帰に入った状況を1行にまとめる。
//
// 単独で壁に刺さった場合と、他車が絡む場合では条件がまったく違う。
// 単独なら後方も横も自由に使えるが、他車が後ろにいれば下がれないし、
// 横に並ばれていれば横へも逃げられない。あとで状況ごとに成功率を
// 見られるように、発生時の配置をそのまま残す。
std::string StuckRecoveryController::situationText() const
{
  if (!odom_) { return "不明"; }
  const double ex = odom_->pose.pose.position.x;
  const double ey = odom_->pose.pose.position.y;
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double fx = std::cos(yaw), fy = std::sin(yaw);
  double front = 1e3, back = 1e3, side_d = 1e3;
  int n_near = 0;
  if (v2x_) {
    for (const auto & v : v2x_->vehicles) {
      const double dx = v.position.x - ex, dy = v.position.y - ey;
      const double f = dx * fx + dy * fy;              // 前後(前が正)
      const double la = -dx * fy + dy * fx;            // 左右(左が正)
      const double d = std::hypot(dx, dy);
      if (d > 12.0) { continue; }
      ++n_near;
      if (std::abs(la) < 1.6) {
        if (f > 0.0) { front = std::min(front, f); }
        else { back = std::min(back, -f); }
      } else if (std::abs(f) < 3.0) {
        side_d = std::min(side_d, std::abs(la));
      }
    }
  }
  const double clear = obstacles_.valid()
                         ? recovery::wallClearanceAt(obstacles_, veh_,
                             [&] { recovery::Pose p; p.x = ex; p.y = ey; p.yaw = yaw; return p; }())
                         : 9.9;
  char buf[192];
  const char * kind = "単独壁";
  if (front < 12.0 && front <= back && front <= side_d) { kind = "前方車"; }
  else if (back < 12.0 && back <= side_d) { kind = "後方車"; }
  else if (side_d < 12.0) { kind = "横並び"; }
  std::snprintf(buf, sizeof(buf),
                "%s 近傍%d台 前%.1f 後%.1f 横%.1f 壁%.2f",
                kind, n_near,
                front > 99.0 ? -1.0 : front,
                back > 99.0 ? -1.0 : back,
                side_d > 99.0 ? -1.0 : side_d,
                clear);
  return std::string(buf);
}

// 復帰経路が避けるべき他車の一覧を作る。
//
// これが無かったため、復帰の経路計画は **壁しか見ていなかった**。
// 前方に止まっている車がいても、その車を突き抜ける円弧を平然と計画し、
// 実行してぶつかり、また stuck になって復帰が再発動する、を繰り返していた
// (ユーザー報告: 「止まっている他車に復帰動作後なんども突っ込んでいる」)。
// 遠くの車まで入れると経路が見つからなくなるので、近傍だけを渡す。
std::vector<recovery::CarObstacle> StuckRecoveryController::carObstacles() const
{
  std::vector<recovery::CarObstacle> out;
  if (!odom_ || !v2x_) { return out; }
  const auto & self = odom_->pose.pose.position;
  for (const auto & v : v2x_->vehicles) {
    const double dx = v.position.x - self.x;
    const double dy = v.position.y - self.y;
    if (std::hypot(dx, dy) > kCarObstacleRange) { continue; }
    out.push_back(recovery::CarObstacle{v.position.x, v.position.y});
  }
  return out;
}

// 後方の他車までの距離[m]。いなければ大きな値。
double StuckRecoveryController::rearRoom() const
{
  if (!odom_ || !v2x_) { return 1e3; }
  const double ex = odom_->pose.pose.position.x;
  const double ey = odom_->pose.pose.position.y;
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double bx = -std::cos(yaw), by = -std::sin(yaw);
  double room = 1e3;
  for (const auto & v : v2x_->vehicles) {
    const double dx = v.position.x - ex;
    const double dy = v.position.y - ey;
    const double back = dx * bx + dy * by;
    const double side = std::abs(-dx * by + dy * bx);
    if (back > 0.0 && side < kRearClearWidth) { room = std::min(room, back); }
  }
  return room;
}

// 前進再開時に、どちら へ舵を切れば障害物を避けられるかを返す。
// +1 = 左へ, -1 = 右へ, 0 = 障害物なし(通常制御へ引き渡してよい)
float StuckRecoveryController::avoidSteerDirection()
{
  if (!odom_) {
    return 0.0f;
  }
  const double ex = odom_->pose.pose.position.x;
  const double ey = odom_->pose.pose.position.y;
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double fx = std::cos(yaw), fy = std::sin(yaw);

  // 前方の他車で最も近いものを探す
  double near_f = 1e9, near_side = 0.0;
  if (v2x_) {
    for (const auto & v : v2x_->vehicles) {
      const double dx = v.position.x - ex, dy = v.position.y - ey;
      const double f = dx * fx + dy * fy;
      const double sd = -dx * fy + dy * fx;
      if (f > 0.0 && f < kAvoidCheckRange && std::abs(sd) < kFrontObstacleHalf &&
          f < near_f) {
        near_f = f;
        near_side = sd;
      }
    }
  }
  if (near_f > kAvoidCheckRange) {
    return 0.0f;                       // 前方に他車なし
  }
  // 相手と反対側へ逃げる。ただしコース境界の余裕がある側を優先する
  float dir = (near_side >= 0.0) ? -1.0f : 1.0f;
  // 壁が迫っている側へは逃げない。左右どちらに余裕があるかで上書きする。
  {
    double lat = 0.0, lo = 0.0, hi = 0.0;
    if (lateralNow(lat, lo, hi)) {
      const double room_left = hi - lat;
      const double room_right = lat - lo;
      const float room = (room_left > room_right) ? 1.0f : -1.0f;
      if (std::abs(room_left - room_right) > 0.5 &&
          ((room > 0 && dir < 0) || (room < 0 && dir > 0))) {
        dir = room;
      }
    }
  }
  return dir;
}

void StuckRecoveryController::onNominalCommand(
  const AckermannControlCommand::ConstSharedPtr msg)
{
  const auto now = this->now();
  // 動作確認用。指示を受けたら数秒だけ全舵で壁へ向かって走る。
  // そのあとは通常の stuck 検出に任せるので、地図を歪めずに
  // 「壁に刺さった状態からの復帰」を再現できる。
  if (crash_me_) {
    const double t = (now - crash_since_).seconds();
    if (t < kCrashDriveSec && !recovery_start_time_.has_value()) {
      publishGear(GearCommand::DRIVE);
      publishCommand(kCrashSpeed, kCrashAccel,
                     static_cast<float>(crash_steer_ * kMaxSteerRad));
      return;
    }
    crash_me_ = false;
    RCLCPP_WARN(get_logger(), "壁当て(動作確認用) 終了");
  }
  if (runRecovery(now)) {
    return;
  }
  // 復帰の前進区間を経路で渡している間は、pure_pursuit の指令をそのまま通す。
  // 復帰中なので停滞判定は回さない(回すと復帰の最中に再突入してしまう)。
  if (recovery_start_time_.has_value() && traj_following_) {
    control_pub_->publish(*msg);
    return;
  }
  // 停滞候補の間は、これから使う復帰計画の舵角へ先回りして向けておく。
  //
  // ここを素通しにすると、壁に刺さって大きく傾いた車に対して pure_pursuit が
  // 目標経路へ向けて大きく舵を切る。それが復帰の向きと逆になるので、
  // 「一度逆へ振ってから正しい向きへ戻して後退する」ように見え、
  // 舵を切り直すぶん復帰が遅れる。車体はほぼ止まっているので、
  // ここで舵を先に向けておいても走行には影響しない。
  if (pre_steer_valid_ && std::abs(latest_velocity_) <= kStuckSpeedThreshold) {
    AckermannControlCommand out = *msg;
    out.lateral.steering_tire_angle = static_cast<float>(pre_steer_);
    control_pub_->publish(out);
  } else {
    control_pub_->publish(*msg);
  }
  updateStuckDetection(*msg, now);
}

void StuckRecoveryController::updateStuckDetection(
  const AckermannControlCommand & command, const rclcpp::Time & now)
{
  const float velocity = latest_velocity_;
  // Require movement once to avoid detecting the initial stationary state as stuck.
  if (velocity >= kMovingSpeedThreshold) {
    moving_observed_ = true;
  }

  // 動作確認用の強制発動。壁に当たらなくなると復帰が動く場面に出会えないため、
  // 走行中に外から発動できるようにしてある。
  //   ros2 topic pub -1 /debug/force_recovery std_msgs/msg/Bool "{data: true}"
  if (force_recovery_) {
    force_recovery_ = false;
    stuck_start_time_.reset();
    recovery_start_time_ = now;
    beginRecovery(now);
    return;
  }

  const bool motion_requested =
    command.longitudinal.speed >= kCommandSpeedThreshold &&
    command.longitudinal.acceleration >= kCommandAccelerationThreshold;

  // 追い越し層が通路なしと判断すると速度指令0を出すため、従来の
  // motion_requested 条件では複数台が接触したまま永久停止する。
  // 車体前方の同一レーンにいる近接車だけを見て、通常の低速走行や横並びを除外する。
  double nearest_forward_vehicle = std::numeric_limits<double>::infinity();
  if (v2x_ && odom_) {
    const auto & self = odom_->pose.pose.position;
    const auto & q = odom_->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    for (const auto & vehicle : v2x_->vehicles) {
      const double dx = vehicle.position.x - self.x;
      const double dy = vehicle.position.y - self.y;
      const double longitudinal = dx * cy + dy * sy;
      const double lateral = -dx * sy + dy * cy;
      if (longitudinal > kBlockedVehicleMinLongitudinal &&
          longitudinal <= kBlockedVehicleMaxLongitudinal &&
          std::abs(lateral) <= kBlockedVehicleMaxLateral)
      {
        nearest_forward_vehicle = std::min(nearest_forward_vehicle, longitudinal);
      }
    }
  }
  // 近くに他車がいて止まっているか。
  //
  // 【直した穴 その1: 指令速度の不感帯】
  // 従来は `指令速度 <= 0.1` が条件だった。一方 `motion_requested` は
  // `指令速度 >= 1.0`。つまり **指令速度が 0.1〜1.0 の帯にいると、
  // どちらの条件も成立せず復帰が永久に起動しない**。
  // 回避層や追従層はこの帯の値をふつうに出すので、実戦で頻繁に踏む。
  // 「動いていないのに復帰しない」を無くすため、上側を 1.0 まで広げる。
  //
  // 【直した穴 その2: 横並びを見ていない】
  // 従来は「前方 0.3-4.0m・横 ±1.6m の箱」に入る車しか見ておらず、
  // **横に並んで押し合っている接触を拾えなかった**
  // (ユーザー報告: idx105 でカート同士がぶつかったのに復帰が出ない)。
  // 前後左右を問わず一定半径内に他車がいれば対象にする。
  //
  // 誤発動の心配は小さい。**実速度 <= 0.1m/s が 3秒続く**ことも同時に要求しており、
  // レース中にその状態が3秒続くのは実際に詰まっているときだけ。
  bool car_near = std::isfinite(nearest_forward_vehicle);
  if (!car_near && v2x_ && odom_) {
    const auto & self = odom_->pose.pose.position;
    for (const auto & vehicle : v2x_->vehicles) {
      const double d = std::hypot(vehicle.position.x - self.x,
                                  vehicle.position.y - self.y);
      if (d < kBlockedVehicleAnyDirRange) { car_near = true; break; }
    }
  }
  const bool blocked_by_vehicle =
    std::abs(command.longitudinal.speed) < kCommandSpeedThreshold &&
    std::abs(velocity) <= kStuckSpeedThreshold &&
    car_near;
  if (blocked_by_vehicle) {
    if (!blocked_vehicle_start_time_) { blocked_vehicle_start_time_ = now; }
  } else {
    blocked_vehicle_start_time_.reset();
  }
  const bool blocked_vehicle_ready =
    blocked_vehicle_start_time_ &&
    (now - blocked_vehicle_start_time_.value()).seconds() >= kBlockedVehicleDurationSec;

  if (!moving_observed_ || (!motion_requested && !blocked_vehicle_ready)) {
    stuck_start_time_.reset();
    pre_steer_valid_ = false;
    return;
  }

  // 近接車条件は専用タイマーで既に3秒を確認済み。通常条件へ切り替わった
  // 時間を流用せず、この周期で復帰を開始する。
  if (blocked_vehicle_ready) {
    stuck_start_time_ = now - rclcpp::Duration::from_seconds(kStuckDurationSec);
  }

  // --- 「進んでいない」を瞬間速度ではなく実際の移動量で見る
  //
  // 【直したバグ(ユーザー報告)】
  // 「P1とP2がぶつかり、復帰処理をせずずっとアクセルを踏み続けて強引に復帰した」。
  //
  // 従来の入口判定は瞬間速度だけ:
  //   if (|velocity| <= kStuckSpeedThreshold(0.1)) { ...stuck 計測開始... }
  // カート同士がぶつかって押し合っている状態は、車体が擦れながら
  // **0.2〜0.5 m/s で動き続ける**。0.1 を超えるので stuck と判定されない。
  // 一方 motion_requested(指令速度 >= 1.0)は成立し続けるので
  // 制御はアクセルを踏み続け、復帰が一度も起動しないまま力任せに押し抜ける。
  // 実測: 車両接触1回に対し復帰完了0回。
  //
  // 復帰処理の**内部**には既に「距離が伸びていなければ停滞」という判定
  // (kStallDist / stall_since_)があるのに、**入口だけが瞬間速度**という
  // 不整合だった。入口も実際の移動量で見る。
  bool no_progress = false;
  {
    recovery::Pose cp;
    if (currentPose(cp)) {
      if (!entry_ref_valid_ ||
          std::hypot(cp.x - entry_ref_x_, cp.y - entry_ref_y_) > kEntryProgressDist)
      {
        entry_ref_x_ = cp.x;
        entry_ref_y_ = cp.y;
        entry_ref_time_ = now;
        entry_ref_valid_ = true;
      } else if ((now - entry_ref_time_).seconds() >= kEntryProgressSec) {
        // kEntryProgressSec 秒かけて kEntryProgressDist も進めていない
        no_progress = true;
      }
    }
  }

  if (std::abs(velocity) <= kStuckSpeedThreshold || no_progress) {
    if (!stuck_start_time_.has_value()) {
      if (no_progress && std::abs(velocity) > kStuckSpeedThreshold) {
        RCLCPP_WARN(get_logger(),
                    "停滞検知(速度は %.2fm/s あるが %.1f秒で %.2fm しか進んでいない)",
                    velocity, kEntryProgressSec, kEntryProgressDist);
      }
      stuck_start_time_ = now;
      // この時点で計画を立てておき、第1区間の舵角へ先回りして向ける。
      // 計算は 1.3ms 程度なので毎回立て直しても問題にならない。
      pre_steer_valid_ = false;
      {
        recovery::Pose p;
        if (currentPose(p) && corridor_.valid()) {
          const auto cars = carObstacles();
          // **本番の makePlan と同じ引数で立てること。**
          //
          // 【直したバグ(ユーザー報告)】
          // 「復帰のときに最初に理想の逆向きにハンドルを切って後退していた」。
          // ここは実際の計画より前に舵を向けておくための先読みだが、
          // 引数が本番と違っていた:
          //   first_phase : ここは 0(無制約)、本番は forward_blocked なら -1
          //   max_reverse : ここは 8.0 固定、本番は後方の他車で制限した値
          // 条件が違えば別の計画が返るので、**先読みで向けた舵が本番の計画と
          // 逆向きになる**。実車はそこから切り直すので、その分だけ余計に
          // 壁や相手へ寄ってしまう。
          const double pre_keep = (unexplained_block_ >= 2) ? kRearKeepTight : kRearKeep;
          const double pre_max_rev = std::clamp(rearRoom() - pre_keep, 0.0, 8.0);
          int pre_first = blocked_vehicle_ready ? -1 : 0;
          if (pre_max_rev < 0.75 && pre_first < 0) { pre_first = 0; }
          const auto pre = recovery::plan(
            corridor_, obstacles_, veh_, p, 4.0, 16.0, pre_first, 0.0, kPlanMinEscape,
            std::min(kBlockedReverseMax, kBlockedReverseStep * unexplained_block_),
            pre_max_rev, cars);
          if (pre.valid && !pre.phases.empty()) {
            pre_steer_ = pre.phases.front().steer;
            pre_steer_valid_ = true;
            RCLCPP_INFO(get_logger(),
                        "復帰 停滞候補。舵を先に%+.0fdeg(%s)へ向ける "
                        "(通常制御の指令は%+.0fdeg)",
                        pre_steer_ * 180.0 / M_PI,
                        pre.phases.front().forward ? "前進" : "後退",
                        command.lateral.steering_tire_angle * 180.0f /
                          static_cast<float>(M_PI));
          }
        }
      }
      // 逃げる向きの決め打ちはやめた。recovery::plan(, cars) が走行可能領域と
      // 車両運動学から、後退・前進それぞれの舵角を計算する。
    } else if ((now - stuck_start_time_.value()).seconds() >= kStuckDurationSec) {
      // 復帰直後は通常制御に発進の機会を与える。
      // これが無いと「後退 -> 動けない -> また stuck -> 後退」を延々と繰り返し、
      // ギアが REVERSE のまま完全に停止したままになる。
      if (recovery_end_time_ &&
          (now - recovery_end_time_.value()).seconds() < kCooldownSec) {
        return;
      }
      stuck_start_time_.reset();
      blocked_vehicle_start_time_.reset();
      recovery_start_time_ = now;
      beginRecovery(now, blocked_vehicle_ready);
      // どこで、どんな姿勢で詰まったのかが分からないと原因を潰せない。
      // 実測で「他車 18.7m 先、壁際で 57 秒間 12 回のもがき」が起きており、
      // 位置・向き・舵角・逃げる向きを残す。
      double lx = 0.0, ly = 0.0, lyaw = 0.0, lroll = 0.0, lpitch = 0.0;
      if (odom_) {
        lx = odom_->pose.pose.position.x;
        ly = odom_->pose.pose.position.y;
        const auto & q = odom_->pose.pose.orientation;
        lyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        lroll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        const double sp = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
        lpitch = std::asin(sp);
      }
      // 走行可能領域に対してどこで詰まったのかも残す。
      // 「領域内なのに壁に当たっている」なら領域が実際の壁とずれている、
      // 「領域外」なら通常制御が寄せすぎている、と切り分けられる。
      double slat = 0.0, slo = 0.0, shi = 0.0, syaw_err = 0.0;
      const bool has_lat = lateralNow(slat, slo, shi);
      const bool has_yaw = headingErrorToTrack(syaw_err);
      RCLCPP_INFO(get_logger(),
                  "stuck detected: velocity=%.3f 位置=(%.1f,%.1f) yaw=%.1f "
                  "roll=%.1f pitch=%.1f deg 舵角=%.1f deg 逃げ=%s "
                  "横=%s 方位差=%s 原因=%s 前方車=%.2fm",
                  velocity, lx, ly, lyaw * 180.0 / M_PI, lroll * 180.0 / M_PI,
                  lpitch * 180.0 / M_PI,
                  command.lateral.steering_tire_angle * 180.0f / static_cast<float>(M_PI),
                  escape_dir_ > 0 ? "左" : "右",
                  has_lat
                    ? (std::to_string(slat).substr(0, 5) + " [" +
                       std::to_string(slo).substr(0, 5) + "," +
                       std::to_string(shi).substr(0, 5) + "]" +
                       (slat < slo || slat > shi ? " 領域外" : " 領域内")).c_str()
                    : "不明",
                  has_yaw ? (std::to_string(syaw_err * 180.0 / M_PI).substr(0, 5) + "deg").c_str()
                          : "不明",
                  blocked_vehicle_ready ? "近接車デッドロック" : "前進不能",
                  nearest_forward_vehicle);
      situation_ = situationText();
      RCLCPP_INFO(get_logger(), "復帰 状況= %s", situation_.c_str());
    }
  } else {
    stuck_start_time_.reset();
    pre_steer_valid_ = false;
  }
}

bool StuckRecoveryController::currentPose(recovery::Pose & p) const
{
  if (!odom_) { return false; }
  const auto & q = odom_->pose.pose.orientation;
  p.x = odom_->pose.pose.position.x;
  p.y = odom_->pose.pose.position.y;
  p.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return true;
}

// 現在姿勢から、走行可能領域を守って目標経路へ戻る操作を計算する。
// 後退と前進それぞれの舵角・距離が決まる。
bool StuckRecoveryController::makePlan(const rclcpp::Time & now, int first_phase)
{
  const auto cars = carObstacles();
  recovery::Pose p;
  if (!currentPose(p) || !corridor_.valid()) { return false; }
  // やり直しのときは「今より改善する計画」だけを求める。
  // 同じ姿勢から同じ条件で引き直しても同じ答えが出るだけで、
  // 実測では前進の計画を5回続けて出して25秒を使い切った。
  const double gain = (replan_count_ > 0) ? 0.05 : 0.0;
  // 地図では動けるはずなのに動けなかったぶん、後退の下限を上げる。
  const double min_rev = std::min(kBlockedReverseMax,
                                  kBlockedReverseStep * unexplained_block_);
  // 後ろに他車がいるぶんだけ後退を制限する。
  // 「6m 空くまで待つ」方式だと、レース中は後ろに車がいるのが普通なので
  // 待っているうちに上限30秒に達する(3台走行の実測で1件目が失敗)。
  // 後方の他車との間に残す距離。詰まりが解けないときは詰めてよい。
  // 30秒を失うより、軽く当たってでも動くほうがよい
  // (実測: 後方1.6m の相手を避けて待ち続け、前進も塞がれて手詰まりになった)。
  const double keep = (unexplained_block_ >= 2 || replan_count_ >= 2)
                        ? kRearKeepTight : kRearKeep;
  const double rear = rearRoom() - keep;
  const double max_rev = std::clamp(rear, 0.0, 8.0);
  if (max_rev < 0.75 && first_phase < 0) { first_phase = 0; }
  plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                         first_phase, gain, kPlanMinEscape, min_rev, max_rev, cars);
  if (!plan_.valid && first_phase != 0) {
    // 縛ったせいで解が無いなら縛りを外す
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           0, gain, kPlanMinEscape, min_rev, max_rev, cars);
  }
  if (!plan_.valid && gain > 0.0) {
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           first_phase, 0.0, kPlanMinEscape, min_rev, max_rev, cars);
  }
  // 壁に挟まれていて 2.5m も動けないときだけ、脱出量の下限を緩める。
  // 緩めるのは脱出量であって後退量ではない。以前は最後のフォールバックで
  // min_reverse まで 0 に落としており、「前進で動けない -> 後退を伸ばせ」と
  // 判断した直後に前進のみの同じ計画が返ってきていた
  // (実測: 後退を 1.8m/3.5m/5.2m/7.0m と上げても毎回「前進 1.5m」)。
  if (!plan_.valid) {
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           0, 0.0, 1.0, min_rev, max_rev, cars);
  }
  if (!plan_.valid) {
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           0, 0.0, 0.0, min_rev, max_rev, cars);
  }
  // 前進が塞がっていると分かっているなら、前進だけの計画は受け取らない。
  // 出しても同じところで止まるだけで、やり直し回数を空費する。
  if (plan_.valid && blocked_dir_ > 0 && !plan_.phases.empty() &&
      plan_.phases.front().forward)
  {
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           -1, 0.0, 0.0, 0.0, std::max(max_rev, 1.5), cars);
  }
  if (!plan_.valid) {
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0, 0, 0.0, 0.0, 0.0,
                           8.0, cars);
  }
  phase_idx_ = 0;
  phase_travelled_ = 0.0;
  phase_start_wall_clear_ = recovery::wallClearanceAt(obstacles_, veh_, p);
  phase_start_car_clear_ = recovery::carClearanceAt(cars, veh_, p);
  braking_ = false;
  last_valid_ = false;
  phase_start_ = now;
  stall_x_ = p.x;
  stall_y_ = p.y;
  stall_since_ = now;
  if (!plan_.valid) {
    RCLCPP_WARN(get_logger(), "復帰 経路を計算できない");
    return false;
  }
  std::string desc;
  for (const auto & ph : plan_.phases) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s舵%+.0fdeg %.1fm ",
                  ph.forward ? "前進" : "後退", ph.steer * 180.0 / M_PI, ph.length);
    desc += buf;
  }
  RCLCPP_INFO(get_logger(), "復帰 計画%d: %s(評価%.2f)",
              replan_count_, desc.c_str(), plan_.cost);
  return true;
}

void StuckRecoveryController::beginRecovery(
  const rclcpp::Time & now, bool forward_blocked)
{
  replan_count_ = 0;
  blocked_dir_ = forward_blocked ? +1 : 0;
  desperate_ = false;
  // 「地図では余裕があるのに動けない」の回数は復帰1回ごとに数え直す。
  // ただし直前の復帰でずれが分かっているなら、1段ぶんは引き継ぐ。
  unexplained_block_ = std::min(unexplained_block_, 1);
  rear_waiting_ = false;
  rear_relaxed_ = false;
  traj_still_ = false;
  traj_giveup_ = false;
  recovery::Pose p;
  if (currentPose(p)) { recovery_start_x_ = p.x; recovery_start_y_ = p.y; }
  makePlan(now, forward_blocked ? -1 : 0);
}

// 計画した区間を順に実行する。
// 区間ごとにギアと舵角が決まっているので、走った距離が区間長に達したら次へ進む。
bool StuckRecoveryController::runRecovery(const rclcpp::Time & now)
{
  if (!recovery_start_time_.has_value()) { return false; }
  const double total = (now - recovery_start_time_.value()).seconds();
  if (total > kRecoveryMaxSec) {
    RCLCPP_WARN(get_logger(), "復帰 上限%.0fs に到達。通常制御へ返す 状況= %s",
                kRecoveryMaxSec, situation_.c_str());
    finishRecovery(now);
    return false;
  }

  recovery::Pose p;
  if (!currentPose(p)) { return false; }

  // 走った距離を積む
  if (last_valid_) {
    phase_travelled_ += std::hypot(p.x - last_x_, p.y - last_y_);
  }
  last_x_ = p.x;
  last_y_ = p.y;
  last_valid_ = true;

  // 領域内へ戻り、向きも揃っていれば通常制御へ返す。
  // 以前は横位置しか見ておらず、コースに対して直角(yaw=111deg)のまま
  // 「復帰 完了」と返して2秒後に再スタックしていた。
  {
    double lat = 0.0, lo = 0.0, hi = 0.0, yaw_err = 0.0;
    // 壁から十分離れていること。境界からわずかに内側という程度で返すと、
    // 通常制御がすぐまた壁へ寄せて同じ場所で詰まる(実測で同一地点8回)。
    const bool inside = lateralNow(lat, lo, hi) &&
                        lat > lo + kHandbackMargin && lat < hi - kHandbackMargin;
    const bool aligned = !headingErrorToTrack(yaw_err) || yaw_err < kAlignYaw;
    // 通常制御が狙う先へ実際に走り出せること。
    // ただし長引いたら見る距離を縮める。12m 先まで完全に空くのを待ち続けると、
    // 狭い区間では条件が満たせず 30 秒を使い切る(実測1件)。
    // 途中まで空いていれば返して、また詰まったら復帰し直すほうが速い。
    const double look = (total > kHandbackRelaxSec)
                          ? kHandbackAhead * 0.5 : kHandbackAhead;
    const bool can_go = handbackPathClear(look);
    // 復帰開始地点から実際に離れていること。後退しただけで
    // 「領域内・向きOK」を満たして返してしまうのを防ぐ。
    const double gone = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
    if (inside && aligned && can_go && gone > kEscapeDist &&
        std::abs(latest_velocity_) > kMovingSpeedThreshold) {
      RCLCPP_INFO(get_logger(),
                  "復帰 完了 %.1fs 計画%d回 方位差%.0fdeg 移動%.1fm 状況= %s",
                  total, replan_count_, yaw_err * 180.0 / M_PI, gone,
                  situation_.c_str());
      finishRecovery(now);
      return false;
    }
  }

  // --- 最終手段 ---
  // 計画をすべて試しても1歩も動けていないなら、強引に動かす。
  // 舵を左右に振りながら前後へ全開で当て、車体の向きを変えて隙間を作る。
  // 相手を押すことにもなるので、他に手が無いときだけ。
  if (desperate_) {
    const double moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
    if (moved > kDesperateFreeDist) {
      RCLCPP_INFO(get_logger(), "復帰 強引な脱出で %.1fm 動けた。計画に戻す", moved);
      desperate_ = false;
      replan_count_ = 0;
      makePlan(now, 0);
      return true;
    }
    const double t = (now - desperate_since_).seconds();
    const int swing = static_cast<int>(t / kDesperateSwingSec);
    if (swing != desperate_swing_) {
      desperate_swing_ = swing;
      RCLCPP_WARN(get_logger(), "復帰 強引な脱出 %d回目 (最終手段)", swing + 1);
    }
    // 前後を交互に、舵も交互に振る。同じ当て方を続けても抜けないため。
    const bool fwd = (swing % 2) == 0;
    const float steer = static_cast<float>(
      ((swing / 2) % 2 == 0 ? 1.0 : -1.0) * kMaxSteerRad);
    publishGear(fwd ? GearCommand::DRIVE : GearCommand::REVERSE);
    publishCommand(fwd ? kDesperateSpeed : -kDesperateSpeed, kDesperateAccel, steer);
    return true;
  }

  if (!plan_.valid || phase_idx_ >= plan_.phases.size()) {
    // 計画を走り切ったのに戻れていない。今の姿勢から引き直す。
    if (replan_count_ >= kReplanMax) {
      const double moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
      if (moved < kDesperateFreeDist) {
        // 計画を出しても1歩も動けていない。走行可能領域では説明のつかない
        // 何か(壁の当たり方、他車)に阻まれている。最終手段に切り替える。
        RCLCPP_WARN(get_logger(),
                    "復帰 計画%d回で %.2fm しか動けない。強引な脱出に切り替える",
                    replan_count_, moved);
        desperate_ = true;
        desperate_since_ = now;
        desperate_swing_ = -1;
        return true;
      }
      RCLCPP_WARN(get_logger(), "復帰 計画%d回でも戻れない。通常制御へ返す 状況= %s",
                  replan_count_, situation_.c_str());
      finishRecovery(now);
      return false;
    }
    ++replan_count_;
    // 動けなかった向きを覚えているなら、その逆から始める計画を求める。
    if (!makePlan(now, blocked_dir_ != 0 ? -blocked_dir_ : 0)) {
      finishRecovery(now);
      return false;
    }
  }

  const auto & ph = plan_.phases[phase_idx_];

  // 前後を切り替える前に止まりきる。
  // 舵は前の区間のまま保つ。ここで舵を先に動かすと、惰性で動いている間に
  // 逆向きに回ってしまう。
  if (braking_) {
    const double bt = (now - brake_since_).seconds();
    if (std::abs(latest_velocity_) < kBrakeDoneSpeed || bt > kBrakeMaxSec) {
      braking_ = false;
      ++phase_idx_;
      phase_travelled_ = 0.0;
      phase_start_ = now;
      stall_since_ = now;
      return true;
    }
    publishGear(brake_was_forward_ ? GearCommand::DRIVE : GearCommand::REVERSE);
    publishCommand(0.0, kBrakeAccel, static_cast<float>(brake_steer_));
    stall_since_ = now;   // 止まろうとしている間は停滞とみなさない
    return true;
  }

  // 後ろに車がいて、この区間で下がる距離が確保できないなら待つ。
  // 下がって当てると Crash(10秒) を食らううえ、同じ相手に再衝突しやすい。
  // ただし計画時点で後方の余裕を上限にしているので、ここで待つのは
  // 計画後に相手が寄ってきた場合だけになる。
  // 後方に残す距離。待ちが長引いたら詰める。
  // 相手が退くのを待ち続けても、相手も詰まっていれば永久に退かない。
  if (rear_waiting_ && (now - rear_wait_since_).seconds() > kRearWaitMax) {
    if (!rear_relaxed_) {
      rear_relaxed_ = true;
      RCLCPP_WARN(get_logger(),
                  "復帰 後方が %.1fs 空かない。相手との間隔を %.1fm まで詰める",
                  kRearWaitMax, kRearKeepTight);
    }
  }
  const double rear_keep_now =
    (rear_relaxed_ || unexplained_block_ >= 2 || replan_count_ >= 2)
      ? kRearKeepTight : kRearKeep;
  // 後方が塞がっていて、計画した後退量が入らない場合。
  //
  // 以前は「入るまで待つ」だけだった。しかし相手も詰まっていれば退かないので、
  // 実測では後方1.4m・必要1.5m のまま11秒以上待ち続けていた。
  // 緩和して残す距離を 0.5m にしても 1.4-0.5=0.9m < 1.5m で足りず、
  // 条件を満たせないまま上限に達する。
  //
  // 待つのではなく、**空いている距離に合わせて後退量を縮めて計画し直す**。
  // 浅い後退でも、前進で回り込めば姿勢は作れる。
  if (!ph.forward && rearRoom() - rear_keep_now < ph.length) {
    const double room = rearRoom() - rear_keep_now;
    if (rear_relaxed_ && room > 0.5 && replan_count_ < kReplanMax) {
      ++replan_count_;
      RCLCPP_WARN(get_logger(),
                  "復帰 後方が空かない(%.1fm)。後退を %.1fm に縮めて引き直す",
                  rearRoom(), room);
      rear_waiting_ = false;
      makePlan(now, -1);
      return true;
    }
    if (rear_relaxed_ && room <= 0.5 && replan_count_ < kReplanMax) {
      // 下がれる余地がまったく無い。前進から始める計画に切り替える。
      ++replan_count_;
      RCLCPP_WARN(get_logger(),
                  "復帰 後方に余地なし(%.1fm)。前進から引き直す", rearRoom());
      rear_waiting_ = false;
      makePlan(now, +1);
      return true;
    }
  }
  if (!ph.forward && rearRoom() - rear_keep_now < ph.length) {
    if (!rear_waiting_) { rear_waiting_ = true; rear_wait_since_ = now; }
    // やり直しを使い切った後は、ここで待ち続けても打つ手が無い。
    // 下の停滞判定はこの return より後ろにあるので到達せず、
    // 復帰の上限(30秒)まで停止したまま終わってしまう。
    // 待ちが停滞と同じだけ続いたら強引な脱出へ移る。
    if (replan_count_ >= kReplanMax &&
        (now - rear_wait_since_).seconds() > kStallSec)
    {
      RCLCPP_WARN(get_logger(),
                  "復帰 やり直し%d回、後方待ち %.1fs。強引な脱出に切り替える",
                  replan_count_, (now - rear_wait_since_).seconds());
      desperate_ = true;
      desperate_since_ = now;
      desperate_swing_ = -1;
      return true;
    }
    publishGear(GearCommand::DRIVE);
    publishCommand(0.0, 0.0);
    // 待ち始めてすぐの間だけ停滞から除外する。
    // ずっと除外すると、やり直し回数が増えないので「強引な脱出」にも
    // 「後方を詰める」処理にも到達せず、30秒まるごと待ち続ける
    // (実測: 状況=後方車 後1.9m 壁0.40m のまま上限に達し続けた)。
    if (!rear_relaxed_) { stall_since_ = now; }
    if ((now - last_trace_).seconds() > 1.0) {
      last_trace_ = now;
      RCLCPP_INFO(get_logger(),
                  "復帰 後方に他車(%.1fm)。%.1fm 下がれるまで待つ(%.1fs)",
                  rearRoom(), ph.length, (now - rear_wait_since_).seconds());
    }
    return true;
  }

  // 待機を抜けたので待ちタイマーは止める。緩和(rear_relaxed_)は
  // この復帰が終わるまで保持する。ここで戻すと基準が元に戻り、
  // また待機に入ってタイマーがリセットされる無限ループになる。
  rear_waiting_ = false;

  // --- 前進中に壁へ近づいたら、動けなくなる前に引き直す
  //
  // 【ユーザー報告】「壁にぶつからない処理を入れているのに、
  //   低速の復帰動作でまた壁に突っ込む」。
  //
  // 【実測(20260828-175600-s0/d2、ヘアピン idx130 付近)】
  //   復帰 計画0: 前進舵-18deg 3.0m            <- 後退なしの前進のみ計画
  //   走行0.00m yaw-34 壁まで 0.45             <- 計画時は 0.45m あった
  //   走行0.18m yaw-47 壁まで 0.14
  //   走行0.24m yaw-51 壁まで-0.10             <- 食い込み
  //   走行0.30m yaw-55 壁まで-0.14
  //   復帰 前進 で動けない。後退から引き直す    <- 楔状に嵌まってから初めて反応
  //
  // 【なぜ計画の壁判定をすり抜けたか】
  //  (1) 計画時の余裕が 0.45m あったので `touching` が偽になり、
  //      「接触中は出だしで食い込みを増やさない」ガードが素通りした。
  //  (2) 経路の検証は自転車モデル(advance())で行うが、
  //      **壁際・低速では車がその通りに動かない**。実測では 0.30m 進む間に
  //      yaw が 21 度も回っている。指令舵 -18deg の自転車モデルでは
  //      説明がつかない(車が並進せずその場で振れている)。
  //      つまり計画時に「3.0m 先まで当たらない」と検証しても、
  //      実際の軌跡は別物になる。
  //
  // 【対策】計画の妥当性をモデルに任せきりにせず、**実測のクリアランス**で
  // 見張る。前進中に余裕が閾値を割ったら、停滞を待たずに即座に後退から引き直す。
  // 停滞判定(kStallSec)を待つと、その頃には壁に押し付けられて動けない。
  if (ph.forward && !desperate_ && replan_count_ < kReplanMax) {
    const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
    // 壁も同じ。狭い所ではもともと余裕が小さいので、
    // 「閾値を割った」だけでは降りない。**区間開始より悪化している**ことを条件にする。
    if (clear_now < kFwdAbortClearance &&
        clear_now < phase_start_wall_clear_ - kFwdAbortWorsen &&
        phase_travelled_ > kFwdAbortMinTravel)
    {
      ++replan_count_;
      blocked_dir_ = 1;                     // 前進は駄目だったと覚える
      RCLCPP_WARN(get_logger(),
                  "復帰 前進中に壁まで %.2fm まで詰まった(走行%.2fm)。"
                  "動けなくなる前に後退から引き直す(%d回目)",
                  clear_now, phase_travelled_, replan_count_);
      makePlan(now, -1);                    // 後退から始める計画を要求
      return true;
    }
  }

  // --- 前進中に他車へ近づいたら、ぶつかる前に引き直す
  // 経路計画にも他車を入れたが、壁のときと同じで**計画どおりに動かない**
  // (壁際・低速では車が並進せず振れる)。実測のクリアランスでも見張る。
  if (ph.forward && !desperate_ && replan_count_ < kReplanMax) {
    // **符号付きの余裕**を使う。`carViolation` は 0 で初期化した「重なりの深さ」で
    // 離れていても 0 を返すので、距離として使うと他車が1台もいなくても
    // 条件が真になり、前進のたびに中断して無限に切り返す(実際に出したバグ)。
    const double cc = recovery::carClearanceAt(carObstacles(), veh_, p);
    // 近いだけでは降りない。**区間開始より悪化している**ときだけ降りる。
    // そうしないと、もともと狭い所では出だしで必ず降りて同じ計画を引き直し続ける。
    if (cc < kFwdAbortCarDist && cc < phase_start_car_clear_ - kFwdAbortWorsen &&
        phase_travelled_ > kFwdAbortMinTravel)
    {
      ++replan_count_;
      blocked_dir_ = 1;
      RCLCPP_WARN(get_logger(),
                  "復帰 前進中に他車まで %.2fm(開始時%.2fm)。"
                  "ぶつかる前に引き直す(%d回目)",
                  cc, phase_start_car_clear_, replan_count_);
      makePlan(now, -1);
      return true;
    }
  }

  // 動けていないなら計画を引き直す。同じ指令を出し続けても出られない。
  // ただし、区間を始めた直後とギアを入れ替えた直後は数に入れない。
  // 停止 -> ギア切替 -> 舵を入れる までに 1 秒近くかかるので、
  // そこを停滞と数えると計画を一度も実行しないまま向きだけ反転し続ける。
  const double since_phase = (now - phase_start_).seconds();
  const double since_gear = (now - gear_changed_).seconds();
  const bool warming_up = since_phase < kPhaseMinSec || since_gear < kActuatorWaitMax;
  if (std::hypot(p.x - stall_x_, p.y - stall_y_) > kStallDist) {
    stall_x_ = p.x;
    stall_y_ = p.y;
    stall_since_ = now;
  } else if (warming_up) {
    stall_since_ = now;
  } else if ((now - stall_since_).seconds() > kStallSec) {
    if (replan_count_ >= kReplanMax && !desperate_) {
      const double moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
      if (moved < kDesperateFreeDist) {
        RCLCPP_WARN(get_logger(),
                    "復帰 やり直し%d回で %.2fm しか動けない。強引な脱出に切り替える",
                    replan_count_, moved);
        desperate_ = true;
        desperate_since_ = now;
        desperate_swing_ = -1;
        return true;
      }
    }
    if (replan_count_ < kReplanMax) {
      ++replan_count_;
      // 動けなかった向きを覚え、次は逆から始めさせる。
      // 走行可能領域だけでは「鼻先が壁に当たっている」ことが分からないので、
      // 実際に動けたかどうかでしか判断できない。
      blocked_dir_ = ph.forward ? 1 : -1;
      const int want = ph.forward ? -1 : 1;
      // 地図の上では余裕があるのに動けないなら、自己位置推定が実際と
      // ずれているとみなす。地図を信じ直しても同じ計画が出るだけなので、
      // 後退の下限を上げて物理的に壁から離す。
      const double clear = recovery::wallClearanceAt(obstacles_, veh_, p);
      if (clear > kExplainedClearance) {
        ++unexplained_block_;
        RCLCPP_WARN(get_logger(),
                    "復帰 %s で動けないが地図では壁まで %.2fm ある。"
                    "自己位置推定のずれとみて後退を %.1fm 以上にする(%d回目)",
                    ph.forward ? "前進" : "後退", clear,
                    std::min(kBlockedReverseMax,
                             kBlockedReverseStep * (unexplained_block_)),
                    unexplained_block_);
      } else {
        RCLCPP_INFO(get_logger(), "復帰 %s で動けない。%sから引き直す",
                    ph.forward ? "前進" : "後退", want > 0 ? "前進" : "後退");
      }
      makePlan(now, want);
      return true;
    }
  }

  // 区間を走り切ったら次へ。ただし前後が入れ替わるなら先に止まりきる。
  if (phase_travelled_ >= ph.length - kPhaseDoneSlack) {
    const bool next_forward =
      (phase_idx_ + 1 < plan_.phases.size()) ? plan_.phases[phase_idx_ + 1].forward
                                             : ph.forward;
    if (next_forward != ph.forward && !braking_) {
      braking_ = true;
      brake_since_ = now;
      brake_steer_ = ph.steer;
      brake_was_forward_ = ph.forward;
      return true;
    }
    ++phase_idx_;
    phase_travelled_ = 0.0;
    phase_start_ = now;
    phase_start_wall_clear_ = recovery::wallClearanceAt(obstacles_, veh_, p);
    phase_start_car_clear_ = recovery::carClearanceAt(carObstacles(), veh_, p);
    return true;
  }

  // 舵は最初から目標値を出す。
  //
  // 以前は 0 から 0.35 秒かけて入れていた(停止状態でフルロックを当てると
  // 壁に押し付けられた車体が動けない、という理由)。しかしその後に
  // 「ギアが入るまでの 0.7 秒は速度0」を入れたので、その間に舵を振り切って
  // しまえば、動き出す時点では既に正しい舵角になっている。
  //
  // しかも経過時間を max(区間開始, ギア切替) で測っていたため、復帰の1周期目
  // だけ gear_changed_ が過去のままで ramp=1 になり、いきなり目標舵角へ振れた
  // 直後に publishGear が gear_changed_ を更新して ramp が 0 に戻っていた。
  // 衝突時の舵(pure_pursuit の値 = 壁へ向かう向き)から見ると
  // 「逆向きに切ってから理想の向きへ戻す」ように見え、そのぶん遅れていた。
  const float steer = static_cast<float>(
    std::clamp(ph.steer, -kMaxSteerRad, kMaxSteerRad));

  publishGear(ph.forward ? GearCommand::DRIVE : GearCommand::REVERSE);
  // 何を指令して、車がどう応じたかを残す。
  // 「前方 1.85m 空いているのに前進できない」が実測で起きており、
  // 幾何では説明がつかない。指令・ギア・実速度を並べないと切り分けられない。
  if ((now - last_trace_).seconds() > 0.5) {
    last_trace_ = now;
    RCLCPP_INFO(get_logger(),
                "復帰追跡 区間%zu/%zu %s 舵指令%+.0f 速度%.2f "
                "走行%.2f/%.2fm ギア%u 位置(%.1f,%.1f) yaw%.0f 壁まで%.2f",
                phase_idx_ + 1, plan_.phases.size(), ph.forward ? "前進" : "後退",
                steer * 180.0 / M_PI,
                latest_velocity_, phase_travelled_, ph.length, gear_now_,
                p.x, p.y, p.yaw * 180.0 / M_PI,
                recovery::wallClearanceAt(obstacles_, veh_, p));
  }
  // ギアが入り、舵が目標へ届くまでは止めておく。
  // 入れ替えと同時に加速指令を出すと、前のギアのまま加速して逆へ進み、
  // 余計に壁へ押し付けることがある。舵が届く前に動くと、計画とは違う
  // 円弧を描いてしまう。
  //
  // 以前は 0.7 秒の固定待ちだったが、これは根拠のない置き値だった。
  // 実測では実ギアが変わるまで 0.012〜0.099s、動き出すまで 0.230〜0.273s で、
  // 3倍以上待ちすぎていた。報告を見て、準備でき次第動く。
  {
    const double waited = (now - gear_changed_).seconds();
    const bool gear_ready = !gear_report_seen_ || gear_report_ == gear_now_;
    const bool steer_ready = std::abs(steer_report_ - steer) < kSteerReadyRad;
    if ((!gear_ready || !steer_ready) && waited < kActuatorWaitMax) {
      publishCommand(0.0, 0.0, steer);
      return true;
    }
  }
  if (ph.forward) {
    // 前進区間は経路を publish して pure_pursuit に追従させる。
    // 舵と速度をこちらで作ると、通常制御へ返す瞬間に不連続が生じ、
    // pure_pursuit が急に遠くの目標を向いて壁へ寄っていた。
    // 軌道で渡しているのに動かないなら、直接制御へ戻す。
    // pure_pursuit が動かせない場合、こちらが指令を出さないままなので
    // 車が完全に止まる(実測: 方位差4度・壁まで0.76m のまま30秒停止)。
    if (traj_following_ && !traj_giveup_) {
      if (std::abs(latest_velocity_) > kMovingSpeedThreshold) {
        traj_still_ = false;
      } else {
        if (!traj_still_) { traj_still_ = true; traj_still_since_ = now; }
        else if ((now - traj_still_since_).seconds() > kTrajStillSec) {
          traj_giveup_ = true;
          RCLCPP_WARN(get_logger(),
                      "復帰 経路で渡しても %.1fs 動かない。直接制御に戻す",
                      kTrajStillSec);
        }
      }
      if (!traj_giveup_) { return false; }   // 指令は pure_pursuit のものを通す
    }
    publishCommand(kRecoverySpeed, kRecoveryAccel, steer);
  } else {
    // AWSIM は後退ギアのとき「負の目標速度・正の加速度」を期待する。
    publishCommand(-kRecoverySpeed, kRecoveryAccel, steer);
  }
  return true;
}

// 復帰の前進区間を「たどってほしい経路」として publish する。
//
// これまでは復帰中ずっと舵と速度を直接作っていた。そのため通常制御へ返す
// 瞬間に不連続が生じ、pure_pursuit が急に遠くの目標を向いて壁へ寄っていた。
// 前進区間を経路として渡せば pure_pursuit がそのまま追従し、
// 経路の終端が目標軌道につながっているので戻りが連続になる。
// 後退区間は pure_pursuit では表現できない(ギアと舵の符号が変わる)ので
// 従来どおり直接制御する。
bool StuckRecoveryController::publishRecoveryTrajectory()
{
  if (!latest_traj_ || !recovery_start_time_.has_value() || !plan_.valid) {
    return false;
  }
  if (phase_idx_ >= plan_.phases.size() || !plan_.phases[phase_idx_].forward) {
    return false;   // 後退中・区間切替中は直接制御に任せる
  }
  if (braking_ || desperate_) { return false; }

  // 計画した前進経路 + そのあと目標軌道へ合流する部分をつなげる
  Trajectory out;
  out.header = latest_traj_->header;
  out.header.stamp = this->now();

  recovery::Pose cur;
  if (!currentPose(cur)) { return false; }

  // 計画の経路のうち、現在地より先の部分だけを使う
  std::size_t from = 0;
  double bd = 1e18;
  for (std::size_t i = 0; i < plan_.path.size(); ++i) {
    const double d = std::hypot(plan_.path[i].x - cur.x, plan_.path[i].y - cur.y);
    if (d < bd) { bd = d; from = i; }
  }
  auto push = [&out](double x, double y, double yaw, double v) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw * 0.5);
    p.pose.orientation.w = std::cos(yaw * 0.5);
    p.longitudinal_velocity_mps = static_cast<float>(v);
    out.points.push_back(p);
  };
  for (std::size_t i = from; i < plan_.path.size(); ++i) {
    push(plan_.path[i].x, plan_.path[i].y, plan_.path[i].yaw, kRecoverySpeed);
  }
  if (out.points.size() < 2) { return false; }

  // 終端から目標軌道へつなぐ。つながっていないと pure_pursuit が
  // 経路の終端で止まってしまい、復帰しても走り出せない。
  const auto & tail = out.points.back().pose.position;
  std::size_t near = 0;
  double nd = 1e18;
  const auto & tp = latest_traj_->points;
  for (std::size_t i = 0; i < tp.size(); ++i) {
    const double d = std::hypot(tp[i].pose.position.x - tail.x,
                                tp[i].pose.position.y - tail.y);
    if (d < nd) { nd = d; near = i; }
  }
  for (std::size_t k = 1; k < tp.size(); ++k) {
    out.points.push_back(tp[(near + k) % tp.size()]);
  }
  traj_pub_->publish(out);
  if ((this->now() - last_traj_log_).seconds() > 1.0) {
    last_traj_log_ = this->now();
    RCLCPP_INFO(get_logger(), "復帰 前進を経路で渡す (%zu点, 計画%zu点ぶん)",
                out.points.size(), plan_.path.size() - from);
  }
  return true;
}

void StuckRecoveryController::finishRecovery(const rclcpp::Time & now)
{
  publishGear(GearCommand::DRIVE);
  recovery_start_time_.reset();
  recovery_end_time_ = now;
}

void StuckRecoveryController::publishCommand(float speed, float acceleration, float steer)
{
  const auto stamp = this->now();
  AckermannControlCommand msg;
  msg.stamp = stamp;
  msg.lateral.stamp = stamp;
  msg.lateral.steering_tire_angle = steer;
  msg.lateral.steering_tire_rotation_rate = 2.0;
  msg.longitudinal.stamp = stamp;
  msg.longitudinal.speed = speed;
  msg.longitudinal.acceleration = acceleration;
  control_pub_->publish(msg);
}

void StuckRecoveryController::publishGear(std::uint8_t command)
{
  if (command != gear_now_) {
    gear_now_ = command;
    gear_changed_ = this->now();
  }
  GearCommand msg;
  msg.stamp = this->now();
  msg.command = command;
  gear_pub_->publish(msg);
}

}  // namespace stuck_recovery_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stuck_recovery_controller::StuckRecoveryController>());
  rclcpp::shutdown();
  return 0;
}
