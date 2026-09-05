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
// --- 復帰へ入る唯一の条件(ユーザー指示 2026-09-06 で統合) ---
//
// 【統合前】同じ「進んでいない」条件が3つあった。
//   停滞      : 移動量が伸びない(2.5秒 / 0.8m)。ただし **motion_requested の門つき**
//   膠着      : 位置が 4.0秒 で 0.8m 進まない(門なし)
//   壁接触膠着: 壁に触れたまま 1.5秒 前進しても動かない
// 門は「指令速度 >= 1.0 かつ指令加速度 >= 0.3」で、**我々自身の追突防止や
// 壁前進禁止が指令を 0 にすると閉じる**。実測で 55秒間そのまま停止していた。
// 膠着と壁接触膠着は、その門を回避するために私が足した重複だった。
//
// **門を外して1つにする。** 誤検知しない根拠:
//   ペナルティのクランプ中でも 5km/h = 1.39m/s 出る -> 1秒で 1.39m
//   徐行通過の最低上限 10.8km/h = 3.0m/s
// 閾値 0.5m/1.0秒 = 0.5m/s は、クランプ中の 2.8分の1、徐行の 6分の1。
// 正常な低速走行と混ざらない。従来の膠着(4.0秒)より **4倍速く** 反応する。
constexpr double kEntryProgressSec = 1.0;
constexpr double kEntryProgressDist = 0.5;
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
// 悪化の判定に余裕を持たせる。わずかな悪化で降りると切り返しが増える。
constexpr double kFwdAbortWorsen = 0.15;
// 前後左右を問わず、この距離[m]以内に他車がいれば「近接」とみなす。
// 横並びで押し合っている接触を拾うために要る(前方の箱だけでは取りこぼす)。
constexpr double kBlockedVehicleAnyDirRange = 2.5;
constexpr double kRearKeepTight = 0.5;      // 手詰まりのときはここまで詰めてよい
constexpr double kRearWaitMax = 2.0;        // 後方が退くのをこの時間[s]しか待たない
constexpr double kRearClearWidth = 1.6;     // 後方判定の横幅[m]
// --- 運動の予測に使う幾何(公式値。2026-09-06 訂正) ---
// 公式 racing_kart_description/config/vehicle_info.param.yaml:
//   wheel_base 1.087 / max_steer_angle 0.64
// 従来ここには 2.14 と 0.6109 が入っていたが、2.14 は**指令の単位変換を
// 吸収した値**(1.087 x 1.97)であって幾何ではない。運動の予測に使うと
// 実際より 12% きつい弧を予測し、車は外側へ膨らんで壁に当たる。
// 指令の単位変換は publishCommand の steer_cmd_scale_ で別に掛ける。
constexpr double kWheelBase = 1.087;         // ホイールベース[m]
// --- 実際に出せる実舵角の上限(実測 2026-09-06) ---
//
// 公式 vehicle_info.param.yaml の max_steer_angle は 0.64rad(36.7deg)だが、
// **この指令経路では到達しない**。実測:
//   指令 35deg -> 報告された実舵角 18deg
//   指令 72deg -> 報告された実舵角 18deg   ← 倍にしても変わらない
// つまり実舵角は 0.31rad(18deg)で飽和する。指令を上げる意味はない。
// (「AWSIM が指令を 1.97 で割る」という仮説は、この2点測定で棄却した。
//  1.97 = 2.14/1.087 と一致したのは偶然だった)
//
// 計画はこの実測値を使う。実舵 18deg と幾何ホイールベース 1.087m から
// 最小旋回半径 3.35m。実測の旋回半径 3.65〜3.73m と 10% 以内で一致する。
constexpr double kMaxSteerRad = 0.31;
constexpr double kHalfWidth = 0.73;         // 車体半幅[m]
constexpr double kAvoidMargin = 0.3;        // 回避時の余裕[m]
constexpr double kAvoidCheckRange = 8.0;    // 前方この距離[m]までを判定対象にする
constexpr double kFrontObstacleHalf = 1.5;  // 前方障害物とみなす横幅の半分[m]

// --- 計画追従の定数 ---
constexpr double kRecoveryMaxSec = 30.0;   // 復帰全体の上限[s]。超えたら通常制御へ返す
// 上限に達したら止まるのではなく、動かない打切り側で 0 に戻して作り直す。
constexpr double kReplanMax = 6;           // 計画のやり直し上限
// 動けているかの判定は「ギアが入ってから」「舵を入れ終わってから」数える。
// 1.2s では、停止 -> ギア切替 -> 舵を入れる の途中で毎回「動けない」と判定され、
// 実測では69回の判定すべてが計画を1つも実行しないまま向きを反転させていた。
constexpr double kStallSec = 4.0;          // この時間[s]動けなければ計画をやり直す
constexpr double kPhaseMinSec = 1.2;       // 区間を始めてこの時間[s]は停滞判定をしない
// 実ギア・実舵角が指令に追いつくのを待つ上限[s]。
// 実測(gearlat2.py): 実ギアの切替 0.012〜0.099s、舵は角速度上限 2.0rad/s なので
// フルロック 0.61rad で 0.31s。両方を見て、届いたら即動く。
constexpr double kActuatorWaitMax = 0.45;
constexpr double kSteerReadyRad = 0.05;    // 舵が目標に届いたとみなす差[rad](2.9deg)
constexpr double kStallDist = 0.15;        // 動けているとみなす距離[m]
// 【余裕を広げた 2026-09-05 ユーザー指示】区間の走破判定が厳しいと、
// あと少しで終わる区間を「未完了」のまま打ち切って計画を作り直し、
// **数回の切り返し**になる。実測では完走率が 0% だった。
constexpr double kPhaseDoneSlack = 0.30;   // 区間の走破判定の余裕[m]
// 前進区間の見込み余裕がこれを下回るなら、モデルのずれで壁へ届く可能性が高い。
// 実測: 予測 +0.22m の計画が 4.5m 走った時点で -0.28m(ずれ 0.5m)。
constexpr double kThinForwardClear = 0.35;
// 薄いときに許す前進の長さ[m]。実測ではこの距離までずれは 0.1m 台に収まる。
constexpr double kThinForwardLen = 2.5;
// 走り切った計画が「やり直し」に数えられない最小の移動量[m]。
constexpr double kPlanProgressDist = 0.5;
// 「改善した」とみなす壁との余裕の増分[m]。
constexpr double kPlanImproveClear = 0.10;
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
// 本線へ戻すときに横ずれを吸収する距離[m]。継ぎ足すだけだと継ぎ目で経路が折れ、
// 速度も段差になる(ユーザー報告「既定の経路への復帰がなめらかでない」)。
constexpr double kBlendLen = 8.0;
// 占有格子を rviz に出すときに、余裕を濃淡で表す範囲[m]。
constexpr double kGridShowRange = 1.5;
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
// 0.05m は姿勢が定まる前で、実測では引き直しの全件がこの距離で起きていた。
constexpr double kFwdAbortMinTravel = 0.15;
// 前進を指令しているのに動かない、と判断する条件。
// 走行距離の条件(kFwdAbortMinTravel)は「動かない」場合に永久に成立しないので、
// 時間で見る枝を別に持つ(2026-09-05 の 340秒の凍結対策)。
constexpr double kFwdNoMoveDist = 0.05;   // これ未満しか走っていない[m]
constexpr double kFwdNoMoveSec = 2.0;     // その状態がこれだけ続いたら[s]
// 復帰の開始時点で壁までの余裕がこれを割っていたら、前進を試さず後退から始める。
// 0.0 は「走行可能領域の縁」。負は既に食い込んでいる状態。
constexpr double kWedgedWallClear = 0.0;
// 計画は「中断されない経路」を出すべきである。中断は壁 kFwdAbortClearance /
// 他車 kFwdAbortCarDist で入るので、計画側にはそれより少し広い余裕を求める。
// これが無いと、計画が通した経路を中断側が 0.23m 手前で落とし、
// 走行 0.05m で必ず引き直しになる(実測: 6回/レース・平均10.5秒)。
constexpr double kPlanWallClear = 0.28;   // 前進区間で確保を試みる壁との余裕[m]
constexpr double kPlanCarClear = 0.32;    // 同 他車との余裕[m]
// 上の条件では見つからない場所もある。そのときは現行どおり緩い条件で計画し、
// 中断閾値の方を「その計画が見込んだ余裕」に合わせて下げる(下の Floor まで)。
constexpr double kFwdAbortWallFloor = 0.02;  // 壁はここまでは許す(実接触の直前)
constexpr double kFwdAbortCarFloor = 0.10;   // 他車の接触は Crash 10秒なので厚めに残す
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
  status_pub_ = create_publisher<std_msgs::msg::String>(
      "/control/debug/recovery_status", rclcpp::QoS(1));

  // 動作確認用の強制発動。壁に当たらなくなると復帰が動く場面に出会えないため。
  // 軌道の経路にも入る。通常は素通しし、復帰の前進区間だけ差し替える。
  // 軌道のトピックは全ノードが best_effort + volatile + KeepLast(1) で
  // そろえてある。既定QoS(reliable)にすると DDS が「QoS 不一致」として
  // メッセージを1件も配送せず、経路がここで切れて pure_pursuit に何も
  // 届かなくなる(実測: rviz に軌道が出ず、車が動かなくなった)。
  const auto traj_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  traj_pub_ = create_publisher<Trajectory>("output/trajectory", traj_qos);
  // 後退用 pure_pursuit へ渡す経路。進行方向(=後退の向き)に点が並ぶ。
  reverse_traj_pub_ = create_publisher<Trajectory>("output/reverse_trajectory", traj_qos);
  // rviz 用。後退->前進->本線復帰 を 1本の Y字として出す。
  // 「復帰で何をしようとしているか」が見えないという指摘への対応(2026-08-31)。
  recovery_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/debug/recovery_path", rclcpp::QoS(1));
  // --- 復帰の当たり判定が見ている占有格子を rviz へ出す(2026-08-31・ユーザー指示)
  //
  // 【ユーザー指摘】rviz の地図が lanelet なので、実際の壁と
  // 走行可能領域の関係が確認できない。
  //
  // この地図は当たり判定に使っている符号付き距離場そのものなので、
  // 「判定が見ているもの」と「表示」が原理的にズレない。
  // rviz が後から起動しても届くよう transient_local(latched)にする。
  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/planning/debug/recovery_grid",
      rclcpp::QoS(1).transient_local().reliable());
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
  // 後退用 pure_pursuit の指令。後退区間ではこちらを中継する(2026-08-31)。
  // 【ユーザー指示】後退も pure_pursuit で追従させ、経路(後退->前進->本線復帰)を
  // 1本として計算・表示する。復帰ノードは元から最終段の中継役なので、
  // 区間に応じてどちらの指令を通すかを選ぶだけで済む(調停の追加は不要)。
  reverse_sub_ = create_subscription<AckermannControlCommand>(
    "/control/command/reverse_control_cmd", 1,
    [this](const AckermannControlCommand::ConstSharedPtr msg) { reverse_cmd_ = msg; });
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

  // 理由を問わない膠着の安全網。既定 0.0 = 無効(現行と完全に同一の挙動)。
  // 後退を経路追従(後退用 pure_pursuit)で走らせるか。
  // false にすると従来どおり舵角と速度を直接指令する。
  // 新しい経路は評価が済んでいないので、問題が出たらここで即座に戻せるようにする。
  reverse_traj_enable_ = declare_parameter<bool>("reverse_traj_enable", true);
  // 復帰の開始時点で既に壁へ食い込んでいるとき、前進を試さず後退から始めるか。
  // false で 2026-09-04 以前の挙動(前進を2秒試してから切り返す)。
  wedge_backward_first_ = declare_parameter<bool>("wedge_backward_first", true);
  // 【既定を切った 2026-09-05 / ユーザー報告「P1P3が全く動かず最悪の状況」】
  //
  // この禁止が**復帰計画の前進区間を毎回0にし**、2秒後に「動かない打切り」で
  // 再計画 → また前進区間 → 0、というライブロックを作っていた。
  // 実測(凍結した車): 前進禁止 216回 / 復帰の計画 104回 / 動かない打切 87回
  // (経路を計算できないは 0。**計画は作れているのに実行できない**)
  //
  // しかも「動かない打切り」を 2秒に速めたことで、30秒待ちだった詰まりが
  // 2秒周期の高速ループに変わり、症状が悪化した。
  //
  // 禁止と計画が矛盾したまま、どちらも譲らない構造が誤り。
  // 壁への前進継続を止める狙い自体は正しい(実機で再起不能)が、
  // **計画側が「後退から始める」ことを保証できるようになるまで既定は false。**
  wall_forward_ban_ = declare_parameter<bool>("wall_forward_ban", false);
  wall_forward_ban_hold_ = declare_parameter<double>("wall_forward_ban_hold", 0.8);
  wall_forward_ban_gain_ = declare_parameter<double>("wall_forward_ban_gain", 0.03);
  wall_forward_ban_speed_ = declare_parameter<double>("wall_forward_ban_speed", 1.0);
  wall_forward_ban_depth_ = declare_parameter<double>("wall_forward_ban_depth", 0.15);
  // 【計測で戻した 2026-09-05】ギアを後退へ入れる実装にしたところ、
  // リタイアが 20% -> 35%、壁ペナルティが 339秒/325秒という値になった。
  //
  // **復帰制御は自前でギアを管理している**(`復帰追跡 ... ギア2 / ギア20`)。
  // このガードが毎周期ギアを上書きすると、復帰の計画実行とギアを取り合って壊す。
  // ギアを入れないと後退しない(実測: 指令だけでは実速度 0.00m/s のまま)ので、
  // **「ガードから後退させる」という形自体が誤り。**
  //
  // 正しくは復帰制御の計画として後退させること。段2(総当りに方位差を入れた)は
  // その方向の変更で、そちらは残してある。
  // ガードは「壁へ食い込んだままの前進を止める」だけに戻す(計測で最良)。
  wall_ban_reverse_ = declare_parameter<bool>("wall_ban_reverse", false);
  wall_ban_reverse_after_ = declare_parameter<double>("wall_ban_reverse_after", 1.5);
  wall_ban_reverse_speed_ = declare_parameter<double>("wall_ban_reverse_speed", 1.0);
  wall_ban_reverse_rear_ = declare_parameter<double>("wall_ban_reverse_rear", 2.0);
  // 前が詰まっているだけの車を復帰にかけない条件。
  queue_wait_enable_ = declare_parameter<bool>("queue_wait_enable", true);
  queue_wait_margin_ = declare_parameter<double>("queue_wait_margin", 0.20);
  queue_wait_yaw_ = declare_parameter<double>("queue_wait_yaw", 0.35);   // 20度
  queue_wait_wall_ = declare_parameter<double>("queue_wait_wall", 0.30);
  queue_wait_max_ = declare_parameter<double>("queue_wait_max", 12.0);
  reject_car_overlap_plan_ = declare_parameter<bool>("reject_car_overlap_plan", false);
  RCLCPP_INFO(get_logger(),
    "壁へ食い込んだままの前進を禁じる: %s (改善猶予 %.1fs / 改善とみなす増分 %.2fm)",
    wall_forward_ban_ ? "する" : "しない", wall_forward_ban_hold_, wall_forward_ban_gain_);
  goal_plan_enable_ = declare_parameter<bool>("goal_plan_enable", true);
  desperate_enable_ = declare_parameter<bool>("desperate_enable", false);
  path_check_enable_ = declare_parameter<bool>("path_check_enable", true);
  path_check_bad_sec_ = declare_parameter<double>("path_check_bad_sec", 0.4);
  // 実測で「指令を倍にしても実舵角は変わらない」ことが分かったので 1.0 に戻す。
  // 仕組みは残すが既定では何もしない。
  steer_cmd_scale_ = declare_parameter<double>("steer_cmd_scale", 1.0);
  // 共通の評価地点までの距離[m]。これを長くすると「復帰後の走り」を重く見る。
  goal_plan_tail_len_ = declare_parameter<double>("goal_plan_tail_len", 40.0);
  goal_plan_max_switch_ = declare_parameter<int>("goal_plan_max_switch", 4);
  goal_plan_ahead_min_ = declare_parameter<double>("goal_plan_ahead_min", 4.0);
  goal_plan_ahead_max_ = declare_parameter<double>("goal_plan_ahead_max", 10.0);
  const auto raceline = declare_parameter<std::string>("raceline_csv", "");
  const auto corridor = declare_parameter<std::string>("corridor_csv", "");
  const auto grid = declare_parameter<std::string>("occupancy_grid_yaml", "");
  loadRaceline(raceline, corridor);
  if (!grid.empty() && obstacles_.load(grid)) {
    RCLCPP_INFO(get_logger(), "復帰用の占有格子を読んだ: %s", grid.c_str());
    publishGrid();
    // 起動時の1回だけでは、後から起動した rviz が Volatile で購読していると
    // 受け取れない(実測: トピックは見えるのに "No map received")。
    // latched に頼らず、低頻度で出し続ける。570KB を 2 秒に1回なので負荷は小さい。
    grid_timer_ = create_wall_timer(std::chrono::milliseconds(2000),
                                    [this]() { publishGrid(); });
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
  // 壁への食い込みの観測は、指令の向きに関わらず毎周期更新する。
  // (外部レビュー レビュー: 前進指令中だけ更新すると古い経過時間が残る)
  updateWallBanState();
  // --- 出口の拒否を「状態遷移」として受け取る(外部レビュー の最優先指摘) ---
  //
  // 【何が壊れていたか】出口(applyWallForwardBan)は前進を止めるだけで、
  // 誰にも知らせていなかった。通常制御は前へ指令を出し続けるので、
  // **速度0のまま前進指令 → 出口で0 → 何も変わらない**が延々と続く。
  // 実測(20260905-150402 d1): 前進を止めた 200回 に対し復帰の計画は 3回。
  // これがユーザー報告の「前進も後退もしない」「永遠に後退」の土台だった。
  //
  // 【直し方】拒否は事実として受け取り、**向きを決めるのは復帰制御**にする。
  //   ・復帰していない → 前進が塞がれている状態として復帰を開始する
  //   ・復帰中で今の区間が前進 → 失敗として数え、後退から引き直す
  // 出口からギアや後退指令は一切出さない(それは復帰のギア管理と競合して
  // リタイアを 20%→35% に増やした失敗として実測済み)。
  if (wall_ban_hit_) {
    wall_ban_hit_ = false;
    // 拒否は毎周期成立する。反応するのは
    //   ・まだこの計画で反応していない、かつ
    //   ・前回の反応から 1 秒以上経っている
    // ときだけ。そうしないと毎周期引き直して計画だけが増える
    // (実測 20260905-201209/d2: 引き直し200回・計画207回・完了0回)。
    const double tnow = now.seconds();
    // 【直したバグ 2026-09-05】「1計画につき1回」を復帰していないときにも
    // 掛けていた。計画を作らない間は plan_seq_ が動かないので `fresh` が
    // 永久に偽になり、**拒否され続けているのに何も起きない**状態になる。
    //
    // 実測(20260905-222851/d4。ユーザー報告「P4 スタートした瞬間壁に
    // ぶつかり動かなくなった」): 復帰の上限30秒で通常制御へ返した後、
    //   「壁へ -0.22m 食い込んだまま N秒 改善しない。前進を止める
    //    (指令 11.67m/s -> 0)」
    // が1秒ごとに15回以上続き、復帰が一度も再開しなかった。
    //
    // 「1計画につき1回」は**復帰中の引き直しを抑えるための規則**なので、
    // 復帰中だけに適用する。復帰していないときは時間だけで見る。
    const bool in_recovery = recovery_start_time_.has_value();
    const bool fresh = (!in_recovery ||
                        !wall_ban_acted_ || wall_ban_acted_seq_ != plan_seq_) &&
                       (wall_ban_acted_at_ < 0.0 ||
                        tnow - wall_ban_acted_at_ >= 1.0);
    const bool in_cooldown =
      recovery_end_time_ &&
      (now - recovery_end_time_.value()).seconds() < kCooldownSec;
    if (fresh) {
      wall_ban_acted_ = true;
      wall_ban_acted_seq_ = plan_seq_;
      wall_ban_acted_at_ = tnow;
      if (!recovery_start_time_.has_value() && !in_cooldown) {
        RCLCPP_WARN(get_logger(),
          "壁前進禁止が前進を止めた。前進が塞がれているとみて復帰を始める");
        stuck_start_time_.reset();
        blocked_vehicle_start_time_.reset();
        recovery_start_time_ = now;
        beginRecovery(now, true);
        if (runRecovery(now)) { return; }
      } else if (recovery_start_time_.has_value() &&
                 plan_.valid && phase_idx_ < plan_.phases.size() &&
                 plan_.phases[phase_idx_].forward)
      {
        blocked_dir_ = +1;
        // 引き直しても出口が拒否し続けるなら、引き直しでは解けない。
        // 実測(20260905-214826/d1): 「後退から引き直す(6回目)」が
        // 28秒間くり返され、毎回同じ前進案が返っていた。
        // 上限に達したら最終手段(食い込み中は後退か待機しか出さない)へ移す。
        if (replan_count_ >= kReplanMax) {
          if (!desperate_) {
            RCLCPP_WARN(get_logger(),
              "壁前進禁止が %d回 前進を止めた。引き直しでは解けないので"
              "後退での脱出に切り替える", replan_count_);
            desperate_ = desperate_enable_;
            desperate_since_ = now;
            desperate_swing_ = -1;
          }
        } else {
          ++replan_count_;
          RCLCPP_WARN(get_logger(),
            "壁前進禁止が復帰の前進区間を止めた。後退から引き直す(%d回目)",
            replan_count_);
          makePlan(now, -1);
        }
        if (runRecovery(now)) { return; }
      }
    }
  }
  if (runRecovery(now)) {
    return;
  }
  // 後退区間を経路で渡している間は、後退用 pure_pursuit の指令を通す。
  // ギアはこちらが持っているので、向きの管理は従来どおり。
  // **前進の判定より先に見る**(前進用の指令を後退中に流さないため)。
  if (recovery_start_time_.has_value() && reverse_following_ && !traj_giveup_) {
    if (reverse_cmd_) { cmd_src_ = "後退経路"; publishFiltered(*reverse_cmd_); return; }
    // 後退用の指令がまだ届いていないなら止めておく(前進用を流さない)
    cmd_src_ = "後退指令待ち";
    publishCommand(0.0f, 0.0f, msg->lateral.steering_tire_angle);
    return;
  }
  // 復帰の前進区間を経路で渡している間は、pure_pursuit の指令をそのまま通す。
  // 復帰中なので停滞判定は回さない(回すと復帰の最中に再突入してしまう)。
  if (recovery_start_time_.has_value() && traj_following_) {
    // 【外部レビュー レビュー 2026-09-05】ここは control_pub_ へ直接流しており、
    // publishCommand の不変条件を通っていなかった。
    // **実測で壁を削っていた「復帰 前進を経路で渡す」はこの経路**なので、
    // 不変条件の穴として最も大きい。フィルタを通す。
    cmd_src_ = "前進経路";
    publishFiltered(*msg);
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
    cmd_src_ = "通常制御(舵先回り)";
    publishFiltered(out);
  } else {
    cmd_src_ = "通常制御";
    publishFiltered(*msg);
  }
  updateStuckDetection(*msg, now);
}

// 車体前方の同一レーンにいる最寄りの他車までの距離[m]。いなければ無限大。
//
// 追い越し層が通路なしと判断すると速度指令 0 を出すため、指令速度だけを見る
// motion_requested 条件では複数台が接触したまま永久停止する。前方の同一レーンに
// 限ることで、通常の低速走行や横並びを誤って拾わない。
double StuckRecoveryController::nearestForwardVehicle() const
{
  double nearest = std::numeric_limits<double>::infinity();
  if (!v2x_ || !odom_) { return nearest; }
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
      nearest = std::min(nearest, longitudinal);
    }
  }
  return nearest;
}

// 前後左右を問わず、一定半径内に他車がいるか。
//
// 前方だけを見ていたときは**横に並んで押し合っている接触を拾えなかった**
// (ユーザー報告: idx105 でカート同士がぶつかったのに復帰が出ない)。
// 誤発動の心配は小さい。呼び出し側は実速度 <= 0.1m/s が 3秒続くことも
// 同時に要求しており、レース中にその状態が続くのは実際に詰まっているときだけ。
bool StuckRecoveryController::anyVehicleNear() const
{
  if (!v2x_ || !odom_) { return false; }
  const auto & self = odom_->pose.pose.position;
  for (const auto & vehicle : v2x_->vehicles) {
    const double d = std::hypot(vehicle.position.x - self.x,
                                vehicle.position.y - self.y);
    if (d < kBlockedVehicleAnyDirRange) { return true; }
  }
  return false;
}

// 「進んでいない」を瞬間速度ではなく実際の移動量で見る。
//
// 【直したバグ(ユーザー報告)】
// 「P1とP2がぶつかり、復帰処理をせずずっとアクセルを踏み続けて強引に復帰した」。
// 従来の入口判定は瞬間速度だけ (|velocity| <= kStuckSpeedThreshold(0.1)) だった。
// カート同士が押し合っている状態は車体が擦れながら **0.2〜0.5 m/s で動き続ける**。
// 0.1 を超えるので stuck と判定されず、一方 motion_requested は成立し続けるので
// アクセルを踏み続け、復帰が一度も起動しないまま力任せに押し抜けていた。
// 実測: 車両接触1回に対し復帰完了0回。
//
// 復帰処理の**内部**には既に「距離が伸びていなければ停滞」という判定
// (kStallDist / stall_since_)があるのに、**入口だけが瞬間速度**という不整合だった。
bool StuckRecoveryController::hasNoProgress(const rclcpp::Time & now)
{
  recovery::Pose cp;
  if (!currentPose(cp)) { return false; }
  if (!entry_ref_valid_ ||
      std::hypot(cp.x - entry_ref_x_, cp.y - entry_ref_y_) > kEntryProgressDist)
  {
    entry_ref_x_ = cp.x;
    entry_ref_y_ = cp.y;
    entry_ref_time_ = now;
    entry_ref_valid_ = true;
    return false;
  }
  // kEntryProgressSec 秒かけて kEntryProgressDist も進めていない
  return (now - entry_ref_time_).seconds() >= kEntryProgressSec;
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

  // 【削除 2026-09-06】`motion_requested`(指令速度>=1.0 かつ指令加速度>=0.3)は
  // 復帰へ入る門として使っていたが、我々自身のガードが指令を 0 にすると閉じる
  // ため、進んでいないのに復帰しない状態を作っていた。門は外した。

  // 追い越し層が通路なしと判断すると速度指令0を出すため、従来の
  // motion_requested 条件では複数台が接触したまま永久停止する。
  // 車体前方の同一レーンにいる近接車だけを見て、通常の低速走行や横並びを除外する。
  const double nearest_forward_vehicle = nearestForwardVehicle();
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
  const bool car_near = std::isfinite(nearest_forward_vehicle) || anyVehicleNear();
  const bool blocked_by_vehicle =
    std::abs(command.longitudinal.speed) < kCommandSpeedThreshold &&
    std::abs(velocity) <= kStuckSpeedThreshold &&
    car_near;
  // --- 前の車に詰まって待っているだけの車を「スタック」にしない ---
  //
  // 【ユーザー報告 2026-09-05】「P1 が必要のない後退を続け、壁に後ろ向きに垂直に
  // ぶつかった。さらに P1 を避けられず他の車両が進めない。」
  //
  // 【実測(20260905-152454 d4)】
  //   stuck detected: velocity=0.070 横=-0.29 [-1.00,1.900] **領域内**
  //                   方位差=**3.719deg** 原因=前進不能 前方車=**2.01m**
  //   復帰 状況= 前方車 近傍2台 前2.0 後2.3 **壁0.80**
  // **領域内・方位差3.7度・壁まで0.80m。この車はスタックしていない。**
  // 前の車に詰まって待っているだけ。それを復帰にかけると、
  // 立つ計画はすべて前の車と重なる評価(車-0.84/-0.80/-0.40)なので動けず、
  // 最後に後退へ escalate して壁へ刺さる。**コース中央で切り返すので
  // 他車も通れなくなる。** 報告の2件はどちらもこれが原因。
  //
  // 車体が健全(走行可能領域の内側 / 向きが揃っている / 壁から離れている)なら、
  // 前が空くのを待つのが正しい。復帰を始めてはいけない。
  bool healthy_wait = false;
  if (queue_wait_enable_ && blocked_by_vehicle) {
    double lat = 0.0, lo = 0.0, hi = 0.0, yerr = 0.0;
    recovery::Pose p;
    const bool inside = lateralNow(lat, lo, hi) &&
                        lat > lo + queue_wait_margin_ && lat < hi - queue_wait_margin_;
    const bool aligned = headingErrorToTrack(yerr) && yerr < queue_wait_yaw_;
    const bool wall_ok = currentPose(p) && obstacles_.valid() &&
                         recovery::wallClearanceAt(obstacles_, veh_, p) > queue_wait_wall_;
    healthy_wait = inside && aligned && wall_ok;
    if (healthy_wait && (now - last_queue_wait_log_).seconds() > 3.0) {
      last_queue_wait_log_ = now;
      RCLCPP_INFO(get_logger(),
        "前が詰まっているだけ(領域内 横%.2f[%.2f,%.2f] 方位差%.1fdeg 壁%.2fm)。"
        "復帰は始めず前が空くのを待つ",
        lat, lo, hi, yerr * 180.0 / M_PI,
        recovery::wallClearanceAt(obstacles_, veh_, p));
    }
  }
  // --- 待機にも上限を置く(安全網) ---
  //
  // 「健全なら待つ」は正しいが、**4台が互いに詰まって全員が健全になると
  // 誰も復帰を始めない永久待機**になる(ユーザー報告「P1P3が全く動かず、
  // それのせいで P2P4 も進めない」に該当しうる)。
  // 列の根本にいる車は普通「健全でない」ので復帰が発火して列は流れるが、
  // 全員健全な配置は起こりうる。長めの上限を置いて、そのときだけ復帰させる。
  if (healthy_wait) {
    if (!healthy_wait_since_) { healthy_wait_since_ = now; }
    else if ((now - healthy_wait_since_.value()).seconds() > queue_wait_max_) {
      healthy_wait = false;          // 待ちすぎ。復帰を始めさせる
      if ((now - last_queue_wait_log_).seconds() > 3.0) {
        last_queue_wait_log_ = now;
        RCLCPP_WARN(get_logger(),
          "前が詰まって %.0f秒 待った。全員が健全な永久待機を避けるため復帰を始める",
          queue_wait_max_);
      }
    }
  } else {
    healthy_wait_since_.reset();
  }
  if (blocked_by_vehicle && !healthy_wait) {
    if (!blocked_vehicle_start_time_) { blocked_vehicle_start_time_ = now; }
  } else {
    blocked_vehicle_start_time_.reset();
  }
  const bool blocked_vehicle_ready =
    blocked_vehicle_start_time_ &&
    (now - blocked_vehicle_start_time_.value()).seconds() >= kBlockedVehicleDurationSec;

  // 膠着(hard_stall)と壁接触膠着は削除した。同じ「進んでいない」条件の重複で、
  // 上の門を回避するために足したものだった。門を外したので不要になった。

  // --- 門を外した(2026-09-06) ---
  // ここは以前 motion_requested(指令速度>=1.0 かつ指令加速度>=0.3)を要求して
  // いた。**指令が0のときに閉じる門**で、我々自身のガードが指令を0にすると
  // 「進んでいない」判定に一度も到達しなかった(実測 55秒間)。
  // 進んでいないことは指令の値とは無関係に判定できるので、門は不要。
  if (!moving_observed_) {
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
  const bool no_progress = hasNoProgress(now);

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
  // 【外部レビュー 指摘 2026-09-05】改善量が 0.05 固定だと、同じ姿勢からは同じ計画が
  // 通り続ける(実測: 同一計画 50回)。やり直すたびに要求を上げ、
  // **別の戦略へ単調に遷移させる。** 候補が尽きれば総当りへ落ち、別の案が出る。
  const double gain = std::min(0.05 * static_cast<double>(replan_count_), 0.60);
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
  // --- バグA の修正(2026-09-05) ---
  //
  // 【何が起きていたか】ここは後退できないときに first_phase を -1 から 0 へ
  // 黙って書き換えていた。0 は「向きの指定なし」なのでプランナは前進を返す。
  // つまり **「後退から引き直す」という判断が、後方に車がいるだけで
  // 前進の再試行に化けていた**。
  //
  // 【実測(20260905-003852 d1)】後方 1.6m に他車。kRearKeep=2.0 なので
  //   rear = 1.6 - 2.0 = -0.4 -> max_rev = 0 < 0.75 -> first_phase = 0
  // その結果 計画2/計画3 が「前進舵+0deg」になり、車は並進せずヨーだけ
  // 91°→60° 回り、壁への食い込みが -0.40 → -0.76m と悪化し続けた。
  // ユーザー報告の「謎の前進を繰り返す」はこれ。
  //
  // 【直し方】壁へ食い込んでいないときだけ従来どおり緩める。
  // 食い込んでいるなら前進は物理的に無意味なので、縛りを外さない。
  // 解が無ければ計画なしで返し、上位が停止を選ぶ(publishCommand の不変条件が
  // 前進を止めるので、押し付け続ける枝はもう存在しない)。
  const double clear_at_plan = recovery::wallClearanceAt(obstacles_, veh_, p);
  if (max_rev < 0.75 && first_phase < 0) {
    if (clear_at_plan >= 0.0) {
      first_phase = 0;
    } else {
      RCLCPP_WARN(get_logger(),
        "復帰 後退したいが下がれない(後方の余地 %.2fm)。壁へ %.2fm 食い込んでいるので "
        "前進へは切り替えない", max_rev, clear_at_plan);
    }
  }
  // --- 目標指向の計画を先に試す(ユーザー指示 2026-09-06) ---
  //
  // 参照経路上の「少し先」を目標に置き、そこへ到達する経路を A* で探す。
  //   ・後退 1m のコストを前進 1m の goal_plan_w_reverse 倍にする
  //   ・切り返し 1 回ごとに goal_plan_w_switch を足す
  // これで **後方が空いていれば切り返し1回(Y字)が最小コストで選ばれ、
  // 後方に壁や他車があるときだけ切り返しが増える。**
  //
  // 終点が参照経路上の目標そのものなので、合流の継ぎ目は構造的に 0 になる。
  // 従来の探索は「後退1本+前進1本」しか作れず、前進が一定舵角の円弧なので
  // 参照経路と繋がらず、実測で継ぎ目が中央 1.16m ずれていた。
  bool used_goal = false;
  // 旧探索が使う印。目標指向で決まったときは両方 false のまま。
  bool plan_strict = false;
  bool plan_best_effort = false;
  plan_from_goal_ = false;
  if (goal_plan_enable_) {
    recovery::GoalPlanParams gp;
    gp.max_switch = goal_plan_max_switch_;
    gp.tail_len = goal_plan_tail_len_;
    gp.goal_ahead_min = goal_plan_ahead_min_;
    gp.goal_ahead_max = goal_plan_ahead_max_;
    std::size_t gi = 0;
    const auto gplan = recovery::planToGoal(corridor_, obstacles_, veh_, p,
                                            cars, gp, &gi);
    if (gplan.valid) {
      plan_ = gplan;
      plan_goal_idx_ = gi;
      plan_from_goal_ = true;
      used_goal = true;
      double rev_len = 0.0, fwd_len = 0.0;
      int sw = 0;
      for (std::size_t k = 0; k < plan_.phases.size(); ++k) {
        if (plan_.phases[k].forward) { fwd_len += plan_.phases[k].length; }
        else { rev_len += plan_.phases[k].length; }
        if (k > 0 && plan_.phases[k].forward != plan_.phases[k - 1].forward) { ++sw; }
      }
      RCLCPP_INFO(get_logger(),
        "復帰 目標指向の計画: 切り返し%d回 後退%.1fm 前進%.1fm 目標idx%zu "
        "所要%.2fs(復帰後%.0fm まで込み) 余裕壁%.2f 車%.2f",
        sw, rev_len, fwd_len, plan_goal_idx_, plan_.cost, goal_plan_tail_len_,
        plan_.min_wall_clear, plan_.min_car_clear);
    }
  }
  if (!used_goal) {
  // まず「中断されない経路」を狙って厳しい余裕つきで計画する。
  // それで見つからない場所もあるので、駄目なら現行どおり緩い条件(余裕0)で
  // 引き直す。フォールバック段が複数あるうち、本命(この呼び出し)だけを
  // 厳→緩の2段にする(全段を厳→緩で一巡させるのは複雑になるため見送り)。
  plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                         first_phase, gain, kPlanMinEscape, min_rev, max_rev, cars,
                         kPlanWallClear, kPlanCarClear);
  plan_strict = plan_.valid;
  if (!plan_.valid) {
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           first_phase, gain, kPlanMinEscape, min_rev, max_rev, cars,
                           0.0, 0.0);
    plan_strict = false;
  }
  // 【外部レビュー レビュー 2026-09-05】この段が first_phase を明示的に 0 へ戻すので、
  // バグA の修正(食い込んでいるときは後退の縛りを外さない)が直後に無効化されていた。
  // 食い込んでいるときはこの段を飛ばす。前進の解を拾っても壁を押すだけ。
  // 【計測で戻した 2026-09-05】ここを飛ばすと計画が取れず desperate が増えた。
  // 食い込んだまま前進する計画は下のバグB の却下で落ちるので、二重に縛らない。
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
    plan_strict = false;
  }
  if (!plan_.valid) {
    // 最後のフォールバック。ここでも解が無いなら best_effort を立てて、
    // 棄却条件を全て外した上で「一番離れられる案」を採らせる
    // (kCarRadius を 1.49m に拡げた副作用で解なしになる距離が伸びたぶんの受け皿)。
    //
    // --- 【重大なバグの修正 2026-09-05】ここは first_phase を 0 に決め打ちして
    // いた。**壁へ食い込んでいても前進から始まる案を返す。**
    // しかも下の却下は `!plan_best_effort` で総当りを除外するので、
    // その前進案がそのまま採用される。
    //
    // 実測(20260905-214826/d1。ユーザー報告「P1がまたずっと後退し続ける」):
    //   計画6: 前進舵+0deg 2.5m 余裕壁**-0.36** 車**-0.97** 総当り1
    //   → 出口の壁前進禁止が拒否 → 後退から引き直す → **同じ前進案** → 以下無限
    // 1レースで計画140回・完了0回。壁 -0.22m のまま 28秒。
    //
    // 食い込んでいるなら総当りにも後退を要求する。
    // 後退の解も無いなら計画は出さない。**そのときは止まって待つのが正しい**
    // (ユーザーの目標「絶対に通過することができない場合はぶつからないように停止」)。
    // desperate の食い込み時の枝が「方向を選んだ後退」か「舵を向けて待つ」を出す。
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           clear_at_plan < 0.0 ? -1 : 0, 0.0, 0.0, 0.0,
                           8.0, cars, 0.0, 0.0, true);
    plan_best_effort = plan_.valid;
  }
  // --- バグB の修正(2026-09-05) ---
  //
  // 【何が起きていたか】総当りのフォールバック(best_effort)は棄却条件を
  // すべて外すので、**自分の評価が「壁や他車に当たる」と言っている計画**
  // (余裕が負)を返す。それをそのまま実行していた。
  //
  // 【実測(20260905-003852)】
  //   d1: 計画2「前進舵+0deg」余裕壁-0.67 車-0.74 / 計画3 余裕壁-0.76 車-0.79
  //   d2: 計画0〜4 がすべて前進で 車-0.90 → -1.05(他車と約1m重なる評価)
  //       しかも「地図では壁まで 1.21m ある。自己位置推定のずれとみて
  //       後退を 1.8→3.5→5.2→7.0m 以上にする」と4回対処しているのに、
  //       返る計画は毎回前進のみ。min_rev は後退区間があるときの下限にしか
  //       ならないので、この対処は一度も効いていなかった。
  //
  // 当たると分かっている前進を出すより、止まって待つほうが常に安い
  // (壁ペナルティは接触が続く間ずっと加算される。実測 3回で103秒)。
  // 【外部レビュー レビュー 2026-09-05】ここは `min_wall_clear` しか見ておらず、
  // ログとコメントが「壁や他車に当たる計画を却下」と書いているのに
  // **他車の余裕は却下条件に入っていなかった**。
  // d2 の実測「計画0〜4 がすべて前進で 車-0.90 → -1.05」はこれで通っていた。
  // 【計測で戻した 2026-09-05】`min_car_clear < 0` も却下すると、計画が取れない
  // 場面が増えて desperate(後方余裕を無視して ±4m/s で動く)へ落ちる回数が増え、
  // 4レースで ペナルティ回数 1.58 -> 2.00 / 壁 7.8 -> 12.1s/車レース と退行した。
  // 指摘(ログは「壁と車」と書いているのに条件は壁だけ)は正しいので、
  // **フラグとして残す。** 有効にする前に desperate 側の受け皿を安全にすること。
  // 【重大な誤りを訂正 2026-09-05】この却下が **desperate(強引な脱出)を
  // 呼ぶ原因**だった。14レースの実測:
  //   「経路を計算できない」 8028回 / 「強引な脱出へ切替」 8677回
  //   (復帰の計画は 273回、復帰 完了は 46回)
  // desperate は**壁も後方の車も無視して前後に ±4m/s で振る**最終手段。
  // ユーザー報告「ずっと壁に向かって後退し続ける」の実体はこれ。
  //
  // 原因は、**最後の手段である総当り(best_effort)の計画まで却下していた**こと。
  // 却下すると `plan_.valid=false` になり計画が何も残らないので desperate に落ちる。
  // **安全のための却下が、より危険なモードを呼んでいた。**
  //
  // 総当りは棄却条件を全部外して「一番離れられる案」を出す最後の受け皿なので、
  // ここで却下してはいけない。前進の押し付けは `wall_forward_ban` が出口で止める。
  if (plan_.valid && !plan_best_effort &&
      !plan_.phases.empty() && plan_.phases.front().forward &&
      (plan_.min_wall_clear < 0.0 || clear_at_plan < 0.0 ||
       (reject_car_overlap_plan_ && plan_.min_car_clear < 0.0)))
  {
    RCLCPP_WARN(get_logger(),
      "復帰 前進から始まる計画を却下(見込み余裕 壁%.2f 車%.2f / 現在の食い込み %.2f)。"
      "当たると分かっている前進は出さない",
      plan_.min_wall_clear, plan_.min_car_clear, clear_at_plan);
    plan_.valid = false;
  }

  // --- 前進区間が「当たる」見込みの計画は、後退だけに切り詰める(2026-09-05) ---
  //
  // 【ユーザー報告の挙動の実体】
  //   復帰 計画0: 後退舵-35deg 1.5m 前進舵-18deg 4.5m 余裕壁**-0.14**
  //   後退で 壁まで -0.40 -> +0.10 (壁から離れる。正しい)
  //   前進で 走行2.16m、壁まで +0.10 -> -0.10 (**同じ壁へ戻る**)
  // 計画自身が -0.14 と予測しているのに実行していた。
  //
  // 【なぜ却下条件をすり抜けたか】バグB の却下は
  // `plan_.phases.front().forward` (最初の区間が前進)に限定していた。
  // **`min_wall_clear` は前進区間の値なので順序に関係なく見るべき**だった。
  //
  // 却下してしまうと計画が無くなり desperate へ落ちる(それは以前の計測で退行した)。
  // 後退区間そのものは壁から離れる正しい動作なので、**後退だけ実行して
  // 良い姿勢から立て直す。** 前進は次の計画で改めて評価する。
  if (plan_.valid && !plan_best_effort &&
      plan_.min_wall_clear < 0.0 && plan_.phases.size() >= 2 &&
      !plan_.phases.front().forward)
  {
    RCLCPP_WARN(get_logger(),
      "復帰 前進区間が当たる見込み(余裕壁%.2f)。後退だけ実行して立て直す",
      plan_.min_wall_clear);
    plan_.phases.resize(1);
    if (plan_.rev_points > 0 && plan_.rev_points <= plan_.path.size()) {
      plan_.path.resize(plan_.rev_points);
    }
    // 前進区間が無くなったので、前進の見込み余裕は「無し」として扱う。
    plan_.min_wall_clear = 1e9;
    plan_.min_car_clear = 1e9;
  }

  // --- 見込みの余裕が薄い前進区間は、その長さぶん走り切らせない(2026-09-05) ---
  //
  // 【実測 20260905-183430/d3】計画0「後退舵+35 1.5m / 前進舵+0 4.5m 余裕壁0.22」。
  // 前進区間は最初の 3.8m まで壁まで 0.63 -> 0.14 と順調に減り、
  // 走り切った時点で **壁まで -0.28m(食い込み)**。計画の予測は +0.22 だった。
  // 自転車モデルは低速・大方位差では実挙動と合わず、走るほどずれが積もる
  // (この例では 4.5m で 0.5m ずれた)。
  //
  // 却下すると計画が消えて desperate に落ちるので、**切り詰める**。
  // 短くても壁から離れた姿勢まで進めれば、次の計画はそこから引き直せる。
  // 走り切った計画はやり直し回数を消費しない(下の「進めた計画は数えない」)ので、
  // 分割しても遅くならない。
  if (plan_.valid && plan_.min_wall_clear < kThinForwardClear) {
    for (size_t i = 0; i < plan_.phases.size(); ++i) {
      if (!plan_.phases[i].forward || plan_.phases[i].length <= kThinForwardLen) { continue; }
      RCLCPP_INFO(get_logger(),
        "復帰 前進の見込み余裕が薄い(壁%.2f)。前進区間を %.1f -> %.1fm に切り詰める",
        plan_.min_wall_clear, plan_.phases[i].length, kThinForwardLen);
      plan_.phases[i].length = kThinForwardLen;
      plan_.phases.resize(i + 1);
      const size_t keep_pts =
        plan_.rev_points + static_cast<size_t>(std::ceil(kThinForwardLen / 0.25)) + 1;
      if (keep_pts < plan_.path.size()) { plan_.path.resize(keep_pts); }
      break;
    }
  }

  // --- 却下で計画が消えたら、最後の受け皿(総当り)をやり直す(2026-09-05) ---
  //
  // 【順序の誤り】総当りの呼び出しは上の `if (!plan_.valid)` の中にあり、
  // **却下より前に一度だけ**評価される。却下で無効化すると総当りをやり直さないまま
  // 「経路を計算できない」に落ちていた(1レースで 1538回)。
  // 計画がゼロだと復帰そのものが動かないので、必ず受け皿へ回す。
  if (!plan_.valid) {
    // 食い込んでいるなら総当りにも「後退から始める」を要求する。
    // 壁に車体が入っている状態から前進しても物理的に出られない。
    plan_ = recovery::plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           clear_at_plan < 0.0 ? -1 : 0, 0.0, 0.0, 0.0,
                           8.0, cars, 0.0, 0.0, true);
    // --- 後退必須の要求をフォールバックで解除しない(外部レビュー 指摘 2026-09-05) ---
    //
    // 【何が起きていたか】ここは「後退の解が無いなら縛りを外す」として、
    // **壁へ食い込んだ状態で前進する計画**を出していた。総当りは棄却条件を
    // 全部外すので、自分の評価が負でもそのまま返る。
    //
    // 【実測 20260905-183430/d3】壁へ -0.22m 食い込んだ姿勢から
    //   計画4: 前進舵+0deg 1.5m 余裕壁**-0.20** 総当り1  -> 5秒間 走行0.02m
    //   計画5: 前進舵+0deg 4.5m 余裕壁**-0.14** 総当り1  -> 2秒間 走行0.018m
    // 壁を押しているだけなので当然動かず、2秒の打切りで引き直して同じ形へ戻る。
    // このレースの計画43回・完了1回はこのループが大半を占めている。
    //
    // 【計画ゼロで凍結しないか】しない。makePlan が false を返すと
    // replanOrEscalate が desperate へ切り替え、desperate は
    // **食い込み中は「方向を選んだ後退」か「舵を向けて待つ」しか出さない**
    // (発動条件が同じ wallClearanceAt < 0 なので必ずそちらへ入る)。
    // 受け皿の役割は残したまま、壁を押す枝だけを消す。
    if (!plan_.valid && clear_at_plan < 0.0) {
      RCLCPP_WARN(get_logger(),
        "復帰 壁へ %.2fm 食い込んでいて後退の解が無い。"
        "前進の計画は出さず、後退での脱出に任せる", clear_at_plan);
    }
    plan_best_effort = plan_.valid;
    if (plan_.valid) {
      RCLCPP_WARN(get_logger(),
        "復帰 却下で計画が消えたので総当りへ回した(見込み余裕 壁%.2f 車%.2f)",
        plan_.min_wall_clear, plan_.min_car_clear);
    }
  }

  }
  // 採用した計画が前進区間で見込んだ余裕を、追従中の中断閾値に使えるよう保存する。
  // 計画が取れなかったときは 1e9 のままにして、中断閾値は既定値を使わせる。
  plan_min_wall_clear_ = plan_.valid ? plan_.min_wall_clear : 1e9;
  plan_min_car_clear_ = plan_.valid ? plan_.min_car_clear : 1e9;
  phase_idx_ = 0;
  phase_travelled_ = 0.0;
  traj_from_ = 0;   // 経路の切り詰め位置も新しい計画に合わせて戻す
  phase_start_wall_clear_ = recovery::wallClearanceAt(obstacles_, veh_, p);
  phase_start_car_clear_ = recovery::carClearanceAt(cars, veh_, p);
  braking_ = false;
  last_valid_ = false;
  phase_start_ = now;
  stall_x_ = p.x;
  stall_y_ = p.y;
  plan_x_ = p.x;
  plan_y_ = p.y;
  plan_wall_clear_ = recovery::wallClearanceAt(obstacles_, veh_, p);
  ++plan_seq_;
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
  RCLCPP_INFO(get_logger(), "復帰 計画%d: %s(評価%.2f) 余裕壁%.2f 車%.2f 厳%d 総当り%d",
              replan_count_, desc.c_str(), plan_.cost,
              plan_min_wall_clear_, plan_min_car_clear_, plan_strict ? 1 : 0,
              plan_best_effort ? 1 : 0);
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
  // --- 既に壁へ食い込んでいるなら、前進を試さず後退から始める ---
  //
  // 【実測(2026-09-04、ユーザー報告「無駄に何度も切り返す」)】
  //   復帰 状況= 後方車 ... 壁-0.30          <- 開始時点で既に 0.30m 食い込み
  //   復帰追跡 前進 舵指令+18 速度0.00 走行0.00/4.50m 壁まで-0.30
  //   (同じ行が2秒間くり返し、走行は 0.06m のみ)
  //   復帰 経路で渡しても 2.0s 動かない
  //   復帰 前進 で動けない。後退から引き直す
  //   → 後退へ切り替えた途端に動いた(速度 -0.22→-1.12、壁まで -0.30→+0.28)
  //
  // 【なぜ既存のガードで捕まらなかったか】前進中断の判定は
  //   clear_now < thr && clear_now < 区間開始より悪化 && 走行 > 0.15m
  // の AND で、「走りながら悪化する」場合を想定している。
  // ところが実際は**開始時点で既に最悪**なので「悪化」が成立せず、
  // 動けないので「0.15m 走った」も成立しない。両方すり抜けて
  // 2秒の汎用タイムアウトを待っていた。
  //
  // 壁に押し付けられた状態で前進しても物理的に出られない。
  // 出口が塞がっていることは、走ってみなくても開始時点で分かる。
  bool wedged = false;
  {
    const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
    if (wedge_backward_first_ && clear_now < kWedgedWallClear) {
      wedged = true;
      RCLCPP_WARN(get_logger(),
                  "復帰 開始時点で壁へ %.2fm 食い込んでいる。前進を試さず後退から始める",
                  clear_now);
    }
  }
  makePlan(now, (forward_blocked || wedged) ? -1 : 0);
}

// 計画した区間を順に実行する。
// 区間ごとにギアと舵角が決まっているので、走った距離が区間長に達したら次へ進む。
// 領域内へ戻り、向きも揃い、走り出せるなら通常制御へ返す。返したら true。
//
// 以前は横位置しか見ておらず、コースに対して直角(yaw=111deg)のまま
// 「復帰 完了」と返して2秒後に再スタックしていた。
bool StuckRecoveryController::tryHandBack(
  const recovery::Pose & p, const rclcpp::Time & now, double total)
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
    return true;
  }
  return false;
}

// 最終手段。計画で抜けられる姿勢ではないときに、強引に動かす。
//
// 舵を左右に振りながら前後へ全開で当て、車体の向きを変えて隙間を作る。
// 相手を押すことにもなるので、他に手が無いときだけ。
// ここへ入るのは (a) 計画を kReplanMax 回出しても 1.2m も動けない
// (b) 経路が 1 本も出せない、のどちらか。(b) は前後とも塞がれている場合で、
// 以前は通常制御へ返して押し付けを繰り返すだけだった。
void StuckRecoveryController::runDesperate(
  const recovery::Pose & p, const rclcpp::Time & now)
{
  const double moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
  if (moved > kDesperateFreeDist) {
    RCLCPP_INFO(get_logger(), "復帰 強引な脱出で %.1fm 動けた。計画に戻す", moved);
    desperate_ = false;
    replan_count_ = 0;
    makePlan(now, 0);
    return;
  }
  const double t = (now - desperate_since_).seconds();
  // 最終手段そのものに上限を付ける。「動けたら終わる」しか無かったので、
  // 動けない間は永久に続いていた(ユーザー報告の無限後退)。
  if (t > desperate_max_sec_) {
    RCLCPP_WARN(get_logger(),
      "復帰 最終手段を %.0f秒 続けても %.2fm しか動けない。通常制御へ返す",
      t, moved);
    desperate_ = false;
    finishRecovery(now);
    return;
  }
  const int swing = static_cast<int>(t / kDesperateSwingSec);
  if (swing != desperate_swing_) {
    desperate_swing_ = swing;
    RCLCPP_WARN(get_logger(), "復帰 強引な脱出 %d回目 (最終手段)", swing + 1);
  }
  // --- 壁へ食い込んでいる間は強引な脱出を使わない(2026-09-05) ---
  //
  // desperate は壁も後方の車も無視して ±4m/s で前後に振る。
  // 壁へ食い込んだ状態でこれをやると、押し付けたまま振り続けることになり
  // 壁ペナルティが加算され続ける(実機では再起不能)。
  // 食い込んでいる間は舵だけ振って待つ。食い込みが解けたら通常の振りに戻る。
  // --- 壁へ食い込んでいる間は「振る」のではなく「下がって回す」(2026-09-05) ---
  //
  // 【なぜ振ってはいけないか】desperate は壁も後方の車も無視して ±4m/s で
  // 前後に振る。壁へ食い込んだ状態でやると押し付けたまま振り続けることになり、
  // 壁ペナルティが加算され続ける(実機では再起不能)。
  //
  // 【ただし止めるだけでも駄目】ユーザー報告「前進も後退もしない。前方には
  // 何もない」。危険な脱出を封じただけで代わりを用意しないと、計画が作れない
  // 状態(実測 1レースで 1538回)と重なって**完全停止**になる。
  //
  // したがって**方向を選んだ後退**を出す。前進は壁へ押し付けるので出さない。
  // 舵は後軸基準の自転車モデルで 2.5m 後退させ、車体の余裕が最も増える側。
  // 後方に車が近いときは出さない(後退の接触は自分と相手の双方に CRASH 10秒)。
  // ここは復帰制御自身の制御経路なので、ギアの管理も自分が持てる
  // (出口のガードから後退させて復帰と競合させた失敗を繰り返さない)。
  {
    recovery::Pose dp;
    if (currentPose(dp) && obstacles_.valid() &&
        recovery::wallClearanceAt(obstacles_, veh_, dp) < 0.0) {
      const double now_clear = recovery::wallClearanceAt(obstacles_, veh_, dp);
      const bool rear_ok = rearRoom() >= wall_ban_reverse_rear_;
      float best_steer = 0.0f; double best = -1e9;
      for (double sg : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        const double st = sg * kMaxSteerRad;
        recovery::Pose q = dp; double worst = 1e9;
        for (int i = 0; i < 25; ++i) {
          q.x += -0.1 * std::cos(q.yaw);
          q.y += -0.1 * std::sin(q.yaw);
          q.yaw += -0.1 * std::tan(st) / std::max(veh_.wheel_base, 0.1);
          worst = std::min(worst, recovery::wallClearanceAt(obstacles_, veh_, q));
        }
        if (worst > best) { best = worst; best_steer = static_cast<float>(st); }
      }
      // 余裕が改善しないまま後退を続けても意味がない。基準から
      // desperate_stall_sec_ のあいだ改善しなければ、後退をやめて待つ。
      if (desperate_ref_clear_ < -1e8 || now_clear > desperate_ref_clear_ + 0.05) {
        desperate_ref_clear_ = now_clear;
        desperate_ref_at_ = now;
      }
      const bool improving =
        (now - desperate_ref_at_).seconds() < desperate_stall_sec_;
      if (rear_ok && improving) {
        publishGear(GearCommand::REVERSE);
        // 加速度が 0 では**ペダルを踏んでいない**ので何も起きない。
        // AWSIM は speed を見ず、acceleration をギアの向きへのペダルとして使う
        // (docs/specifications/interface.ja.md: longitudinal.speed は未使用)。
        // REVERSE ギアなので正の加速度で後退へ踏む。
        cmd_src_ = "最終手段(後退)";
        publishCommand(static_cast<float>(-wall_ban_reverse_speed_),
                       kRecoveryAccel, best_steer);
      } else {
        publishGear(GearCommand::DRIVE);
        cmd_src_ = "最終手段(待ち)";
        publishCommand(0.0f, kBrakeAccel, best_steer);
      }
      if ((now - last_desperate_hold_log_).seconds() > 1.5) {
        last_desperate_hold_log_ = now;
        RCLCPP_WARN(get_logger(),
          "壁へ %.2fm 食い込んでいる。振らずに%s(舵 %+.0fdeg / 見込みの余裕 %.2fm / 後方 %.1fm)",
          now_clear,
          (rear_ok && improving) ? "後退して回す"
            : (rear_ok ? "待つ(後退しても改善しない)" : "待つ(後方に余地なし)"),
          best_steer * 180.0 / M_PI, best, rearRoom());
      }
      return;
    }
  }
  // 前後を交互に、舵も交互に振る。同じ当て方を続けても抜けないため。
  const bool fwd = (swing % 2) == 0;
  const float steer = static_cast<float>(
    ((swing / 2) % 2 == 0 ? 1.0 : -1.0) * kMaxSteerRad);
  publishGear(fwd ? GearCommand::DRIVE : GearCommand::REVERSE);
  cmd_src_ = "最終手段(振り)";
  publishCommand(fwd ? kDesperateSpeed : -kDesperateSpeed, kDesperateAccel, steer);
}

// 計画を走り切った / そもそも計画が無いときに、引き直すか最終手段へ移る。
//
// 戻り値: 値があれば runRecovery はそれをそのまま返す(この周期は終わり)。
//         std::nullopt なら計画が用意できたので、そのまま実行へ進む。
std::optional<bool> StuckRecoveryController::replanOrEscalate(
  const recovery::Pose & p, const rclcpp::Time & now)
{
  // 区間をすべて走り切り、計画を採用した地点から実際に離れているなら、
  // それは同じ計画の作り直しではなく**続き**である(下の「数えない」を参照)。
  const bool plan_ran_through = plan_.valid && phase_idx_ >= plan_.phases.size();
  const double moved_on_plan = std::hypot(p.x - plan_x_, p.y - plan_y_);
  // --- 「進めた」には改善も要る(2026-09-05 に自分で作った退行の修正) ---
  //
  // 距離だけで判定すると、**同じ場所で 0.5m ずつ後退し続ける計画**が
  // 永久に「進捗」と数えられ、やり直し上限に永遠に達しない。
  // 実測(20260905-193122/d2): 壁まで -0.10m のまま
  //   計画2 後退0.8m -> 計画3 後退0.8m -> 計画4 後退1.5m を 1.65秒で3回。
  // 壁から離れられていないのだから、これは続きではなく同じ失敗の繰り返し。
  const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
  const bool situation_better =
    clear_now >= 0.0 || clear_now > plan_wall_clear_ + kPlanImproveClear;
  const bool plan_made_progress =
    plan_ran_through && moved_on_plan >= kPlanProgressDist && situation_better;

  // 計画を走り切ったのに戻れていない。今の姿勢から引き直す。
  // 進めている間は上限に掛けない。上限は「動けない」ことへの歯止めであり、
  // 段を重ねて前進できている復帰を打ち切って desperate に落とす理由はない。
  if (replan_count_ >= kReplanMax && !plan_made_progress) {
    const double moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
    if (moved < kDesperateFreeDist) {
      // 計画を出しても1歩も動けていない。走行可能領域では説明のつかない
      // 何か(壁の当たり方、他車)に阻まれている。最終手段に切り替える。
      RCLCPP_WARN(get_logger(),
                  "復帰 計画%d回で %.2fm しか動けない。強引な脱出に切り替える",
                  replan_count_, moved);
      desperate_ = desperate_enable_;
      desperate_since_ = now;
      desperate_swing_ = -1;
      desperate_ref_clear_ = -1e9;
      return true;
    }
    RCLCPP_WARN(get_logger(), "復帰 計画%d回でも戻れない。通常制御へ返す 状況= %s",
                replan_count_, situation_.c_str());
    finishRecovery(now);
    return false;
  }
  // --- 走り切った計画は「やり直し」に数えない(2026-09-05) ---
  //
  // 【何が起きていたか】replan_count_ は「同じ姿勢から同じ計画が出る」ループを
  // 止めるために、引き直すたびに要求(gain)を上げ、kReplanMax=6 で desperate へ
  // 落とすカウンタである。しかしここは**計画が成功して次の段へ進む場合も
  // 同じカウンタを進めていた**。
  //
  // 【実測 20260905-183430/d3】計画3「後退0.8m + 前進6.0m」は
  // 速度 1.78 -> 2.36m/s、壁まで 0.45 -> 2.04m と理想的に走り切っている。
  // それでも1回ぶん消費し、数回そういう段を重ねると上限に達して
  // 「強引な脱出」へ落ちていた。ユーザーの目標「復帰動作は一回で成功」から見ると、
  // 数えるべきは**失敗した試行**だけである。
  //
  // 区間をすべて走り切り、計画を採用した地点から実際に離れているなら、
  // それは同じ計画の作り直しではなく**続き**なので数えない。
  // 動いていない場合は従来どおり数えるので、同一計画ループの歯止めは残る。
  if (plan_made_progress) {
    RCLCPP_INFO(get_logger(),
      "復帰 計画を走り切って %.2fm 進めた(壁まで %.2f -> %.2f)。"
      "やり直しには数えない(計画%d回)",
      moved_on_plan, plan_wall_clear_, clear_now, replan_count_);
  } else {
    ++replan_count_;
  }
  // 動けなかった向きを覚えているなら、その逆から始める計画を求める。
  if (!makePlan(now, blocked_dir_ != 0 ? -blocked_dir_ : 0)) {
    // --- 経路が1本も出せない = 前後とも塞がれている
    //
    // 【直したバグ(ユーザー報告: ぶつかって復帰が働かずアクセル踏みっぱなし)】
    // ここは finishRecovery() で通常制御へ返していた。しかし通常制御は
    // 前へ指令を出し続けるだけなので、返した瞬間にまた押し付けが始まり、
    // stuck 判定 -> 経路が出せない -> 返す、を延々と繰り返す。
    // 実測(3台走行 d2): `復帰 経路を計算できない` が 12 回、
    // stuck 判定が 8 回。その間ずっと前の車へ押し付けていた。
    //
    // 強引な脱出(desperate_)の仕組みは下に既にあるが、入口が
    // `replan_count_ >= kReplanMax` だけで、**経路が出せない場合は
    // そこへ到達する前にここで降りていた**。だから一度も発動していない。
    //
    // 経路が出せないのは「計画で抜けられる姿勢ではない」ということなので、
    // まさに強引な脱出が要る場面。ここから直接切り替える
    // (ユーザー指示: 完全に動けないのであれば強引に切り返しを何度も行う)。
    RCLCPP_WARN(get_logger(),
                "復帰 経路が出せない(%d回目)。強引な脱出に切り替える 状況= %s",
                replan_count_, situation_.c_str());
    desperate_ = desperate_enable_;
    desperate_since_ = now;
    desperate_swing_ = -1;
    desperate_ref_clear_ = -1e9;
    return true;
  }
  return std::nullopt;
}

// 前進中に壁へ近づいたら、動けなくなる前に引き直す。引き直したら true。
bool StuckRecoveryController::replanIfWallNear(
  const recovery::Pose & p, const recovery::Phase & ph, const rclcpp::Time & now)
{
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
  // --- 前進を指令しているのに1mmも動かないなら、待たずに後退から引き直す ---
  //
  // 【実測 2026-09-05 / ユーザー報告「前進も後退もしない」】
  //   +0s   後退 ギア20 位置(89655.30,43132.10) 壁まで+0.20   <- 後退は成功していた
  //   +20s  前進 ギア2  速度+0.00 位置(89654.80,43134.40) 壁まで-0.30
  //   +361s 前進 ギア2  速度+0.00 位置(89654.70,43134.60) 壁まで-0.50
  //   **340秒 まったく動かず。** ギアの内訳は DRIVE 461 / REVERSE 52。
  //
  // 前進区間に入り、壁へ食い込んでいるので `wall_forward_ban` が速度を0にする。
  // ところが**下の打切り条件は「0.15m 以上走った」を要求している**ので、
  // 動かないと成立せず、前進区間のまま永久に待つ。
  // (前セッションの引き継ぎに記録があった問題を、壁前進禁止が永久化させた)
  //
  // 走行距離ではなく**時間**で見る。前進を指令しているのに動かないのは、
  // それ自体が「その前進は成立しない」という十分な証拠。
  // 【重要】ここに `replan_count_ < kReplanMax` を掛けてはいけない。
  // 上限に達した後は打切りが止まり、**前進区間のまま永久に凍結する**
  // (ユーザー報告「P2 が前進も後退もしない」の再発)。
  // 動かない前進を出し続けることに価値は無いので、回数に関係なく降りる。
  if (ph.forward && !desperate_ &&
      phase_travelled_ < kFwdNoMoveDist &&
      (now - phase_start_).seconds() > kFwdNoMoveSec)
  {
    // 【外部レビュー 指摘 2026-09-05】カウンタを 0 に戻してはいけない。
    // 戻すと要求する改善量が最小に戻り、**同じ計画を再生成する**。
    // 実測: d1 は 70回の計画のうち **50回が完全に同一**
    // (後退舵+18deg 0.8m 前進舵-35deg 1.5m)。後退 0.8m を繰り返すので
    // 外から見ると「永遠に後退し続ける」ように見える。
    ++replan_count_;
    RCLCPP_WARN(get_logger(),
      "復帰 前進を指令して %.1fs 動かない(走行 %.3fm 壁まで %.2f)。後退から引き直す(%d回目)",
      (now - phase_start_).seconds(), phase_travelled_,
      recovery::wallClearanceAt(obstacles_, veh_, p), replan_count_);
    makePlan(now, -1);
    return true;
  }

  if (ph.forward && !desperate_ && replan_count_ < kReplanMax) {
  const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
  // 計画が見込んだ余裕より下げない。計画が「0.05m しかない所を通す」と決めたなら、
  // 中断側が 0.20m で落とすのは矛盾であり、必ず引き直しになる。
  const double thr = std::max(kFwdAbortWallFloor,
      std::min(kFwdAbortClearance, plan_min_wall_clear_ - kFwdAbortWorsen));
  // 壁も同じ。狭い所ではもともと余裕が小さいので、
  // 「閾値を割った」だけでは降りない。**区間開始より悪化している**ことを条件にする。
  if (clear_now < thr &&
      clear_now < phase_start_wall_clear_ - kFwdAbortWorsen &&
      phase_travelled_ > kFwdAbortMinTravel)
  {
    ++replan_count_;
    blocked_dir_ = 1;                     // 前進は駄目だったと覚える
    RCLCPP_WARN(get_logger(),
                "復帰 前進中に壁まで %.2fm まで詰まった(走行%.2fm しきい%.2f)。"
                "動けなくなる前に後退から引き直す(%d回目)",
                clear_now, phase_travelled_, thr, replan_count_);
    makePlan(now, -1);                    // 後退から始める計画を要求
    return true;
  }
  }
  return false;
}

// 前進中に他車へ近づいたら、ぶつかる前に引き直す。引き直したら true。
bool StuckRecoveryController::replanIfCarNear(
  const recovery::Pose & p, const recovery::Phase & ph, const rclcpp::Time & now)
{
  // --- 前進中に他車へ近づいたら、ぶつかる前に引き直す
  // 経路計画にも他車を入れたが、壁のときと同じで**計画どおりに動かない**
  // (壁際・低速では車が並進せず振れる)。実測のクリアランスでも見張る。
  if (ph.forward && !desperate_ && replan_count_ < kReplanMax) {
  // **符号付きの余裕**を使う。`carViolation` は 0 で初期化した「重なりの深さ」で
  // 離れていても 0 を返すので、距離として使うと他車が1台もいなくても
  // 条件が真になり、前進のたびに中断して無限に切り返す(実際に出したバグ)。
  const double cc = recovery::carClearanceAt(carObstacles(), veh_, p);
  // 計画が見込んだ余裕より下げない。理由は壁側の thr と同じ。
  const double thr = std::max(kFwdAbortCarFloor,
      std::min(kFwdAbortCarDist, plan_min_car_clear_ - kFwdAbortWorsen));
  // 近いだけでは降りない。**区間開始より悪化している**ときだけ降りる。
  // そうしないと、もともと狭い所では出だしで必ず降りて同じ計画を引き直し続ける。
  if (cc < thr && cc < phase_start_car_clear_ - kFwdAbortWorsen &&
      phase_travelled_ > kFwdAbortMinTravel)
  {
    ++replan_count_;
    blocked_dir_ = 1;
    RCLCPP_WARN(get_logger(),
                "復帰 前進中に他車まで %.2fm(開始時%.2fm しきい%.2f)。"
                "ぶつかる前に引き直す(%d回目)",
                cc, phase_start_car_clear_, thr, replan_count_);
    makePlan(now, -1);
    return true;
  }
  }
  return false;
}

// 前後を切り替える前に止まりきる。まだ止まりきっていなければ true。
//
// 舵は前の区間のまま保つ。ここで舵を先に動かすと、惰性で動いている間に
// 逆向きに回ってしまう。
bool StuckRecoveryController::runBraking(const rclcpp::Time & now)
{
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
    cmd_src_ = "制動";
    publishCommand(0.0, kBrakeAccel, static_cast<float>(brake_steer_));
    stall_since_ = now;   // 止まろうとしている間は停滞とみなさない
    return true;
  }
  return false;
}

// 後ろに車がいて、この区間で下がる距離が確保できないときの扱い。
//
// 下がって当てると Crash(10秒)を食らううえ、同じ相手に再衝突しやすい。
// ただし相手も詰まっていれば永久に退かないので、待ち続けるのではなく
// 空いている距離へ後退量を合わせる。
//
// 戻り値: 値があれば runRecovery はそれをそのまま返す。nullopt なら先へ進む。
std::optional<bool> StuckRecoveryController::waitForRearRoom(
  const recovery::Phase & ph, const rclcpp::Time & now)
{
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
      desperate_ = desperate_enable_;
      desperate_since_ = now;
      desperate_swing_ = -1;
      desperate_ref_clear_ = -1e9;
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
  return std::nullopt;
}

// 動けていないなら計画を引き直す。同じ指令を出し続けても出られない。
//
// ただし区間を始めた直後とギアを入れ替えた直後は数に入れない。
// 停止 -> ギア切替 -> 舵を入れる までに 1 秒近くかかるので、そこを停滞と
// 数えると計画を一度も実行しないまま向きだけ反転し続ける。
//
// 戻り値: 値があれば runRecovery はそれをそのまま返す。nullopt なら先へ進む。
std::optional<bool> StuckRecoveryController::handleStall(
  const recovery::Pose & p, const recovery::Phase & ph, const rclcpp::Time & now)
{
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
        desperate_ = desperate_enable_;
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
  return std::nullopt;
}

bool StuckRecoveryController::runRecovery(const rclcpp::Time & now)
{
  if (!recovery_start_time_.has_value()) { return false; }
  const double total = (now - recovery_start_time_.value()).seconds();
  if (total > kRecoveryMaxSec) {
    // --- 【撤回 2026-09-06】食い込み中も通常制御へ返す ---
    //
    // 「返した瞬間に全開が通って壁へ押し込む」のを防ぐために入れたが、
    // 押し込みを防ぐのは壁前進禁止の仕事で、そちらは実測で機能している
    // (不変条件の計測で「壁の外で前進」が 758回 -> 1回)。
    // 一方この変更は**時間の上限を持つ唯一の出口を塞いだ**。
    // 最終手段の終了条件は「動けたら」だけなので、動けない間は永久に続く。
    // ユーザー報告「30秒以上ずっと後退。それ以上の可能性もある」はこの構造。
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

  if (tryHandBack(p, now, total)) { return false; }

  

  if (desperate_) { runDesperate(p, now); return true; }

  if (!plan_.valid || phase_idx_ >= plan_.phases.size()) {
    const auto done = replanOrEscalate(p, now);
    if (done.has_value()) { return done.value(); }
  }

  const auto & ph = plan_.phases[phase_idx_];

  if (runBraking(now)) { return true; }

  {
    const auto done = waitForRearRoom(ph, now);
    if (done.has_value()) { return done.value(); }
  }

  // --- 固定した経路を毎周期検査する(内容は変えない) ---
  // 塞がれた状態が path_check_bad_sec_ 続いたときだけ作り直す。
  // 瞬間的な判定で差し替えると、境界で振動して同じ計画を出し続ける。
  if (path_check_enable_) {
    if (plannedPathStillClear()) {
      path_bad_since_ = -1.0;
    } else {
      const double tnow = now.seconds();
      if (path_bad_since_ < 0.0) { path_bad_since_ = tnow; }
      else if (tnow - path_bad_since_ >= path_check_bad_sec_) {
        path_bad_since_ = -1.0;
        if ((now - last_path_bad_log_).seconds() > 1.0) {
          last_path_bad_log_ = now;
          RCLCPP_WARN(get_logger(),
            "復帰 他車が動いて経路が塞がれた。作り直す(計画%d回)", replan_count_);
        }
        if (replan_count_ < kReplanMax) { ++replan_count_; }
        makePlan(now, 0);
        return true;
      }
    }
  }

  if (replanIfWallNear(p, ph, now)) { return true; }

  if (replanIfCarNear(p, ph, now)) { return true; }

  {
    const auto done = handleStall(p, ph, now);
    if (done.has_value()) { return done.value(); }
  }

  // 区間を走り切ったら次へ。ただし前後が入れ替わるなら先に止まりきる。
  // --- 短い区間では走破判定の余裕を比例させる(2026-09-05) ---
  //
  // kPhaseDoneSlack は 0.30m の固定値だが、**後退区間は 0.75〜0.8m しかない**
  // ことが多い。固定 0.30m だと指定の 60% で「走り切った」と扱われ、
  // 計画が必要とした後退量に届かない。実測では壁から離れられないまま
  // 0.5m ずつ後退して引き直す動きになっていた。
  // 長い前進区間では従来どおり 0.30m の余裕を残す。
  const double done_slack = std::min(kPhaseDoneSlack, ph.length * 0.15);
  if (phase_travelled_ >= ph.length - done_slack) {
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
                "復帰追跡 区間%zu/%zu %s 舵狙い%+.0f 舵指令%+.0f 実舵%+.0f 実速度%.2f "
                "送出[加速度%+.2f 速度%+.2f ギア%u 出=%s] "
                "走行%.2f/%.2fm 位置(%.1f,%.1f) yaw%.0f 壁まで%.2f 経路渡し%d",
                phase_idx_ + 1, plan_.phases.size(), ph.forward ? "前進" : "後退",
                steer * 180.0 / M_PI, sent_steer_cmd_ * 180.0 / M_PI,
                steer_report_ * 180.0 / M_PI, latest_velocity_,
                sent_accel_, sent_speed_, sent_gear_, cmd_src_,
                phase_travelled_, ph.length,
                p.x, p.y, p.yaw * 180.0 / M_PI,
                recovery::wallClearanceAt(obstacles_, veh_, p),
                (reverse_following_ || traj_following_) ? 1 : 0);
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
    // --- 舵の条件を外した(2026-09-06) ---
    // 追従を pure_pursuit へ一本化したあと、実舵角は**経路追従が毎周期動かす値**
    // であり、`steer`(区間の代表舵角)とは一致しない。実測(20260906-034518)では
    // 代表舵 0deg に対し追従が実舵を -3→-8→-15→-18deg と動かし、差が閾値を
    // 超えて「準備できていない」と判定され、**後退の途中で指令が 0 になった**。
    // 1台テストで 47エピソード中 45回がこれ。舵は pure_pursuit が持つので、
    // 復帰側が届いたかを判定してはいけない。ギアの条件だけ残す。
    if (!gear_ready && waited < kActuatorWaitMax) {
      cmd_src_ = "ギア/舵の待ち";
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
                      "復帰 経路で渡しても %.1fs 動かない(追従は続ける)",
                      kTrajStillSec);
        }
      }
      if (!traj_giveup_) { return false; }   // 指令は pure_pursuit のものを通す
    }
    // --- 一定舵角の直接制御を廃止した(ユーザー指示 2026-09-06) ---
    //
    // 【何が問題だったか】ここは `ph.steer`(区間の代表舵角)を区間の全長に
    // わたって出していた。目標指向の探索は経路を素片(0.5m)の列として作り、
    // 素片ごとに舵角が違うのに、**区間にまとめるとき最初の素片の舵角を
    // 区間全体の代表値にしていた**。
    // その結果 7〜12m の前進区間を一定舵角(-18deg = R 3.35m)で走り、
    // 120度以上回ってコースを横断し、向かいの壁に当たっていた
    // (ユーザー報告「ハンドルを右に切ったまま前進を続け、向かいの壁に
    //  ぶつかっています」)。
    //
    // 【なぜ廃止でよいか】追従は通常の pure_pursuit に一本化する
    // (ユーザーの想定した流れ)。ここへ落ちる原因だった
    // 「経路を渡しても動かない」は、`reverse_pure_pursuit` が後退のとき
    // 負の加速度(ブレーキ)を出していたことが原因で、今日修正済み。
    // 物理的に嵌まって何をしても動かない場合は runDesperate の
    // 「余裕が最も増える舵角を選んで後退する」枝が受け持つ
    // (実測でその枝が 4.7m 動かして脱出できている)。
    //
    // 速度と加速度だけは出す。舵は pure_pursuit の値をそのまま使う
    // (publishCommand ではなく、素通しで通る)。
    return false;
  } else {
    // 後退区間も経路を publish して後退用 pure_pursuit に追従させる
    // (2026-08-31・ユーザー指示)。前進とまったく同じ形にする。
    // 【訂正 2026-09-06】以前はここで自前の一定舵角制御へ落としていたが、
    // その制御は区間の代表舵角を区間全体に適用するもので、7〜15m の円弧を
    // 描いて向かいの壁に当たっていた。**削除済み**。
    // いまは追従を続けるだけ(舵は pure_pursuit が持つ)。
    if (reverse_following_ && !traj_giveup_) {
      if (std::abs(latest_velocity_) > kMovingSpeedThreshold) {
        traj_still_ = false;
      } else {
        if (!traj_still_) { traj_still_ = true; traj_still_since_ = now; }
        else if ((now - traj_still_since_).seconds() > kTrajStillSec) {
          traj_giveup_ = true;
          RCLCPP_WARN(get_logger(),
                      "復帰 後退を経路で渡しても %.1fs 動かない(追従は続ける)",
                      kTrajStillSec);
        }
      }
      if (!traj_giveup_) { return false; }   // 指令は後退用 pure_pursuit のものを通す
    }
    // 後退も同じ。一定舵角の直接制御は廃止し、後退用 pure_pursuit に任せる。
    // ここへ落ちるのは経路が渡っていない(まだ publish していない)ときだけ。
    return false;
  }
  return true;
}

// 復帰の前進区間を「たどってほしい経路」として publish する。
//
// これまでは復帰中ずっと舵と速度を直接作っていた。そのため通常制御へ返す
// 瞬間に不連続が生じ、pure_pursuit が急に遠くの目標を向いて壁へ寄っていた。
// 前進区間を経路として渡せば pure_pursuit がそのまま追従し、
// 経路の終端が目標軌道につながっているので戻りが連続になる。
// 【訂正 2026-09-06】「後退区間は pure_pursuit では表現できないので直接制御する」
// と書いていたが誤り。専用ノード `reverse_pure_pursuit` が後退経路を追従できる。
// できていなかった原因は次の3つで、いずれも修正済み。
//   1. 加速度の符号が逆(REVERSE ギアで負 = 常にブレーキ)
//   2. 経路が pure_pursuit の目標距離より短い(1.5m < 2.5m)ので目標点が無い
//   3. 舵の符号が二重反転(前方基準の角度に後方基準の反転を掛けていた)
// 1 が 2 と 3 を隠していた(動かない車の舵の向きは観測できない)。

// 固定した経路が、いまの他車位置でまだ通れるかを検査する。
//
// 【なぜ検査だけにするか】経路の内容を毎周期作り直すと追従側の進捗が壊れる
// (外部レビュー 指摘)。一方で他車は動くので、無視することもできない。
// そこで **内容は固定し、検査は毎周期行い、無効になったときだけ作り直す**。
// 検査は残りの経路(いま切り詰めている位置から先)だけを見る。
bool StuckRecoveryController::plannedPathStillClear() const
{
  if (!plan_.valid || plan_.path.empty()) { return true; }
  const auto cars = const_cast<StuckRecoveryController *>(this)->carObstacles();
  if (cars.empty()) { return true; }
  for (std::size_t i = traj_from_; i < plan_.path.size(); ++i) {
    if (recovery::carClearanceAt(cars, veh_, plan_.path[i]) < 0.0) { return false; }
  }
  return true;
}

bool StuckRecoveryController::publishRecoveryTrajectory()
{
  if (!latest_traj_ || !recovery_start_time_.has_value() || !plan_.valid) {
    reverse_following_ = false;
    return false;
  }
  if (braking_ || desperate_ || phase_idx_ >= plan_.phases.size()) {
    reverse_following_ = false;
    return false;
  }

  recovery::Pose cur;
  if (!currentPose(cur)) { reverse_following_ = false; return false; }

  // --- 経路の内容を固定する(外部レビュー 指摘 / ユーザー確認 2026-09-06) ---
  //
  // 【何が問題だったか】ここは毎周期、現在地から `plan_.path` 全体の最近傍点
  // `from` を探し直し、**そこから先だけ**を publish していた。つまり
  // **経路の内容が毎周期変わる**。追従側(pure_pursuit)は毎回新しい経路で
  // 最近傍探索をやり直すので、**経路上の単調な進捗が残らない**。
  // 外部レビュー の指摘: 「車両が追っている対象が固定経路ではなく、動く経路になる」。
  // さらに最近傍探索は姿勢も経路上の順序も見ないので、切り返しで交差する
  // 経路では `from` が別の枝へ飛ぶ(前進区間や尾へ)ことがある。
  //
  // 【直し方】`from` を**単調にする**。一度進んだら戻らない。
  // 探索範囲も直前の位置から前方に限る。これで publish する内容は
  // 「同じ経路を、進んだぶんだけ切り詰めたもの」になり、幾何が変わらない。
  // 計画を作り直したときだけ 0 に戻す(makePlan で traj_from_ = 0)。
  std::size_t from = traj_from_;
  {
    double bd = 1e18;
    std::size_t best = traj_from_;
    for (std::size_t i = traj_from_; i < plan_.path.size(); ++i) {
      const double d = std::hypot(plan_.path[i].x - cur.x, plan_.path[i].y - cur.y);
      if (d < bd) { bd = d; best = i; }
    }
    from = best;
    traj_from_ = best;   // 単調
  }

  auto push = [](Trajectory & t, double x, double y, double yaw, double v) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw * 0.5);
    p.pose.orientation.w = std::cos(yaw * 0.5);
    p.longitudinal_velocity_mps = static_cast<float>(v);
    t.points.push_back(p);
  };

  const std::size_t rev_n = std::min(plan_.rev_points, plan_.path.size());
  const bool in_reverse = reverse_traj_enable_ && !plan_.phases[phase_idx_].forward;
  if (!reverse_traj_enable_ && !plan_.phases[phase_idx_].forward) {
    reverse_following_ = false;
    publishRecoveryPathMarker(from);   // 表示だけは従来どおり出す
    return false;
  }

  // --- 後退区間: 後退用 pure_pursuit へ、後退の向きに並んだ経路を渡す
  if (in_reverse) {
    Trajectory rev;
    rev.header = latest_traj_->header;
    rev.header.stamp = this->now();
    for (std::size_t i = from; i < rev_n; ++i) {
      push(rev, plan_.path[i].x, plan_.path[i].y, plan_.path[i].yaw, kRecoverySpeed);
    }
    // --- 経路は必ず目標距離より長く渡す(2026-09-06) ---
    //
    // 【これが「経路を渡しても動かない」の原因だった】
    // pure pursuit は「後軸から目標距離以上離れた最初の経路点」を目標にする。
    // `reverse_pure_pursuit` は lookahead_gain=0 / lookahead_min_distance=2.5 なので
    // **目標距離は固定 2.5m**。ところが後退区間は 0.5〜3.0m しかない。
    // 経路全体が目標距離に収まると目標点が見つからず、終端を目標にするしかなく、
    // そこへ近づくと幾何が破綻して **加速度0・速度0** を出す。
    //
    // 実測(20260906-032551): 後退経路 4点(1.5m)を渡し、走行 0.76m の時点で
    // 送出が [加速度+0.00 速度+0.00] になり惰性で停止。
    // 80エピソード中 **79回**「2.0s 動かない」に落ちていた。
    //
    // 対策は経路の延長。**最後の姿勢から同じ舵角で弧を延ばした「尾」**を付ける。
    // 区間の終了は走行距離で判定するので、車が尾に到達することはない。
    // pure pursuit には常に目標点が存在するようになる。
    {
      constexpr double kTailLen = 4.0;    // 目標距離 2.5m より確実に長く
      constexpr double kTailStep = 0.25;
      if (rev.points.size() >= 1 && rev_n >= 1) {
        const auto & last = plan_.path[rev_n - 1];
        // --- 尾は「最後の2点から求めた実際の曲率」で延ばす(2026-09-06) ---
        //
        // 【バグだった点】区間の**代表舵角**(最初の素片の舵)で延ばしていた。
        // 後退区間の舵が途中で変わる場合、尾は最後の姿勢から**間違った向き**へ
        // 曲がる。`reverse_pure_pursuit` はその尾を目標にするので、
        // 逆向きに舵を切る(ユーザー報告「後退の経路が、車体前方が通常の経路の
        // 向きに向かない方にハンドルを切っていた」)。
        // 代表値を使わず、経路そのものの曲率を継ぐ。
        double curv = 0.0;   // [rad/m]
        if (rev_n >= 2) {
          const auto & prev = plan_.path[rev_n - 2];
          const double ds = std::hypot(last.x - prev.x, last.y - prev.y);
          if (ds > 1e-6) {
            double dy = last.yaw - prev.yaw;
            while (dy > M_PI) { dy -= 2.0 * M_PI; }
            while (dy < -M_PI) { dy += 2.0 * M_PI; }
            curv = dy / ds;
          }
        }
        double x = last.x, y = last.y, yaw = last.yaw;
        for (double run = 0.0; run < kTailLen; run += kTailStep) {
          const double dyaw = -kTailStep * curv;
          const double mid = yaw + dyaw * 0.5;
          x += -kTailStep * std::cos(mid);
          y += -kTailStep * std::sin(mid);
          yaw += dyaw;
          push(rev, x, y, yaw, kRecoverySpeed);
        }
      }
    }
    publishRecoveryPathMarker(from);
    if (rev.points.size() < 2) { reverse_following_ = false; return false; }
    reverse_traj_pub_->publish(rev);
    reverse_following_ = true;
    if ((this->now() - last_traj_log_).seconds() > 1.0) {
      last_traj_log_ = this->now();
      RCLCPP_INFO(get_logger(), "復帰 後退を経路で渡す (%zu点)", rev.points.size());
    }
    // traj_following_(前進用)は立てない。呼び出し側は戻り値でそれを決めるので
    // false を返す。前進用 pure_pursuit には通常の軌道を流しておいてよい
    // (後退中はその指令を使わないため)。
    return false;
  }

  reverse_following_ = false;

  // --- 前進区間: 計画した前進経路 + 本線へ「なめらかに」合流する部分
  Trajectory out;
  out.header = latest_traj_->header;
  out.header.stamp = this->now();
  const std::size_t f0 = std::max(from, rev_n);
  for (std::size_t i = f0; i < plan_.path.size(); ++i) {
    push(out, plan_.path[i].x, plan_.path[i].y, plan_.path[i].yaw, kRecoverySpeed);
  }
  // 前進側は下で参照経路を継ぎ足すので、経路が目標距離より短くなることはない。
  if (out.points.size() < 2) { return false; }

  // 終端から目標軌道へつなぐ。
  //
  // 【ユーザー報告】「生成した経路が短く、既定の経路への復帰がなめらかでない」。
  // 以前はここで最近傍点から先を **そのまま継ぎ足していた** ため、
  // 計画の終端が本線から横にずれていると継ぎ目で経路が折れ、
  // 速度も kRecoverySpeed から本線の速度へ段差になっていた。
  // 終端の横ずれを kBlendLen[m] かけて指数的に 0 へ近づけ、
  // 速度も同じ区間で本線の値へ寄せる。
  const auto & tail = out.points.back().pose.position;
  std::size_t near = 0;
  double nd = 1e18;
  const auto & tp = latest_traj_->points;
  for (std::size_t i = 0; i < tp.size(); ++i) {
    const double d = std::hypot(tp[i].pose.position.x - tail.x,
                                tp[i].pose.position.y - tail.y);
    if (d < nd) { nd = d; near = i; }
  }
  // 継ぎ目での横ずれを、本線の法線方向に符号つきで測る
  double nx = 0.0, ny = 0.0;
  {
    const std::size_t a = near;
    const std::size_t b = (near + 1) % tp.size();
    const double dx = tp[b].pose.position.x - tp[a].pose.position.x;
    const double dy = tp[b].pose.position.y - tp[a].pose.position.y;
    const double L = std::hypot(dx, dy);
    if (L > 1e-6) { nx = -dy / L; ny = dx / L; }
  }
  const double lat0 = (tail.x - tp[near].pose.position.x) * nx +
                      (tail.y - tp[near].pose.position.y) * ny;

  double run = 0.0;
  for (std::size_t k = 1; k < tp.size(); ++k) {
    const std::size_t i = (near + k) % tp.size();
    const std::size_t j = (near + k - 1) % tp.size();
    run += std::hypot(tp[i].pose.position.x - tp[j].pose.position.x,
                      tp[i].pose.position.y - tp[j].pose.position.y);
    auto pt = tp[i];
    if (run < kBlendLen && std::abs(lat0) > 1e-3) {
      const double w = 1.0 - run / kBlendLen;      // 1 -> 0
      double bx = 0.0, by = 0.0;
      const std::size_t i2 = (i + 1) % tp.size();
      const double dx = tp[i2].pose.position.x - tp[i].pose.position.x;
      const double dy = tp[i2].pose.position.y - tp[i].pose.position.y;
      const double L = std::hypot(dx, dy);
      if (L > 1e-6) { bx = -dy / L; by = dx / L; }
      pt.pose.position.x += lat0 * w * bx;
      pt.pose.position.y += lat0 * w * by;
      // 速度も段差にしない
      const double vt = pt.longitudinal_velocity_mps;
      pt.longitudinal_velocity_mps =
          static_cast<float>(kRecoverySpeed * w + vt * (1.0 - w));
    }
    out.points.push_back(pt);
  }
  publishRecoveryPathMarker(from);
  traj_pub_->publish(out);
  if ((this->now() - last_traj_log_).seconds() > 1.0) {
    last_traj_log_ = this->now();
    RCLCPP_INFO(get_logger(),
                "復帰 前進を経路で渡す (%zu点, 計画%zu点ぶん, 継ぎ目の横ずれ%.2fm)",
                out.points.size(), plan_.path.size() - f0, lat0);
  }
  return true;
}

// 復帰の当たり判定が見ている占有格子を、そのまま rviz へ出す。
//
// 値は「余裕の大きさ」を段階で表す。単なる白黒より、
// **どこがぎりぎりなのか** が見えるほうがデバッグに役立つ。
//   壁の中(clearance <= 0)          -> 100 (黒)
//   余裕 0 〜 kGridShowRange[m]      -> 99 〜 1 (濃いほど壁に近い)
//   それ以上                         -> 0 (白)
void StuckRecoveryController::publishGrid()
{
  if (!grid_pub_ || !obstacles_.valid()) { return; }
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.header.stamp = this->now();
  g.info.resolution = static_cast<float>(obstacles_.resolution());
  g.info.width = static_cast<unsigned int>(obstacles_.width());
  g.info.height = static_cast<unsigned int>(obstacles_.height());
  g.info.origin.position.x = obstacles_.originX();
  g.info.origin.position.y = obstacles_.originY();
  g.info.origin.orientation.w = 1.0;
  g.data.resize(static_cast<std::size_t>(g.info.width) * g.info.height);
  for (int iy = 0; iy < obstacles_.height(); ++iy) {
    for (int ix = 0; ix < obstacles_.width(); ++ix) {
      const double c = obstacles_.distAt(ix, iy);
      int8_t v;
      if (c <= 0.0) {
        v = 100;
      } else if (c >= kGridShowRange) {
        v = 0;
      } else {
        v = static_cast<int8_t>(99.0 * (1.0 - c / kGridShowRange));
        if (v < 1) { v = 1; }
      }
      g.data[static_cast<std::size_t>(iy) * g.info.width + ix] = v;
    }
  }
  grid_pub_->publish(g);
  if (!grid_logged_) {
    grid_logged_ = true;
    RCLCPP_INFO(get_logger(),
                "復帰用の占有格子を rviz へ出した: %ux%u 解像度%.3fm 原点(%.1f, %.1f) "
                "topic=/planning/debug/recovery_grid (2秒ごとに再送)",
                g.info.width, g.info.height, g.info.resolution,
                g.info.origin.position.x, g.info.origin.position.y);
  }
}

// rviz へ「後退 -> 前進 -> 本線復帰」を1本の Y字として出す。
// 後退部を赤、前進部を緑で色分けする。
void StuckRecoveryController::publishRecoveryPathMarker(std::size_t from)
{
  if (!recovery_path_pub_ || !plan_.valid || plan_.path.empty()) { return; }
  // 表示専用の間引き。0.2s では復帰の動きが追えないので詰める。
  if ((this->now() - last_path_marker_).seconds() < 0.05) { return; }
  last_path_marker_ = this->now();
  const std::size_t rev_n = std::min(plan_.rev_points, plan_.path.size());
  visualization_msgs::msg::MarkerArray arr;
  for (int part = 0; part < 2; ++part) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "recovery_path";
    m.id = part;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.15;
    m.color.a = 0.95f;
    m.color.r = (part == 0) ? 1.0f : 0.1f;
    m.color.g = (part == 0) ? 0.2f : 1.0f;
    m.color.b = 0.2f;
    m.pose.orientation.w = 1.0;
    const std::size_t b = (part == 0) ? 0 : rev_n;
    const std::size_t e = (part == 0) ? rev_n : plan_.path.size();
    for (std::size_t i = b; i < e; ++i) {
      geometry_msgs::msg::Point q;
      q.x = plan_.path[i].x; q.y = plan_.path[i].y;
      q.z = (i < from) ? 0.1 : 0.3;   // 通過済みは低く描く
      m.points.push_back(q);
    }
    if (m.points.size() >= 2) { arr.markers.push_back(m); }
  }
  if (!arr.markers.empty()) { recovery_path_pub_->publish(arr); }
}

void StuckRecoveryController::finishRecovery(const rclcpp::Time & now)
{
  publishGear(GearCommand::DRIVE);
  recovery_start_time_.reset();
  recovery_end_time_ = now;
}

// 壁への食い込みの観測。**前進指令の有無に関わらず毎周期更新する。**
//
// 【外部レビュー レビュー 2026-09-05】更新が前進指令中だけだと、停止・後退で壁から
// 離れても基準と時刻が古いまま残り、次に前進を始めた瞬間に「改善していない」と
// 判定されて即座に禁止される。脱出の機会を奪うデッドロック要因になる。
//
// また `clear >= 0` で即リセットすると、境界付近の自己位置ノイズで
// 0.8秒の前進許可を何度も取り直せてしまう。解除側にヒステリシスを入れる。
void StuckRecoveryController::updateWallBanState()
{
  recovery::Pose p;
  if (!currentPose(p) || !obstacles_.valid()) { return; }
  const double clear = recovery::wallClearanceAt(obstacles_, veh_, p);
  // 計測用に毎周期控える(禁止が無効でも記録する)。
  wall_clear_now_ = clear;
  if (!wall_forward_ban_) { return; }
  const double t = this->now().seconds();
  // 解除は「余裕が正側へ十分出た」ときだけ。ゼロ跨ぎのノイズで戻さない。
  if (clear >= wall_forward_ban_gain_ * 2.0) {
    wall_ban_since_ = -1.0;
    wall_ban_ref_clear_ = 1e9;
    return;
  }
  if (clear >= 0.0) { return; }   // 0 付近は状態を保持(進めも戻しもしない)
  // 地図誤差の範囲の食い込みでは効かせない。実測の破綻ケースは 0.40〜0.76m。
  if (clear > -wall_forward_ban_depth_) { return; }
  if (wall_ban_since_ < 0.0 || clear > wall_ban_ref_clear_ + wall_forward_ban_gain_) {
    wall_ban_since_ = t;
    wall_ban_ref_clear_ = clear;
  }
}

// 壁へ食い込んだまま前進し続けない(絶対の不変条件)。
//
// 【なぜ絶対にするか】運営アナウンス(2026-09-03):
//   「実機は一度壁にぶつかったままアクセルを踏み続けると再起不能(≒最下位)」
// SIM でも最大の損失だった(決勝条件の4台走行で壁ペナルティ3台合計193秒、
// うち1台は3回で103秒。wall は接触が続く間ずっと加算される)。
//
// 一律禁止ではなく **「食い込んでいて、かつ改善していないなら禁止」**。
// 斜めに刺さっている場合は前進で抜けられるので、改善している間は通す。

// 「壁の外に出たまま前進の指令が出た」を抜け道ごとに数えるだけの関数。
// 挙動は変えない。0 でなければ不変条件を守れていない。
void StuckRecoveryController::noteForwardInWall(int reason, float speed)
{
  if (reason < 0 || reason >= kFwdWallReasons) { return; }
  if (speed <= 0.5f) { return; }
  if (!(wall_clear_now_ < 0.0)) { return; }
  const double t = this->now().seconds();
  fwd_in_wall_cnt_[reason]++;
  if (fwd_in_wall_last_t_ > 0.0) {
    const double dt = t - fwd_in_wall_last_t_;
    if (dt > 0.0 && dt < 0.5) { fwd_in_wall_sec_[reason] += dt; }
  }
  fwd_in_wall_last_t_ = t;
  fwd_in_wall_worst_ = std::min(fwd_in_wall_worst_, wall_clear_now_);
  if ((this->now() - last_invariant_log_).seconds() > 5.0) {
    last_invariant_log_ = this->now();
    long total = 0;
    for (int i = 0; i < kFwdWallReasons; ++i) { total += fwd_in_wall_cnt_[i]; }
    RCLCPP_WARN(get_logger(),
      "不変条件 壁の外で前進 計%ld回 "
      "[発進前 %ld(%.1fs) 禁止off %ld(%.1fs) 復帰の前進 %ld(%.1fs) "
      "高速 %ld(%.1fs) 浅い %ld(%.1fs) 猶予 %ld(%.1fs)] 最深 %.2fm",
      total,
      fwd_in_wall_cnt_[0], fwd_in_wall_sec_[0],
      fwd_in_wall_cnt_[1], fwd_in_wall_sec_[1],
      fwd_in_wall_cnt_[2], fwd_in_wall_sec_[2],
      fwd_in_wall_cnt_[3], fwd_in_wall_sec_[3],
      fwd_in_wall_cnt_[4], fwd_in_wall_sec_[4],
      fwd_in_wall_cnt_[5], fwd_in_wall_sec_[5],
      fwd_in_wall_worst_);
  }
}

void StuckRecoveryController::applyWallForwardBan(float & speed, float & acceleration)
{
  // --- 一度も走り出していない間は効かせない(2026-09-05) ---
  //
  // 【ユーザー報告】「P4 スタートした瞬間壁にぶつかり動かなくなりました」。
  //
  // 【実測 20260905-222851/d4】止まっていた位置 (89630.8, 43134.9) は
  // **P4 のグリッド座標そのもの**(launch の grid_slots の4番目と一致)。
  // 車は一度も動いていない。ところが復帰用の占有格子はその地点を
  // 「壁へ -0.36m 食い込んでいる」と判定する。**地図がグリッドを壁の中に
  // 置いている。** その結果:
  //   合図 → 全開 11.67m/s → 禁止が「食い込んだまま0.8秒改善しない」と判断
  //   → スロットルを0 → 一度も発進できない → 復帰 → 30秒上限 → 以下無限
  //
  // この禁止の意味は「ぶつかった後に押し付け続けない」ことなので、
  // **まだ一度も走っていない車には適用しない。** 一度動けば有効になる。
  if (!moving_observed_) { noteForwardInWall(0, speed); return; }
  if (!wall_forward_ban_) { noteForwardInWall(1, speed); }
  if (!wall_forward_ban_ || speed <= 0.05f) {
    if (wall_ban_gear_rev_ && speed > 0.05f) {
      publishGear(GearCommand::DRIVE);
      wall_ban_gear_rev_ = false;
    }
    return;
  }
  // --- 復帰計画の前進区間は止めない(ユーザー報告 2026-09-05) ---
  //
  // 【報告された挙動】「壁に衝突して全く前進せず、リバースに入れて後退、
  // ギアを入れ替えて前進せず、また後退をしだす」。
  //
  // 【原因】この禁止が**復帰計画の前進区間を毎回0にしていた。**
  // 実測(20260905-150402 d1): 前進を止めた 200回 に対し復帰の計画は 3回。
  // 計画は「後退 → 前進」で立つので、後退で壁から離れた直後に前進区間が
  // 潰され、動かないので計画をやり直し、また後退から始まる循環になる。
  //
  // 【切り分け】計画は前進区間の最小余裕を `min_wall_clear` として持っている。
  // **計画自身が「当たらない」と評価している前進は止めない。**
  // 「当たる」と評価している前進は下の (B) で計画ごと却下するので、
  // ここへは来ない。止めるべきなのは計画の裏付けが無い前進だけ。
  // --- 計画の予測ではなく、いまの実測で判断する(2026-09-05) ---
  //
  // 【計測でこの抜け道が最大だと分かった】不変条件の計測(1レース):
  //   d1: 壁の外で前進 758回 のうち **復帰の前進 264回(2.6秒)**
  //   d3: 935回 のうち **復帰の前進 533回(5.3秒)** 最深 -0.50m
  // ここは「計画自身が当たらないと評価した前進は止めない」という免除だが、
  // **その予測は実測とずれる**ことが既に分かっている
  // (予測 +0.22m の前進区間が 4.5m 走って -0.28m になった実測がある)。
  //
  // 車体が**いま**壁の外に出ているなら、計画の予測は当てにならない。
  // 計画は「壁の中にいる姿勢」から立てたものなので前提が崩れている。
  // 実測が食い込みを示している間は免除しない。
  if (recovery_start_time_.has_value() && plan_.valid &&
      phase_idx_ < plan_.phases.size() && plan_.phases[phase_idx_].forward &&
      plan_min_wall_clear_ >= 0.0 && wall_clear_now_ >= 0.0)
  {
    noteForwardInWall(2, speed);
    return;
  }
  // **走っている車には効かせない。** 押し付けられて動けない状態が前提であり、
  // 走行中に壁を掠めた程度でスロットルを切ると、コース上に停止して
  // 全車の玉突きを招く(2026-09-05 に実測。1レースで4台全滅)。
  if (std::abs(latest_velocity_) >= wall_forward_ban_speed_) {
    noteForwardInWall(3, speed);
    return;
  }
  // wall_ban_since_ が立っていない = 深さ(wall_forward_ban_depth_)に届いていない。
  if (wall_ban_since_ < 0.0) { noteForwardInWall(4, speed); return; }
  const double held = this->now().seconds() - wall_ban_since_;
  // 深く食い込んでいるなら「改善するか様子を見る」意味がない。猶予を縮める。
  // 実測: 猶予 0.8秒のあいだ全開が通り、返すたびに壁へ数cm押し込んでいた。
  const double hold = (wall_ban_ref_clear_ < -0.4)
                        ? std::min(wall_forward_ban_hold_, 0.2)
                        : wall_forward_ban_hold_;
  if (held <= hold) { noteForwardInWall(5, speed); return; }
  // 拒否したという事実だけを残す。ここでは向きもギアも作らない。
  wall_ban_hit_ = true;
  if ((this->now() - last_wall_ban_log_).seconds() > 1.0) {
    last_wall_ban_log_ = this->now();
    RCLCPP_WARN(get_logger(),
      "壁へ %.2fm 食い込んだまま %.1fs 改善しない。前進を止める "
      "(指令 %.2fm/s -> 0 実速度 %.2fm/s)。実機ではこの前進継続が再起不能になる",
      wall_ban_ref_clear_, held, static_cast<double>(speed), latest_velocity_);
  }
  // --- 止めるだけでは姿勢が直らない。後退しながら回転して抜ける ---
  //
  // 【実測 2026-09-05】崩壊したレースの内訳:
  //   d2: 「前進を止めた」146回 / 復帰の計画 3回 / 壁ペナ 236秒
  //   d3: 123回 / 5回
  // **止められたまま放置されている。** カートは速度0では回転できないので、
  // 止めるだけでは方位差が永久に残る。壁ペナルティは接触が続く間ずっと加算される。
  //
  // 壁接触の原因は姿勢だと実測で分かっている
  // (方位差 <10度で接触0% / >=45度で100%)。**回転そのものが脱出**になる。
  // 前進は禁じたままにして、**後退で回転する**。後退はこの禁止を受けない。
  //
  // 後方に車がいるときは出さない。後退での接触は自分と相手の双方に
  // CRASH 10秒が付く(公式のペナルティ表)。
  if (wall_ban_reverse_ && held > wall_ban_reverse_after_) {
    if (rearRoom() >= wall_ban_reverse_rear_) {
      // 舵は「後退したときに壁から離れる向き」を選ぶ。
      // 後軸基準の自転車モデルで 1m ぶん後退させ、車体の余裕が増える側を採る。
      recovery::Pose p;
      if (currentPose(p)) {
        // 【修正 2026-09-05】1m の後退では足りず、3案すべてが現状より悪化する
        // 場面があった(食い込み -0.20m に対し見込み -0.50m)。
        // 距離を伸ばし、**余裕が改善しないなら回転量そのもの**で選ぶ。
        // 壁接触の原因は姿勢なので、回転が進むこと自体が価値を持つ。
        const double now_clear = recovery::wallClearanceAt(obstacles_, veh_, p);
        double best_clear = -1e9, best_turn = -1e9;
        float best_steer = 0.0f; bool by_turn = false;
        for (double sg : {-1.0, -0.5, 0.5, 1.0}) {
          const double st = sg * kMaxSteerRad;
          recovery::Pose q = p;
          const double ds = -0.1;
          double bestq = 1e9;
          for (int i = 0; i < 25; ++i) {          // 2.5m ぶん後退させて見る
            q.x += ds * std::cos(q.yaw);
            q.y += ds * std::sin(q.yaw);
            q.yaw += ds * std::tan(st) / std::max(veh_.wheel_base, 0.1);
            bestq = std::min(bestq, recovery::wallClearanceAt(obstacles_, veh_, q));
          }
          const double turn = std::abs(q.yaw - p.yaw);
          // 余裕が現状より改善する案があればそれを採る。
          if (bestq > now_clear + 0.02 && bestq > best_clear) {
            best_clear = bestq; best_steer = static_cast<float>(st); by_turn = false;
          } else if (best_clear < -1e8 && turn > best_turn) {
            // どれも改善しない。**最も回る案**を採る(姿勢を直すのが目的)。
            best_turn = turn; best_steer = static_cast<float>(st); by_turn = true;
          }
        }
        const double best = by_turn ? now_clear : best_clear;
        // **ギアを後退へ入れる。** 負の目標速度だけでは後退しない
        // (実測: 指令を出しても実速度 0.00m/s のまま食い込み時間が伸び続けた)。
        publishGear(GearCommand::REVERSE);
        wall_ban_gear_rev_ = true;
        speed = static_cast<float>(-wall_ban_reverse_speed_);
        acceleration = 0.0f;
        if ((this->now() - last_wall_ban_rev_log_).seconds() > 1.0) {
          last_wall_ban_rev_log_ = this->now();
          RCLCPP_WARN(get_logger(),
            "壁へ食い込んだまま %.1fs。後退して回転で抜ける(舵 %+.0fdeg / "
            "選択=%s / 現在の余裕 %.2f -> 見込み %.2fm / 後方 %.1fm)",
            held, best_steer * 180.0 / M_PI, by_turn ? "回転量" : "余裕",
            now_clear, best, rearRoom());
        }
        // 舵は呼び出し側の値を上書きする。回転が目的なので舵が本体。
        wall_ban_steer_ = best_steer;
        wall_ban_steer_valid_ = true;
        return;
      }
    } else if ((this->now() - last_wall_ban_rev_log_).seconds() > 2.0) {
      last_wall_ban_rev_log_ = this->now();
      RCLCPP_WARN(get_logger(),
        "壁へ食い込んだまま %.1fs だが後方に余地が無い(%.1fm)。"
        "後退での接触は双方に CRASH が付くので待つ", held, rearRoom());
    }
  }
  if (wall_ban_gear_rev_) {
    // 後退をやめたらギアを戻す。戻さないと通常制御の前進指令が後退になる。
    publishGear(GearCommand::DRIVE);
    wall_ban_gear_rev_ = false;
  }
  speed = 0.0f;
  acceleration = kBrakeAccel;
}

// control_pub_ への唯一の出口。素通しの指令もここを通す。

// ギアと指令速度の符号を必ず一致させる(2026-09-05)。
//
// 【ユーザー報告】「ごくたまに AWSIM 上のギアの表示がリバースになっているのに
// 前進をすることがある。ギアチェンジの処理にバグが起きている可能性は?」
//
// 【何が起きうるか】ギアを publish するのは復帰の直接制御・desperate・
// 復帰の終了だけで、**経路を素通しする経路(publishFiltered)ではギアを出さない。**
// 後退区間の途中で計画が差し替わる・素通しへ切り替わると、
// ギアが REVERSE のまま前向きの指令が流れる状態が生じる。
// AWSIM の表示は最後に送ったギアなので、報告どおり「表示は R、動きは前進」になる。
//
// 【直し方】速度指令の符号でギアを決める、を**出口の不変条件**にする。
// publishGear は値が変わったときだけ送るので毎周期呼んでも負荷にならない。
// ここは新しい行動を作っていない(すでに出す指令に正しい札を付けるだけ)ので、
// 「出口は拒否だけ」の原則にも反しない。
void StuckRecoveryController::alignGearToSpeed(float speed)
{
  constexpr float kDeadband = 0.05f;
  if (speed > kDeadband) {
    publishGear(GearCommand::DRIVE);
  } else if (speed < -kDeadband) {
    publishGear(GearCommand::REVERSE);
  }
  // 停止指令のときはギアを触らない。切り返しの待ち時間中に
  // 意図した向きのギアを保つ必要がある。
}

void StuckRecoveryController::publishFiltered(AckermannControlCommand cmd)
{
  float speed = cmd.longitudinal.speed;
  float accel = cmd.longitudinal.acceleration;
  wall_ban_steer_valid_ = false;
  applyWallForwardBan(speed, accel);
  cmd.longitudinal.speed = speed;
  cmd.longitudinal.acceleration = accel;
  // 回転で抜けるときは舵が本体なので上書きする。
  if (wall_ban_steer_valid_) {
    // wall_ban_steer_ は実舵角で作っているので、ここだけ指令の単位へ変換する。
    cmd.lateral.steering_tire_angle =
      static_cast<float>(wall_ban_steer_ * steer_cmd_scale_);
  }
  // ここを通る指令は pure_pursuit / reverse_pure_pursuit が作ったもので、
  // **すでに指令の単位**(あちらは wheel_base=2.14 で変換を吸収している)。
  // 二重に steer_cmd_scale_ を掛けてはいけない。
  alignGearToSpeed(cmd.longitudinal.speed);
  sent_speed_ = cmd.longitudinal.speed;
  sent_accel_ = cmd.longitudinal.acceleration;
  sent_gear_ = gear_now_;
  control_pub_->publish(cmd);
}

void StuckRecoveryController::publishCommand(float speed, float acceleration, float steer)
{
  const auto stamp = this->now();
  // --- 出力段で必ず範囲に収める(唯一の publish 地点なので、ここだけで全経路を守れる) ---
  //
  // 【なぜここか】publishCommand の呼び出しは7箇所あり、そのうち4箇所は
  // 舵角をクランプせずに渡していた(1256 / 1701 / 1726 / 1746行)。
  // 呼び出し側それぞれにクランプを足すと同じ判定が散らばるので、出口に1つ置く。
  //
  // 【OVER ペナルティ】大会ルールでは加速度指令の絶対値が ±3 m/s^2 を超えると
  // 2秒間 5km/h に制限される(simple_pure_pursuit 側のコメントに実装当時の調査あり)。
  // AWSIM は入力を ±1.37 m/s^2 に clamp するので、2.0 に抑えても性能は落ちない。
  // 復帰の定数は現状すべて 1.37 以下だが、将来の変更で踏まないように出口で止める。
  //
  // 非有限値(NaN/Inf)も落とす。計算のどこかで 0 除算が起きたときに
  // そのまま車両へ流すと、何が起きたか分からない挙動になる。
  constexpr float kOutMaxSteer = static_cast<float>(kMaxSteerRad);
  constexpr float kOutMaxAccel = 2.0f;
  constexpr float kOutMaxSpeed = 10.0f;
  if (!std::isfinite(steer)) { steer = 0.0f; }
  if (!std::isfinite(acceleration)) { acceleration = 0.0f; }
  if (!std::isfinite(speed)) { speed = 0.0f; }
  steer = std::clamp(steer, -kOutMaxSteer, kOutMaxSteer);
  acceleration = std::clamp(acceleration, -kOutMaxAccel, kOutMaxAccel);
  speed = std::clamp(speed, -kOutMaxSpeed, kOutMaxSpeed);

  wall_ban_steer_valid_ = false;
  applyWallForwardBan(speed, acceleration);
  if (wall_ban_steer_valid_) { steer = wall_ban_steer_; }
  alignGearToSpeed(speed);
  // ここまでの steer は**実舵角**。publish する値は指令の単位なので変換する
  // (他のノードは wheel_base=2.14 を使って式の中で同じ変換を吸収している)。
  sent_steer_phys_ = steer;
  steer = static_cast<float>(steer * steer_cmd_scale_);
  sent_speed_ = speed;
  sent_steer_cmd_ = steer;
  sent_accel_ = acceleration;
  sent_gear_ = gear_now_;

  AckermannControlCommand msg;
  msg.stamp = stamp;
  msg.lateral.stamp = stamp;
  msg.lateral.steering_tire_angle = steer;
  msg.lateral.steering_tire_rotation_rate = 2.0;
  msg.longitudinal.stamp = stamp;
  msg.longitudinal.speed = speed;
  msg.longitudinal.acceleration = acceleration;
  // --- publish 間隔の自己監視(観測のみ) ---
  // 大会ルールでは 250Hz 以上で publish しても OVER ペナルティになる。
  // 現構成では pure_pursuit は nominal_control_cmd へ出しており、
  // /control/command/control_cmd の publisher はこのノードだけなので
  // 二重 publish は起きないはずだが、**確かめずに「起きない」と書かない**。
  // 4ms(=250Hz)を下回る間隔が出たら記録する。
  {
    const double t = stamp.seconds();
    if (last_pub_t_ > 0.0) {
      const double dt = t - last_pub_t_;
      if (dt < 0.004 && dt > 0.0) {
        ++fast_pub_cnt_;
        if ((t - last_fast_log_t_) > 2.0) {
          last_fast_log_t_ = t;
          RCLCPP_WARN(get_logger(),
                      "指令の publish 間隔が短い %.1fms (250Hz超) 累計%d回",
                      dt * 1000.0, fast_pub_cnt_);
        }
      }
    }
    last_pub_t_ = t;
  }
  control_pub_->publish(msg);
  // GUI 用。ここが呼ばれている = 復帰が車両を直接動かしている。
  // 呼ばれなくなれば GUI 側が時間切れで「通常」に戻す。
  if (status_pub_) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "RECOVERY  cmd %.1f km/h  steer %.0f deg",
                  speed * 3.6f, steer * 180.0f / static_cast<float>(M_PI));
    std_msgs::msg::String m;
    m.data = buf;
    status_pub_->publish(m);
  }
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
