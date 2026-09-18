#include "stuck_recovery_controller/stuck_recovery_controller.hpp"
#include "stuck_recovery_controller/contact_lease.hpp"
#include "stuck_recovery_controller/phase_path.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <deque>
#include <cstdio>
#include <cstdlib>
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
// 壁に刺さってから復帰を始めるまでの待ち。短いほど、pure_pursuit が。
constexpr double kStuckDurationSec = 0.6;
// 入口の停滞判定。この秒数のあいだに この距離[m] も進めていなければ、。
constexpr double kEntryProgressSec = 1.0;
constexpr double kEntryProgressDist = 0.5;
// 通常側が「前車が近いので停止」を出し続ける玉突き状態も復帰対象にする。
constexpr double kBlockedVehicleDurationSec = 3.0;
constexpr double kBlockedVehicleMinLongitudinal = 0.3;
constexpr double kBlockedVehicleMaxLongitudinal = 4.0;
constexpr double kBlockedVehicleMaxLateral = 1.6;
constexpr float kStoppedCommandThreshold = 0.1;
constexpr float kCommandSpeedThreshold = 1.0;
constexpr float kCommandAccelerationThreshold = 0.3;
constexpr float kMovingSpeedThreshold = 0.5;
constexpr double kRearClearDist = 6.0;      // 後方この距離[m]以内に車がいたら下がらない
constexpr double kBodyFront = 1.554;        // 後軸中心から前端[m](公式)
constexpr double kBodyRear  = 0.510;        // 後軸中心から後端[m](公式)
constexpr double kRearTouch = kBodyRear + kBodyFront;   // 2.064m で当たる
constexpr double kRearKeep = kRearTouch + 0.12;         // = 2.184m
// 復帰の経路計画で避ける他車を拾う範囲[m]。遠くまで入れると経路が見つからない。
constexpr double kCarObstacleRange = 12.0;
// 前進中に他車がこれ[m]より近づいたら、停滞を待たずに引き直す。
constexpr double kFwdAbortCarDist = 0.25;
// 「閾値を割った」だけでは降りず、区間開始よりこれだけ[m]悪化したときに降りる。
constexpr double kFwdAbortWorsen = 0.15;
// 前後左右を問わず、この距離[m]以内に他車がいれば「近接」とみなす。
constexpr double kBlockedVehicleAnyDirRange = 2.5;
// 手詰まりのときに詰めてよい距離[m]。
constexpr double kRearKeepTight = kRearTouch - 0.20;   // = 1.864m(触れる手前)
constexpr double kRearWaitMax = 2.0;        // 後方が退くのをこの時間[s]しか待たない
constexpr double kRearClearWidth = 1.6;     // 後方判定の横幅[m]
// 運動の予測に使う幾何 ---。
constexpr double kWheelBase = 1.087;         // ホイールベース[m]
// 計画は実舵上限0.31radを使い、送出時だけsteer_cmd_scale_で指令単位へ変換する。
constexpr double kMaxSteerRad = 0.31;
constexpr double kHalfWidth = 0.73;         // 車体半幅[m]
constexpr double kAvoidMargin = 0.3;        // 回避時の余裕[m]
constexpr double kAvoidCheckRange = 8.0;    // 前方この距離[m]までを判定対象にする
constexpr double kFrontObstacleHalf = 1.5;  // 前方障害物とみなす横幅の半分[m]

// --- 計画追従の定数 ---
constexpr double kRecoveryMaxSec = 30.0;   // 1回の復帰episodeの上限[s]
// 上限に達したら止まるのではなく、動かない打切り側で 0 に戻して作り直す。
constexpr double kReplanMax = 6;           // 計画のやり直し上限
// 食い込みからの前進脱出で「浅くなった」と認める最小の改善量[m]。
constexpr double kEmbedImprove = 0.05;
// 動けているかの判定は「ギアが入ってから」「舵を入れ終わってから」数える。
constexpr double kStallSec = 4.0;          // この時間[s]動けなければ計画をやり直す
constexpr double kPhaseMinSec = 1.2;       // 区間を始めてこの時間[s]は停滞判定をしない
// 実ギアが指令に追いつくのを待つ上限[s]。舵の遅れは到達可能壁予測で扱う。
constexpr double kActuatorWaitMax = 0.15;
constexpr double kStallDist = 0.15;        // 動けているとみなす距離[m]
// 区間の走破判定が厳しいと、。
constexpr double kPhaseDoneSlack = 0.30;   // 区間の走破判定の余裕[m]
// 前進区間の見込み余裕がこれを下回るなら、モデルのずれで壁へ届く可能性が高い。
constexpr double kThinForwardClear = 0.35;
constexpr double kThinForwardLen = 2.5;
// 走り切った計画が「やり直し」に数えられない最小の移動量[m]。
constexpr double kPlanProgressDist = 0.5;
// 「改善した」とみなす壁との余裕の増分[m]。
constexpr double kPlanImproveClear = 0.10;
constexpr double kAlignYaw = 0.22;         // コース方位とのずれ[rad](12.6deg)
// 通常制御に返す前に、この距離[m]ぶん先まで追従経路を検査する。
constexpr double kHandbackAhead = 12.0;
constexpr double kHandbackLookahead = 3.5;      // pure_pursuit の lookahead_min_distance
constexpr double kHandbackLookaheadGain = 0.20; // pure_pursuit の lookahead_gain
constexpr double kHandbackRelaxSec = 12.0;      // これを過ぎたら検査距離を半分にする
constexpr double kTrajStillSec = 2.0;           // 経路で渡して動かない時間[s]の上限
// 本線へ戻すときに横ずれを吸収する距離[m]。継ぎ足すだけだと継ぎ目で経路が折れ、。
constexpr double kBlendLen = 8.0;
// 占有格子を rviz に出すときに、余裕を濃淡で表す範囲[m]。
constexpr double kGridShowRange = 1.5;
constexpr double kPlanMinEscape = 2.5;     // 計画の終端は詰まった場所からこれだけ[m]離れること
// 地図の上では余裕があるのに動けなかったとき、後退の下限をこれだけ[m]ずつ伸ばす。
constexpr double kBlockedReverseStep = 1.75;
constexpr double kBlockedReverseMax = 7.0;
constexpr double kExplainedClearance = 0.30;   // これだけ[m]余裕があれば「当たっていないはず」
constexpr double kFwdAbortClearance = 0.20;
// 区間の出だしは姿勢が定まらないので、少し走ってから見る。
constexpr double kFwdAbortMinTravel = 0.15;
// 前進を指令しているのに動かない、と判断する条件。
constexpr double kFwdNoMoveDist = 0.05;   // これ未満しか走っていない[m]
constexpr double kFwdNoMoveSec = 2.0;     // その状態がこれだけ続いたら[s]
// 復帰の開始時点で壁までの余裕がこれを割っていたら、前進を試さず後退から始める。
constexpr double kWedgedWallClear = 0.0;
// 計画は「中断されない経路」を出すべきである。中断は壁 kFwdAbortClearance /。
constexpr double kPlanWallClear = 0.28;   // 前進区間で確保を試みる壁との余裕[m]
constexpr double kPlanCarClear = 0.32;    // 同 他車との余裕[m]
// 上の条件では見つからない場所もある。そのときは現行どおり緩い条件で計画し、。
constexpr double kFwdAbortWallFloor = 0.02;  // 壁はここまでは許す(実接触の直前)
constexpr double kFwdAbortCarFloor = 0.10;   // 他車の接触は Crash 10秒なので厚めに残す
constexpr float kBrakeDoneSpeed = 0.25f;    // これ以下[m/s]なら止まったとみなす
constexpr double kBrakeMaxSec = 1.5;        // この時間[s]で止まれなければ診断を出す
constexpr float kBrakeAccel = -1.37f;       // 減速指令
constexpr double kEscapeDist = 2.5;        // 復帰開始地点からこれだけ[m]離れたら脱出とみなす
constexpr double kHandbackMargin = 0.60;   // 壁からこれだけ[m]離れて初めて通常制御へ返す
// 復帰中の目標速度[m/s]。計画どおり走らせるため低速で一定。
constexpr float kRecoverySpeed = 2.5f;
constexpr float kRecoveryAccel = 1.0f;     // 加速度上限(parameter.md MaxAccelerationInput)
constexpr double kCooldownSec = 2.0;       // 復帰完了後、再突入を禁止する時間[s]
constexpr double kNoPlanRetrySec = 0.25;   // 安全停止中の再探索は4Hz以下
constexpr double kContactLeaseSlotSec = 4.0;
constexpr double kContactLeaseGuardSec = 0.30;
// --- 最終手段 ---。
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
  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
  control_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
  gear_pub_ = create_publisher<GearCommand>("/control/command/gear_cmd", 1);
  status_pub_ = create_publisher<std_msgs::msg::String>(
      "/control/debug/recovery_status", rclcpp::QoS(1));
  recovery_layers_pub_ = create_publisher<std_msgs::msg::String>(
      "/control/debug/recovery_layers", rclcpp::QoS(1));

  // 動作確認用の強制発動。壁に当たらなくなると復帰が動く場面に出会えないため。
  const auto traj_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  traj_pub_ = create_publisher<Trajectory>("output/trajectory", traj_qos);
  // 後退用 pure_pursuit へ渡す経路。進行方向(=後退の向き)に点が並ぶ。
  reverse_traj_pub_ = create_publisher<Trajectory>("output/reverse_trajectory", traj_qos);
  // rviz 用。後退->前進->本線復帰 を 1本の Y字として出す。
  recovery_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/debug/recovery_path", rclcpp::QoS(1));
  // 復帰の当たり判定が見ている占有格子を rviz へ出す。
  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/planning/debug/recovery_grid",
      rclcpp::QoS(1).transient_local().reliable());
  traj_sub_ = create_subscription<Trajectory>(
    "input/trajectory", traj_qos,
    [this](const Trajectory::ConstSharedPtr msg) {
      const auto cb_t0 = std::chrono::steady_clock::now();
      latest_traj_ = msg;
      // 復帰中は認証済み経路を維持し、無動作時の方向変更はrunPhaseで行う。
      traj_following_ = publishRecoveryTrajectory();
      if (!traj_following_) { traj_pub_->publish(*msg); }
      profOtherCallback("経路の受信(publishRecoveryTrajectory)", cb_t0);
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
      reachable_wall_guard_have_steer_ = true;
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

  // Start前のグリッド待機はスタックではない。状態はlatched配信なので、ノードが。
  launch_no_recovery_sec_ = declare_parameter<double>("launch_no_recovery_sec", 8.0);
  // 【2026-09-17】AWSIM は state を transient_local で1レース数回しか送らない。volatile だけだと
  // 送出より後に起動したノードが開始を受け取れない。安全ゲート(volatile と見られる)にも
  // 対応するため両方の QoS で購読する。処理は !race_started_ で冪等。
  const auto on_race_state = [this](const std_msgs::msg::String::ConstSharedPtr msg) {
      // 公式 autostart_orchestrator と同じく Grounded / Ready / Start で開始とみなす。
      // 後から起動すると最後の Ready しか受け取れないため Ready も必要(2026-09-18 gate2 で再現)。
      // 実移動を観測するまで判定しないので待機中は安全。
      if (!race_started_ &&
          (msg->data == "Start" || msg->data == "Grounded" || msg->data == "Ready")) {
        race_started_ = true;
        moving_observed_ = false;
        stuck_start_time_.reset();
        blocked_vehicle_start_time_.reset();
        healthy_wait_since_.reset();
        pre_steer_valid_ = false;
        RCLCPP_INFO(get_logger(),
          "レース開始を検知(%s)。実移動を観測するまで復帰判定は開始しない", msg->data.c_str());
      }
    };
  race_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/awsim/state", rclcpp::QoS(10), on_race_state);
  race_state_sub_latched_ = create_subscription<std_msgs::msg::String>(
    "/awsim/state", rclcpp::QoS(1).transient_local().reliable(), on_race_state);

  nominal_sub_ = create_subscription<AckermannControlCommand>(
    "/control/command/nominal_control_cmd", 1,
    std::bind(&StuckRecoveryController::onNominalCommand, this, std::placeholders::_1));
  // 後退用 pure_pursuit の指令。後退区間ではこちらを中継する。
  reverse_sub_ = create_subscription<AckermannControlCommand>(
    "/control/command/reverse_control_cmd", 1,
    [this](const AckermannControlCommand::ConstSharedPtr msg) {
      reverse_cmd_ = msg; reverse_cmd_time_ = this->now();
    });
  // 復帰経路の計算に使う情報:。
  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state",
    rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
    [this](const Odometry::ConstSharedPtr msg) { odom_ = msg; });
  v2x_sub_ = create_subscription<V2XVehiclePositionArray>(
    "/v2x/vehicle_positions", rclcpp::QoS(10),
    [this](const V2XVehiclePositionArray::ConstSharedPtr msg) { v2x_ = msg; });

  reverse_traj_enable_ = declare_parameter<bool>("reverse_traj_enable", true);
  // 復帰の開始時点で既に壁へ食い込んでいるとき、前進を試さず後退から始めるか。
  wedge_backward_first_ = declare_parameter<bool>("wedge_backward_first", true);
  // 処理内容を示す。
  rev_gear_fix_ = declare_parameter<bool>("rev_gear_fix", false);
  rev_cmd_hold_sec_ = declare_parameter<double>("rev_cmd_hold_sec", 0.5);
  // 処理時間の計測(ログを出すだけで挙動は変えない)。
  slow_cycle_warn_ms_ = declare_parameter<double>("slow_cycle_warn_ms", 50.0);
  cycle_gap_warn_ms_ = declare_parameter<double>("cycle_gap_warn_ms", 200.0);
  wall_forward_ban_ = declare_parameter<bool>("wall_forward_ban", false);
  wall_forward_ban_hold_ = declare_parameter<double>("wall_forward_ban_hold", 0.8);
  wall_forward_ban_gain_ = declare_parameter<double>("wall_forward_ban_gain", 0.03);
  wall_forward_ban_speed_ = declare_parameter<double>("wall_forward_ban_speed", 1.0);
  wall_forward_ban_depth_ = declare_parameter<double>("wall_forward_ban_depth", 0.15);
  // ギアを後退へ入れる実装にしたところ、。
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
  // 完全停止のときは待ちを短くする ---。
  queue_wait_max_stopped_ = declare_parameter<double>("queue_wait_max_stopped", 4.0);
  // 待ちの条件を「自分の真正面に車がいる」ときだけにする。
  queue_wait_front_only_ = declare_parameter<bool>("queue_wait_front_only", true);
  // 手詰まり(計画を使い切って1歩も動けない)のときにフルブレーキをやめて後退する。
  deadlock_release_enable_ = declare_parameter<bool>("deadlock_release_enable", false);
  deadlock_release_sec_ = declare_parameter<double>("deadlock_release_sec", 5.0);
  deadlock_release_accel_ = declare_parameter<double>("deadlock_release_accel", 0.5);
  // 後退してよいと判断する、後方の他車までの最小距離[m]。
  deadlock_release_rear_m_ = declare_parameter<double>("deadlock_release_rear_m", 2.5);
  // 止まって壁に食い込んでいる間、到達可能壁予測の「停止」を効かせない。
  reachable_guard_allow_escape_ = declare_parameter<bool>("reachable_guard_allow_escape", true);
  reachable_guard_escape_sec_ = declare_parameter<double>("reachable_guard_escape_sec", 2.0);
  escape_probe_enable_ = declare_parameter<bool>("escape_probe_enable", false);
  escape_probe_sec_ = declare_parameter<double>("escape_probe_sec", 1.0);
  escape_probe_dist_ = declare_parameter<double>("escape_probe_dist", 0.5);
  escape_probe_gain_ = declare_parameter<double>("escape_probe_gain", 0.05);
  stopped_box_enable_ = declare_parameter<bool>("stopped_box_enable", true);
  // 向きの不確かさの増え方[rad/s]と上限[rad]。上限に達したら円へ戻す。
  stopped_yaw_rate_ = declare_parameter<double>("stopped_yaw_rate", 0.02);
  stopped_yaw_max_ = declare_parameter<double>("stopped_yaw_max", 0.15);
  // 復帰を速くする ---。
  recovery_speed_ = declare_parameter<double>("recovery_speed", 4.0);
  recovery_accel_ = declare_parameter<double>("recovery_accel", 1.37);
  // 計画が無いときの再探索の間隔[s]。0.25 = 4Hz は判断が遅い。
  no_plan_retry_sec_ = declare_parameter<double>("no_plan_retry_sec", 0.10);
  // 後退の解が無いとき、食い込みが浅くなる前進計画なら採る。
  embed_forward_escape_ = declare_parameter<bool>("embed_forward_escape", true);
  // 食い込み中に後退を試す最小距離[m]。0.75 固定だと 0.6m 使えても諦めていた。
  embed_min_reverse_ = declare_parameter<double>("embed_min_reverse", 0.20);
  // 食い込み中、区間の最初のこの距離[m]だけ計画の舵を直接出す。
  plan_steer_kick_ = declare_parameter<bool>("plan_steer_kick", true);
  plan_steer_kick_m_ = declare_parameter<double>("plan_steer_kick_m", 1.0);
  plan_steer_kick_speed_ = declare_parameter<double>("plan_steer_kick_speed", 2.0);
  plan_steer_kick_accel_ = declare_parameter<double>("plan_steer_kick_accel", 1.37);
  // 押し出しを続ける時間の上限[s]。動かないまま居座らせない。
  plan_steer_kick_sec_ = declare_parameter<double>("plan_steer_kick_sec", 1.2);
  // 復帰の区間中、追従が出す舵を計画の舵 ±これ[rad]に収める。0 で無効。
  plan_steer_band_ = declare_parameter<double>("plan_steer_band", 0.15);
  // 切り出し直後(この距離[m]まで)に壁が turn_in_abort_worsen 以上悪化したら引き直す。
  plan_local_steer_ = declare_parameter<bool>("plan_local_steer", true);
  // 復帰の新方式(前進/後退で pure_pursuit の目標点へ戻る)。
  recovery_simple_ = declare_parameter<bool>("recovery_simple", true);
  simple_ahead_m_ = declare_parameter<double>("simple_ahead_m", 6.0);
  simple_back_m_ = declare_parameter<double>("simple_back_m", 4.0);
  simple_probe_m_ = declare_parameter<double>("simple_probe_m", 4.0);
  // 後退で発振し目標に着かないため範囲を広げる。
  simple_reach_m_ = declare_parameter<double>("simple_reach_m", 2.0);
  // 一つの向きで動いてよい最大距離[m]。発振しても必ず終わらせる。
  simple_dir_max_m_ = declare_parameter<double>("simple_dir_max_m", 6.0);
  simple_wall_keep_ = declare_parameter<double>("simple_wall_keep", 0.05);
  simple_car_keep_ = declare_parameter<double>("simple_car_keep", 0.05);
  simple_speed_ = declare_parameter<double>("simple_speed", 3.0);
  simple_accel_ = declare_parameter<double>("simple_accel", 1.37);
  simple_steer_steps_ = declare_parameter<int>("simple_steer_steps", 8);
  simple_steer_frac_ = declare_parameter<double>("simple_steer_frac", 0.75);
  // 既に重なっているときは「これ以上悪化しない」を条件にする。その許容量[m]。
  simple_worsen_tol_ = declare_parameter<double>("simple_worsen_tol", 0.02);
  // 既に他車と重なっているときの許容[m]。壁より広く取る。
  simple_car_worsen_tol_ = declare_parameter<double>("simple_car_worsen_tol", 0.25);
  // 一度決めた向きを保持する時間[s]。ギアの往復を防ぐ。
  simple_dir_hold_sec_ = declare_parameter<double>("simple_dir_hold_sec", 1.5);
  // 候補が有効なのに実際には動けていないとき、。
  simple_stall_sec_ = declare_parameter<double>("simple_stall_sec", 3.0);
  press_flip_sec_ = declare_parameter<double>("press_flip_sec", 1.0);
  press_wall_m_ = declare_parameter<double>("press_wall_m", 0.2);
  steer_clamp_test_ = declare_parameter<double>("steer_clamp_test", 0.0);
  if (steer_clamp_test_ > 0.0) {
    RCLCPP_WARN(get_logger(), "【調査用】指令舵の上限を %.2frad(%.0fdeg)に上げている",
                steer_clamp_test_, steer_clamp_test_ * 180.0 / M_PI);
  }
  simple_stall_min_m_ = declare_parameter<double>("simple_stall_min_m", 0.15);
  simple_lost_confirm_sec_ =
    declare_parameter<double>("simple_lost_confirm_sec", 0.5);
  simple_short_probe_m_ = declare_parameter<double>("simple_short_probe_m", 1.0);
  // 壁に食い込んでいるとき、候補に要求する改善量[m]。
  simple_embed_improve_ = declare_parameter<double>("simple_embed_improve", 0.05);
  embed_prefer_reverse_ = declare_parameter<bool>("embed_prefer_reverse", true);
  simple_dir_stable_n_ = declare_parameter<int>("simple_dir_stable_n", 5);
  simple_dir_forward_ = true;   // 復帰は前進から試す
  turn_in_abort_ = declare_parameter<bool>("turn_in_abort", true);
  turn_in_abort_travel_ = declare_parameter<double>("turn_in_abort_travel", 0.5);
  turn_in_abort_worsen_ = declare_parameter<double>("turn_in_abort_worsen", 0.25);
  // 接触移動権(接触中の車で1台ずつ動く持ち回り)。本番の相手は同じコードを。
  contact_lease_enable_ = declare_parameter<bool>("contact_lease_enable", false);
  // 証明に合格した復帰計画の実行中は、到達可能壁予測の stop を適用しない。
  guard_trust_certificate_ = declare_parameter<bool>("guard_trust_certificate", true);
  // 送出経路が2つあり片方にクランプが無かった。両方に掛ける。
  steer_clamp_all_ = declare_parameter<bool>("steer_clamp_all", true);
  // 区間を始めてから停滞判定をしない時間[s]。1.2 は様子見が長い。
  phase_min_sec_ = declare_parameter<double>("phase_min_sec", 0.6);
  // 「列でじりじり動きながら進めない」形にも待ちを掛けるか。
  queue_wait_moving_ = declare_parameter<bool>("queue_wait_moving", false);
  reject_car_overlap_plan_ = declare_parameter<bool>("reject_car_overlap_plan", false);
  RCLCPP_INFO(get_logger(),
    "壁へ食い込んだままの前進を禁じる: %s (改善猶予 %.1fs / 改善とみなす増分 %.2fm)",
    wall_forward_ban_ ? "する" : "しない", wall_forward_ban_hold_, wall_forward_ban_gain_);
  goal_plan_enable_ = declare_parameter<bool>("goal_plan_enable", true);
  desperate_enable_ = declare_parameter<bool>("desperate_enable", false);
  path_check_enable_ = declare_parameter<bool>("path_check_enable", true);
  path_check_bad_sec_ = declare_parameter<double>("path_check_bad_sec", 0.4);
  steer_cmd_scale_ = declare_parameter<double>("steer_cmd_scale", 1.6666667);
  reachable_wall_guard_enable_ = declare_parameter<bool>("reachable_wall_guard_enable", true);
  reachable_wall_guard_options_.horizon = declare_parameter<double>("reachable_wall_horizon", 0.8);
  reachable_wall_guard_options_.step = declare_parameter<double>("reachable_wall_step", 0.02);
  reachable_wall_guard_options_.delay = declare_parameter<double>("reachable_wall_delay", 0.20);
  reachable_wall_guard_options_.steering_rate =
    declare_parameter<double>("reachable_wall_steering_rate", 1.05);
  reachable_wall_guard_options_.spatial_step =
    declare_parameter<double>("reachable_wall_spatial_step", 0.05);
  reachable_wall_guard_options_.clearance_margin =
    declare_parameter<double>("reachable_wall_margin", 0.0);
  reachable_wall_guard_options_.escape_improvement =
    declare_parameter<double>("reachable_wall_escape_improvement", 0.01);
  reachable_wall_guard_options_.candidates = declare_parameter<int>("reachable_wall_candidates", 41);
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
    // 起動時の1回だけでは、後から起動した rviz が Volatile で購読していると。
    grid_timer_ = create_wall_timer(std::chrono::milliseconds(2000),
                                    [this]() {
                                      const auto cb_t0 = std::chrono::steady_clock::now();
                                      publishGrid();
                                      profOtherCallback("占有格子の送出(publishGrid)", cb_t0);
                                    });
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
void StuckRecoveryController::updateOpponentTracks() const
{
  if (!v2x_) { return; }
  constexpr double kMovingOn = 0.60;    // これ以上で「走行中」に入る[m/s]
  constexpr double kMovingOff = 0.30;   // これ以下で「停止中」に戻る[m/s]
  constexpr double kTau = 0.30;         // 速度のなまし時定数[s]
  constexpr double kStaleSec = 1.0;     // これ以上空いたら履歴を捨てる[s]
  const rclcpp::Time now = v2x_->header.stamp;
  for (const auto & v : v2x_->vehicles) {
    auto & t = opp_track_[v.vehicle_id];
    if (!t.init) {
      t.x = v.position.x; t.y = v.position.y; t.stamp = now; t.init = true;
      continue;
    }
    const double dt = (now - t.stamp).seconds();
    if (dt <= 1e-3) { continue; }
    if (dt > kStaleSec) {   // 欠測が長い。速度は作らず位置だけ更新する
      t.x = v.position.x; t.y = v.position.y; t.stamp = now;
      t.vx = 0.0; t.vy = 0.0; t.moving = false;
      continue;
    }
    const double vx = (v.position.x - t.x) / dt;
    const double vy = (v.position.y - t.y) / dt;
    const double a = dt / (kTau + dt);
    t.vx += (vx - t.vx) * a;
    t.vy += (vy - t.vy) * a;
    t.x = v.position.x; t.y = v.position.y; t.stamp = now;
    const double sp = std::hypot(t.vx, t.vy);
    const bool was_moving = t.moving;
    if (!t.moving && sp >= kMovingOn) { t.moving = true; }
    else if (t.moving && sp <= kMovingOff) { t.moving = false; }
    // 動いている間は向きを控え続ける。止まった瞬間の時刻も残す。
    if (t.moving) {
      t.last_yaw = std::atan2(t.vy, t.vx);
      t.last_yaw_valid = true;
      t.stopped_since_valid = false;
    } else if (was_moving && !t.stopped_since_valid) {
      t.stopped_since = now;
      t.stopped_since_valid = true;
    }
  }
}

std::vector<recovery::CarObstacle> StuckRecoveryController::carObstacles() const
{
  std::vector<recovery::CarObstacle> out;
  if (!odom_ || !v2x_) { return out; }
  updateOpponentTracks();
  const auto & self = odom_->pose.pose.position;
  for (const auto & v : v2x_->vehicles) {
    const double dx = v.position.x - self.x;
    const double dy = v.position.y - self.y;
    if (std::hypot(dx, dy) > kCarObstacleRange) { continue; }
    recovery::CarObstacle c{};
    c.x = v.position.x;
    c.y = v.position.y;
    c.id = v.vehicle_id;
    const auto it = opp_track_.find(v.vehicle_id);
    if (it != opp_track_.end() && it->second.moving) {
      c.moving = true;
      c.yaw = std::atan2(it->second.vy, it->second.vx);
    } else if (stopped_box_enable_ && it != opp_track_.end() && it->second.last_yaw_valid) {
      // 止まった相手も矩形で扱う ---。
      const double held = it->second.stopped_since_valid
        ? (this->now() - it->second.stopped_since).seconds() : 0.0;
      // idx22-25 の有効幅は 3.30〜3.65m。自車半幅 0.65 と。
      const double yaw_err = std::min(stopped_yaw_rate_ * held, stopped_yaw_max_);
      c.moving = true;                 // 矩形として扱う
      c.yaw = it->second.last_yaw;
      c.extra_half_width = 1.032 * std::sin(yaw_err);   // 半長 x sin(誤差)
    }
    out.push_back(c);
  }
  return out;
}

// 後方の他車までの距離[m]。いなければ大きな値。
void StuckRecoveryController::logV2XSelfOffset()
{
  if (!v2x_ || !odom_ || vehicle_id_.empty()) { return; }
  const auto now_t = this->now();
  if ((now_t - last_v2x_self_log_).seconds() < 2.0) { return; }
  for (const auto & v : v2x_->vehicles) {
    if (v.vehicle_id != vehicle_id_) { continue; }
    last_v2x_self_log_ = now_t;
    const double ex = odom_->pose.pose.position.x;
    const double ey = odom_->pose.pose.position.y;
    const auto & q = odom_->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double dx = v.position.x - ex, dy = v.position.y - ey;
    const double lon = dx * std::cos(yaw) + dy * std::sin(yaw);
    const double lat = -dx * std::sin(yaw) + dy * std::cos(yaw);
    RCLCPP_WARN(get_logger(),
      "V2X自己照合 id=%s 進行方向%+.3fm 横%+.3fm 速度%.1fkm/h "
      "(0なら odom と同じ点 / +1.55 なら odom は後軸で V2X は前端)",
      vehicle_id_.c_str(), lon, lat, std::abs(latest_velocity_) * 3.6);
    return;
  }
  // 自車が V2X に載っていないことも1回だけ残す。
  if (!v2x_self_absent_logged_) {
    v2x_self_absent_logged_ = true;
    last_v2x_self_log_ = now_t;
    RCLCPP_WARN(get_logger(), "V2X自己照合 自車 %s は V2X に載っていない(%zu台配信中)",
                vehicle_id_.c_str(), v2x_->vehicles.size());
  }
}

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


// ===================================================================。
void StuckRecoveryController::traceInit()
{
  if (trace_ready_) { return; }
  trace_ready_ = true;
  const char * ld = std::getenv("LOG_DIR");
  const char * dom = std::getenv("ROS_DOMAIN_ID");
  if (!ld || !*ld) { return; }
  trace_dir_ = std::string(ld) + "/d" + (dom ? dom : "0");
  std::filesystem::create_directories(trace_dir_ + "/events");
  trace_ofs_.open(trace_dir_ + "/trace_recovery.tsv", std::ios::out);
  if (trace_ofs_) { trace_ofs_ << traceHeader() << "\n"; trace_ofs_.flush(); }
}

std::string StuckRecoveryController::traceHeader() const
{
  return "t\tcb_seq\tdt\tx\ty\tyaw\tv\tsteer_rep\tgear_rep\tsent_accel\tsent_speed"
         "\tsent_gear\tsent_steer_cmd\tsent_steer_phys\tcmd_src\tselection_reason"
         "\twall_clear\tin_recovery\telapsed\tphase\tphases"
         "\tphase_fwd\treplan\tdesperate\tsituation\tstuck_sec\tmoved_1s"
         "\ttraj_from\tphase_path_begin\tphase_path_end\tpath_points"
         "\treverse_following\ttraj_giveup\tbraking\trear_waiting"
         "\tphase_travelled\tphase_length\tplan_wall_clear\tphase_start_wall_clear"
         "\ttarget_lon\ttarget_lat\ttarget_wall_clear"
         "\treverse_cmd_age\treverse_cmd_accel\treverse_cmd_speed\treverse_cmd_steer"
         "\tgear_cmd_age\treverse_deadlock\trev_off_why"
         // V2X の基準点を測るための生値。
         "\tnear_id\tnear_v2x_x\tnear_v2x_y";
}

std::string StuckRecoveryController::traceLine(const rclcpp::Time & now)
{
  recovery::Pose p;
  const bool have = currentPose(p);
  const double dt = (last_trace_t_.nanoseconds() > 0)
                      ? (now - last_trace_t_).seconds() : -1.0;
  last_trace_t_ = now;
  const bool inrec = recovery_start_time_.has_value();
  const bool have_phase = plan_.valid && phase_idx_ < plan_.phases.size();
  const bool reverse_phase = inrec && have_phase && !plan_.phases[phase_idx_].forward;
  const std::string source = cmd_src_ ? cmd_src_ : "-";
  const char * selection_reason = "not_reverse_phase";
  if (reverse_phase) {
    if (source.rfind("後退経路", 0) == 0) {
      selection_reason = "reverse_selected";
    } else if (braking_) {
      selection_reason = "braking";
    } else if (rear_waiting_) {
      selection_reason = "rear_wait";
    } else if (!reverse_following_) {
      selection_reason = "reverse_following_off";
    } else if (!reverse_cmd_) {
      selection_reason = "reverse_cmd_missing";
    } else {
      selection_reason = "other";
    }
  }
  const double reverse_age = reverse_cmd_ ? (now - reverse_cmd_time_).seconds() : -1.0;
  const double reverse_accel = reverse_cmd_ ? reverse_cmd_->longitudinal.acceleration : 0.0;
  const double reverse_speed = reverse_cmd_ ? reverse_cmd_->longitudinal.speed : 0.0;
  const double reverse_steer = reverse_cmd_
    ? reverse_cmd_->lateral.steering_tire_angle * 180.0 / M_PI : 0.0;
  double target_lon = 0.0;
  double target_lat = 0.0;
  double target_clear = 999.0;
  if (have && have_phase && plan_.phases[phase_idx_].path_end > 0 &&
      plan_.phases[phase_idx_].path_begin < plan_.path.size())
  {
    const std::size_t end = std::min(plan_.phases[phase_idx_].path_end, plan_.path.size());
    const std::size_t target_idx = std::min(
      std::max(traj_from_ + 1, plan_.phases[phase_idx_].path_begin), end - 1);
    const auto & target = plan_.path[target_idx];
    const double dx = target.x - p.x;
    const double dy = target.y - p.y;
    target_lon = dx * std::cos(p.yaw) + dy * std::sin(p.yaw);
    target_lat = -dx * std::sin(p.yaw) + dy * std::cos(p.yaw);
    if (obstacles_.valid()) {
      target_clear = recovery::wallClearanceAt(obstacles_, veh_, target);
    }
  }
  const double gear_age = gear_changed_.nanoseconds() > 0
    ? (now - gear_changed_).seconds() : -1.0;
  char buf[1800];
  // 最も近い他車の V2X 位置。
  std::string near_id = "-"; double near_x = 0.0, near_y = 0.0;
  if (v2x_ && odom_) {
    const auto & me = odom_->pose.pose.position;
    double bd = 1e18;
    for (const auto & vv : v2x_->vehicles) {
      if (vv.vehicle_id == vehicle_id_) { continue; }
      const double dd = std::hypot(vv.position.x - me.x, vv.position.y - me.y);
      if (dd < bd) { bd = dd; near_id = vv.vehicle_id; near_x = vv.position.x; near_y = vv.position.y; }
    }
  }

  std::snprintf(buf, sizeof(buf),
    "%.3f\t%lu\t%.4f\t%.2f\t%.2f\t%.1f\t%.3f\t%.1f\t%u\t%+.2f\t%+.2f\t%u"
    "\t%+.1f\t%+.1f\t%s\t%s"
    "\t%.2f\t%d\t%.2f\t%zu\t%zu\t%d\t%d\t%d\t%s\t%.2f\t%.3f"
    "\t%zu\t%zu\t%zu\t%zu\t%d\t%d\t%d\t%d"
    "\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f"
    "\t%.3f\t%+.2f\t%+.2f\t%+.1f\t%.3f\t%d\t%s\t%s\t%.3f\t%.3f",
    now.seconds(), trace_cb_seq_, dt,
    have ? p.x : 0.0, have ? p.y : 0.0, have ? p.yaw * 180.0 / M_PI : 0.0,
    latest_velocity_, steer_report_ * 180.0 / M_PI,
    static_cast<unsigned>(gear_report_),
    sent_accel_, sent_speed_, static_cast<unsigned>(sent_gear_),
    sent_steer_cmd_ * 180.0 / M_PI, sent_steer_phys_ * 180.0 / M_PI,
    source.c_str(), selection_reason,
    wall_clear_now_,
    inrec ? 1 : 0,
    inrec ? (now - recovery_start_time_.value()).seconds() : -1.0,
    phase_idx_, plan_.valid ? plan_.phases.size() : 0,
    have_phase ? (plan_.phases[phase_idx_].forward ? 1 : 0) : -1,
    replan_count_, desperate_ ? 1 : 0,
    situation_.empty() ? "-" : situation_.c_str(),
    stuck_start_time_.has_value() ? (now - stuck_start_time_.value()).seconds() : -1.0,
    trace_moved_1s_, traj_from_,
    have_phase ? plan_.phases[phase_idx_].path_begin : 0,
    have_phase ? plan_.phases[phase_idx_].path_end : 0,
    plan_.valid ? plan_.path.size() : 0,
    reverse_following_ ? 1 : 0, 0,
    braking_ ? 1 : 0, rear_waiting_ ? 1 : 0,
    phase_travelled_, have_phase ? plan_.phases[phase_idx_].length : -1.0,
    plan_wall_clear_, phase_start_wall_clear_, target_lon, target_lat, target_clear,
    reverse_age, reverse_accel, reverse_speed, reverse_steer,
    gear_age, reverse_deadlock_active_ ? 1 : 0,
    rev_off_why_ ? rev_off_why_ : "-",
    near_id.c_str(), near_x, near_y);
  return std::string(buf);
}

void StuckRecoveryController::publishRecoveryMeasure(const rclcpp::Time & now)
{
  if (!recovery_layers_pub_) { return; }
  if ((now - last_recovery_measure_).seconds() < 0.2) { return; }
  last_recovery_measure_ = now;
  const bool in_rec = recovery_start_time_.has_value();
  const double rec_sec = in_rec ? (now - recovery_start_time_.value()).seconds() : 0.0;
  // rviz は日本語を文字化けさせるので英字で出す(日本語の内容は autoware.log にある)。
  const std::string src = cmd_src_ ? cmd_src_ : "-";
  const auto has = [&src](const char * k) { return src.find(k) != std::string::npos; };
  std::string src_en = "OTHER(" + src + ")";
  if (has("通常制御(舵先回り)")) { src_en = "NORMAL (pre-steer to plan)"; }
  else if (has("通常制御")) { src_en = "NORMAL (pure_pursuit)"; }
  else if (has("後退経路")) { src_en = "RECOVERY reverse path"; }
  else if (has("後退指令待ち")) { src_en = "RECOVERY waiting reverse cmd"; }
  else if (has("前進経路")) { src_en = "RECOVERY forward path"; }
  else if (has("復帰(簡易)")) { src_en = "RECOVERY simple"; }
  else if (has("最終手段")) { src_en = "RECOVERY last resort"; }
  else if (has("制動")) { src_en = "RECOVERY braking"; }
  else if (has("待ち")) { src_en = "RECOVERY waiting"; }
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    "[CTRL] out = %s\n"
    "  recovery = %s%s | plan = %s phase %zu/%zu%s\n"
    "  cmd %.1fkm/h accel %+.2f steer %+.0fdeg gear %s | ego %.1fkm/h\n"
    "  wall clearance %.2fm%s",
    src_en.c_str(),
    in_rec ? "ON" : "off",
    in_rec ? (" " + std::to_string(static_cast<int>(rec_sec)) + "s").c_str() : "",
    plan_.valid ? "yes" : "no", phase_idx_, plan_.phases.size(),
    (plan_.valid && phase_idx_ < plan_.phases.size())
      ? (plan_.phases[phase_idx_].forward ? " forward" : " reverse") : "",
    sent_speed_ * 3.6, sent_accel_, sent_steer_phys_ * 180.0 / M_PI,
    sent_gear_ == GearCommand::REVERSE ? "R" : "D",
    latest_velocity_ * 3.6,
    wall_clear_now_,
    (wall_forward_ban_ && wall_clear_now_ < 0.0) ? " | INSIDE WALL (forward banned)" : "");
  std_msgs::msg::String m;
  m.data = buf;
  recovery_layers_pub_->publish(m);
}

void StuckRecoveryController::traceSample(const rclcpp::Time & now)
{
  ProfScope prof_scope(prof_, "traceSample");
  publishRecoveryMeasure(now);
  traceInit();
  // 「1秒で何m進んだか」= 停滞判定の入力そのもの。判断の材料は必ず残す。
  recovery::Pose p;
  if (currentPose(p)) {
    if (trace_ref_t_.nanoseconds() == 0 || (now - trace_ref_t_).seconds() >= 1.0) {
      trace_moved_1s_ = std::hypot(p.x - trace_ref_x_, p.y - trace_ref_y_);
      trace_ref_x_ = p.x; trace_ref_y_ = p.y; trace_ref_t_ = now;
    }
  }
  detectReverseDeadlock(now);
  detectUnexpectedReverse(now, "control_publish");
  if (trace_dir_.empty()) { return; }
  ++trace_cb_seq_;
  const std::string line = traceLine(now);
  ring_.push_back(line);
  if (ring_.size() > kRingMax) { ring_.pop_front(); }
  if (trace_ofs_ &&
      (last_trace_write_.nanoseconds() == 0 ||
       (now - last_trace_write_).seconds() >= 0.2))     // 5Hz
  {
    last_trace_write_ = now;
    trace_ofs_ << line << "\n";
    trace_ofs_.flush();
  }
}

void StuckRecoveryController::detectReverseDeadlock(const rclcpp::Time & now)
{
  recovery::Pose p;
  const bool have_pose = currentPose(p);
  const bool reverse_phase = recovery_start_time_.has_value() && plan_.valid &&
    phase_idx_ < plan_.phases.size() && !plan_.phases[phase_idx_].forward;
  const double gear_age = gear_changed_.nanoseconds() > 0
    ? (now - gear_changed_).seconds() : -1.0;
  const bool stalled = reverse_phase && gear_report_seen_ &&
    gear_report_ == GearCommand::REVERSE && gear_age >= kActuatorWaitMax &&
    std::abs(latest_velocity_) < 0.05 && trace_moved_1s_ < 0.05;

  if (!stalled) {
    reverse_deadlock_since_.reset();
    reverse_deadlock_active_ = false;
    return;
  }
  if (!reverse_deadlock_since_) {
    reverse_deadlock_since_ = now;
    if (have_pose) {
      reverse_deadlock_x_ = p.x;
      reverse_deadlock_y_ = p.y;
    }
    return;
  }
  const double held = (now - reverse_deadlock_since_.value()).seconds();
  if (reverse_deadlock_active_ || held < 3.0) { return; }

  reverse_deadlock_active_ = true;
  ++reverse_deadlock_count_;
  const std::string source = cmd_src_ ? cmd_src_ : "-";
  const bool reverse_source = source.rfind("後退経路", 0) == 0;
  const char * classification = !reverse_source
    ? "WRONG_COMMAND_SELECTION"
    : (sent_accel_ >= 0.8f ? "PHYSICAL_OR_GEOMETRY_LOCK" : "NONPROPULSIVE_REVERSE");
  bool loop = false;
  double loop_dt = -1.0;
  double loop_dist = -1.0;
  if (last_reverse_deadlock_at_ && have_pose) {
    loop_dt = (now - last_reverse_deadlock_at_.value()).seconds();
    loop_dist = std::hypot(p.x - last_reverse_deadlock_x_, p.y - last_reverse_deadlock_y_);
    loop = loop_dt >= 0.0 && loop_dt <= 45.0 && loop_dist <= 0.5;
  }
  RCLCPP_ERROR(get_logger(),
    "自動検知 REVERSE停止 #%d 分類=%s 継続=%.1fs ループ=%d(前回から%.1fs/%.2fm) "
    "出=%s 加速度=%+.2f 指令速度=%+.2f ギア報告/送出=%u/%u 実速度=%.3f "
    "移動1秒=%.3f 諦め=%d 後退追従=%d 引き直し=%d 区間=%zu/%zu "
    "走行=%.2f/%.2fm 壁余裕=%.2f",
    reverse_deadlock_count_, classification, held, loop ? 1 : 0, loop_dt, loop_dist,
    source.c_str(), sent_accel_, sent_speed_, static_cast<unsigned>(gear_report_),
    static_cast<unsigned>(sent_gear_), latest_velocity_, trace_moved_1s_,
    0, reverse_following_ ? 1 : 0, replan_count_,
    phase_idx_, plan_.phases.size(), phase_travelled_,
    plan_.phases[phase_idx_].length, wall_clear_now_);
  traceDump(now, loop ? "reverse_recovery_loop" : "reverse_deadlock");
  last_reverse_deadlock_at_ = now;
  if (have_pose) {
    last_reverse_deadlock_x_ = p.x;
    last_reverse_deadlock_y_ = p.y;
  }
}

void StuckRecoveryController::detectUnexpectedReverse(
  const rclcpp::Time & now, const char * trigger)
{
  const bool have_phase = recovery_start_time_.has_value() && plan_.valid &&
    phase_idx_ < plan_.phases.size();
  const bool reverse_phase = have_phase && !plan_.phases[phase_idx_].forward;
  const bool reverse_braking = recovery_start_time_.has_value() && braking_ &&
    !brake_was_forward_;
  const bool reverse_expected = desperate_ || reverse_phase || reverse_braking;
  const bool command_reverse = gear_now_ == GearCommand::REVERSE;
  const bool reported_reverse = gear_report_seen_ && gear_report_ == GearCommand::REVERSE;
  const bool unexpected = !reverse_expected && (command_reverse || reported_reverse);
  if (!unexpected) {
    unexpected_reverse_active_ = false;
    return;
  }
  if (unexpected_reverse_active_) { return; }

  unexpected_reverse_active_ = true;
  ++unexpected_reverse_count_;
  const bool nonpropulsive_stop = command_reverse && std::abs(sent_speed_) <= 0.05f &&
    sent_accel_ < 0.0f;
  const char * classification = nonpropulsive_stop
    ? "REVERSE_GEAR_SAFE_STOP"
    : (command_reverse ? "WRONG_REVERSE_COMMAND" : "REVERSE_REPORT_LAG");
  RCLCPP_ERROR(get_logger(),
    "自動検知 非後退状態REVERSE #%d 分類=%s trigger=%s 復帰中=%d 計画有効=%d "
    "区間=%zu/%zu 区間前進=%d 制動=%d 最終手段=%d 指令ギア=%u 報告ギア=%u "
    "出=%s 指令速度=%+.2f 加速度=%+.2f 実速度=%+.2f",
    unexpected_reverse_count_, classification, trigger,
    recovery_start_time_.has_value() ? 1 : 0,
    plan_.valid ? 1 : 0, phase_idx_, plan_.phases.size(),
    have_phase ? (plan_.phases[phase_idx_].forward ? 1 : 0) : -1,
    braking_ ? 1 : 0, desperate_ ? 1 : 0,
    static_cast<unsigned>(gear_now_), static_cast<unsigned>(gear_report_),
    cmd_src_ ? cmd_src_ : "-", sent_speed_, sent_accel_, latest_velocity_);
  traceDump(now, "unexpected_reverse");
}

void StuckRecoveryController::traceDump(const rclcpp::Time & now, const char * kind)
{
  ProfScope prof_scope(prof_, "traceDump");
  if (trace_dir_.empty() || ring_.empty()) { return; }
  char name[256];
  std::snprintf(name, sizeof(name), "%s/events/%.1f_%s.tsv",
                trace_dir_.c_str(), now.seconds(), kind);
  std::ofstream f(name);
  if (!f) { return; }
  f << traceHeader() << "\n";
  for (const auto & l : ring_) { f << l << "\n"; }
}

void StuckRecoveryController::profCycleBegin(std::chrono::steady_clock::time_point t0)
{
  prof_.clear();
  if (prof_have_prev_) {
    const double gap_ms =
      std::chrono::duration<double, std::milli>(t0 - prof_prev_begin_).count();
    const double t0_s = std::chrono::duration<double>(t0.time_since_epoch()).count();
    if (gap_ms >= cycle_gap_warn_ms_ && t0_s - prof_last_gap_log_s_ >= 1.0) {
      prof_last_gap_log_s_ = t0_s;
      // 間隔が空いた理由の切り分け:
      //   直前の周期の所要が大きい      -> この周期自身が重かった(内訳は直前の「処理が遅い」)
      //   他のコールバックの所要が大きい -> 同じスレッドの別処理が塞いだ
      //   どちらも小さい                -> 入力(pure_pursuit)が来なかった、または CPU を取れなかった
      RCLCPP_WARN(get_logger(),
        "処理時間 指令周期の間隔が空いた %.0fms 直前の周期の所要%.0fms "
        "間に走った他のコールバックで最大=%s %.0fms",
        gap_ms, prof_prev_cycle_ms_, prof_other_name_, prof_other_ms_);
    }
  }
  prof_have_prev_ = true;
  prof_prev_begin_ = t0;
  prof_other_name_ = "-";
  prof_other_ms_ = 0.0;
}

void StuckRecoveryController::profCycleEnd(std::chrono::steady_clock::time_point t0)
{
  const double total_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0).count();
  prof_prev_cycle_ms_ = total_ms;
  if (total_ms < slow_cycle_warn_ms_) { return; }
  // 同じ名前を合算する。時間は入れ子を含む(makePlan は beginRecovery の内側に含まれる)。
  struct Agg { const char * name; double sum; double max; int n; };
  std::vector<Agg> agg;
  for (const auto & e : prof_) {
    auto it = std::find_if(agg.begin(), agg.end(), [&](const Agg & a) {
      return std::string(a.name) == e.name;
    });
    if (it == agg.end()) { agg.push_back({e.name, e.ms, e.ms, 1}); }
    else { it->sum += e.ms; it->max = std::max(it->max, e.ms); ++it->n; }
  }
  std::sort(agg.begin(), agg.end(), [](const Agg & a, const Agg & b) { return a.sum > b.sum; });
  std::string desc;
  for (std::size_t i = 0; i < agg.size() && i < 10; ++i) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), " %s=%.0fms(x%d 最大%.0fms)",
                  agg[i].name, agg[i].sum, agg[i].n, agg[i].max);
    desc += buf;
  }
  RCLCPP_WARN(get_logger(),
    "処理時間 処理が遅い 指令周期の所要%.0fms 出=%s 内訳(入れ子を含む):%s",
    total_ms, cmd_src_ ? cmd_src_ : "-", desc.empty() ? " なし" : desc.c_str());
}

void StuckRecoveryController::profOtherCallback(
  const char * name, std::chrono::steady_clock::time_point t0)
{
  const double ms = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0).count();
  if (ms > prof_other_ms_) {
    prof_other_ms_ = ms;
    prof_other_name_ = name;
  }
  if (ms >= slow_cycle_warn_ms_) {
    RCLCPP_WARN(get_logger(), "処理時間 処理が遅い コールバック=%s 所要%.0fms", name, ms);
  }
}

void StuckRecoveryController::onNominalCommand(
  const AckermannControlCommand::ConstSharedPtr msg)
{
  // 周期の所要時間を測る。途中の return すべてで終わりを記録するため RAII にする。
  const auto cycle_t0 = std::chrono::steady_clock::now();
  profCycleBegin(cycle_t0);
  struct CycleGuard
  {
    StuckRecoveryController & self;
    std::chrono::steady_clock::time_point t0;
    ~CycleGuard() { self.profCycleEnd(t0); }
  } cycle_guard{*this, cycle_t0};
  const auto now = this->now();
  // 動作確認用。指示を受けたら数秒だけ全舵で壁へ向かって走る。
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
  updateWallBanState();
  if (wall_ban_hit_) {
    wall_ban_hit_ = false;
    // 拒否は毎周期成立する。反応するのは。
    const double tnow = now.seconds();
    // 「1計画につき1回」を復帰していないときにも。
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
      const bool in_launch_hold =
        launch_moving_since_.has_value() &&
        (now - launch_moving_since_.value()).seconds() < launch_no_recovery_sec_;
      if (!recovery_start_time_.has_value() && !in_cooldown && !in_launch_hold) {
        RCLCPP_WARN(get_logger(),
          "壁前進禁止が前進を止めた。前進が塞がれているとみて復帰を始める");
        stuck_start_time_.reset();
        blocked_vehicle_start_time_.reset();
        recovery_start_time_ = now;
        simple_tgt_valid_ = false;   // 復帰の開始時に目標点を取り直す
        beginRecovery(now, true, false);   // この経路は上で has_value() を確認済み
        if (runRecovery(now)) { return; }
      } else if (recovery_start_time_.has_value() &&
                 plan_.valid && phase_idx_ < plan_.phases.size() &&
                 plan_.phases[phase_idx_].forward)
      {
        blocked_dir_ = +1;
        // 引き直しても出口が拒否し続けるなら、引き直しでは解けない。
        if (replan_count_ >= kReplanMax) {
          if (!desperate_ && desperate_enable_) {
            RCLCPP_WARN(get_logger(),
              "壁前進禁止が %d回 前進を止めた。引き直しでは解けないので"
              "後退での脱出に切り替える", replan_count_);
            desperate_ = true;
            desperate_since_ = now;
            desperate_swing_ = -1;
          } else if (!desperate_ &&
                     (now - last_desperate_log_).seconds() > 3.0) {
            last_desperate_log_ = now;
            RCLCPP_WARN(get_logger(),
              "壁前進禁止が %d回 前進を止めた。最終手段は無効なので、"
              "証明可能な後退経路を待つ", replan_count_);
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
  // 後退phaseはアクチュエータを所有する。新鮮な専用指令だけを通し、。
  const bool rev_phase_now =
    recovery_start_time_.has_value() && plan_.valid &&
    phase_idx_ < plan_.phases.size() && !plan_.phases[phase_idx_].forward;
  if (rev_phase_now && reverse_cmd_) {
    const double age = (now - reverse_cmd_time_).seconds();
    if (age >= 0.0 && age <= rev_cmd_hold_sec_) {
      if (!reverse_following_) { ++rev_hold_used_; }
      cmd_src_ = reverse_following_ ? "後退経路" : "後退経路(鮮度保持)";
      publishFiltered(*reverse_cmd_);
      return;
    }
    ++rev_hold_stale_;
    if ((now - last_rev_stale_log_).seconds() > 2.0) {
      last_rev_stale_log_ = now;
      RCLCPP_WARN(get_logger(),
        "後退指令が古い(%.2fs > %.2fs)。通常前進へは落とさず停止して更新を待つ "
        "(鮮度保持で出した周期%d 古くて停止%d)",
        age, rev_cmd_hold_sec_, rev_hold_used_, rev_hold_stale_);
    }
  }
  if (rev_phase_now) {
    // The reverse phase owns the actuator.  Missing/stale reverse output may。
    cmd_src_ = "後退指令待ち";
    // publishCommand は**実舵角**を受け取り、内部で。
    publishCommand(0.0f, 0.0f,
                   static_cast<float>(msg->lateral.steering_tire_angle /
                                      std::max(steer_cmd_scale_, 1e-3)));
    return;
  }
  // 復帰の前進区間を経路で渡している間は、pure_pursuit の指令をそのまま通す。
  if (recovery_start_time_.has_value() && traj_following_) {
    // ここは control_pub_ へ直接流しており、。
    cmd_src_ = "前進経路";
    publishFiltered(*msg);
    return;
  }
  // 停滞候補の間は、これから使う復帰計画の舵角へ先回りして向けておく。
  if (pre_steer_valid_ && std::abs(latest_velocity_) <= kStuckSpeedThreshold) {
    AckermannControlCommand out = *msg;
    out.lateral.steering_tire_angle = static_cast<float>(pre_steer_);
    // この分岐が復帰のどの状態で走るのかを記録する ---。
    if (recovery_start_time_.has_value() &&
        (now - last_presteer_log_).seconds() > 0.5)
    {
      last_presteer_log_ = now;
      RCLCPP_WARN(get_logger(),
        "舵先回り診断 計画有効=%d 区間=%zu/%zu 向き=%s 後退追従=%d 前進追従=%d "
        "諦め=%d 加速度=%+.2f 実速度=%.2f",
        plan_.valid ? 1 : 0, phase_idx_, plan_.phases.size(),
        (plan_.valid && phase_idx_ < plan_.phases.size())
          ? (plan_.phases[phase_idx_].forward ? "前進" : "後退") : "なし",
        reverse_following_ ? 1 : 0, traj_following_ ? 1 : 0,
        0,
        out.longitudinal.acceleration, latest_velocity_);
    }
    // 舵先回りの制動を後退区間で外す、という案 ---。
    cmd_src_ = "通常制御(舵先回り)";
    publishFiltered(out);
  } else {
    cmd_src_ = "通常制御";
    publishFiltered(*msg);
  }
  updateStuckDetection(*msg, now);
}

// 車体前方の同一レーンにいる最寄りの他車までの距離[m]。いなければ無限大。
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
  ProfScope prof_scope(prof_, "updateStuckDetection");
  const float velocity = latest_velocity_;
  logV2XSelfOffset();   // V2X の基準点を実測する(2秒に1回)
  // Ready中は指令値・近接車・経過時間にかかわらず復帰状態を作らない。
  if (!race_started_) {
    moving_observed_ = false;
    stuck_start_time_.reset();
    blocked_vehicle_start_time_.reset();
    healthy_wait_since_.reset();
    entry_ref_valid_ = false;
    pre_steer_valid_ = false;
    return;
  }
  // Require movement once to avoid detecting the initial stationary state as stuck.
  if (velocity >= kMovingSpeedThreshold) {
    if (!moving_observed_) {
      launch_moving_since_ = now;
      RCLCPP_INFO(get_logger(), "発進を観測。%.1f 秒間は復帰を始めない", launch_no_recovery_sec_);
    }
    moving_observed_ = true;
  }
  // 動けているなら手詰まりではない。計時を落とす。
  if (std::abs(velocity) >= kBrakeDoneSpeed) { deadlock_since_.reset(); }

  // 動作確認用の強制発動。壁に当たらなくなると復帰が動く場面に出会えないため、。
  if (force_recovery_) {
    force_recovery_ = false;
    stuck_start_time_.reset();
    recovery_start_time_ = now;
    beginRecovery(now, false, false);   // 強制発動(動作確認用)
    return;
  }

  // `motion_requested`(指令速度>=1.0 かつ指令加速度>=0.3)は。

  const double nearest_forward_vehicle = nearestForwardVehicle();
  // 近くに他車がいて止まっているか。
  const bool car_near = std::isfinite(nearest_forward_vehicle) || anyVehicleNear();
  const bool blocked_by_vehicle =
    std::abs(command.longitudinal.speed) < kCommandSpeedThreshold &&
    std::abs(velocity) <= kStuckSpeedThreshold &&
    car_near;
  // --- 前の車に詰まって待っているだけの車を「スタック」にしない ---。
  bool healthy_wait = false;
  // 以前は blocked_by_vehicle(指令速度も実速度も。
  const bool front_car = std::isfinite(nearest_forward_vehicle);
  const bool queue_wait_wide = queue_wait_moving_ && front_car;
  // 待つのは真正面に車がいるときだけ。
  const bool blocked_for_wait =
    blocked_by_vehicle && (!queue_wait_front_only_ || front_car);
  if (queue_wait_enable_ && (blocked_for_wait || queue_wait_wide)) {
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
  // --- 待機にも上限を置く(安全網) ---。
  if (healthy_wait) {
    if (!healthy_wait_since_) { healthy_wait_since_ = now; }
    else {
      const bool fully_stopped = std::abs(velocity) < kStuckSpeedThreshold;
      const double cap = (fully_stopped && queue_wait_max_stopped_ > 0.0)
                           ? std::min(queue_wait_max_stopped_, queue_wait_max_)
                           : queue_wait_max_;
      if ((now - healthy_wait_since_.value()).seconds() > cap) {
        healthy_wait = false;          // 待ちすぎ。復帰を始めさせる
        if ((now - last_queue_wait_log_).seconds() > 3.0) {
          last_queue_wait_log_ = now;
          RCLCPP_WARN(get_logger(),
            "前が詰まって %.1f秒 待った(%s)。復帰を始める",
            cap, fully_stopped ? "完全停止" : "微速前進");
        }
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

  // 膠着(hard_stall)と壁接触膠着は削除した。同じ「進んでいない」条件の重複で、。

  // 門を外した ---。
  if (!moving_observed_) {
    stuck_start_time_.reset();
    pre_steer_valid_ = false;
    return;
  }

  // 近接車条件は専用タイマーで既に3秒を確認済み。通常条件へ切り替わった。
  if (blocked_vehicle_ready) {
    stuck_start_time_ = now - rclcpp::Duration::from_seconds(kStuckDurationSec);
  }

  // --- 「進んでいない」を瞬間速度ではなく実際の移動量で見る。
  const bool no_progress = hasNoProgress(now);

  // 前の車に詰まっているだけで自分は健全なら、復帰を始めずに待つ。
  if (healthy_wait) {
    stuck_start_time_.reset();
    pre_steer_valid_ = false;
    return;
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
      pre_steer_valid_ = false;
      {
        recovery::Pose p;
        if (currentPose(p) && corridor_.valid()) {
          const auto cars = carObstacles();
          // **本番の makePlan と同じ引数で立てること。**。
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
      // 逃げる向きの決め打ちはやめた。recovery::plan(, cars) が走行可能領域と。
    } else if (recovery_start_time_.has_value() &&
               (now - stuck_start_time_.value()).seconds() >= kStuckDurationSec) {
      // 既に復帰中なら開始し直さない ---。
      stuck_start_time_.reset();
      ++stall_suppressed_;
      if ((now - last_stall_suppress_log_).seconds() > 2.0) {
        last_stall_suppress_log_ = now;
        RCLCPP_WARN(get_logger(),
          "復帰中の停滞は開始し直さない(累計%d回) 経過=%.1fs 計画%d回 状況= %s",
          stall_suppressed_,
          (now - recovery_start_time_.value()).seconds(), replan_count_,
          situation_.c_str());
      }
    } else if ((now - stuck_start_time_.value()).seconds() >= kStuckDurationSec) {
      // 復帰直後は通常制御に発進の機会を与える。
      if (recovery_end_time_ &&
          (now - recovery_end_time_.value()).seconds() < kCooldownSec) {
        return;
      }
      // 発進から launch_no_recovery_sec 秒は復帰を始めない(スタート直後の誤起動対策)。
      if (launch_moving_since_.has_value() &&
          (now - launch_moving_since_.value()).seconds() < launch_no_recovery_sec_) {
        if ((now - last_launch_hold_log_).seconds() > 1.0) {
          last_launch_hold_log_ = now;
          RCLCPP_INFO(get_logger(), "発進から%.1fs: 復帰の開始を見送る(%.1fs まで)",
            (now - launch_moving_since_.value()).seconds(), launch_no_recovery_sec_);
        }
        return;
      }
      // **代入の前に**「既に復帰中だったか」を控える(順序が逆だと必ず真になる)
      const bool was_active_stall = recovery_start_time_.has_value();
      stuck_start_time_.reset();
      blocked_vehicle_start_time_.reset();
      recovery_start_time_ = now;
      beginRecovery(now, blocked_vehicle_ready, was_active_stall);
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
      double slat = 0.0, slo = 0.0, shi = 0.0, syaw_err = 0.0;
      const bool has_lat = lateralNow(slat, slo, shi);
      const bool has_yaw = headingErrorToTrack(syaw_err);
      RCLCPP_INFO(get_logger(),
                  "stuck detected: velocity=%.3f 位置=(%.1f,%.1f) yaw=%.1f "
                  "roll=%.1f pitch=%.1f deg 舵角=%.1f deg "
                  "横=%s 方位差=%s 原因=%s 前方車=%.2fm",
                  velocity, lx, ly, lyaw * 180.0 / M_PI, lroll * 180.0 / M_PI,
                  lpitch * 180.0 / M_PI,
                  command.lateral.steering_tire_angle * 180.0f / static_cast<float>(M_PI),
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
bool StuckRecoveryController::makePlan(const rclcpp::Time & now, int first_phase)
{
  ProfScope prof_scope(prof_, "makePlan");
  last_plan_attempt_ = now;
  const auto cars = carObstacles();
  // 探索器の呼び出しごとの時間を測る(呼び出し回数も内訳に出る)。
  auto timed_plan = [this](auto &&... args) {
    ProfScope s(prof_, "makePlan/recovery::plan");
    return recovery::plan(std::forward<decltype(args)>(args)...);
  };
  recovery::Pose p;
  if (!currentPose(p) || !corridor_.valid()) { return false; }
  escape_certificate_ = {};
  escape_best_wall_clear_ = -1e9;
  // やり直しのときは「今より改善する計画」だけを求める。
  const double gain = std::min(0.05 * static_cast<double>(replan_count_), 0.60);
  // 地図では動けるはずなのに動けなかったぶん、後退の下限を上げる。
  const double min_rev = std::min(kBlockedReverseMax,
                                  kBlockedReverseStep * unexplained_block_);
  // 後ろに他車がいるぶんだけ後退を制限する。
  const double keep = (unexplained_block_ >= 2 || replan_count_ >= 2)
                        ? kRearKeepTight : kRearKeep;
  const double rear = rearRoom() - keep;
  const double max_rev = std::clamp(rear, 0.0, 8.0);
  // バグA の修正 ---。
  const double clear_at_plan = recovery::wallClearanceAt(obstacles_, veh_, p);
  const double car_clear_at_plan = recovery::carClearanceAt(cars, veh_, p);
  auto certify_candidate = [&](const char * source) {
    ProfScope certify_scope(prof_, "makePlan/certify_candidate");
    std::vector<double> wall_clearances;
    std::vector<double> car_clearances;
    wall_clearances.reserve(plan_.path.size());
    car_clearances.reserve(plan_.path.size());
    for (const auto & pose : plan_.path) {
      wall_clearances.push_back(recovery::wallClearanceAt(obstacles_, veh_, pose));
      car_clearances.push_back(recovery::carClearanceAt(cars, veh_, pose));
    }
    std::vector<recovery::CarClearanceTrace> car_traces;
    car_traces.reserve(cars.size());
    for (const auto & car : cars) {
      recovery::CarClearanceTrace trace;
      trace.vehicle_id = car.id;
      trace.start_clearance = recovery::carClearanceAt(car, veh_, p);
      trace.path_clearance.reserve(plan_.path.size());
      for (const auto & pose : plan_.path) {
        trace.path_clearance.push_back(recovery::carClearanceAt(car, veh_, pose));
      }
      car_traces.push_back(std::move(trace));
    }
    escape_certificate_ = recovery::certifyEscapePlan(
      plan_, clear_at_plan, wall_clearances, car_clear_at_plan, car_clearances,
      {}, car_traces);
    if (!escape_certificate_.valid) {
      if (plan_.valid) {
        RCLCPP_WARN(get_logger(),
          "復帰 計画証明を却下 source=%s 理由=%s "
          "壁[開始%.2f 最小%.2f 終端%.2f] 車[開始%.2f 最小%.2f 終端%.2f] 点数%zu",
          source, escape_certificate_.reason.c_str(),
          escape_certificate_.start_wall_clearance,
          escape_certificate_.minimum_wall_clearance,
          escape_certificate_.terminal_wall_clearance,
          escape_certificate_.start_car_clearance,
          escape_certificate_.minimum_car_clearance,
          escape_certificate_.terminal_car_clearance, plan_.path.size());
      }
      plan_.valid = false;
      return false;
    }
    RCLCPP_INFO(get_logger(),
      "復帰 計画証明に合格 source=%s 壁[開始%.2f 最小%.2f 終端%.2f] "
      "車[開始%.2f 最小%.2f 終端%.2f] 点数%zu",
      source, escape_certificate_.start_wall_clearance,
      escape_certificate_.minimum_wall_clearance,
      escape_certificate_.terminal_wall_clearance,
      escape_certificate_.start_car_clearance,
      escape_certificate_.minimum_car_clearance,
      escape_certificate_.terminal_car_clearance, plan_.path.size());
    return true;
  };
  const double min_rev_try = (clear_at_plan < 0.0) ? embed_min_reverse_ : 0.75;
  if (max_rev < min_rev_try && first_phase < 0) {
    if (clear_at_plan >= 0.0) {
      first_phase = 0;
    } else {
      RCLCPP_WARN(get_logger(),
        "復帰 後退したいが下がれない(後方の余地 %.2fm)。壁へ %.2fm 食い込んでいるので "
        "前進へは切り替えない", max_rev, clear_at_plan);
    }
  }
  // 「食い込んでいるなら後退から始めろ」を**1か所で決める** ---。
  int wedge_first = 0;
  if (clear_at_plan < 0.0 && obstacles_.valid()) {
    recovery::Pose wp;
    if (currentPose(wp)) {
      double fc = 1e3, rc = 1e3;
      recovery::wallClearanceSplit(obstacles_, veh_, wp, fc, rc);
      // 後ろが当たっている(後ろのほうが余裕が小さい)なら前進で逃げる。
      wedge_first = (rc < fc) ? +1 : -1;
      if ((this->now() - last_wedge_side_log_).seconds() > 2.0) {
        last_wedge_side_log_ = this->now();
        RCLCPP_INFO(get_logger(),
          "復帰 食い込みの向き 前%.2fm 後%.2fm -> %sから逃げる",
          fc, rc, wedge_first > 0 ? "前進" : "後退");
      }
    } else {
      wedge_first = -1;   // 姿勢が取れないときは従来どおり後退
    }
  }
  const int eff_first = (first_phase != 0) ? first_phase : wedge_first;
  if (eff_first != first_phase && (this->now() - last_efffirst_log_).seconds() > 2.0) {
    last_efffirst_log_ = this->now();
    RCLCPP_INFO(get_logger(),
      "復帰 壁へ %.2fm 食い込んでいるので、すべての計画を後退から始めさせる",
      clear_at_plan);
  }
  first_phase = eff_first;
  // 目標指向の計画を先に試す ---。
  bool used_goal = false;
  bool plan_strict = false;
  bool plan_best_effort = false;
  if (goal_plan_enable_) {
    recovery::GoalPlanParams gp;
    gp.max_switch = goal_plan_max_switch_;
    gp.tail_len = goal_plan_tail_len_;
    gp.goal_ahead_min = goal_plan_ahead_min_;
    gp.goal_ahead_max = goal_plan_ahead_max_;
    // 呼び出し側の「後退から引き直す」をこの探索にも効かせる。
    gp.first_phase = first_phase;   // 上で実効値に揃えてある
    std::size_t gi = 0;
    const auto gplan = [&]() {
      ProfScope goal_scope(prof_, "makePlan/planToGoal");
      return recovery::planToGoal(corridor_, obstacles_, veh_, p, cars, gp, &gi);
    }();
    if (gplan.valid) {
      plan_ = gplan;
    }
    if (gplan.valid && certify_candidate("goal")) {
      plan_goal_idx_ = gi;
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
  plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                         first_phase, gain, kPlanMinEscape, min_rev, max_rev, cars,
                         kPlanWallClear, kPlanCarClear);
  plan_strict = plan_.valid;
  if (!plan_.valid) {
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           first_phase, gain, kPlanMinEscape, min_rev, max_rev, cars,
                           0.0, 0.0);
    plan_strict = false;
  }
  // この段が first_phase を明示的に 0 へ戻すので、。
  if (!plan_.valid && first_phase != 0) {
    // 縛ったせいで解が無いなら縛りを外す
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           0, gain, kPlanMinEscape, min_rev, max_rev, cars);
  }
  if (!plan_.valid && gain > 0.0) {
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           first_phase, 0.0, kPlanMinEscape, min_rev, max_rev, cars);
  }
  // 壁に挟まれていて 2.5m も動けないときだけ、脱出量の下限を緩める。
  if (!plan_.valid) {
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           0, 0.0, 1.0, min_rev, max_rev, cars);
  }
  if (!plan_.valid) {
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           0, 0.0, 0.0, min_rev, max_rev, cars);
  }
  // 前進が塞がっていると分かっているなら、前進だけの計画は受け取らない。
  if (plan_.valid && blocked_dir_ > 0 && !plan_.phases.empty() &&
      plan_.phases.front().forward)
  {
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           -1, 0.0, 0.0, 0.0, std::max(max_rev, 1.5), cars);
    plan_strict = false;
  }
  if (!plan_.valid) {
    // 最後のフォールバック。ここでも解が無いなら best_effort を立てて、。
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           eff_first, 0.0, 0.0, 0.0,
                           8.0, cars, 0.0, 0.0, true);
    plan_best_effort = plan_.valid;
  }
  // どの探索器の候補も、全区間を一体として安全に脱出できるか同じ規則で判定する。
  if (plan_.valid) {
    certify_candidate(plan_best_effort ? "legacy_best_effort" : "legacy");
  }
  // バグB の修正 ---。
  if (plan_.valid && !escape_certificate_.valid && !plan_best_effort &&
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

  // 前進区間が「当たる」見込みの計画は、後退だけに切り詰める ---。
  if (plan_.valid && !escape_certificate_.valid && !plan_best_effort &&
      plan_.min_wall_clear < 0.0 && plan_.phases.size() >= 2 &&
      !plan_.phases.front().forward)
  {
    // 切断前の全体計画が「壁内から改善して外へ出る途中」なのか、最後まで。
    const auto & reverse_phase = plan_.phases.front();
    const std::size_t reverse_end_idx = reverse_phase.path_end > 0
      ? std::min(reverse_phase.path_end, plan_.path.size()) - 1 : 0;
    const double reverse_end_clear = plan_.path.empty() ? clear_at_plan
      : recovery::wallClearanceAt(obstacles_, veh_, plan_.path[reverse_end_idx]);
    const auto forward_it = std::find_if(
      plan_.phases.begin(), plan_.phases.end(), [](const recovery::Phase & ph) {
        return ph.forward;
      });
    double forward_start_clear = 999.0;
    double forward_end_clear = 999.0;
    double first_nonnegative_m = -1.0;
    if (forward_it != plan_.phases.end() && !plan_.path.empty()) {
      const std::size_t begin = std::min(forward_it->path_begin, plan_.path.size() - 1);
      const std::size_t end = std::min(forward_it->path_end, plan_.path.size());
      forward_start_clear = recovery::wallClearanceAt(obstacles_, veh_, plan_.path[begin]);
      if (end > begin) {
        forward_end_clear = recovery::wallClearanceAt(obstacles_, veh_, plan_.path[end - 1]);
        double travelled = 0.0;
        for (std::size_t k = begin; k < end; ++k) {
          if (k > begin) {
            travelled += std::hypot(plan_.path[k].x - plan_.path[k - 1].x,
                                     plan_.path[k].y - plan_.path[k - 1].y);
          }
          if (recovery::wallClearanceAt(obstacles_, veh_, plan_.path[k]) >= 0.0) {
            first_nonnegative_m = travelled;
            break;
          }
        }
      }
    }
    RCLCPP_WARN(get_logger(),
      "復帰 計画切断前診断 現在余裕%.2f 後退終端%.2f(改善%+.2f) "
      "前進先頭%.2f 前進終端%.2f(改善%+.2f) 前進最小%.2f 壁外到達=%.2fm",
      clear_at_plan, reverse_end_clear, reverse_end_clear - clear_at_plan,
      forward_start_clear, forward_end_clear, forward_end_clear - clear_at_plan,
      plan_.min_wall_clear, first_nonnegative_m);
    RCLCPP_WARN(get_logger(),
      "復帰 前進区間が当たる見込み(余裕壁%.2f)。後退だけ実行して立て直す",
      plan_.min_wall_clear);
    plan_.phases.resize(1);
    if (plan_.phases.front().path_end <= plan_.path.size()) {
      plan_.path.resize(plan_.phases.front().path_end);
    }
    // 前進区間が無くなったので、前進の見込み余裕は「無し」として扱う。
    plan_.min_wall_clear = 1e9;
    plan_.min_car_clear = 1e9;
  }

  // 見込みの余裕が薄い前進区間は、その長さぶん走り切らせない ---。
  if (plan_.valid && !escape_certificate_.valid &&
      plan_.min_wall_clear < kThinForwardClear) {
    for (size_t i = 0; i < plan_.phases.size(); ++i) {
      if (!plan_.phases[i].forward || plan_.phases[i].length <= kThinForwardLen) { continue; }
      RCLCPP_INFO(get_logger(),
        "復帰 前進の見込み余裕が薄い(壁%.2f)。前進区間を %.1f -> %.1fm に切り詰める",
        plan_.min_wall_clear, plan_.phases[i].length, kThinForwardLen);
      if (!recovery::truncatePhasePath(plan_, i, kThinForwardLen)) {
        RCLCPP_ERROR(get_logger(),
          "復帰 前進区間の切り詰め失敗 phase=%zu begin=%zu end=%zu points=%zu",
          i, plan_.phases[i].path_begin, plan_.phases[i].path_end, plan_.path.size());
        plan_.valid = false;
      }
      break;
    }
  }

  // 却下で計画が消えたら、最後の受け皿(総当り)をやり直す ---。
  if (!plan_.valid) {
    // 食い込んでいるなら総当りにも「後退から始める」を要求する。
    plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                           eff_first, 0.0, 0.0, 0.0,
                           8.0, cars, 0.0, 0.0, true);
    // 後退必須の要求をフォールバックで解除しない ---。
    if (!plan_.valid && clear_at_plan < 0.0 && eff_first < 0) {
      // ---。
      bool took_forward = false;
      if (embed_forward_escape_) {
        auto fwd = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                                  +1, 0.0, 0.0, 0.0,
                                  8.0, cars, 0.0, 0.0, true);
        if (fwd.valid && fwd.min_wall_clear > clear_at_plan + kEmbedImprove) {
          plan_ = fwd;
          took_forward = true;
          RCLCPP_WARN(get_logger(),
            "復帰 後退の解が無いので前進で逃げる 食い込み%.2fm -> 見込み%.2fm",
            clear_at_plan, fwd.min_wall_clear);
        }
      }
      if (!took_forward) {
        RCLCPP_WARN(get_logger(),
          "復帰 壁へ %.2fm 食い込んでいて後退の解が無い。"
          "前進の計画も食い込みを浅くできない", clear_at_plan);
      }
    } else if (!plan_.valid && clear_at_plan < 0.0 && eff_first > 0) {
      // 後ろ当たり。前進で逃げる計画を、向きの縛りなしで探し直す。
      plan_ = timed_plan(corridor_, obstacles_, veh_, p, 4.0, 16.0,
                             +1, 0.0, 0.0, 0.0,
                             8.0, cars, 0.0, 0.0, true);
      RCLCPP_WARN(get_logger(),
        "復帰 壁へ %.2fm 食い込んでいるが**後端**なので前進で逃げる(計画%s)",
        clear_at_plan, plan_.valid ? "あり" : "なし");
    }
    plan_best_effort = plan_.valid;
    if (plan_.valid) {
      RCLCPP_WARN(get_logger(),
        "復帰 却下で計画が消えたので総当りへ回した(見込み余裕 壁%.2f 車%.2f)",
        plan_.min_wall_clear, plan_.min_car_clear);
      certify_candidate("legacy_retry_best_effort");
    }
  }

  }
  // 将来探索分岐が増えても、未証明の plan_ を実行側へ渡さない。
  if (plan_.valid && !escape_certificate_.valid) {
    certify_candidate(used_goal ? "goal_final" : "legacy_final");
  }
  // 採用した計画が前進区間で見込んだ余裕を、追従中の中断閾値に使えるよう保存する。
  plan_min_wall_clear_ = plan_.valid ? plan_.min_wall_clear : 1e9;
  plan_min_car_clear_ = plan_.valid ? plan_.min_car_clear : 1e9;
  phase_idx_ = 0;
  phase_travelled_ = 0.0;
  traj_from_ = 0;   // 経路の切り詰め位置も新しい計画に合わせて戻す
  // これは復帰episodeではなく「いまの計画を追っても動かなかった」状態。
  traj_still_ = false;
  phase_start_wall_clear_ = recovery::wallClearanceAt(obstacles_, veh_, p);
  escape_best_wall_clear_ = phase_start_wall_clear_;
  phase_start_car_clear_ = recovery::carClearanceAt(cars, veh_, p);
  escape_best_car_clearance_.clear();
  for (const auto & contract : escape_certificate_.certified_cars) {
    escape_best_car_clearance_[contract.vehicle_id] = contract.start_clearance;
  }
  braking_ = false;
  brake_advance_phase_ = true;
  brake_replan_after_ = false;
  brake_timeout_logged_ = false;
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
    escape_certificate_ = {};
    RCLCPP_WARN(get_logger(), "復帰 経路を計算できない");
    return false;
  }
  // A plan is geometrically certified from `p`.  If the vehicle is still。
  if (!plan_.phases.empty()) {
    const bool first_forward = plan_.phases.front().forward;
    const bool moving_against_first =
      (first_forward && latest_velocity_ < -kBrakeDoneSpeed) ||
      (!first_forward && latest_velocity_ > kBrakeDoneSpeed);
    if (moving_against_first) {
      braking_ = true;
      brake_since_ = now;
      brake_steer_ = sent_steer_phys_;
      brake_was_forward_ = latest_velocity_ >= 0.0;
      brake_advance_phase_ = false;
      brake_replan_after_ = true;
      RCLCPP_WARN(get_logger(),
        "復帰 新計画の先頭%sに対し実速度%+.2fm/sは逆向き。"
        "計画の距離に数えず、停止後の姿勢から再計画する",
        first_forward ? "前進" : "後退", latest_velocity_);
    }
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
  const rclcpp::Time & now, bool forward_blocked, bool was_active)
{
  ProfScope prof_scope(prof_, "beginRecovery");
  // 開始回数を数えるための専用ログ ---。
  traceDump(now, "recovery_begin");
  ++recovery_begin_seq_;
  // 「既に復帰中に呼ばれたか」を必ず残す ---。
  RCLCPP_WARN(get_logger(),
              "復帰開始 #%d 前進が塞がれている=%d 既に復帰中=%d 前回からの経過=%.1fs",
              recovery_begin_seq_, forward_blocked ? 1 : 0, was_active ? 1 : 0,
              (last_begin_time_.nanoseconds() > 0)
                ? (now - last_begin_time_).seconds() : -1.0);
  replan_count_ = 0;
  blocked_dir_ = forward_blocked ? +1 : 0;
  desperate_ = false;
  // 簡易復帰の向き・起点・目標点が前回の復帰から。
  if (!was_active) {
    simple_tgt_valid_ = false;       // 次の周期に現在地から目標点と起点を取り直す
    simple_dir_forward_ = true;      // 両方動けるなら前進を優先
    simple_dir_since_ = now;
    simple_dir_stable_ = 0;
    simple_lost_valid_ = false;
    simple_reversing_ = false;
    simple_dir_fresh_ = true;
  }
  // 「地図では余裕があるのに動けない」の回数は復帰1回ごとに数え直す。
  {
    recovery::Pose q;
    const double dt = (last_begin_time_.nanoseconds() > 0)
                        ? (now - last_begin_time_).seconds() : 1e9;
    const bool same_place = currentPose(q) && last_begin_valid_ &&
                            std::hypot(q.x - last_begin_x_, q.y - last_begin_y_) < 1.5;
    if (!(same_place && dt < 3.0)) {
      unexplained_block_ = std::min(unexplained_block_, 1);
    }
    if (currentPose(q)) {
      last_begin_x_ = q.x; last_begin_y_ = q.y; last_begin_valid_ = true;
    }
    last_begin_time_ = now;
  }
  rear_waiting_ = false;
  rear_relaxed_ = false;
  traj_still_ = false;
  recovery::Pose p;
  if (currentPose(p)) {
    recovery_start_x_ = p.x; recovery_start_y_ = p.y;
    recovery_start_clear_ = obstacles_.valid()
        ? recovery::wallClearanceAt(obstacles_, veh_, p) : 9.99;
  }
  // --- 既に壁へ食い込んでいるなら、前進を試さず後退から始める ---。
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
bool StuckRecoveryController::tryHandBack(
  const recovery::Pose & p, const rclcpp::Time & now, double total)
{
  double lat = 0.0, lo = 0.0, hi = 0.0, yaw_err = 0.0;
  // 壁から十分離れていること。境界からわずかに内側という程度で返すと、。
  const bool inside = lateralNow(lat, lo, hi) &&
                      lat > lo + kHandbackMargin && lat < hi - kHandbackMargin;
  const bool aligned = !headingErrorToTrack(yaw_err) || yaw_err < kAlignYaw;
  // 通常制御が狙う先へ実際に走り出せること。
  const double look = (total > kHandbackRelaxSec)
                        ? kHandbackAhead * 0.5 : kHandbackAhead;
  const bool can_go = handbackPathClear(look);
  // 復帰開始地点から実際に離れていること。後退しただけで。
  const double gone = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
  if (inside && aligned && can_go && gone > kEscapeDist &&
      std::abs(latest_velocity_) > kMovingSpeedThreshold) {
    RCLCPP_INFO(get_logger(),
                "復帰 完了 %.1fs 計画%d回 方位差%.0fdeg 移動%.1fm 状況= %s",
                total, replan_count_, yaw_err * 180.0 / M_PI, gone,
                situation_.c_str());
    finishRecovery(now, "完了");
    return true;
  }
  return false;
}

// 最終手段。計画で抜けられる姿勢ではないときに、強引に動かす。
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
  // 最終手段そのものに上限を付ける。「動けたら終わる」しか無かったので、。
  if (t > desperate_max_sec_) {
    RCLCPP_WARN(get_logger(),
      "復帰 最終手段を %.0f秒 続けても %.2fm しか動けない。通常制御へ返す",
      t, moved);
    desperate_ = false;
    finishRecovery(now, "最終手段の打切");
    return;
  }
  const int swing = static_cast<int>(t / kDesperateSwingSec);
  if (swing != desperate_swing_) {
    desperate_swing_ = swing;
    RCLCPP_WARN(get_logger(), "復帰 強引な脱出 %d回目 (最終手段)", swing + 1);
  }
  // 壁へ食い込んでいる間は強引な脱出を使わない ---。
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
      // 余裕が改善しないまま後退を続けても意味がない。基準から。
      if (desperate_ref_clear_ < -1e8 || now_clear > desperate_ref_clear_ + 0.05) {
        desperate_ref_clear_ = now_clear;
        desperate_ref_at_ = now;
      }
      const bool improving =
        (now - desperate_ref_at_).seconds() < desperate_stall_sec_;
      if (rear_ok && improving) {
        publishGear(GearCommand::REVERSE);
        // 加速度が 0 では**ペダルを踏んでいない**ので何も起きない。
        cmd_src_ = "最終手段(後退)";
        publishCommand(static_cast<float>(-wall_ban_reverse_speed_),
                       static_cast<float>(recovery_accel_), best_steer);
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
std::optional<bool> StuckRecoveryController::replanOrEscalate(
  const recovery::Pose & p, const rclcpp::Time & now)
{
  // A failed planner used to run twice per 100Hz control callback, consuming。
  if (!plan_.valid && (now - last_plan_attempt_).seconds() < no_plan_retry_sec_) {
    cmd_src_ = "計画再探索待ち";
    publishSafeStop(sent_steer_phys_);
    return true;
  }
  // 区間をすべて走り切り、計画を採用した地点から実際に離れているなら、。
  const bool plan_ran_through = plan_.valid && phase_idx_ >= plan_.phases.size();
  const double moved_on_plan = std::hypot(p.x - plan_x_, p.y - plan_y_);
  // 「進めた」には改善も要る ---。
  const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
  const bool situation_better =
    clear_now >= 0.0 || clear_now > plan_wall_clear_ + kPlanImproveClear;
  const bool plan_made_progress =
    plan_ran_through && moved_on_plan >= kPlanProgressDist && situation_better;

  // 計画を走り切ったのに戻れていない。今の姿勢から引き直す。
  if (replan_count_ >= kReplanMax && !plan_made_progress) {
    const double moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
    if (moved < kDesperateFreeDist) {
      // 計画を出しても1歩も動けていない。走行可能領域では説明のつかない。
      if (desperate_enable_) {
        RCLCPP_WARN(get_logger(),
                    "復帰 計画%d回で %.2fm しか動けない。強引な脱出に切り替える",
                    replan_count_, moved);
        desperate_ = true;
        desperate_since_ = now;
        desperate_swing_ = -1;
        desperate_ref_clear_ = -1e9;
        return true;
      }
      if ((now - last_desperate_log_).seconds() > 3.0) {
        last_desperate_log_ = now;
        RCLCPP_WARN(get_logger(),
                    "復帰 計画%d回で %.2fm。最終手段は無効なので、"
                    "停止して証明可能な経路を再探索する",
                    replan_count_, moved);
      }
      // ここでフルブレーキを出し続けると、。
      if (deadlock_release_enable_ && std::abs(latest_velocity_) < kBrakeDoneSpeed) {
        if (!deadlock_since_) { deadlock_since_ = now; }
        if ((now - deadlock_since_.value()).seconds() >= deadlock_release_sec_) {
          const double rear = rearRoom();
          recovery::Pose behind;
          bool wall_ok = true;
          if (currentPose(behind) && obstacles_.valid()) {
            // 0.5m 後ろの姿勢で壁余裕を見る。負なら後ろは壁。
            behind.x -= 0.5 * std::cos(behind.yaw);
            behind.y -= 0.5 * std::sin(behind.yaw);
            wall_ok = recovery::wallClearanceAt(obstacles_, veh_, behind) >
                      recovery::wallClearanceAt(obstacles_, veh_, p) - 0.05;
          }
          const bool rear_ok = rear >= deadlock_release_rear_m_;
          if (rear_ok && wall_ok) {
            // 舵は直進寄りに戻す。切ったまま下がると壁沿いに擦る。
            publishGear(GearCommand::REVERSE);
            publishCommand(-1.0f, static_cast<float>(deadlock_release_accel_), 0.0f);
            cmd_src_ = "手詰まり解除(後退)";
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "復帰 手詰まり %.1f秒。後方 %.1fm 空き・壁も可なので制動をやめて下がる"
              "(移動%.2fm 壁%.2fm)",
              (now - deadlock_since_.value()).seconds(), rear, moved, plan_wall_clear_);
            return true;
          }
          // 単純な後退が使えない。**試して測る**反復探索へ回す。
          if (runEscapeProbe(p, now)) { return true; }
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "復帰 手詰まり %.1f秒だが後退しない(後方%.1fm 要%.1fm 壁の向き=%s)。安全停止を続ける",
            (now - deadlock_since_.value()).seconds(), rear, deadlock_release_rear_m_,
            wall_ok ? "可" : "不可");
        }
      }
      publishSafeStop(sent_steer_phys_);
      makePlan(now, blocked_dir_ != 0 ? -blocked_dir_ : -1);
      return true;
    }
    RCLCPP_WARN(get_logger(), "復帰 計画%d回でも戻れない。通常制御へ返す 状況= %s",
                replan_count_, situation_.c_str());
    finishRecovery(now, "計画しても戻れない");
    return false;
  }
  // 走り切った計画は「やり直し」に数えない ---。
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
    // --- 経路が1本も出せない = 前後とも塞がれている。
    if (desperate_enable_) {
      RCLCPP_WARN(get_logger(),
                  "復帰 経路が出せない(%d回目)。強引な脱出に切り替える 状況= %s",
                  replan_count_, situation_.c_str());
      desperate_ = true;
      desperate_since_ = now;
      desperate_swing_ = -1;
      desperate_ref_clear_ = -1e9;
      return true;
    }
    if ((now - last_desperate_log_).seconds() > 3.0) {
      last_desperate_log_ = now;
      RCLCPP_WARN(get_logger(),
                  "復帰 経路が出せない(%d回目)。最終手段は無効なので安全停止 "
                  "状況= %s", replan_count_, situation_.c_str());
    }
    publishSafeStop(sent_steer_phys_);
    return true;
  }
  return std::nullopt;
}

// 前進中に壁へ近づいたら、動けなくなる前に引き直す。引き直したら true。
bool StuckRecoveryController::replanIfWallNear(
  const recovery::Pose & p, const recovery::Phase & ph, const rclcpp::Time & now)
{
  // --- 前進中に壁へ近づいたら、動けなくなる前に引き直す。
  if (ph.forward && !desperate_ &&
      phase_travelled_ < kFwdNoMoveDist &&
      (now - phase_start_).seconds() > kFwdNoMoveSec)
  {
    // カウンタを 0 に戻してはいけない。
    ++replan_count_;
    // 地図では空いているのに動かないなら後退を伸ばす ---。
    {
      const double clear = recovery::wallClearanceAt(obstacles_, veh_, p);
      if (clear > kExplainedClearance) { ++unexplained_block_; }
      RCLCPP_WARN(get_logger(),
        "復帰 前進を指令して %.1fs 動かない(走行 %.3fm 壁まで %.2f)。"
        "後退から引き直す(%d回目 後退の下限%.1fm)",
        (now - phase_start_).seconds(), phase_travelled_, clear, replan_count_,
        std::min(kBlockedReverseMax, kBlockedReverseStep * unexplained_block_));
    }
    makePlan(now, -1);
    return true;
  }

  // 区間の切り出しで角が壁を舐めるのを止める ---。
  if (turn_in_abort_ && !desperate_ && replan_count_ < kReplanMax &&
      phase_travelled_ < turn_in_abort_travel_)
  {
    const double clear_turn = recovery::wallClearanceAt(obstacles_, veh_, p);
    if (clear_turn < phase_start_wall_clear_ - turn_in_abort_worsen_) {
      ++replan_count_;
      blocked_dir_ = ph.forward ? 1 : -1;
      RCLCPP_WARN(get_logger(),
        "復帰 切り出しで壁が %.2f -> %.2fm へ悪化(走行%.2fm 舵狙い%.0fdeg)。"
        "回頭で角が当たっている。引き直す(%d回目)",
        phase_start_wall_clear_, clear_turn, phase_travelled_,
        ph.steer * 180.0 / M_PI, replan_count_);
      makePlan(now, ph.forward ? -1 : 1);
      return true;
    }
  }
  if (ph.forward && !desperate_ && replan_count_ < kReplanMax) {
  const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
  // 計画が見込んだ余裕より下げない。計画が「0.05m しかない所を通す」と決めたなら、。
  const double thr = std::max(kFwdAbortWallFloor,
      std::min(kFwdAbortClearance, plan_min_wall_clear_ - kFwdAbortWorsen));
  // 壁も同じ。狭い所ではもともと余裕が小さいので、。
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
  // --- 前進中に他車へ近づいたら、ぶつかる前に引き直す。
  if (ph.forward && !desperate_ && replan_count_ < kReplanMax) {
  // **符号付きの余裕**を使う。`carViolation` は 0 で初期化した「重なりの深さ」で。
  const double cc = recovery::carClearanceAt(carObstacles(), veh_, p);
  const double thr = std::max(kFwdAbortCarFloor,
      std::min(kFwdAbortCarDist, plan_min_car_clear_ - kFwdAbortWorsen));
  // 近いだけでは降りない。**区間開始より悪化している**ときだけ降りる。
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
bool StuckRecoveryController::runBraking(const rclcpp::Time & now)
{
  if (braking_) {
    const double bt = (now - brake_since_).seconds();
    if (std::abs(latest_velocity_) < kBrakeDoneSpeed) {
      braking_ = false;
      const bool replan = brake_replan_after_;
      brake_replan_after_ = false;
      if (brake_advance_phase_) { ++phase_idx_; }
      traj_from_ = phase_idx_ < plan_.phases.size()
        ? plan_.phases[phase_idx_].path_begin : plan_.path.size();
      phase_travelled_ = 0.0;
      last_valid_ = false;
      phase_start_ = now;
      stall_since_ = now;
      brake_timeout_logged_ = false;
      if (replan) {
        RCLCPP_INFO(get_logger(), "復帰 方向転換の停止完了。現在姿勢から再計画する");
        makePlan(now, 0);
      }
      return true;
    }
    // The former `bt > 1.5s` branch advanced while still moving at +3.14m/s.。
    if (bt > kBrakeMaxSec && !brake_timeout_logged_) {
      brake_timeout_logged_ = true;
      RCLCPP_WARN(get_logger(),
        "復帰 制動開始から%.1fs後も実速度%+.2fm/s。"
        "未停止のまま次区間へは進まない", bt, latest_velocity_);
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
std::optional<bool> StuckRecoveryController::waitForRearRoom(
  const recovery::Phase & ph, const rclcpp::Time & now)
{
  // 後ろに車がいて、この区間で下がる距離が確保できないなら待つ。
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
    if (replan_count_ >= kReplanMax &&
        (now - rear_wait_since_).seconds() > kStallSec)
    {
      // 無効化されているときは「切り替える」と書いてはいけない。
      if ((now - last_desperate_log_).seconds() > 3.0) {
        last_desperate_log_ = now;
        RCLCPP_WARN(get_logger(),
                    "復帰 やり直し%d回、後方待ち %.1fs。%s",
                    replan_count_, (now - rear_wait_since_).seconds(),
                    desperate_enable_ ? "強引な脱出に切り替える"
                                      : "最終手段は無効なので打つ手が無い");
      }
      if (desperate_enable_) {
        desperate_ = true;
        desperate_since_ = now;
        desperate_swing_ = -1;
        desperate_ref_clear_ = -1e9;
        return true;
      }
      // Disabled means this mode cannot claim the cycle.  Fall through to the。
    }
    publishGear(GearCommand::DRIVE);
    publishCommand(0.0, 0.0);
    // 待ち始めてすぐの間だけ停滞から除外する。
    if (!rear_relaxed_) { stall_since_ = now; }
    if ((now - last_trace_).seconds() > 1.0) {
      last_trace_ = now;
      RCLCPP_INFO(get_logger(),
                  "復帰 後方に他車(%.1fm)。%.1fm 下がれるまで待つ(%.1fs)",
                  rearRoom(), ph.length, (now - rear_wait_since_).seconds());
    }
    return true;
  }

  // 待機を抜けたので待ちタイマーは止める。緩和(rear_relaxed_)は。
  rear_waiting_ = false;
  return std::nullopt;
}

// 動けていないなら計画を引き直す。同じ指令を出し続けても出られない。
std::optional<bool> StuckRecoveryController::handleStall(
  const recovery::Pose & p, const recovery::Phase & ph, const rclcpp::Time & now)
{
  const double since_phase = (now - phase_start_).seconds();
  const double since_gear = (now - gear_changed_).seconds();
  // 方向転換の直後は猶予を短くする。
  const double warm_need = reverse_following_ ? std::min(phase_min_sec_, 0.20)
                                              : phase_min_sec_;
  const bool warming_up = since_phase < warm_need || since_gear < kActuatorWaitMax;
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
        if (desperate_enable_) {
          RCLCPP_WARN(get_logger(),
                      "復帰 やり直し%d回で %.2fm しか動けない。強引な脱出に切り替える",
                      replan_count_, moved);
          desperate_ = true;
          desperate_since_ = now;
          desperate_swing_ = -1;
          return true;
        }
        // The disabled fallback used to return `true` here on every callback.。
        RCLCPP_WARN(get_logger(),
                    "復帰 やり直し%d回で %.2fm。最終手段は無効なので、"
                    "証明済み%s経路の送出を継続する",
                    replan_count_, moved, ph.forward ? "前進" : "後退");
        stall_since_ = now;
      }
    }
    if (replan_count_ < kReplanMax) {
      ++replan_count_;
      // 動けなかった向きを覚え、次は逆から始めさせる。
      blocked_dir_ = ph.forward ? 1 : -1;
      const int want = ph.forward ? -1 : 1;
      // 地図の上では余裕があるのに動けないなら、自己位置推定が実際と。
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

// ===================================================================。
bool StuckRecoveryController::runRecoverySimple(const rclcpp::Time & now)
{
  ProfScope prof_scope(prof_, "runRecoverySimple");
  recovery::Pose p;
  if (!currentPose(p) || line_x_.size() < 3) { return false; }
  const auto cars = carObstacles();

  // --- 目標点: レースライン上の最近傍から前/後ろへ一定距離 ---
  std::size_t ni = 0; double bd = 1e18;
  for (std::size_t i = 0; i < line_x_.size(); ++i) {
    const double d = (line_x_[i]-p.x)*(line_x_[i]-p.x) + (line_y_[i]-p.y)*(line_y_[i]-p.y);
    if (d < bd) { bd = d; ni = i; }
  }
  const std::size_t n = line_x_.size();
  auto point_at = [&](double along) {
    double acc = 0.0; std::size_t i = ni;
    const int step = (along >= 0.0) ? 1 : -1;
    const double want = std::abs(along);
    for (std::size_t k = 0; k < n; ++k) {
      const std::size_t j = (i + n + step) % n;
      acc += std::hypot(line_x_[j]-line_x_[i], line_y_[j]-line_y_[i]);
      i = j;
      if (acc >= want) { break; }
    }
    return std::pair<double,double>(line_x_[i], line_y_[i]);
  };
  if (!simple_tgt_valid_) {
    simple_tgt_f_ = point_at(+simple_ahead_m_);
    simple_tgt_b_ = point_at(-simple_back_m_);
    simple_tgt_valid_ = true;
    simple_dir_from_ = {p.x, p.y};
    RCLCPP_INFO(get_logger(),
      "復帰(簡易) 目標点を固定 前(%.1f,%.1f) 後(%.1f,%.1f)",
      simple_tgt_f_.first, simple_tgt_f_.second,
      simple_tgt_b_.first, simple_tgt_b_.second);
  }
  const auto tgt_f = simple_tgt_f_;
  const auto tgt_b = simple_tgt_b_;

  // --- 候補の評価。到達点が目標にどれだけ近いか + 途中で当たらないか ---。
  const double start_wall = recovery::wallClearanceAt(obstacles_, veh_, p);
  const double start_car =
    cars.empty() ? 9.9 : recovery::carClearanceAt(cars, veh_, p);
  const double wall_floor =
    std::min(simple_wall_keep_, start_wall - simple_worsen_tol_);
  const double car_tol = (start_car < 0.0) ? simple_car_worsen_tol_
                                           : simple_worsen_tol_;
  const double car_floor = std::min(simple_car_keep_, start_car - car_tol);
  auto try_dir = [&](bool forward, const std::pair<double,double> & tgt,
                     double & best_steer, double & best_dist,
                     double probe_m, double steer_frac, double embed_improve) {
    bool found = false; best_steer = 0.0; best_dist = 1e18;
    const int N = simple_steer_steps_;
    for (int k = -N; k <= N; ++k) {
      // 舵は余裕を持たせる。全舵で経路を引くと追従の。
      const double st_max = kMaxSteerRad * steer_frac;
      const double st = st_max * static_cast<double>(k) / static_cast<double>(N);
      recovery::Pose q = p;
      bool hit = false; double travelled = 0.0;
      const double ds = 0.25;
      while (travelled < probe_m) {
        q = recovery::advanceBicycleExact(q, forward ? ds : -ds, st, veh_.wheel_base);
        travelled += ds;
        if (recovery::wallClearanceAt(obstacles_, veh_, q) < wall_floor) { hit = true; break; }
        if (!cars.empty() &&
            recovery::carClearanceAt(cars, veh_, q) < car_floor) { hit = true; break; }
      }
      if (hit) { continue; }
      const double end_wall = recovery::wallClearanceAt(obstacles_, veh_, q);
      if (start_wall < 0.0) {
        if (end_wall < start_wall + embed_improve) { continue; }
        // 食い込みが最も浅くなる候補を選ぶ(小さいほど良い指標に揃える)
        const double score = -end_wall;
        if (score < best_dist) { best_dist = score; best_steer = st; found = true; }
        continue;
      }
      const double d = std::hypot(q.x - tgt.first, q.y - tgt.second);
      if (d < best_dist) { best_dist = d; best_steer = st; found = true; }
    }
    return found;
  };

  double steer_f = 0.0, dist_f = 1e18, steer_b = 0.0, dist_b = 1e18;
  bool ok_f = try_dir(true,  tgt_f, steer_f, dist_f,
                      simple_probe_m_, simple_steer_frac_, simple_embed_improve_);
  bool ok_b = try_dir(false, tgt_b, steer_b, dist_b,
                      simple_probe_m_, simple_steer_frac_, simple_embed_improve_);
  // 壁に 0.22m 食い込み、方位差 61度・コリドア外の姿勢で、。
  if (!ok_f && !ok_b) {
    const double short_m = std::min(simple_probe_m_, simple_short_probe_m_);
    const double short_improve =
      simple_embed_improve_ * short_m / std::max(simple_probe_m_, 1e-3);
    ok_f = try_dir(true,  tgt_f, steer_f, dist_f, short_m, 1.0, short_improve);
    ok_b = try_dir(false, tgt_b, steer_b, dist_b, short_m, 1.0, short_improve);
    if ((ok_f || ok_b) && (now - last_short_probe_log_).seconds() > 1.0) {
      last_short_probe_log_ = now;
      RCLCPP_INFO(get_logger(),
        "復帰(簡易) %.1fm では候補なし。%.1fm・全舵で探し直し 前進=%d 後退=%d 壁まで%.2fm",
        simple_probe_m_, short_m, ok_f ? 1 : 0, ok_b ? 1 : 0, start_wall);
    }
  }

  // 後退で目標点に着いたら前進へ。到達は半径 simple_reach_m_ 以内。

  auto set_dir = [&](bool fwd, const char * why) {
    simple_dir_forward_ = fwd;
    simple_dir_since_ = now;
    simple_dir_stable_ = 0;
    simple_dir_from_ = {p.x, p.y};
    simple_tgt_valid_ = false;      // 次の周期に現在地から目標点を取り直す
    simple_lost_valid_ = false;
    RCLCPP_INFO(get_logger(), "復帰(簡易) 向きを切り替え -> %s (理由=%s)",
                fwd ? "前進" : "後退", why);
  };

  // 「壁に当たっているのに前進を選択するのは間違い」。
  if (simple_dir_fresh_) {
    simple_dir_fresh_ = false;
    if (start_wall < 0.0 && embed_prefer_reverse_ && ok_b && simple_dir_forward_) {
      RCLCPP_INFO(get_logger(),
        "復帰(簡易) 壁へ%.2fm 食い込み -> 後退を優先する", start_wall);
      set_dir(false, "開始時に壁へ食い込み");
    }
  }
  bool switch_dir = false;
  const char * why_switch = "";
  const bool cur_ok = simple_dir_forward_ ? ok_f : ok_b;
  // 【2026-09-18】壁に押し付いて動けない状態を、前進・後退のどちらでも見る。
  // 進もうとしている側(前進なら前端、後退なら後端)の壁までの距離で判定する。
  bool press_flip = false;
  {
    double fc = 1e3, rc = 1e3;
    if (obstacles_.valid()) {
      recovery::wallClearanceSplit(obstacles_, veh_, p, fc, rc);
    }
    const double clear_dir = simple_dir_forward_ ? fc : rc;
    const bool moving_now = std::abs(latest_velocity_) > 0.15;
    if (moving_now || clear_dir >= press_wall_m_) {
      simple_press_since_ = -1.0;
    } else {
      if (simple_press_since_ < 0.0) { simple_press_since_ = now.seconds(); }
      if (now.seconds() - simple_press_since_ >= press_flip_sec_) {
        press_flip = true;
      }
    }
    if (press_flip) {
      const bool other_ok_press = simple_dir_forward_ ? ok_b : ok_f;
      RCLCPP_WARN(get_logger(),
        "復帰(簡易) %sで壁に押し付いて %.1fs 動けない(その側の壁まで%.2fm 実速度%.2f)。"
        "%s",
        simple_dir_forward_ ? "前進" : "後退",
        now.seconds() - simple_press_since_, clear_dir, latest_velocity_,
        other_ok_press ? "反対へ切り替える" : "反対にも候補が無いので続行");
      simple_press_since_ = -1.0;
      if (other_ok_press) {
        switch_dir = true;
        why_switch = simple_dir_forward_ ? "前進で壁に押し付いた" : "後退で壁に押し付いた";
      }
    }
  }
  // 経路の喪失は連続 simple_lost_confirm_sec_ 続いたときだけ確定する。
  if (cur_ok) {
    simple_lost_valid_ = false;
  } else if (!simple_lost_valid_) {
    simple_lost_valid_ = true;
    simple_lost_since_ = now;
  }
  const double lost_sec =
    simple_lost_valid_ ? (now - simple_lost_since_).seconds() : 0.0;
  const bool lost_confirmed = !cur_ok && lost_sec >= simple_lost_confirm_sec_;
  // (a) 到達したか。前進の目標点/後退の目標点それぞれで見る。
  const double d_cur_tgt = simple_dir_forward_
    ? std::hypot(p.x - tgt_f.first, p.y - tgt_f.second)
    : std::hypot(p.x - tgt_b.first, p.y - tgt_b.second);
  // 固定した目標点の周りで発振して着かない場合の保険。
  const double moved_since_dir =
    std::hypot(p.x - simple_dir_from_.first, p.y - simple_dir_from_.second);
  const bool over_run = moved_since_dir > simple_dir_max_m_;
  const bool reached = (d_cur_tgt < simple_reach_m_) || over_run;
  if (over_run && (now - last_simple_log_).seconds() > 1.0) {
    last_simple_log_ = now;
    RCLCPP_INFO(get_logger(),
      "復帰(簡易) %s で %.1fm 動いた(上限%.1f)。到達扱いにする(目標点まで%.2fm)",
      simple_dir_forward_ ? "前進" : "後退", moved_since_dir,
      simple_dir_max_m_, d_cur_tgt);
  }
  if (press_flip) {
    // 押し付きの判断を優先する(下のチェーンは上書きしない)
  } else if (reached) {
    // 目標点に着いたのに切り替え先の候補が無いと、その場で。
    const bool other_ok2 = simple_dir_forward_ ? ok_b : ok_f;
    if (!other_ok2 && start_wall < 0.0) {
      // 壁から -0.58 -> -0.22m まで出たところで。
      set_dir(simple_dir_forward_, "到達扱いだが壁に食い込んだまま");
    } else if (!other_ok2) {
      RCLCPP_INFO(get_logger(),
        "復帰(簡易) 目標点に到達(%.2fm)し、次の候補が無い。通常制御へ返す",
        d_cur_tgt);
      finishRecovery(now, "簡易復帰の目標点に到達");
      return false;
    }
    if (!simple_dir_forward_) { switch_dir = true; why_switch = "目標点に到達"; }
  } else if (lost_confirmed) {
    switch_dir = true; why_switch = "経路を辿れない";
  } else if (cur_ok) {
    const double dir_elapsed = (now - simple_dir_since_).seconds();
    const bool other_ok_now = simple_dir_forward_ ? ok_b : ok_f;
    const bool stalled =
      dir_elapsed > std::max(simple_dir_hold_sec_, simple_stall_sec_) &&
      moved_since_dir < simple_stall_min_m_;
    if (stalled && other_ok_now) {
      switch_dir = true; why_switch = "動けないため反対へ";
    } else if (stalled && (now - last_stall_log_).seconds() > 1.0) {
      last_stall_log_ = now;
      RCLCPP_INFO(get_logger(),
        "復帰(簡易) %s で%.1fs 動けず(移動%.2fm)。反対側も候補なし、続行",
        simple_dir_forward_ ? "前進" : "後退", dir_elapsed, moved_since_dir);
    }
  }
  if (switch_dir) {
    // 切り替え先が成立するときだけ実際に切り替える。
    const bool other_ok = simple_dir_forward_ ? ok_b : ok_f;
    if (other_ok) {
      set_dir(!simple_dir_forward_, why_switch);
    }
  }
  const bool forward = simple_dir_forward_;
  double steer = 0.0;
  if (forward && ok_f) { steer = steer_f; simple_reversing_ = false; }
  else if (!forward && ok_b) { steer = steer_b; simple_reversing_ = true; }
  else if (ok_f || ok_b) {
    // いまの向きの経路が見つからないが、まだ喪失を確定していない。
    cmd_src_ = "復帰(簡易) 経路の喪失を確認中";
    publishCommand(0.0f, kBrakeAccel, sent_steer_phys_);
    if ((now - last_simple_log_).seconds() > 1.0) {
      last_simple_log_ = now;
      RCLCPP_INFO(get_logger(),
        "復帰(簡易) %s の経路が%.2fs 見つからない。確定(%.2fs)まで停止して待つ",
        forward ? "前進" : "後退", lost_sec, simple_lost_confirm_sec_);
    }
    return true;
  }
  else {
    // 経路が見つからない。ここだけ 0km/h。
    cmd_src_ = "復帰(簡易) 経路なし";
    publishSafeStop(sent_steer_phys_);
    if ((now - last_simple_log_).seconds() > 1.0) {
      last_simple_log_ = now;
      RCLCPP_WARN(get_logger(),
        "復帰(簡易) 前進も後退も当たる。停止 壁まで%.2fm 車まで%.2fm "
        "(許容 壁%.2f 車%.2f)",
        start_wall, start_car, wall_floor, car_floor);
    }
    return true;
  }

  publishGear(forward ? GearCommand::DRIVE : GearCommand::REVERSE);
  const float st_cmd = static_cast<float>(std::clamp(steer, -kMaxSteerRad, kMaxSteerRad));
  cmd_src_ = forward ? "復帰(簡易)前進" : "復帰(簡易)後退";
  // ギアが実際に切り替わるまで加速指令を出さない。
  const bool gear_ready = !gear_report_seen_ || gear_report_ == gear_now_;
  if (!gear_ready) {
    const double waited = (now - gear_changed_).seconds();
    if (waited < kActuatorWaitMax) {
      cmd_src_ = forward ? "復帰(簡易)ギア待ち(前進)" : "復帰(簡易)ギア待ち(後退)";
      // 速度 0 だと alignGearToSpeed が向きを判断できないので、。
      publishCommand(static_cast<float>(forward ? 0.01 : -0.01), 0.0f, st_cmd);
      if ((now - last_simple_log_).seconds() > 0.5) {
        last_simple_log_ = now;
        RCLCPP_INFO(get_logger(),
          "復帰(簡易) ギア待ち %s 舵%+.0fdeg 待ち%.2fs 実ギア=%u 指令=%u",
          forward ? "前進" : "後退", st_cmd * 180.0 / M_PI, waited,
          gear_report_, gear_now_);
      }
      return true;
    }
  }
  // AWSIM は速度指令を使わず加速度だけを見る。一定の加速度を出し続けると速度が
  // 上がり続ける(4台レースで復帰の前進が 5m/s に達し、舵を振ったまま壁へ衝突)。
  // simple_speed に達したら踏むのをやめ、超えたら制動する。
  double simple_a = simple_accel_;
  const double v_abs_now = std::abs(latest_velocity_);
  if (v_abs_now > simple_speed_ + 0.3) {
    simple_a = kBrakeAccel;
  } else if (v_abs_now >= simple_speed_) {
    simple_a = 0.0;
  }
  publishCommand(static_cast<float>(forward ? simple_speed_ : -simple_speed_),
                 static_cast<float>(simple_a), st_cmd);
  if ((now - last_simple_log_).seconds() > 0.5) {
    last_simple_log_ = now;
    RCLCPP_INFO(get_logger(),
      "復帰(簡易) %s 舵%+.0fdeg 目標点まで%.2fm (前進候補=%d 到達%.2f / 後退候補=%d 到達%.2f) "
      "実速度%.2f 壁まで%.2f 位置(%.1f,%.1f) yaw%.0f",
      forward ? "前進" : "後退", st_cmd * 180.0 / M_PI,
      forward ? dist_f : dist_b, ok_f ? 1 : 0, dist_f, ok_b ? 1 : 0, dist_b,
      latest_velocity_, recovery::wallClearanceAt(obstacles_, veh_, p),
      p.x, p.y, p.yaw * 180.0 / M_PI);
  }
  return true;
}

bool StuckRecoveryController::runRecovery(const rclcpp::Time & now)
{
  ProfScope prof_scope(prof_, "runRecovery");
  if (!recovery_start_time_.has_value()) { return false; }
  const double total = (now - recovery_start_time_.value()).seconds();
  if (recovery_simple_) {
    if (total > kRecoveryMaxSec) {
      RCLCPP_WARN(get_logger(), "復帰(簡易) 上限%.0fs に到達。通常制御へ返す", kRecoveryMaxSec);
      finishRecovery(now, "30秒の上限");
      return false;
    }
    if (runRecoverySimple(now)) { return true; }
    return false;
  }
  if (total > kRecoveryMaxSec) {
    // 食い込み中も通常制御へ返す ---。
    RCLCPP_WARN(get_logger(), "復帰 上限%.0fs に到達。通常制御へ返す 状況= %s",
                kRecoveryMaxSec, situation_.c_str());
    finishRecovery(now, "30秒の上限");
    return false;
  }

  recovery::Pose p;
  if (!currentPose(p)) { return false; }

  if (!missing_contact_id_.empty()) {
    const bool present = v2x_ && std::any_of(
      v2x_->vehicles.begin(), v2x_->vehicles.end(),
      [&](const auto & vehicle) { return vehicle.vehicle_id == missing_contact_id_; });
    if (!present) {
      cmd_src_ = "V2X接触車待ち";
      publishSafeStop(sent_steer_phys_);
      return true;
    }
    missing_contact_id_.clear();
    plan_.valid = false;
    escape_certificate_ = {};
  }
  // Two controllers planning against the other's frozen snapshot can choose。
  if (contact_lease_enable_ && !vehicle_id_.empty()) {
    std::vector<std::string> contact_ids;
    for (const auto & car : carObstacles()) {
      if (!car.id.empty() && recovery::carClearanceAt(car, veh_, p) < 0.0) {
        contact_ids.push_back(car.id);
      }
    }
    const auto lease = recovery::decideContactLease(
      std::move(contact_ids), vehicle_id_, now.seconds(),
      kContactLeaseSlotSec, kContactLeaseGuardSec);
    if (lease.applicable && !lease.is_owner) {
      // Waiting is not evidence that the certified plan became invalid.。
      cmd_src_ = "接触リース待ち";
      publishSafeStop(sent_steer_phys_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "復帰 接触移動権 owner=%s self=%s guard=%d slot=%llu。非所有中は停止",
        lease.lease_owner.c_str(), vehicle_id_.c_str(), lease.guard ? 1 : 0,
        static_cast<unsigned long long>(lease.slot));
      return true;
    }
  }

  if (tryHandBack(p, now, total)) { return false; }

  

  if (desperate_) { runDesperate(p, now); return true; }

  if (!plan_.valid || phase_idx_ >= plan_.phases.size()) {
    const auto done = replanOrEscalate(p, now);
    if (done.has_value()) { return done.value(); }
  }

  const auto & ph = plan_.phases[phase_idx_];

  // Braking is a state between executable phases.  Do it before checking the。
  if (runBraking(now)) { return true; }

  // Accumulate signed longitudinal progress for this phase.  Euclidean。
  if (last_valid_) {
    const recovery::Pose previous{last_x_, last_y_, last_yaw_};
    phase_travelled_ += recovery::directedTravelIncrement(previous, p, ph.forward);
  }
  last_x_ = p.x;
  last_y_ = p.y;
  last_yaw_ = p.yaw;
  last_valid_ = true;

  // The accepted path is an indivisible escape contract.  Verify both that no。
  const bool same_certified_plan = escape_certificate_.matchesPlan(plan_);
  const std::size_t phase_progress =
    traj_from_ >= ph.path_begin && traj_from_ < ph.path_end ? traj_from_ : ph.path_begin;
  const auto nearest = recovery::nearestInPhase(plan_, phase_idx_, phase_progress, p);
  const double measured_wall = recovery::wallClearanceAt(obstacles_, veh_, p);
  double absolute_floor = nearest.has_value()
    ? escape_certificate_.runtimeWallFloor(nearest.value())
    : std::numeric_limits<double>::infinity();
  // Once an initially-overlapped vehicle reaches free space, never allow it to。
  if (escape_certificate_.starts_inside_wall && escape_best_wall_clear_ >= 0.0) {
    absolute_floor = -escape_certificate_.options.runtime_start_worsen_tolerance;
  }
  const double progress_floor =
    escape_certificate_.starts_inside_wall && escape_best_wall_clear_ < 0.0
    ? escape_best_wall_clear_ - escape_certificate_.options.runtime_progress_worsen_tolerance
    : -std::numeric_limits<double>::infinity();
  const double required_floor = std::max(absolute_floor, progress_floor);
  const bool inside_envelope = nearest.has_value() &&
    std::isfinite(measured_wall) && measured_wall >= required_floor;
  // Check every vehicle separately.  An aggregate minimum can hide a new。
  bool inside_car_envelope = true;
  std::string bad_car_id;
  double bad_car_clearance = 1e3;
  double bad_car_floor = -escape_certificate_.options.runtime_start_worsen_tolerance;
  const auto current_cars = carObstacles();
  for (const auto & car : current_cars) {
    const auto contract = std::find_if(
      escape_certificate_.certified_cars.begin(), escape_certificate_.certified_cars.end(),
      [&](const recovery::CertifiedCarTrace & item) { return item.vehicle_id == car.id; });
    const double measured = recovery::carClearanceAt(car, veh_, p);
    double floor = -escape_certificate_.options.runtime_start_worsen_tolerance;
    if (contract != escape_certificate_.certified_cars.end() && contract->starts_overlapping) {
      const auto best_it = escape_best_car_clearance_.find(car.id);
      const double best = best_it == escape_best_car_clearance_.end()
        ? contract->start_clearance : best_it->second;
      floor = contract->start_clearance -
        escape_certificate_.options.runtime_start_worsen_tolerance;
      if (best >= 0.0) {
        floor = -escape_certificate_.options.runtime_start_worsen_tolerance;
      } else {
        floor = std::max(
          floor, best - escape_certificate_.options.runtime_car_progress_worsen_tolerance);
      }
    }
    if (!std::isfinite(measured) || measured < floor) {
      inside_car_envelope = false;
      bad_car_id = car.id;
      bad_car_clearance = measured;
      bad_car_floor = floor;
      break;
    }
  }
  // A vehicle that was overlapping at certification may not silently vanish。
  if (inside_car_envelope) {
    for (const auto & contract : escape_certificate_.certified_cars) {
      if (!contract.starts_overlapping) { continue; }
      const bool present = v2x_ && std::any_of(
        v2x_->vehicles.begin(), v2x_->vehicles.end(),
        [&](const auto & vehicle) { return vehicle.vehicle_id == contract.vehicle_id; });
      if (!present) {
        inside_car_envelope = false;
        bad_car_id = contract.vehicle_id + "(V2X消失)";
        missing_contact_id_ = contract.vehicle_id;
        break;
      }
    }
  }
  if (!same_certified_plan || !inside_envelope || !inside_car_envelope) {
    RCLCPP_WARN(get_logger(),
      "復帰 計画証明の実行拒否 同一%d idx=%zu "
      "実測壁%.2f 許容%.2f 車%s 実測%.2f 許容%.2f。"
      "全計画を破棄して現在姿勢から再計画",
      same_certified_plan ? 1 : 0, nearest.value_or(phase_progress), measured_wall,
      required_floor, bad_car_id.empty() ? "-" : bad_car_id.c_str(),
      bad_car_clearance, bad_car_floor);
    // sent_steer_cmd_ は指令単位。publishCommand は実舵角を。
    publishCommand(0.0, -2.0, sent_steer_phys_);
    plan_.valid = false;
    escape_certificate_ = {};
    if (replan_count_ < kReplanMax) { ++replan_count_; }
    makePlan(now, 0);
    return true;
  }
  escape_best_wall_clear_ = std::max(escape_best_wall_clear_, measured_wall);
  for (const auto & car : current_cars) {
    const double measured = recovery::carClearanceAt(car, veh_, p);
    auto best = escape_best_car_clearance_.find(car.id);
    if (best != escape_best_car_clearance_.end()) {
      best->second = std::max(best->second, measured);
    }
  }

  {
    const auto done = waitForRearRoom(ph, now);
    if (done.has_value()) { return done.value(); }
  }

  // --- 固定した経路を毎周期検査する(内容は変えない) ---。
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
      brake_advance_phase_ = true;
      brake_replan_after_ = false;
      brake_timeout_logged_ = false;
      return true;
    }
    ++phase_idx_;
    traj_from_ = phase_idx_ < plan_.phases.size()
      ? plan_.phases[phase_idx_].path_begin : plan_.path.size();
    phase_travelled_ = 0.0;
    last_valid_ = false;
    phase_start_ = now;
    phase_start_wall_clear_ = recovery::wallClearanceAt(obstacles_, veh_, p);
    phase_start_car_clear_ = recovery::carClearanceAt(carObstacles(), veh_, p);
    return true;
  }

  // 舵は最初から目標値を出す。
  const float steer = static_cast<float>(
    std::clamp(ph.steer, -kMaxSteerRad, kMaxSteerRad));

  publishGear(ph.forward ? GearCommand::DRIVE : GearCommand::REVERSE);
  // 何を指令して、車がどう応じたかを残す。
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
  {
    const double waited = (now - gear_changed_).seconds();
    const bool gear_ready = !gear_report_seen_ || gear_report_ == gear_now_;
    if (!gear_ready && waited < kActuatorWaitMax) {
      cmd_src_ = "ギアの待ち";
      publishCommand(0.0, 0.0, steer);
      return true;
    }
  }
  // ---。
  if (plan_steer_kick_ && phase_travelled_ < plan_steer_kick_m_ &&
      (now - phase_start_).seconds() < plan_steer_kick_sec_ &&
      recovery::wallClearanceAt(obstacles_, veh_, p) < 0.0)
  {
    cmd_src_ = ph.forward ? "計画舵で押し出し(前進)" : "計画舵で押し出し(後退)";
    publishCommand(
      static_cast<float>(plan_steer_kick_speed_),
      static_cast<float>(plan_steer_kick_accel_), steer);
    return true;
  }
  if (ph.forward) {
    // 前進区間は経路を publish して pure_pursuit に追従させる。
    if (traj_following_) {
      if (std::abs(latest_velocity_) > kMovingSpeedThreshold) {
        traj_still_ = false;
      } else {
        if (!traj_still_) { traj_still_ = true; traj_still_since_ = now; }
        else if ((now - traj_still_since_).seconds() > kTrajStillSec) {
          ++replan_count_;
          blocked_dir_ = 1;
          RCLCPP_WARN(get_logger(),
                      "復帰 前進経路で %.1fs 動かない。通常軌道へは戻さず、"
                      "後退から引き直す(%d回目)",
                      kTrajStillSec, replan_count_);
          makePlan(now, -1);
          return true;
        }
      }
      return false;   // 指令は pure_pursuit のものを通す
    }
    // 一定舵角の直接制御を廃止した ---。
    return false;
  } else {
    // 後退区間も経路を publish して後退用 pure_pursuit に追従させる。
    if (reverse_following_) {
      if (std::abs(latest_velocity_) > kMovingSpeedThreshold) {
        traj_still_ = false;
      } else {
        if (!traj_still_) { traj_still_ = true; traj_still_since_ = now; }
        else if ((now - traj_still_since_).seconds() > kTrajStillSec) {
          ++replan_count_;
          blocked_dir_ = -1;
          RCLCPP_WARN(get_logger(),
                      "復帰 後退経路で %.1fs 動かない。通常前進へは落とさず、"
                      "前進から引き直す(%d回目)",
                      kTrajStillSec, replan_count_);
          makePlan(now, 1);
          return true;
        }
      }
      return false;
    }
    // 後退も同じ。一定舵角の直接制御は廃止し、後退用 pure_pursuit に任せる。
    return false;
  }
  return true;
}

// 現在phaseの経路を、前進・後退それぞれのpure pursuitへpublishする。

// 固定した経路が、いまの他車位置でまだ通れるかを検査する。
bool StuckRecoveryController::plannedPathStillClear() const
{
  if (!plan_.valid || plan_.path.empty()) { return true; }
  const auto cars = const_cast<StuckRecoveryController *>(this)->carObstacles();
  if (cars.empty()) { return true; }
  for (const auto & car : cars) {
    const auto contract = std::find_if(
      escape_certificate_.certified_cars.begin(), escape_certificate_.certified_cars.end(),
      [&](const recovery::CertifiedCarTrace & item) { return item.vehicle_id == car.id; });
    double floor = 0.0;  // a new or initially-clear vehicle may never be touched
    if (contract != escape_certificate_.certified_cars.end() && contract->starts_overlapping) {
      const auto best_it = escape_best_car_clearance_.find(car.id);
      const double best = best_it == escape_best_car_clearance_.end()
        ? contract->start_clearance : best_it->second;
      floor = best >= 0.0 ? 0.0 : std::max(
        contract->start_clearance - escape_certificate_.options.plan_worsen_tolerance,
        best - escape_certificate_.options.runtime_car_progress_worsen_tolerance);
    }
    for (std::size_t i = traj_from_; i < plan_.path.size(); ++i) {
      if (recovery::carClearanceAt(car, veh_, plan_.path[i]) < floor) { return false; }
    }
  }
  return true;
}

bool StuckRecoveryController::publishRecoveryTrajectory()
{
  if (!latest_traj_ || !recovery_start_time_.has_value() || !plan_.valid) {
    rev_off_why_ = !latest_traj_ ? "軌道なし"
                 : !recovery_start_time_.has_value() ? "復帰中でない" : "計画なし";
    reverse_following_ = false;
    return false;
  }
  if (braking_ || desperate_ || phase_idx_ >= plan_.phases.size()) {
    rev_off_why_ = braking_ ? "制動中"
                 : desperate_ ? "最終手段" : "区間を使い切った";
    reverse_following_ = false;
    return false;
  }

  recovery::Pose cur;
  if (!currentPose(cur)) {
    rev_off_why_ = "姿勢なし"; reverse_following_ = false; return false;
  }

  // 同じ経路上でも切り返し後の枝は現在地へ近づき得る。
  const auto nearest = recovery::nearestInPhase(plan_, phase_idx_, traj_from_, cur);
  if (!nearest) {
    rev_off_why_ = "区間の経路範囲が不正";
    reverse_following_ = false;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "復帰 経路範囲が不正 phase=%zu begin=%zu end=%zu points=%zu",
      phase_idx_, plan_.phases[phase_idx_].path_begin,
      plan_.phases[phase_idx_].path_end, plan_.path.size());
    return false;
  }
  const std::size_t from = *nearest;
  traj_from_ = from;
  const auto & path_phase = plan_.phases[phase_idx_];
  const std::size_t phase_end = path_phase.path_end;

  auto push = [](Trajectory & t, double x, double y, double yaw, double v) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw * 0.5);
    p.pose.orientation.w = std::cos(yaw * 0.5);
    p.longitudinal_velocity_mps = static_cast<float>(v);
    t.points.push_back(p);
  };

  const bool in_reverse = reverse_traj_enable_ && !plan_.phases[phase_idx_].forward;
  if (!reverse_traj_enable_ && !plan_.phases[phase_idx_].forward) {
    rev_off_why_ = "後退経路が無効";
    reverse_following_ = false;
    publishRecoveryPathMarker(from);   // 表示だけは従来どおり出す
    return false;
  }

  // --- 後退区間: 後退用 pure_pursuit へ、後退の向きに並んだ経路を渡す
  if (in_reverse) {
    Trajectory rev;
    rev.header = latest_traj_->header;
    rev.header.stamp = this->now();
    for (std::size_t i = from; i < phase_end; ++i) {
      push(rev, plan_.path[i].x, plan_.path[i].y, plan_.path[i].yaw, static_cast<float>(recovery_speed_));
    }
    // 1.5mのlookaheadより十分長い尾を付け、区間終端でも目標点を確保する。
    {
      constexpr double kTailLen = 4.0;
      constexpr double kTailStep = 0.25;
      if (rev.points.size() >= 1) {
        const auto & last = plan_.path[phase_end - 1];
        // 尾は「最後の2点から求めた実際の曲率」で延ばす ---。
        std::optional<recovery::Pose> previous;
        if (phase_end - path_phase.path_begin >= 2) {
          previous = plan_.path[phase_end - 2];
        }
        for (const auto & p : recovery::makeReverseTail(
            last, previous, kTailLen, kTailStep))
        {
          push(rev, p.x, p.y, p.yaw, static_cast<float>(recovery_speed_));
        }
      }
    }
    publishRecoveryPathMarker(from);
    if (rev.points.size() < 2) {
      rev_off_why_ = "後退経路の点が2未満";
      reverse_following_ = false; return false;
    }
    reverse_traj_pub_->publish(rev);
    rev_off_why_ = "-";
    reverse_following_ = true;
    if ((this->now() - last_traj_log_).seconds() > 1.0) {
      last_traj_log_ = this->now();
      RCLCPP_INFO(get_logger(), "復帰 後退を経路で渡す (%zu点)", rev.points.size());
    }
    // traj_following_(前進用)は立てない。呼び出し側は戻り値でそれを決めるので。
    return false;
  }

  rev_off_why_ = "前進区間";
  reverse_following_ = false;

  // --- 前進区間: 計画した前進経路 + 本線へ「なめらかに」合流する部分
  Trajectory out;
  out.header = latest_traj_->header;
  out.header.stamp = this->now();
  for (std::size_t i = from; i < phase_end; ++i) {
    push(out, plan_.path[i].x, plan_.path[i].y, plan_.path[i].yaw, static_cast<float>(recovery_speed_));
  }
  // 前進側は下で参照経路を継ぎ足すので、経路が目標距離より短くなることはない。
  if (out.points.size() < 2) { return false; }

  // 終端から目標軌道へつなぐ。
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
          static_cast<float>(recovery_speed_ * w + vt * (1.0 - w));
    }
    // 復帰中の経路は全区間を recovery_speed 以下にする(先頭の合流区間だけでなく)。
    pt.longitudinal_velocity_mps =
        std::min(pt.longitudinal_velocity_mps, static_cast<float>(recovery_speed_));
    out.points.push_back(pt);
  }
  publishRecoveryPathMarker(from);
  traj_pub_->publish(out);
  if ((this->now() - last_traj_log_).seconds() > 1.0) {
    last_traj_log_ = this->now();
    RCLCPP_INFO(get_logger(),
                "復帰 前進を経路で渡す (%zu点, 計画%zu点ぶん, 継ぎ目の横ずれ%.2fm)",
                out.points.size(), phase_end - from, lat0);
  }
  return true;
}

// 復帰の当たり判定が見ている占有格子を、そのまま rviz へ出す。
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

void StuckRecoveryController::finishRecovery(const rclcpp::Time & now,
                                             const char * why)
{
  deadlock_since_.reset();   // 手詰まりの計時は復帰の切れ目で必ず落とす
  probe_idx_ = -1;           // 反復探索も切れ目で最初から
  // 復帰1回ぶんの要約を必ず1行残す ---。
  recovery::Pose p;
  double clear_now = 9.99, moved = 0.0;
  if (currentPose(p)) {
    if (obstacles_.valid()) { clear_now = recovery::wallClearanceAt(obstacles_, veh_, p); }
    moved = std::hypot(p.x - recovery_start_x_, p.y - recovery_start_y_);
  }
  RCLCPP_WARN(get_logger(),
    "復帰終了 理由=%s 継続=%.1fs 移動=%.2fm 壁 開始%.2f -> 終了%.2f "
    "計画%d回 最終手段=%d 状況= %s",
    why,
    recovery_start_time_.has_value()
      ? (now - recovery_start_time_.value()).seconds() : -1.0,
    moved, recovery_start_clear_, clear_now, replan_count_,
    desperate_ ? 1 : 0, situation_.c_str());
  traceDump(now, "recovery_end");
  publishGear(GearCommand::DRIVE);
  recovery_start_time_.reset();
  recovery_end_time_ = now;
}

// 壁への食い込みの観測。**前進指令の有無に関わらず毎周期更新する。**。
void StuckRecoveryController::updateWallBanState()
{
  ProfScope prof_scope(prof_, "updateWallBanState");
  recovery::Pose p;
  if (!currentPose(p) || !obstacles_.valid()) { return; }
  const double clear = recovery::wallClearanceAt(obstacles_, veh_, p);
  // 用に毎周期控える(禁止が無効でも記録する)。
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
  if (clear > -wall_forward_ban_depth_) { return; }
  if (wall_ban_since_ < 0.0 || clear > wall_ban_ref_clear_ + wall_forward_ban_gain_) {
    wall_ban_since_ = t;
    wall_ban_ref_clear_ = clear;
  }
}

// 壁へ食い込んだまま前進し続けない(絶対の不変条件)。

// 「壁の外に出たまま前進の指令が出た」を抜け道ごとに数えるだけの関数。
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
  // 一度も走り出していない間は効かせない ---。
  if (!moving_observed_) { noteForwardInWall(0, speed); return; }
  if (!wall_forward_ban_) { noteForwardInWall(1, speed); }
  if (!wall_forward_ban_ || speed <= 0.05f) {
    if (wall_ban_gear_rev_ && speed > 0.05f) {
      publishGear(GearCommand::DRIVE);
      wall_ban_gear_rev_ = false;
    }
    return;
  }
  // 復帰計画の前進区間は止めない ---。
  if (recovery_start_time_.has_value() && plan_.valid &&
      escape_certificate_.matchesPlan(plan_) &&
      phase_idx_ < plan_.phases.size() && plan_.phases[phase_idx_].forward)
  {
    recovery::Pose p;
    if (currentPose(p)) {
      const auto & phase = plan_.phases[phase_idx_];
      const std::size_t progress =
        traj_from_ >= phase.path_begin && traj_from_ < phase.path_end
        ? traj_from_ : phase.path_begin;
      const auto nearest = recovery::nearestInPhase(plan_, phase_idx_, progress, p);
      double floor = nearest.has_value()
        ? escape_certificate_.runtimeWallFloor(nearest.value())
        : std::numeric_limits<double>::infinity();
      if (escape_certificate_.starts_inside_wall && escape_best_wall_clear_ >= 0.0) {
        floor = -escape_certificate_.options.runtime_start_worsen_tolerance;
      } else if (escape_certificate_.starts_inside_wall && escape_best_wall_clear_ < 0.0) {
        floor = std::max(
          floor, escape_best_wall_clear_ -
          escape_certificate_.options.runtime_progress_worsen_tolerance);
      }
      if (nearest.has_value() && std::isfinite(wall_clear_now_) && wall_clear_now_ >= floor)
      {
        noteForwardInWall(2, speed);
        return;
      }
    }

    // runRecovery normally catches this before a command is selected.  Keep an。
    wall_ban_hit_ = true;
    speed = 0.0f;
    acceleration = std::min(acceleration, -2.0f);
    return;
  }
  // **走っている車には効かせない。** 押し付けられて動けない状態が前提であり、。
  if (std::abs(latest_velocity_) >= wall_forward_ban_speed_) {
    noteForwardInWall(3, speed);
    return;
  }
  // wall_ban_since_ が立っていない = 深さ(wall_forward_ban_depth_)に届いていない。
  if (wall_ban_since_ < 0.0) { noteForwardInWall(4, speed); return; }
  const double held = this->now().seconds() - wall_ban_since_;
  // 深く食い込んでいるなら「改善するか様子を見る」意味がない。猶予を縮める。
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
  // --- 止めるだけでは姿勢が直らない。後退しながら回転して抜ける ---。
  if (wall_ban_reverse_ && held > wall_ban_reverse_after_) {
    if (rearRoom() >= wall_ban_reverse_rear_) {
      // 舵は「後退したときに壁から離れる向き」を選ぶ。
      recovery::Pose p;
      if (currentPose(p)) {
        // 1m の後退では足りず、3案すべてが現状より悪化する。
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
        // **ギアを後退へ入れる。** 負の目標速度だけでは後退しない。
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

// ギアと指令速度の符号を必ず一致させる。
void StuckRecoveryController::alignGearToSpeed(float speed)
{
  constexpr float kDeadband = 0.05f;
  if (speed > kDeadband) {
    publishGear(GearCommand::DRIVE);
  } else if (speed < -kDeadband) {
    publishGear(GearCommand::REVERSE);
  }
  // 停止指令のときはギアを触らない。切り返しの待ち時間中に。
}

void StuckRecoveryController::applyReachableWallGuard(
  float & speed, float & acceleration, float & command_steer)
{
  if (!reachable_wall_guard_enable_ || !obstacles_.valid() ||
      !reachable_wall_guard_have_steer_) { return; }
  recovery::Pose pose;
  if (!currentPose(pose)) { return; }
  if (!std::isfinite(steer_cmd_scale_) || steer_cmd_scale_ <= 0.0) {
    speed = 0.0f;
    acceleration = kBrakeAccel;
    return;
  }
  recovery::SteeringMotion motion;
  motion.pose = pose;
  motion.speed = latest_velocity_;
  motion.steering = steer_report_;
  // Acceleration is a pedal: propulsion follows gear; braking opposes measured motion.
  const double pedal = std::clamp(static_cast<double>(acceleration), -1.37, 1.37);
  const double gear_direction = gear_now_ == GearCommand::REVERSE ? -1.0 : 1.0;
  motion.acceleration = pedal >= 0.0 ? gear_direction * pedal :
    (std::abs(motion.speed) < 1e-6 ? 0.0 : std::copysign(pedal, -motion.speed));
  const double nominal = std::clamp(
    static_cast<double>(command_steer) / steer_cmd_scale_, -veh_.max_steer, veh_.max_steer);
  const auto decision = recovery::selectReachableSteering(
    obstacles_, veh_, motion, nominal, reachable_wall_guard_options_);
  if (!decision.valid) {
    speed = 0.0f;
    acceleration = kBrakeAccel;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
      "到達可能壁予測の入力が無効。安全停止");
    return;
  }
  // 止まって食い込んでいる間は「停止」を効かせない ---。
  const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, pose);
  const bool halted = std::abs(latest_velocity_) < kBrakeDoneSpeed;
  if (halted && clear_now < 0.0) {
    if (!guard_stuck_since_) { guard_stuck_since_ = this->now(); }
  } else {
    guard_stuck_since_.reset();
  }
  const bool allow_escape = reachable_guard_allow_escape_ && guard_stuck_since_ &&
    (this->now() - guard_stuck_since_.value()).seconds() >= reachable_guard_escape_sec_;

  if (!decision.overridden && !decision.stop) { return; }
  if (decision.overridden) {
    command_steer = static_cast<float>(decision.steering * steer_cmd_scale_);
  }
  // ---。
  const bool certified_now =
    guard_trust_certificate_ && escape_certificate_.valid && plan_.valid;
  if (decision.stop && certified_now && !allow_escape) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "到達可能壁予測 停止を出したが、計画は証明に合格している"
      "(壁 開始%.2f 最小%.2f 終端%.2f)。指令を通す(出=%s)",
      escape_certificate_.start_wall_clearance,
      escape_certificate_.minimum_wall_clearance,
      escape_certificate_.terminal_wall_clearance,
      cmd_src_ ? cmd_src_ : "-");
  } else if (decision.stop && allow_escape) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "到達可能壁予測 停止を出したが、止まって壁に %.2fm 食い込んで %.1f秒 経っている。"
      "停止では出られないので指令を通す(出=%s)",
      clear_now, (this->now() - guard_stuck_since_.value()).seconds(),
      cmd_src_ ? cmd_src_ : "-");
  } else if (decision.stop) {
    speed = 0.0f; acceleration = kBrakeAccel;
  }
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
    "到達可能壁予測 出=%s 実速%+.2f 実舵%+.3f 指令物理%+.3f->%+.3f "
    "最小余裕%.2f->%.2f 終端%.2f 停止%d",
    cmd_src_ ? cmd_src_ : "-", motion.speed, motion.steering, nominal, decision.steering,
    decision.nominal.minimum_clearance, decision.selected.minimum_clearance,
    decision.selected.terminal_clearance, decision.stop ? 1 : 0);
}

void StuckRecoveryController::publishFiltered(AckermannControlCommand cmd)
{
  ProfScope prof_scope(prof_, "publishFiltered");
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
  // ここを通る指令は pure_pursuit / reverse_pure_pursuit が作ったもので、。
  if (rev_gear_fix_ && recovery_start_time_.has_value() && plan_.valid &&
      phase_idx_ < plan_.phases.size() && !plan_.phases[phase_idx_].forward &&
      cmd.longitudinal.speed > 0.0f)
  {
    cmd.longitudinal.speed = -std::abs(cmd.longitudinal.speed);
    ++rev_gear_fixed_;
    const auto tnow = this->now();
    if ((tnow - last_rev_gear_log_).seconds() > 2.0) {
      last_rev_gear_log_ = tnow;
      RCLCPP_WARN(get_logger(),
        "後退区間なので送出ギアを後退にそろえる(累計%d周期) "
        "出=%s 加速度%+.2f 実速度%.2f 区間%zu/%zu",
        rev_gear_fixed_, cmd_src_ ? cmd_src_ : "-",
        cmd.longitudinal.acceleration, latest_velocity_,
        phase_idx_, plan_.phases.size());
    }
  }
  // 後退区間で後退用コントローラを通らなかった周期 ---。
  if (recovery_start_time_.has_value() && plan_.valid &&
      phase_idx_ < plan_.phases.size() && !plan_.phases[phase_idx_].forward &&
      cmd_src_ && std::string(cmd_src_).find("後退") == std::string::npos)
  {
    const auto tnow = this->now();
    if ((tnow - last_rev_off_log_).seconds() > 1.0) {
      last_rev_off_log_ = tnow;
      RCLCPP_WARN(get_logger(),
        "後退区間なのに後退用が外れている 外れた理由=%s 出=%s "
        "加速度%+.2f 実速度%.2f 諦め=%d 引き直し%d 区間%zu/%zu "
        "経路の現在地=%zu 区間経路=[%zu,%zu) 後退点数=%zu 経路点数=%zu "
        "区間走行=%.2f/%.2fm",
        rev_off_why_, cmd_src_, cmd.longitudinal.acceleration,
        latest_velocity_, 0, replan_count_,
        phase_idx_, plan_.phases.size(),
        traj_from_, plan_.phases[phase_idx_].path_begin,
        plan_.phases[phase_idx_].path_end, plan_.rev_points, plan_.path.size(),
        phase_travelled_,
        (phase_idx_ < plan_.phases.size()) ? plan_.phases[phase_idx_].length : -1.0);
    }
  }
  // ---。
  if (steer_clamp_all_ && std::isfinite(steer_cmd_scale_) && steer_cmd_scale_ > 1e-6) {
    const double phys = static_cast<double>(cmd.lateral.steering_tire_angle) / steer_cmd_scale_;
    const double lim = std::clamp(phys, -steerCmdMax(), steerCmdMax());
    if (std::abs(lim - phys) > 1e-6) {
      cmd.lateral.steering_tire_angle = static_cast<float>(lim * steer_cmd_scale_);
      if ((this->now() - last_steer_clamp_log_).seconds() > 1.0) {
        last_steer_clamp_log_ = this->now();
        RCLCPP_WARN(get_logger(),
          "舵を実舵上限で切り詰め %.1fdeg -> %.1fdeg (出=%s)",
          phys * 180.0 / M_PI, lim * 180.0 / M_PI, cmd_src_ ? cmd_src_ : "-");
      }
    }
  }
  // ---。
  if (plan_steer_band_ > 0.0 && plan_.valid && (traj_following_ || reverse_following_) &&
      phase_idx_ < plan_.phases.size() &&
      std::isfinite(steer_cmd_scale_) && steer_cmd_scale_ > 1e-6)
  {
    double want = std::clamp(
      static_cast<double>(plan_.phases[phase_idx_].steer), -kMaxSteerRad, kMaxSteerRad);
    if (plan_local_steer_) {
      recovery::Pose pp;
      if (currentPose(pp)) {
        const std::size_t pp0 =
          traj_from_ < plan_.path.size() ? traj_from_ : 0;
        const auto nb = recovery::nearestInPhase(plan_, phase_idx_, pp0, pp);
        if (nb.has_value()) {
          const std::size_t i0 = nb.value();
          const std::size_t i1 = std::min(i0 + 2, plan_.path.size() - 1);
          if (i1 > i0) {
            const double dx = plan_.path[i1].x - plan_.path[i0].x;
            const double dy = plan_.path[i1].y - plan_.path[i0].y;
            const double ds = std::hypot(dx, dy);
            double dyaw = plan_.path[i1].yaw - plan_.path[i0].yaw;
            while (dyaw > M_PI) { dyaw -= 2.0 * M_PI; }
            while (dyaw < -M_PI) { dyaw += 2.0 * M_PI; }
            if (ds > 1e-3) {
              const double local = std::atan(kWheelBase * dyaw / ds);
              if (std::isfinite(local)) {
                want = std::clamp(local, -kMaxSteerRad, kMaxSteerRad);
              }
            }
          }
        }
      }
    }
    const double phys = static_cast<double>(cmd.lateral.steering_tire_angle) / steer_cmd_scale_;
    const double lim = std::clamp(phys, want - plan_steer_band_, want + plan_steer_band_);
    if (std::abs(lim - phys) > 1e-6) {
      cmd.lateral.steering_tire_angle = static_cast<float>(lim * steer_cmd_scale_);
      if ((this->now() - last_plan_band_log_).seconds() > 1.0) {
        last_plan_band_log_ = this->now();
        RCLCPP_WARN(get_logger(),
          "舵を計画の帯へ収めた 追従%.1fdeg -> %.1fdeg (計画%.1fdeg ±%.1fdeg 出=%s)",
          phys * 180.0 / M_PI, lim * 180.0 / M_PI, want * 180.0 / M_PI,
          plan_steer_band_ * 180.0 / M_PI, cmd_src_ ? cmd_src_ : "-");
      }
    }
  }
  alignGearToSpeed(cmd.longitudinal.speed);
  // `復帰(簡易)前進 ... 最小余裕-0.10 終端0.20 停止1`。
  const bool in_simple_recovery =
    recovery_simple_ && cmd_src_ != nullptr &&
    std::string(cmd_src_).rfind("復帰(簡易)", 0) == 0;
  if (!in_simple_recovery)
  applyReachableWallGuard(
    cmd.longitudinal.speed, cmd.longitudinal.acceleration, cmd.lateral.steering_tire_angle);
  sent_speed_ = cmd.longitudinal.speed;
  sent_accel_ = cmd.longitudinal.acceleration;
  sent_steer_cmd_ = cmd.lateral.steering_tire_angle;
  sent_steer_phys_ = static_cast<float>(sent_steer_cmd_ / steer_cmd_scale_);
  sent_gear_ = gear_now_;
  control_pub_->publish(cmd);
  traceSample(this->now());
}

void StuckRecoveryController::publishSafeStop(float steer)
{
  // A stale REVERSE report while speed=0 looked exactly like the original。
  if (std::abs(latest_velocity_) < kBrakeDoneSpeed) {
    publishGear(GearCommand::DRIVE);
  }
  publishCommand(0.0f, kBrakeAccel, steer);
}


// 反復探索。試す -> 測る -> 効いた向きを残す。
bool StuckRecoveryController::runEscapeProbe(
  const recovery::Pose & p, const rclcpp::Time & now)
{
  if (!escape_probe_enable_) { return false; }
  const double clear_now = recovery::wallClearanceAt(obstacles_, veh_, p);
  // 候補表。後退を先に置く(壁へ食い込んでいるときは前進より下がるほうが効く)。
  struct Cand { int dir; double steer_ratio; };
  static const Cand kCands[10] = {
    {-1,  0.0}, {-1, +1.0}, {-1, -1.0}, {-1, +0.5}, {-1, -0.5},
    {+1,  0.0}, {+1, +1.0}, {+1, -1.0}, {+1, +0.5}, {+1, -0.5},
  };
  constexpr int kN = 10;

  if (probe_idx_ < 0) {           // 開始
    probe_idx_ = 0; probe_best_idx_ = -1; probe_best_gain_ = 0.0;
    probe_since_ = now;
    probe_start_clear_ = clear_now; probe_start_x_ = p.x; probe_start_y_ = p.y;
    RCLCPP_WARN(get_logger(), "反復探索 開始 壁%.2fm", clear_now);
  }
  const double held = (now - probe_since_.value()).seconds();
  const double moved = std::hypot(p.x - probe_start_x_, p.y - probe_start_y_);
  if (held >= escape_probe_sec_ || moved >= escape_probe_dist_) {
    // 1回ぶん終わった。結果を測る。
    const double gain = clear_now - probe_start_clear_;
    if (gain > probe_best_gain_) { probe_best_gain_ = gain; probe_best_idx_ = probe_idx_; }
    RCLCPP_WARN(get_logger(),
      "反復探索 候補%d(%s 舵%+.1f) %.1f秒 %.2fm 動いて 壁 %.2f -> %.2f (%+.2f) %s",
      probe_idx_, kCands[probe_idx_].dir > 0 ? "前進" : "後退",
      kCands[probe_idx_].steer_ratio, held, moved,
      probe_start_clear_, clear_now, gain,
      gain > escape_probe_gain_ ? "効いた。続ける" : "効かない。次へ");
    if (gain <= escape_probe_gain_) {
      probe_idx_ = (probe_idx_ + 1) % kN;
      // 一巡したら、いちばん効いた候補へ戻る(何も効かなければ 0 から)。
      if (probe_idx_ == 0 && probe_best_idx_ >= 0) { probe_idx_ = probe_best_idx_; }
    }
    probe_since_ = now;
    probe_start_clear_ = clear_now; probe_start_x_ = p.x; probe_start_y_ = p.y;
  }
  const Cand & cd = kCands[probe_idx_];
  // 後退するなら後方が空いていることを確かめる(自分で当てると Crash は自分に付く)。
  if (cd.dir < 0 && rearRoom() < deadlock_release_rear_m_) {
    probe_idx_ = 5;               // 前進側の先頭へ飛ばす
  }
  const Cand & use = kCands[probe_idx_];
  publishGear(use.dir > 0 ? GearCommand::DRIVE : GearCommand::REVERSE);
  publishCommand(static_cast<float>(use.dir * 1.0),
                 static_cast<float>(deadlock_release_accel_),
                 static_cast<float>(use.steer_ratio * kMaxSteerRad));
  cmd_src_ = "反復探索";
  return true;
}

void StuckRecoveryController::logNoMove(
  const rclcpp::Time & now, float speed, float accel, float steer_phys)
{
  // 「動かせと指令しているのに動いていない」状態が続いたら、その場の条件をまとめて出す。
  const double t = now.seconds();
  const bool want_move = std::abs(speed) > 0.2f || std::abs(accel) > 0.3f;
  const bool moving = std::abs(latest_velocity_) > 0.15;
  if (!want_move || moving) {
    no_move_since_ = -1.0;
    return;
  }
  if (no_move_since_ < 0.0) { no_move_since_ = t; }
  if (t - no_move_since_ < 1.0 || t - last_no_move_log_ < 1.0) { return; }
  last_no_move_log_ = t;
  recovery::Pose p;
  const bool have_pose = currentPose(p);
  // いちばん近い他車(V2X)。押し合いで動けない場合を切り分ける。
  std::string near_id = "-";
  double near_d = -1.0;
  if (v2x_ && have_pose) {
    for (const auto & v : v2x_->vehicles) {
      const double d = std::hypot(v.position.x - p.x, v.position.y - p.y);
      if (d < 0.5) { continue; }
      if (near_d < 0.0 || d < near_d) { near_d = d; near_id = v.vehicle_id; }
    }
  }
  const double wall_f = wall_clear_now_;
  RCLCPP_WARN(get_logger(),
    "動かない診断 %.1fs 指令[速度%+.2f 加速度%+.2f 舵%+.0fdeg ギア%s] "
    "禁止前[速度%+.2f 加速度%+.2f] 実[速度%+.2f 舵%+.0fdeg ギア%s] "
    "壁まで%.2fm 復帰=%s 出=%s 近傍車=%s %.2fm 位置=(%.1f,%.1f) 方位%.0fdeg",
    t - no_move_since_, speed, accel, steer_phys * 180.0 / M_PI,
    gear_now_ == GearCommand::REVERSE ? "R" : "D",
    pre_ban_speed_, pre_ban_accel_,
    latest_velocity_, steer_report_ * 180.0 / M_PI,
    gear_report_ == GearReport::REVERSE ? "R" : (gear_report_seen_ ? "D" : "?"),
    wall_f, recovery_start_time_.has_value() ? "中" : "していない",
    cmd_src_ ? cmd_src_ : "-", near_id.c_str(), near_d,
    have_pose ? p.x : 0.0, have_pose ? p.y : 0.0,
    have_pose ? p.yaw * 180.0 / M_PI : 0.0);
}

void StuckRecoveryController::publishCommand(float speed, float acceleration, float steer)
{
  const auto stamp = this->now();
  // --- 出力段で必ず範囲に収める(唯一の publish 地点なので、ここだけで全経路を守れる) ---。
  const float kOutMaxSteer = static_cast<float>(steerCmdMax());
  constexpr float kOutMaxAccel = 2.0f;
  constexpr float kOutMaxSpeed = 10.0f;
  if (!std::isfinite(steer)) { steer = 0.0f; }
  if (!std::isfinite(acceleration)) { acceleration = 0.0f; }
  if (!std::isfinite(speed)) { speed = 0.0f; }
  steer = std::clamp(steer, -kOutMaxSteer, kOutMaxSteer);
  acceleration = std::clamp(acceleration, -kOutMaxAccel, kOutMaxAccel);
  speed = std::clamp(speed, -kOutMaxSpeed, kOutMaxSpeed);

  wall_ban_steer_valid_ = false;
  pre_ban_speed_ = speed;
  pre_ban_accel_ = acceleration;
  applyWallForwardBan(speed, acceleration);
  if (wall_ban_steer_valid_) { steer = wall_ban_steer_; }
  alignGearToSpeed(speed);
  // ここまでの steer は**実舵角**。publish する値は指令の単位なので変換する。
  steer = static_cast<float>(steer * steer_cmd_scale_);
  {
    const bool in_simple_rec =
      recovery_simple_ && cmd_src_ != nullptr &&
      std::string(cmd_src_).rfind("復帰(簡易)", 0) == 0;
    if (!in_simple_rec) {
      applyReachableWallGuard(speed, acceleration, steer);
    }
  }
  logNoMove(stamp, speed, acceleration, static_cast<float>(steer / steer_cmd_scale_));
  sent_steer_phys_ = static_cast<float>(steer / steer_cmd_scale_);
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
  // --- publish 間隔の自己監視(観測のみ) ---。
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
  traceSample(this->now());
  // GUI 用。ここが呼ばれている = 復帰が車両を直接動かしている。
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
    detectUnexpectedReverse(gear_changed_, "gear_command_edge");
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
