#ifndef STUCK_RECOVERY_CONTROLLER_HPP_
#define STUCK_RECOVERY_CONTROLLER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "stuck_recovery_controller/escape_certificate.hpp"
#include "stuck_recovery_controller/recovery_planner.hpp"
#include "stuck_recovery_controller/reachable_steering.hpp"

#include <string>
#include <map>
#include <unordered_map>
#include <vector>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <cstdint>
#include <optional>

namespace stuck_recovery_controller
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;

class StuckRecoveryController : public rclcpp::Node
{
public:
  StuckRecoveryController();

private:
  void onNominalCommand(const AckermannControlCommand::ConstSharedPtr msg);
  void updateStuckDetection(
    const AckermannControlCommand & command, const rclcpp::Time & now);
  bool runRecovery(const rclcpp::Time & now);
  bool runRecoverySimple(const rclcpp::Time & now);
  void publishCommand(float speed, float acceleration, float steer = 0.0f);
  void publishSafeStop(float steer);
  void applyReachableWallGuard(float & speed, float & acceleration, float & command_steer);
  bool reachable_wall_guard_enable_{true};
  bool reachable_wall_guard_have_steer_{false};
  recovery::SteeringPredictionOptions reachable_wall_guard_options_;
  // publish 間隔の自己監視用(OVER ペナルティは 250Hz 以上でも付く)
  double last_pub_t_{-1.0};
  double last_fast_log_t_{-1.0};
  int fast_pub_cnt_{0};
  void loadRaceline(const std::string & raceline_csv, const std::string & corridor_csv);
  bool isRearClear();
  // --- stuck 判定の材料(updateStuckDetection から切り出したもの) ---
  // 車体前方の同一レーンにいる最寄りの他車までの距離[m]。いなければ無限大。
  double nearestForwardVehicle() const;
  // 前後左右を問わず一定半径内に他車がいるか。横並びの押し合いを拾うため。
  bool anyVehicleNear() const;
  // 「進んでいない」を瞬間速度ではなく実際の移動量で判定する。
  bool hasNoProgress(const rclcpp::Time & now);
  // 後方の他車までの距離[m]。いなければ大きな値。
  // 「6m 空くまで待つ」と、レース中は後ろに車がいるのが普通なので
  // 待っているうちに上限に達する。空いている距離ぶんだけ下がる計画にする。
  double rearRoom() const;
  // 復帰に入った状況を1行にまとめる。
  // 単独で壁に刺さった場合と、他車に当てた/当てられた場合では
  // 使える空間も相手の動きもまったく違うので、分けて集計できるようにする。
  std::string situationText() const;
  // 領域内へ戻り向きも揃い走り出せるなら通常制御へ返す。返したら true。
  bool tryHandBack(const recovery::Pose & p, const rclcpp::Time & now, double total);
  // 最終手段。舵を左右に振りながら前後へ全開で当て、強引に隙間を作る。
  void runDesperate(const recovery::Pose & p, const rclcpp::Time & now);
  // 計画を引き直すか最終手段へ移る。値があれば runRecovery はそれを返す。
  std::optional<bool> replanOrEscalate(const recovery::Pose & p, const rclcpp::Time & now);
  // 前進中に壁へ近づいたら引き直す。引き直したら true。
  bool replanIfWallNear(const recovery::Pose & p, const recovery::Phase & ph,
                        const rclcpp::Time & now);
  // 前進中に他車へ近づいたら引き直す。引き直したら true。
  bool replanIfCarNear(const recovery::Pose & p, const recovery::Phase & ph,
                       const rclcpp::Time & now);
  // 前後を切り替える前に止まりきる。まだ止まりきっていなければ true。
  bool runBraking(const rclcpp::Time & now);
  // 後方に余地が無いときの待ち/後退量の調整。値があれば runRecovery はそれを返す。
  std::optional<bool> waitForRearRoom(const recovery::Phase & ph, const rclcpp::Time & now);
  // 停滞していたら計画を引き直す。値があれば runRecovery はそれを返す。
  std::optional<bool> handleStall(const recovery::Pose & p, const recovery::Phase & ph,
                                  const rclcpp::Time & now);
  // 復帰区間を進行方向別の軌道としてpublishする。戻せたらtrue。
  bool publishRecoveryTrajectory();
  // 自車の横位置とその地点のコリドア境界を返す。コリドアが無ければ false。
  bool lateralNow(double & lat, double & lo, double & hi);
  // コース方位に対する車体の向きのずれ[rad]。壁へ突き刺さった判定に使う
  bool headingErrorToTrack(double & err);
  bool handbackPathClear(double dist) const;
  bool canSteerAround();
  float avoidSteerDirection();
  void publishGear(std::uint8_t command);
  // 指令速度の符号にギアを合わせる(出口の不変条件)。
  void alignGearToSpeed(float speed);
  // 実舵で計画し、送出時だけ指令単位へ変換する。比率は実走ログから校正した。
  double steer_cmd_scale_{1.6666667};
  float sent_speed_{0.0f};
  const char * cmd_src_{"未"};
  // 強引な脱出は既定off。有効時も時間と無動作の上限を持つ。
  bool desperate_enable_{false};
  double desperate_max_sec_{10.0};
  double desperate_stall_sec_{3.0};
  double desperate_ref_clear_{-1e9};
  rclcpp::Time desperate_ref_at_{0, 0, RCL_ROS_TIME};
  float sent_steer_cmd_{0.0f};   // 実際に publish した舵角(指令の単位)
  float sent_steer_phys_{0.0f};  // その狙いの実舵角[rad]
  float sent_accel_{0.0f};
  std::uint8_t sent_gear_{0};
  bool unexpected_reverse_active_{false};
  int unexpected_reverse_count_{0};

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr control_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr nominal_sub_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr reverse_sub_;
  AckermannControlCommand::ConstSharedPtr reverse_cmd_;
  // 後退指令が届いた時刻。**古い指令を出し続けないため**に鮮度で切る。
  rclcpp::Time reverse_cmd_time_{0, 0, RCL_ROS_TIME};
  const char * rev_off_why_ = "-";
  // **注意**: `rev_off_why_` はメンバなので、`publishRecoveryTrajectory()` が。
  rclcpp::Time last_rev_off_log_{0, 0, RCL_ROS_TIME};
  bool rev_gear_fix_;          // 後退区間では送出ギアを REVERSE にそろえる
  int rev_gear_fixed_ = 0;     // そろえた周期数(発火の確認用)
  rclcpp::Time last_rev_gear_log_{0, 0, RCL_ROS_TIME};
  double rev_cmd_hold_sec_;    // 後退指令を有効とみなす鮮度上限[s]
  int rev_hold_used_ = 0;      // 保持した指令を出した周期数(効果の確認用)
  int rev_hold_stale_ = 0;     // 古すぎて使えなかった周期数
  rclcpp::Time last_rev_stale_log_{0, 0, RCL_ROS_TIME};
  // 後退区間を経路で渡している最中か。true の間は後退用 pure_pursuit を中継する。
  bool reverse_following_{false};
  bool reverse_traj_enable_{true};
  bool wedge_backward_first_{true};   // 壁へ食い込んでいたら後退から始める

  bool wall_forward_ban_{true};          // 壁へ食い込んだままの前進を禁じるか
  double wall_forward_ban_hold_{0.8};    // 改善しないまま前進を許す時間[s]
  double wall_forward_ban_gain_{0.03};   // 「改善した」とみなす最小の増分[m]
  double wall_forward_ban_speed_{1.0};   // この速度[m/s]未満のときだけ効かせる
  double wall_forward_ban_depth_{0.15};  // この深さ[m]を超えて食い込んだときだけ
  bool wall_ban_reverse_{true};          // 後退で回転して抜けるか
  double wall_ban_reverse_after_{1.5};   // 止めてからこの秒数で後退へ移る
  double wall_ban_reverse_speed_{1.0};   // 後退の速度[m/s]
  double wall_ban_reverse_rear_{2.0};    // 後方にこれだけ[m]空いていないと出さない
  float wall_ban_steer_{0.0f};           // 回転で抜けるときの舵
  bool wall_ban_steer_valid_{false};
  bool wall_ban_gear_rev_{false};   // 後退で抜けるためにギアを後退へ入れたか

  // 前の車に詰まって待っているだけの車を「スタック」にしないための条件。
  // 車体が健全(領域内 / 向きが揃う / 壁から離れている)なら待つのが正しい。
  bool queue_wait_enable_{true};
  double queue_wait_margin_{0.20};   // 走行可能領域の端からこれだけ内側なら健全[m]
  double queue_wait_yaw_{0.35};      // 方位差がこれ未満なら健全[rad](20度)
  double queue_wait_wall_{0.30};     // 壁までこれ以上あれば健全[m]
  // 待機の上限[s]。全員が健全な永久待機を避けるための安全網。
  double queue_wait_max_{12.0};
  // 完全停止(実速度 < 0.1m/s)のときの短い上限[s]。0以下で無効。
  double queue_wait_max_stopped_{4.0};
  // 待ちに入るのは「自分の真正面に車がいる」ときだけにする。
  bool queue_wait_front_only_{true};

  bool deadlock_release_enable_{false};
  // 手詰まりと判定するまでに止まっていた秒数[s]。
  double deadlock_release_sec_{5.0};
  double embed_min_reverse_{0.20};   // 食い込み中に後退を試す最小距離[m]
  bool plan_steer_kick_{true};       // 食い込み中は区間の頭で計画の舵を直接出す
  double plan_steer_kick_m_{1.0};    // その距離[m]
  double plan_steer_kick_speed_{2.0};
  double plan_steer_kick_accel_{1.37};
  double plan_steer_kick_sec_{1.2};  // 押し出しの時間上限[s]
  double plan_steer_band_{0.15};     // 追従の舵を計画±これ[rad]に収める
  bool   plan_local_steer_{true};    // 舵の帯を経路の局所曲率から出す
  bool   recovery_simple_{true};
  double simple_ahead_m_{6.0};
  double simple_back_m_{4.0};
  double simple_probe_m_{4.0};
  double simple_reach_m_{2.0};
  double simple_wall_keep_{0.05};
  double simple_car_keep_{0.05};
  double simple_speed_{3.0};
  double simple_accel_{1.37};
  int    simple_steer_steps_{8};
  double simple_steer_frac_{0.75};   // 使う舵は最大の何割まで
  double simple_worsen_tol_{0.02};   // 重なっているときの悪化の許容[m]
  double simple_car_worsen_tol_{0.25};// 他車と重なっているときの許容[m]
  double simple_dir_hold_sec_{1.5};  // 向きを保持する最短時間[s]
  // 候補(ok_f/ok_b)が有効でも、方位差が大きい。
  double simple_stall_sec_{3.0};     // この時間[s]実質動かなければ停滞とみなす
  double simple_stall_min_m_{0.15};  // 停滞とみなす移動量の下限[m]
  // いまの向きの候補が連続してこの時間[s]見つからなければ経路の喪失を確定する。
  double simple_lost_confirm_sec_{0.5};
  // 両方向とも候補が無いときに探し直す短い距離[m](全舵)。
  double simple_short_probe_m_{1.0};
  rclcpp::Time last_short_probe_log_{0, 0, RCL_ROS_TIME};
  bool   simple_lost_valid_{false};
  rclcpp::Time simple_lost_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stall_log_{0, 0, RCL_ROS_TIME};
  // 復帰を新しく始めた最初の周期か(開始時だけ壁食い込みの後退優先を掛ける)。
  bool   simple_dir_fresh_{true};
  double simple_embed_improve_{0.05};// 食い込み中に要求する改善量[m]
  bool   simple_tgt_valid_{false};   // 目標点を固定済みか
  double simple_dir_max_m_{6.0};     // 一つの向きで動いてよい最大距離[m]
  std::pair<double,double> simple_dir_from_{0.0,0.0};
  std::pair<double,double> simple_tgt_f_{0.0,0.0};
  std::pair<double,double> simple_tgt_b_{0.0,0.0};
  bool   embed_prefer_reverse_{true};// 食い込み中は後退を優先する
  int    simple_dir_stable_n_{5};    // 向きの変更に要る連続周期数
  int    simple_dir_stable_{0};
  bool   simple_dir_forward_{true};  // いま選んでいる向き
  rclcpp::Time simple_dir_since_{0, 0, RCL_ROS_TIME};
  bool   simple_reversing_{false};
  rclcpp::Time last_simple_log_{0, 0, RCL_ROS_TIME};
  bool   turn_in_abort_{true};       // 切り出しの回頭で壁が悪化したら引き直す
  double turn_in_abort_travel_{0.5};
  double turn_in_abort_worsen_{0.25};
  rclcpp::Time last_plan_band_log_{0, 0, RCL_ROS_TIME};
  bool contact_lease_enable_{false}; // 接触移動権(本番では前提が成立しない)
  // 後退の加速度指令。弱くてよい(接触を切るだけ)。
  double deadlock_release_accel_{0.5};
  // 後退してよい後方の最小空き[m]。空いていなければ下がらない(失格を避ける)。
  double deadlock_release_rear_m_{2.5};
  // 止まって壁に食い込んでいる間は到達可能壁予測の「停止」を効かせない
  bool reachable_guard_allow_escape_{true};
  double reachable_guard_escape_sec_{2.0};
  std::optional<rclcpp::Time> guard_stuck_since_;

  bool escape_probe_enable_{false};
  double escape_probe_sec_{1.0};      // 1回の試行の長さ[s]
  double escape_probe_dist_{0.5};     // 1回で動かす目安[m]
  double escape_probe_gain_{0.05};    // これだけ[m]改善したら「効いた」
  int probe_idx_{-1};                 // いま試している候補(-1 = 未開始)
  double probe_start_clear_{0.0};
  double probe_start_x_{0.0}, probe_start_y_{0.0};
  std::optional<rclcpp::Time> probe_since_;
  int probe_best_idx_{-1};
  double probe_best_gain_{0.0};
  // 1回の試行を実行する。true を返したらこの周期の指令は出し終えている。
  bool runEscapeProbe(const recovery::Pose & p, const rclcpp::Time & now);
  bool stopped_box_enable_{true};   // 止まった相手を矩形で扱う
  double stopped_yaw_rate_{0.02};   // 向きの不確かさの増え方[rad/s]
  double stopped_yaw_max_{0.15};    // その上限[rad]。8.6度。円へは戻さない
  // V2X の基準点を自分で測るための診断。
  void logV2XSelfOffset();
  rclcpp::Time last_v2x_self_log_{0, 0, RCL_ROS_TIME};
  bool v2x_self_absent_logged_{false};
  // 手詰まり状態に入った時刻。
  std::optional<rclcpp::Time> deadlock_since_;

  double recovery_speed_{4.0};      // 復帰の追従速度[m/s](旧 2.5)
  double recovery_accel_{1.37};     // 復帰の加速度指令(旧 1.0。実測上限は 1.37)
  double no_plan_retry_sec_{0.10};  // 計画が無いときの再探索間隔[s](旧 0.25=4Hz)
  bool embed_forward_escape_{true};  // 後退不能な食い込みで前進脱出を許す
  bool guard_trust_certificate_{true};  // 証明済み計画中は壁予測の停止を適用しない
  bool steer_clamp_all_{true};          // すべての送出経路で実舵上限にクランプ
  rclcpp::Time last_steer_clamp_log_{0, 0, RCL_ROS_TIME};
  double phase_min_sec_{0.6};       // 区間開始後に停滞判定をしない時間[s](旧 1.2)
  std::optional<rclcpp::Time> healthy_wait_since_;
  rclcpp::Time last_queue_wait_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stall_suppress_log_{0, 0, RCL_ROS_TIME};
  int stall_suppressed_ = 0;      // 復帰中に停滞判定を握り潰した回数
  rclcpp::Time last_desperate_hold_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wall_ban_rev_log_{0, 0, RCL_ROS_TIME};
  bool reject_car_overlap_plan_{false};
  double wall_ban_ref_clear_{1e9};       // 基準にしている壁の余裕[m]
  double wall_ban_since_{-1.0};          // その基準を更新した時刻[s]
  rclcpp::Time last_wall_ban_log_{0, 0, RCL_ROS_TIME};
  // 壁への食い込みの観測。前進指令の有無に関わらず毎周期更新する
  // (外部レビュー レビュー: 更新が前進指令中だけだと、停止・後退で壁から離れても
  //  基準と時刻が古いまま残り、次の前進開始時に即座に禁止されてしまう)。
  void updateWallBanState();
  // 壁へ食い込んだままの前進を止める。全 publish 経路がここを通る。
  void applyWallForwardBan(float & speed, float & acceleration);
  // control_pub_ への唯一の出口。素通しの指令もここを通す。
  void publishFiltered(AckermannControlCommand cmd);
  rclcpp::Publisher<Trajectory>::SharedPtr reverse_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr recovery_path_pub_;
  rclcpp::Time last_path_marker_{0, 0, RCL_ROS_TIME};
  void publishRecoveryPathMarker(std::size_t from);
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  void publishGrid();
  rclcpp::TimerBase::SharedPtr grid_timer_;
  bool grid_logged_{false};
  // 前進・後退とも経路をpublishし、方向別pure pursuitへ追従させる。
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  Trajectory::ConstSharedPtr latest_traj_;
  // 前進区間を経路で渡している間は、舵と速度を pure_pursuit に任せる。
  bool traj_following_{false};
  // デバッグ表示(GUI)用。復帰が車両へ指令を出している間だけ流す。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Time last_traj_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time traj_still_since_{0, 0, RCL_ROS_TIME};
  bool traj_still_{false};
  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<GearReport>::SharedPtr gear_report_sub_;
  rclcpp::Subscription<SteeringReport>::SharedPtr steer_report_sub_;
  // 復帰動作を任意のタイミングで発動させるための入力(動作確認用)。
  // 壁に当たらなくなると復帰が動く場面に出会えず、検証できないため。
  //   ros2 topic pub -1 /debug/force_recovery std_msgs/msg/Bool "{data: true}"
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr crash_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr race_state_sub_;
  bool force_recovery_{false};
  bool race_started_{false};

  // 復帰経路の計算に使う情報
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  v2x_msgs::msg::V2XVehiclePositionArray::ConstSharedPtr v2x_;
  std::string missing_contact_id_;
  std::string vehicle_id_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr v2x_sub_;
  std::vector<double> line_x_, line_y_, corr_lo_, corr_hi_;
  float latest_velocity_{0.0};
  bool moving_observed_{false};
  std::optional<rclcpp::Time> stuck_start_time_;
  // 通常側が停止指令を出す前方車デッドロックは、通常の壁スタックとは別に
  // 継続時間を測る。条件の切替だけで待ち時間を引き継がないため。
  std::optional<rclcpp::Time> blocked_vehicle_start_time_;
  std::optional<rclcpp::Time> recovery_start_time_;
  std::optional<rclcpp::Time> recovery_end_time_;  // 復帰完了時刻(再突入の抑制に使う)

  recovery::Corridor corridor_;
  // 実際の壁。コリドアは中心線に対する横方向しか持たないので、
  // ヘアピンで鼻先が壁に当たっている状態を表現できない。壁の判定はこちらで行う。
  recovery::ObstacleMap obstacles_;
  recovery::VehicleParams veh_;
  recovery::Plan plan_;
  // A plan is executable only while this certificate matches its complete,
  // unmodified path.  Replanning replaces both together; phases are never cut.
  recovery::EscapeCertificate escape_certificate_;
  double escape_best_wall_clear_{-1e9};
  std::unordered_map<std::string, double> escape_best_car_clearance_;
  std::size_t phase_idx_{0};        // 実行中の区間
  double phase_travelled_{0.0};     // その区間の方向付き進捗[m]
  double last_x_{0.0}, last_y_{0.0}, last_yaw_{0.0};
  bool last_valid_{false};
  int replan_count_{0};
  rclcpp::Time last_plan_attempt_{0, 0, RCL_ROS_TIME};
  // 動けなかった向き。走行可能領域だけでは「鼻先が壁に当たっている」ことが
  // 分からないので、失敗した向きを覚えて次は逆から始めさせる。
  int blocked_dir_{0};        // +1=前進が駄目 / -1=後退が駄目 / 0=未知
  // 最終手段。計画をすべて試しても動けないときだけ使う。
  // 壁へ押し当てて車体の向きを変えたり、詰まった相手を押しのけたりする。
  // ふさわしい動きではないので、他に手が無いときに限る。
  bool desperate_{false};
  rclcpp::Time desperate_since_{0, 0, RCL_ROS_TIME};
  int desperate_swing_{0};
  double recovery_start_x_{0.0}, recovery_start_y_{0.0};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  double stall_x_{0.0}, stall_y_{0.0};   // 動けているかの確認用
  // 計画を採用した地点。走り切った計画が実際に前進できたかを見て、
  // 「やり直し」に数えるかどうかを決める。
  double plan_x_{0.0}, plan_y_{0.0};
  // 計画を採用した時点の壁との余裕[m]。走り切った計画を「進めた」と数えるには、
  // 距離だけでなく状況が改善していることも要る。
  double plan_wall_clear_{0.0};
  bool goal_plan_enable_{true};
  // 共通の評価地点までの距離[m]。復帰後の走りをどれだけ重く見るか。
  double goal_plan_tail_len_{40.0};
  int goal_plan_max_switch_{4};
  double goal_plan_ahead_min_{8.0};
  double goal_plan_ahead_max_{16.0};
  // 到達した目標の中心線 index。合流用の経路をここから継ぐ。
  std::size_t plan_goal_idx_{0};
  std::size_t traj_from_{0};
  bool path_check_enable_{true};
  double path_check_bad_sec_{0.4};   // 無効がこの時間続いたら作り直す
  double path_bad_since_{-1.0};
  rclcpp::Time last_path_bad_log_{0, 0, RCL_ROS_TIME};
  // 固定した経路が、いまの他車位置でまだ通れるか。通れなければ false。
  bool plannedPathStillClear() const;
  bool wall_ban_hit_{false};
  unsigned long plan_seq_{0};
  unsigned long wall_ban_acted_seq_{0};
  bool wall_ban_acted_{false};

  double wall_clear_now_{1e9};       // 毎周期の食い込み量[m]。正=余裕あり
  static constexpr int kFwdWallReasons = 6;
  long fwd_in_wall_cnt_[kFwdWallReasons]{};
  double fwd_in_wall_sec_[kFwdWallReasons]{};
  double fwd_in_wall_worst_{0.0};    // 最も深かった食い込み[m]
  double fwd_in_wall_last_t_{-1.0};
  rclcpp::Time last_invariant_log_{0, 0, RCL_ROS_TIME};
  void noteForwardInWall(int reason, float speed);
  double wall_ban_acted_at_{-1.0};
  rclcpp::Time stall_since_{0, 0, RCL_ROS_TIME};
  // ギアを入れ替えた時刻。AWSIM のギアは切り替わるまで間があるので、
  // 入れ替え直後に動けないことを「詰まっている」と誤判定しないために持つ。
  rclcpp::Time gear_changed_{0, 0, RCL_ROS_TIME};
  std::uint8_t gear_now_{0};
  rclcpp::Time last_trace_{0, 0, RCL_ROS_TIME};
  // 地図の上では余裕があるのに動けなかった回数。
  // 自己位置推定が実際とずれていると起きる。起きたぶんだけ後退を伸ばす。
  int unexplained_block_{0};
  // 停滞候補の間に、これから使う復帰計画の舵角へ先回りして向けておく。
  // 素通しにすると pure_pursuit が目標経路へ向けて大きく舵を切り、
  // それが復帰の向きと逆になるため、動き出す前に一度逆へ振ってしまう。
  bool pre_steer_valid_{false};
  std::string situation_;      // この復帰に入ったときの状況
  // 後方の他車が退くのを待ち始めた時刻。待ちっぱなしを防ぐために持つ。
  rclcpp::Time rear_wait_since_{0, 0, RCL_ROS_TIME};
  bool rear_waiting_{false};
  bool rear_relaxed_{false};
  double pre_steer_{0.0};
  std::uint8_t gear_report_{0};
  bool gear_report_seen_{false};
  double steer_report_{0.0};
  bool braking_{false};
  rclcpp::Time brake_since_{0, 0, RCL_ROS_TIME};
  double brake_steer_{0.0};
  bool brake_was_forward_{true};
  // A normal phase boundary advances after stopping.  A newly planned route
  // selected against current motion must instead be planned again from the
  // actual stopped pose; its old origin is no longer executable.
  bool brake_advance_phase_{true};
  bool brake_replan_after_{false};
  bool brake_timeout_logged_{false};
  // 動作確認用。わざと壁へ突っ込ませて復帰動作を発生させる。
  // 走行ラインを壁へずらす方法だと、コリドアも一緒にずれて
  // 復帰の狙い先まで壁の中に入ってしまい、検証にならないため。
  bool crash_me_{false};
  rclcpp::Time crash_since_{0, 0, RCL_ROS_TIME};
  double crash_steer_{0.0};

  // 現在姿勢から復帰経路を計算する。
  // first_phase: 0=任意 / +1=前進から / -1=後退から
  // 復帰を始めた通し番号。集計をログの文字列推定に頼らないための計数。
  rclcpp::Time last_desperate_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_presteer_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wedge_side_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_efffirst_log_{0, 0, RCL_ROS_TIME};
  bool queue_wait_moving_{true};
  int recovery_begin_seq_{0};
  // --- トレース(3層) ---
  void traceInit();
  void traceSample(const rclcpp::Time & now);
  void detectReverseDeadlock(const rclcpp::Time & now);
  void detectUnexpectedReverse(const rclcpp::Time & now, const char * trigger);
  void traceDump(const rclcpp::Time & now, const char * kind);
  std::string traceHeader() const;
  std::string traceLine(const rclcpp::Time & now);
  static constexpr std::size_t kRingMax = 200;   // 20Hz x 10秒
  bool trace_ready_{false};
  std::string trace_dir_;
  std::ofstream trace_ofs_;
  std::deque<std::string> ring_;
  rclcpp::Time last_trace_write_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trace_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trace_ref_t_{0, 0, RCL_ROS_TIME};
  double trace_ref_x_{0.0}, trace_ref_y_{0.0}, trace_moved_1s_{0.0};
  unsigned long trace_cb_seq_{0};
  std::optional<rclcpp::Time> reverse_deadlock_since_;
  bool reverse_deadlock_active_{false};
  int reverse_deadlock_count_{0};
  std::optional<rclcpp::Time> last_reverse_deadlock_at_;
  double reverse_deadlock_x_{0.0}, reverse_deadlock_y_{0.0};
  double last_reverse_deadlock_x_{0.0}, last_reverse_deadlock_y_{0.0};
  double recovery_start_clear_{9.99}; // 復帰開始時の壁への食い込み
  // 直前に復帰を始めた時刻と場所。同じ詰まりの続きかを判断して、
  // 伸ばした後退の下限を引き継ぐために使う。
  rclcpp::Time last_begin_time_{0, 0, RCL_ROS_TIME};
  double last_begin_x_{0.0};
  double last_begin_y_{0.0};
  bool last_begin_valid_{false};
  bool makePlan(const rclcpp::Time & now, int first_phase = 0);
  // 復帰経路が避けるべき近傍の他車。壁しか見ていなかった問題への対応。
  std::vector<recovery::CarObstacle> carObstacles() const;

  struct OppTrack
  {
    double x{0.0};
    double y{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double vx{0.0};          // なました速度[m/s]
    double vy{0.0};
    bool moving{false};      // ヒステリシス付きの判定
    bool init{false};
    double last_yaw{0.0};        // 最後に動いていたときの進行方向[rad]
    bool last_yaw_valid{false};
    rclcpp::Time stopped_since{0, 0, RCL_ROS_TIME};   // 止まった時刻
    bool stopped_since_valid{false};
  };
  mutable std::map<std::string, OppTrack> opp_track_;
  void updateOpponentTracks() const;
  // 区間を始めた時点の余裕[m]。「悪化したときだけ降りる」判定に使う。
  // 入口の停滞判定用。実際に進んだ距離で見るための基準点。
  double entry_ref_x_{0.0};
  double entry_ref_y_{0.0};
  rclcpp::Time entry_ref_time_{0, 0, RCL_ROS_TIME};
  bool entry_ref_valid_{false};
  double phase_start_wall_clear_{1e3};
  double phase_start_car_clear_{1e3};
  // 採用した計画が前進区間で見込んだ最小余裕[m]。中断閾値をこれに合わせて
  // 「計画が通した幾何を中断側が落とす」食い違いを無くすために持つ。
  // 計画が取れなかったときは 1e9 のまま(=中断閾値は既定値を使う)。
  double plan_min_wall_clear_{1e9};
  double plan_min_car_clear_{1e9};
  void beginRecovery(const rclcpp::Time & now, bool forward_blocked = false,
                     bool was_active = false);
  void finishRecovery(const rclcpp::Time & now, const char * why = "不明");
  bool currentPose(recovery::Pose & p) const;
};

}  // namespace stuck_recovery_controller

#endif  // STUCK_RECOVERY_CONTROLLER_HPP_
