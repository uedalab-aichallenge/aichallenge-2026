#ifndef STUCK_RECOVERY_CONTROLLER_HPP_
#define STUCK_RECOVERY_CONTROLLER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "stuck_recovery_controller/recovery_planner.hpp"

#include <string>
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
  void publishCommand(float speed, float acceleration, float steer = 0.0f);
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
  // 復帰の前進区間を軌道として publish する。戻せたら true。
  bool publishRecoveryTrajectory();
  // 自車の横位置とその地点のコリドア境界を返す。コリドアが無ければ false。
  bool lateralNow(double & lat, double & lo, double & hi);
  // コース方位に対する車体の向きのずれ[rad]。壁へ突き刺さった判定に使う
  bool headingErrorToTrack(double & err);
  // 通常制御(pure_pursuit)に返したあと、実際にたどる経路が壁に当たらないか。
  //
  // 以前は「今の向きのまま真っ直ぐ dist[m] 進めるか」だけを見ていた。
  // しかし pure_pursuit はカーブの先の目標点へ向けて曲がるので、
  // 目の前が直線でも曲がった先で壁に当たる。
  // カーブ手前で復帰したときに再接触するのはこれが原因だった。
  // ここでは pure_pursuit の追従則をそのまま模擬して経路を作り、
  // その全体で車体が壁に触れないことを確かめる。
  bool handbackPathClear(double dist) const;
  bool canSteerAround();
  float avoidSteerDirection();
  void publishGear(std::uint8_t command);
  // 指令速度の符号にギアを合わせる(出口の不変条件)。
  void alignGearToSpeed(float speed);

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr control_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr nominal_sub_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr reverse_sub_;
  AckermannControlCommand::ConstSharedPtr reverse_cmd_;
  // 後退区間を経路で渡している最中か。true の間は後退用 pure_pursuit を中継する。
  bool reverse_following_{false};
  bool reverse_traj_enable_{true};
  bool wedge_backward_first_{true};   // 壁へ食い込んでいたら後退から始める

  // --- 壁へ食い込んだまま前進し続けることを禁じる(絶対の不変条件) ---
  //
  // 【なぜ絶対にするか】運営アナウンス(2026-09-03):
  //   「実機はシミュレーションと違い、一度壁にぶつかったまま
  //     アクセルを踏み続けると再起不能(≒最下位)になります」
  // SIM でも実測で最大の損失だった。決勝条件の4台走行(355秒)で
  // 壁ペナルティが3台合計193秒、うち1台は3回で103秒。
  // wall は接触が続いている間ずっと加算されるので、回数ではなく
  // 「押し付け続けた時間」がそのまま損失になる。
  //
  // publishCommand は唯一の publish 地点なので、そこで止めれば
  // 経路追従・直接制御・最後の手段のどの枝から来ても守れる。
  bool wall_forward_ban_{true};          // 壁へ食い込んだままの前進を禁じるか
  double wall_forward_ban_hold_{0.8};    // 改善しないまま前進を許す時間[s]
  double wall_forward_ban_gain_{0.03};   // 「改善した」とみなす最小の増分[m]
  // 【誤爆の修正 2026-09-05】この不変条件を素通しの publish まで広げたところ、
  // **29km/h で走行中の車のスロットルを切って**コース上に停止させ、
  // 全車の玉突きを招いた(1レースで4台全滅、壁ペナルティ 322/249/166秒。
  // 発火回数 112〜219回、ログは「壁へ -0.10m 食い込んだまま … 指令 8.04m/s -> 0」)。
  //
  // 不変条件の前提は「壁に押し付けられて動けない」状態であって、
  // 走行中に壁を掠めることではない。実際に守りたかった実測ケースは
  // 速度 0.01〜0.03m/s・食い込み 0.40〜0.76m だった。
  // 2つの条件で場面を限定する。
  double wall_forward_ban_speed_{1.0};   // この速度[m/s]未満のときだけ効かせる
  double wall_forward_ban_depth_{0.15};  // この深さ[m]を超えて食い込んだときだけ
  // 止めた後に「後退しながら回転して抜ける」。止めるだけでは姿勢が直らない
  // (実測: 前進を止めた146回に対し復帰の計画が3回しか出ず、壁ペナ236秒)。
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
  std::optional<rclcpp::Time> healthy_wait_since_;
  rclcpp::Time last_queue_wait_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_desperate_hold_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wall_ban_rev_log_{0, 0, RCL_ROS_TIME};
  // 他車と重なる評価の前進計画も却下するか(既定 false。計測で退行したため)
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
  // 復帰の前進区間は、指令を直接作るのではなく「たどってほしい経路」を
  // publish して pure_pursuit に追従させる。そのほうが通常制御への
  // 戻りが連続になり、後退に頼る量も減る。後退は pure_pursuit では
  // 表現できない(ギアと舵の符号が変わる)ので直接制御のまま。
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  Trajectory::ConstSharedPtr latest_traj_;
  // 前進区間を経路で渡している間は、舵と速度を pure_pursuit に任せる。
  bool traj_following_{false};
  // デバッグ表示(GUI)用。復帰が車両へ指令を出している間だけ流す。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Time last_traj_log_{0, 0, RCL_ROS_TIME};
  // 軌道で渡しているのに動かないときの安全弁。
  // pure_pursuit が何らかの理由で動かせない場合、こちらが指令を出さない
  // ままなので車が完全に止まる(実測: 方位差4度・壁まで0.76m で30秒停止)。
  rclcpp::Time traj_still_since_{0, 0, RCL_ROS_TIME};
  bool traj_still_{false};
  bool traj_giveup_{false};
  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_sub_;
  // 実際のギアと舵角。固定時間で待つのをやめ、準備でき次第動き出すために使う。
  // 実測(gearlat2.py)では、指令から実ギアが変わるまで 0.012〜0.099s、
  // 動き出すまで 0.230〜0.273s。置き値の 0.7s は3倍以上待ちすぎだった。
  rclcpp::Subscription<GearReport>::SharedPtr gear_report_sub_;
  rclcpp::Subscription<SteeringReport>::SharedPtr steer_report_sub_;
  // 復帰動作を任意のタイミングで発動させるための入力(動作確認用)。
  // 壁に当たらなくなると復帰が動く場面に出会えず、検証できないため。
  //   ros2 topic pub -1 /debug/force_recovery std_msgs/msg/Bool "{data: true}"
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr crash_sub_;
  bool force_recovery_{false};

  // 復帰経路の計算に使う情報
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  v2x_msgs::msg::V2XVehiclePositionArray::ConstSharedPtr v2x_;
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
  float escape_dir_{0.0f};   // 逃げる向き(+1=左)。後退と前進で共有する
  std::optional<rclcpp::Time> recovery_end_time_;  // 復帰完了時刻(再突入の抑制に使う)

  // --- 理由を問わない最後の安全網(hard stall)---
  // 実測(20260829-181500 d1 レース277s〜): preventRearEnd が上限0を出し続け、
  // 実速度 -0.00m/s のまま **55秒** 一度も復帰へ入らなかった。
  // 入口のゲートが motion_requested(指令速度>=1.0) か blocked_by_vehicle の
  // どちらかを要求しており、指令0かつ前方車の箱に入らない相手だと
  // hasNoProgress を評価する前に return してしまう。
  // ここは指令も他車も見ず、「車体が動いていない」事実だけで拾う。
  double hard_stall_sec_{0.0};    // 0 = 無効(退避スイッチ)
  double hard_stall_dist_{0.8};
  double hard_ref_x_{0.0}, hard_ref_y_{0.0};
  rclcpp::Time hard_ref_time_{0, 0, RCL_ROS_TIME};
  bool hard_ref_valid_{false};

  // 復帰は「計算した経路を走る」方式。
  //
  // 以前は決め打ちの後退時間と舵角で切り返しを繰り返していた。どの向きへ
  // どれだけ切れば出られるかを判断できず、抜けられないまま25秒を使い切ることが
  // 多かった。実測では「復帰 完了」の2秒後に横4.4m・車体111度で再スタックし、
  // そこから53秒を失っている。
  //
  // 今は recovery::plan() が車両運動学と走行可能領域から、後退と前進の
  // 舵角・距離を計算する。ここはその区間を順に実行するだけ。
  recovery::Corridor corridor_;
  // 実際の壁。コリドアは中心線に対する横方向しか持たないので、
  // ヘアピンで鼻先が壁に当たっている状態を表現できない。壁の判定はこちらで行う。
  recovery::ObstacleMap obstacles_;
  recovery::VehicleParams veh_;
  recovery::Plan plan_;
  std::size_t phase_idx_{0};        // 実行中の区間
  double phase_travelled_{0.0};     // その区間で走った距離[m]
  double last_x_{0.0}, last_y_{0.0};
  bool last_valid_{false};
  int replan_count_{0};
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
  // 出口の壁前進禁止が実際に前進を止めたか。
  // 【外部レビュー の最優先指摘 2026-09-05】
  //   「出口が前進を拒否したまま、論理上の区間を FORWARD に残さない」
  //   「出口から直接ギアや後退指令を作らない。後退の方向・舵・ギアは復帰制御が所有する」
  // 出口は拒否したことを**報告するだけ**。それを受けて向きを変えるのは復帰制御。
  bool wall_ban_hit_{false};
  // 計画の通し番号。出口の拒否に反応するのは「1つの計画につき1回だけ」。
  // 拒否は毎周期成立するので、無条件に反応すると毎周期引き直すことになる
  // (実測: 1レースで 引き直し200回・計画207回・完了0回)。
  unsigned long plan_seq_{0};
  unsigned long wall_ban_acted_seq_{0};
  bool wall_ban_acted_{false};
  // 壁に触れたまま前進指令が出ていて動かない状態の計測。
  // 側面だけの接触は食い込みが浅く、壁前進禁止(深さ 0.15m 必要)が発火しない。
  // 理由不問の膠着(hard_stall_sec=4秒)を待つと、その間ずっと壁を押し続ける。
  double wall_contact_stall_sec_{1.5};
  bool wall_stall_valid_{false};
  double wall_stall_x_{0.0}, wall_stall_y_{0.0};
  rclcpp::Time wall_stall_since_{0, 0, RCL_ROS_TIME};
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
  // 一度「待っても無駄」と判断したら、この復帰の間はずっと詰めてよい。
  // 判断のたびに元に戻すと、緩和 -> 待機解除 -> フラグ消滅 -> また元の基準、
  // を繰り返して永久に下がれない(実測: 経過1.0秒のまま無限ループ)。
  bool rear_relaxed_{false};
  double pre_steer_{0.0};
  std::uint8_t gear_report_{0};
  bool gear_report_seen_{false};
  double steer_report_{0.0};
  // 前後を切り替える前に止まりきるための状態。
  // 実測では、後退区間が終わった時点でまだ -0.8m/s で下がっており、
  // そのまま 0.3m 行き過ぎてからギアが入り、同じ舵のまま前進して
  // 後退した円弧をそのまま戻り、同じ壁に当たっていた。
  bool braking_{false};
  rclcpp::Time brake_since_{0, 0, RCL_ROS_TIME};
  double brake_steer_{0.0};
  bool brake_was_forward_{true};
  // 動作確認用。わざと壁へ突っ込ませて復帰動作を発生させる。
  // 走行ラインを壁へずらす方法だと、コリドアも一緒にずれて
  // 復帰の狙い先まで壁の中に入ってしまい、検証にならないため。
  bool crash_me_{false};
  rclcpp::Time crash_since_{0, 0, RCL_ROS_TIME};
  double crash_steer_{0.0};

  // 現在姿勢から復帰経路を計算する。
  // first_phase: 0=任意 / +1=前進から / -1=後退から
  bool makePlan(const rclcpp::Time & now, int first_phase = 0);
  // 復帰経路が避けるべき近傍の他車。壁しか見ていなかった問題への対応。
  std::vector<recovery::CarObstacle> carObstacles() const;
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
  void beginRecovery(const rclcpp::Time & now, bool forward_blocked = false);
  void finishRecovery(const rclcpp::Time & now);
  bool currentPose(recovery::Pose & p) const;
};

}  // namespace stuck_recovery_controller

#endif  // STUCK_RECOVERY_CONTROLLER_HPP_
