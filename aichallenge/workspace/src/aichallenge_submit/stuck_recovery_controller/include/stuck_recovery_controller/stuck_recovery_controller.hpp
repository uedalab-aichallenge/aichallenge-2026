#ifndef STUCK_RECOVERY_CONTROLLER_HPP_
#define STUCK_RECOVERY_CONTROLLER_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

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

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr control_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr nominal_sub_;
  // 復帰の前進区間は、指令を直接作るのではなく「たどってほしい経路」を
  // publish して pure_pursuit に追従させる。そのほうが通常制御への
  // 戻りが連続になり、後退に頼る量も減る。後退は pure_pursuit では
  // 表現できない(ギアと舵の符号が変わる)ので直接制御のまま。
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  Trajectory::ConstSharedPtr latest_traj_;
  // 前進区間を経路で渡している間は、舵と速度を pure_pursuit に任せる。
  bool traj_following_{false};
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
  void beginRecovery(const rclcpp::Time & now, bool forward_blocked = false);
  void finishRecovery(const rclcpp::Time & now);
  bool currentPose(recovery::Pose & p) const;
};

}  // namespace stuck_recovery_controller

#endif  // STUCK_RECOVERY_CONTROLLER_HPP_
