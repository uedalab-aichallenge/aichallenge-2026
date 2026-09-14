#include <cstddef>
#include <vector>
#include <string>
#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;  

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;



  // pure pursuit parameters
  const double wheel_base_;
  double lookahead_gain_;
  double lookahead_min_distance_;
  double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  double steering_tire_angle_gain_;
  // ラインから離れているときに lookahead を伸ばす係数(横ずれ e に対し e*gain)
  const double lookahead_cte_gain_;
  // 低速時だけ曲率と速度で lookahead に上限を掛ける。
  // 基準速度以上では変更しない。
  const double lookahead_slow_speed_;   // これ[m/s]以上なら従来どおり
  const double lookahead_curve_k_;      // 上限 = 曲率半径 x この係数
  const double lookahead_slow_min_;     // 縮めすぎ防止の下限[m]
  const double lookahead_slow_exp_;     // 低速での縮め方の鋭さ(指数)
  // 追い越し試行中は目標点を近づける。横にずらした軌道へ素早く追従させるため。
  const double lookahead_overtake_scale_;
  bool overtaking_{false};
  double overtake_scale_now_{1.0};   // なましてから掛ける
  // 場所を指定して lookahead を縮める区間。"開始:終了:倍率" をカンマ区切り。
  // 区間指定は、曲率だけでは不要な区間まで縮めて発振し得るために使う。
  const std::string lookahead_zone_spec_;
  struct LookaheadZone { std::size_t from; std::size_t to; double scale; };
  std::vector<LookaheadZone> lookahead_zones_;
  double lookahead_scale_now_{1.0};
  double lookahead_slow_now_{1e9};   // 低速縮小の上限[m]。なましてから使う
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_overtaking_;
  // 壁ガードの舵角範囲。最後に受け取った値と、受け取った時刻を持つ。
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_steer_limit_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_steer_override_;
  bool steer_limit_valid_{false};        // 一度でも受け取ったか
  double steer_limit_lo_{0.0};           // 右いっぱい側[rad]
  double steer_limit_hi_{0.0};           // 左いっぱい側[rad]
  double steer_limit_flag_{0.0};         // 1.0 なら壁予測が違反を検知している
  rclcpp::Time steer_limit_time_{0, 0, RCL_ROS_TIME};   // 受信時刻
  rclcpp::Time last_steer_override_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_steer_diag_log_{0, 0, RCL_ROS_TIME};
  double prev_requested_steer_{0.0};
  static constexpr double kScaleSmooth = 0.08;   // なまし係数(1周期あたり)
  const double lookahead_curve_ahead_;
  // 自車の少し先の軌道の曲率半径[m]を返す。取れなければ大きな値
  double localTurnRadius(size_t closest_idx) const;
  // 低速時の操舵角制限(スタート時の振られ対策)
  const double start_steer_speed_;   // この速度[m/s]未満で制限を掛ける
  const double start_steer_limit_;   // 停止時の操舵角上限[rad]
  const double stuck_steer_free_speed_;  // これ未満の速度では制限を外す(壁からの脱出用)
  const bool sat_accel_guard_;    // 舵が飽和している間は前へ加速しない
  const double sat_steer_rad_;    // 実舵の上限[rad](実測 0.31 = 18deg)
  const double sat_accel_max_;    // そのときに許す最大加速度
  const double sat_guard_min_speed_; // この速度[m/s]以下ではガードを効かせない
  rclcpp::Time last_sat_log_{0, 0, RCL_ROS_TIME};
  double max_acceleration_;
  // --- 壁ガードによる舵角クランプ(最終手段の安全網)
  // v2x_overtaker が /control/wall_guard/steer_limit へ流す
  // 「壁に当たらない舵角の範囲」で、計算した舵角を最後にクランプする。
  // フェイルオープン: メッセージが来ない / 古い / 無効化されている場合は
  // 一切クランプしない(安全網の不具合で操縦不能になるほうが危険)。
  const bool wall_guard_clamp_enable_;   // 丸ごと無効化する
  const double wall_guard_stale_sec_;    // これ[s]以上古い制限は使わない


 private:
  void onTimer();
  bool subscribeMessageAvailable();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ten_param_cb_;
  double speed_scale_{1.0};   // 走行ラインの目標速度に掛ける倍率(実機で調整)
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
