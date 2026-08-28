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
  const double lookahead_gain_;
  const double lookahead_min_distance_;
  const double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  const double steering_tire_angle_gain_;
  // ラインから離れているときに lookahead を伸ばす係数(横ずれ e に対し e*gain)
  const double lookahead_cte_gain_;
  // 曲率に応じて lookahead を縮める。半径 curve_ref 以上で等倍、
  // それより曲がっているほど curve_min まで縮める
  const double lookahead_curve_ref_;
  const double lookahead_curve_min_;
  // --- 低速でだけ曲率に応じて目標点を近づける(ユーザー指示)
  // 基準速度以上では一切変更しない。以下では曲率と現在速度で lookahead に上限を掛ける。
  const double lookahead_slow_speed_;   // これ[m/s]以上なら従来どおり
  const double lookahead_slow_full_;    // これ[m/s]以下で効果100%
  const double lookahead_curve_k_;      // 上限 = 曲率半径 x この係数
  const double lookahead_slow_min_;     // 縮めすぎ防止の下限[m]
  const double lookahead_slow_exp_;     // 低速での縮め方の鋭さ(指数)
  const double lookahead_slow_far_;     // 低速でも直線ならここまで[m]
  // 追い越し試行中は目標点を近づける。横にずらした軌道へ素早く追従させるため。
  const double lookahead_overtake_scale_;
  bool overtaking_{false};
  double overtake_scale_now_{1.0};   // なましてから掛ける
  // 場所を指定して lookahead を縮める区間。"開始:終了:倍率" をカンマ区切り。
  // 曲率で決めると、直したい idx165-185(R7.0m) より きつい
  // idx51-65(R4.0m) 等のほうが強く縮まって左右に発振する。
  const std::string lookahead_zone_spec_;
  struct LookaheadZone { std::size_t from; std::size_t to; double scale; };
  std::vector<LookaheadZone> lookahead_zones_;
  double lookahead_scale_now_{1.0};
  double lookahead_slow_now_{1e9};   // 低速縮小の上限[m]。なましてから使う
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_overtaking_;
  static constexpr double kScaleSmooth = 0.08;   // なまし係数(1周期あたり)
  const double lookahead_curve_ahead_;
  // 自車の少し先の軌道の曲率半径[m]を返す。取れなければ大きな値
  double localTurnRadius(size_t closest_idx) const;
  // 低速時の操舵角制限(スタート時の振られ対策)
  const double start_steer_speed_;   // この速度[m/s]未満で制限を掛ける
  const double start_steer_limit_;   // 停止時の操舵角上限[rad]
  const double stuck_steer_free_speed_;  // これ未満の速度では制限を外す(壁からの脱出用)
  const double max_acceleration_;


 private:
  void onTimer();
  bool subscribeMessageAvailable();
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
