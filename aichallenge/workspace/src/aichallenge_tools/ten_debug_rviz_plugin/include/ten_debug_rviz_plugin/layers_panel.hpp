// 今どの層・処理が軌道と速度に効いているかを rviz のパネル(Displays などが並ぶ側)に出す。
// 3D 画面(地図の上)に文字を重ねると見づらいというユーザー指摘(2026-09-18)への対応。
#ifndef TEN_DEBUG_RVIZ_PLUGIN__LAYERS_PANEL_HPP_
#define TEN_DEBUG_RVIZ_PLUGIN__LAYERS_PANEL_HPP_

#include <QLabel>
#include <QTimer>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace ten_debug_rviz_plugin
{
class LayersPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LayersPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void tick();
  QLabel * plan_{nullptr};      // 横目標・速度上限・状態(v2x_overtaker)
  QLabel * ctrl_{nullptr};      // 最終指令の出どころ(stuck_recovery_controller)
  QTimer * timer_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctrl_sub_;
  std::string plan_text_{"(まだ受信していない: /planning/debug/v2x_layers)"};
  std::string ctrl_text_{"(まだ受信していない: /control/debug/recovery_layers)"};
  rclcpp::Time plan_stamp_;
  rclcpp::Time ctrl_stamp_;
};
}  // namespace ten_debug_rviz_plugin

#endif  // TEN_DEBUG_RVIZ_PLUGIN__LAYERS_PANEL_HPP_
