#include "ten_debug_rviz_plugin/layers_panel.hpp"

#include <QFont>
#include <QVBoxLayout>
#include <memory>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace ten_debug_rviz_plugin
{
LayersPanel::LayersPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;
  QFont f("Monospace");
  f.setStyleHint(QFont::TypeWriter);
  f.setPointSize(9);
  plan_ = new QLabel(QString::fromStdString(plan_text_));
  ctrl_ = new QLabel(QString::fromStdString(ctrl_text_));
  for (QLabel * l : {plan_, ctrl_}) {
    l->setFont(f);
    l->setTextInteractionFlags(Qt::TextSelectableByMouse);
    l->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    l->setWordWrap(false);
    layout->addWidget(l);
  }
  layout->addStretch();
  setLayout(layout);
}

void LayersPanel::onInitialize()
{
  // rviz の実行ノードとは別に、このパネル専用のノードを持つ(rviz の版差に依存しない)。
  node_ = std::make_shared<rclcpp::Node>("ten_layers_panel");
  plan_stamp_ = node_->now();
  ctrl_stamp_ = node_->now();
  plan_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/planning/debug/v2x_layers", rclcpp::QoS(1),
    [this](const std_msgs::msg::String::SharedPtr m) {
      plan_text_ = m->data;
      plan_stamp_ = node_->now();
    });
  ctrl_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/control/debug/recovery_layers", rclcpp::QoS(1),
    [this](const std_msgs::msg::String::SharedPtr m) {
      ctrl_text_ = m->data;
      ctrl_stamp_ = node_->now();
    });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &LayersPanel::tick);
  timer_->start(100);
}

void LayersPanel::tick()
{
  if (!node_) { return; }
  rclcpp::spin_some(node_);
  const auto age = [this](const rclcpp::Time & t) { return (node_->now() - t).seconds(); };
  // 古い情報をそのまま出すと誤解のもとなので、経過秒を添える。
  const auto stale = [](double s) { return s > 1.0 ? QString(" [%1s 前]").arg(s, 0, 'f', 1) : QString(); };
  plan_->setText(QString::fromStdString(plan_text_) + stale(age(plan_stamp_)));
  ctrl_->setText(QString::fromStdString(ctrl_text_) + stale(age(ctrl_stamp_)));
}
}  // namespace ten_debug_rviz_plugin

PLUGINLIB_EXPORT_CLASS(ten_debug_rviz_plugin::LayersPanel, rviz_common::Panel)
