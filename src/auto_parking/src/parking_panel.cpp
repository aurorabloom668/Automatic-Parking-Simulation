#include "auto_parking/parking_panel.hpp"

#include <algorithm>
#include <cmath>
#include <QComboBox>
#include <QDockWidget>
#include <QEvent>
#include <QGridLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace auto_parking
{
using Control = prius_msgs::msg::Control;

ParkingPanel::ParkingPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  auto * layout = new QVBoxLayout(this);
  mode_ = new QComboBox(this);
  mode_->setObjectName("parking_mode");
  mode_->addItems({"手动驾驶", "自动泊车"});
  layout->addWidget(mode_);

  manual_controls_ = new QWidget(this);
  auto * buttons = new QGridLayout(manual_controls_);
  struct Button { const char * label; int row; int column; double gas; double steer; uint8_t gear; double brake; };
  const Button commands[] = {
    {"前进左 Q", 0, 0, throttle_, turn_, Control::FORWARD, 0},
    {"前进 W", 0, 1, throttle_, 0, Control::FORWARD, 0},
    {"前进右 E", 0, 2, throttle_, -turn_, Control::FORWARD, 0},
    {"左 A", 1, 0, 0, turn_, Control::NO_COMMAND, 0},
    {"刹车 S", 1, 1, 0, 0, Control::NEUTRAL, 1},
    {"右 D", 1, 2, 0, -turn_, Control::NO_COMMAND, 0},
    {"倒车左 Z", 2, 0, throttle_, turn_, Control::REVERSE, 0},
    {"倒车 X", 2, 1, throttle_, 0, Control::REVERSE, 0},
    {"倒车右 C", 2, 2, throttle_, -turn_, Control::REVERSE, 0},
  };
  for (const auto & command : commands) {
    auto * button = new QPushButton(command.label, manual_controls_);
    buttons->addWidget(button, command.row, command.column);
    connect(button, &QPushButton::pressed, this, [this, command]() {
      setDrive(command.gas == 0 ? 0 : throttle_, command.steer == 0 ? 0 : std::copysign(turn_, command.steer),
        command.gear, command.brake);
    });
    connect(button, &QPushButton::released, this, [this]() { stopManual(); });
  }
  manual_controls_->setEnabled(false);
  layout->addWidget(manual_controls_);
  auto * help = new QLabel("按住按钮或按键驾驶，松开即刹车。\n键盘焦点需在面板；空格刹车。\nR/F 调油门，T/G 调转向。", this);
  help->setWordWrap(true);
  layout->addWidget(help);
  slot_ = new QLabel("未选择车位", this);
  distance_ = new QLabel("车位边界距离：—", this);
  distance_->setObjectName("parking_distance");
  stage_ = new QLabel("等待后端状态", this);
  reason_ = new QLabel(this);
  reason_->setWordWrap(true);
  metrics_ = new QLabel(this);
  metrics_->setWordWrap(true);
  for (auto * label : {slot_, distance_, stage_, reason_, metrics_}) {
    layout->addWidget(label);
  }
  start_ = new QPushButton("开始自动泊车", this);
  start_->setObjectName("parking_start");
  start_->setEnabled(false);
  auto * cancel = new QPushButton("取消", this);
  auto * estop = new QPushButton("急停", this);
  estop->setStyleSheet("background: #b52b2b; color: white; font-weight: bold;");
  layout->addWidget(start_);
  layout->addWidget(cancel);
  layout->addWidget(estop);
  layout->addStretch();
  connect(mode_, QOverload<int>::of(&QComboBox::activated), this, [this](int index) {
    stopManual();
    manual_enabled_ = false;
    manual_controls_->setEnabled(false);
    start_->setEnabled(false);
    sendRequest(index == 0 ? "manual" : "auto");
  });
  connect(start_, &QPushButton::clicked, this, [this]() {
    stopManual();
    manual_enabled_ = false;
    manual_controls_->setEnabled(false);
    start_->setEnabled(false);
    sendRequest("start");
  });
  connect(cancel, &QPushButton::clicked, this, [this]() { stopManual(); sendRequest("cancel"); });
  connect(estop, &QPushButton::clicked, this, [this]() { stopManual(); sendRequest("estop"); });
  installEventFilter(this);
  for (auto * widget : findChildren<QWidget *>()) {
    widget->installEventFilter(this);
  }
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { publishManual(); });
  stopManual();
}

void ParkingPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  manual_pub_ = node_->create_publisher<Control>("/parking/manual_control", 10);
  request_pub_ = node_->create_publisher<std_msgs::msg::String>("/parking/command", 10);
  status_sub_ = node_->create_subscription<msg::ParkingStatus>(
    "/parking/status", rclcpp::QoS(1).transient_local(),
    [this](msg::ParkingStatus::ConstSharedPtr status) {
      QMetaObject::invokeMethod(this, [this, status]() { updateStatus(*status); }, Qt::QueuedConnection);
    });
  timer_->start(50);
  // RViz 添加面板后再定位到左侧原生停靠区。
  QTimer::singleShot(0, this, [this]() {
    auto * dock = qobject_cast<QDockWidget *>(parentWidget());
    if (dock) {
      auto * window = qobject_cast<QMainWindow *>(dock->parentWidget());
      if (window) {
        dock->setFloating(false);
        window->addDockWidget(Qt::LeftDockWidgetArea, dock);
      }
    }
  });
}

void ParkingPanel::sendRequest(const std::string & command)
{
  if (!request_pub_) {return;}
  std_msgs::msg::String msg;
  msg.data = command;
  request_pub_->publish(msg);
}

void ParkingPanel::setDrive(double throttle, double steer, uint8_t gear, double brake)
{
  if (!manual_enabled_) {return;}
  manual_.throttle = throttle;
  manual_.steer = steer;
  manual_.shift_gears = gear;
  manual_.brake = brake;
  publishManual();
}

void ParkingPanel::stopManual()
{
  active_key_ = 0;
  manual_.throttle = manual_.steer = 0;
  manual_.brake = 1;
  manual_.shift_gears = Control::NEUTRAL;
  publishManual();
}

void ParkingPanel::publishManual()
{
  if (manual_pub_ && manual_enabled_) {
    manual_.header.stamp = node_->now();
    manual_pub_->publish(manual_);
  }
}

void ParkingPanel::updateStatus(const msg::ParkingStatus & status)
{
  QSignalBlocker blocker(mode_);
  mode_->setCurrentIndex(status.mode == "AUTO" ? 1 : 0);
  const bool driving = status.state == "MANUAL" || status.state == "SLOT_SELECTED" ||
    status.state == "MANUAL_APPROACH" || status.state == "READY_TO_PARK";
  const bool enabled = status.mode == "MANUAL" && driving;
  if (manual_enabled_ != enabled) {stopManual();}
  manual_enabled_ = enabled;
  manual_controls_->setEnabled(enabled);
  start_->setEnabled(status.ready_to_park && status.state == "READY_TO_PARK");
  slot_->setText(status.selected_slot == 0 ? "未选择车位" :
    QString("车位 %1 · %2 · %3").arg(status.selected_slot)
    .arg(QString::fromStdString(status.slot_label)).arg(status.occupied ? "占用" : "空闲"));
  distance_->setText(QString("后轴到车位边界：%1 m / 附近范围 %2 m")
    .arg(status.entry_distance, 0, 'f', 2).arg(status.activation_radius, 0, 'f', 1));
  stage_->setText(QString("%1\n%2").arg(QString::fromStdString(status.state), QString::fromStdString(status.phase)));
  reason_->setText(QString::fromStdString(status.reason));
  metrics_->setText(QString("路径 %1 m · 最小间隙 %2 m\n纵向 %3 m · 横向 %4 m\n航向 %5° · 车身%6 · 纠偏 %7")
    .arg(status.path_length, 0, 'f', 2).arg(status.min_clearance, 0, 'f', 2)
    .arg(status.longitudinal_error, 0, 'f', 3).arg(status.lateral_error, 0, 'f', 3)
    .arg(status.yaw_error * 180.0 / 3.141592653589793, 0, 'f', 2)
    .arg(status.body_inside ? "完整在位" : "未完整在位").arg(status.adjustments));
}

bool ParkingPanel::keyCommand(int key)
{
  switch (key) {
    case Qt::Key_W: setDrive(throttle_, 0, Control::FORWARD, 0); break;
    case Qt::Key_Q: setDrive(throttle_, turn_, Control::FORWARD, 0); break;
    case Qt::Key_E: setDrive(throttle_, -turn_, Control::FORWARD, 0); break;
    case Qt::Key_X: setDrive(throttle_, 0, Control::REVERSE, 0); break;
    case Qt::Key_Z: setDrive(throttle_, turn_, Control::REVERSE, 0); break;
    case Qt::Key_C: setDrive(throttle_, -turn_, Control::REVERSE, 0); break;
    case Qt::Key_A: setDrive(0, turn_, Control::NO_COMMAND, 0); break;
    case Qt::Key_D: setDrive(0, -turn_, Control::NO_COMMAND, 0); break;
    case Qt::Key_S:
    case Qt::Key_Space: stopManual(); break;
    case Qt::Key_R: throttle_ = std::min(1.0, throttle_ * 1.1); return true;
    case Qt::Key_F: throttle_ = std::max(0.05, throttle_ * 0.9); return true;
    case Qt::Key_T: turn_ = std::min(1.0, turn_ * 1.1); return true;
    case Qt::Key_G: turn_ = std::max(0.1, turn_ * 0.9); return true;
    default: return false;
  }
  active_key_ = key;
  return true;
}

bool ParkingPanel::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == QEvent::KeyPress && manual_enabled_) {
    auto * key = static_cast<QKeyEvent *>(event);
    if (!key->isAutoRepeat() && keyCommand(key->key())) {return true;}
  } else if (event->type() == QEvent::KeyRelease) {
    auto * key = static_cast<QKeyEvent *>(event);
    if (!key->isAutoRepeat() && key->key() == active_key_) {stopManual(); return true;}
  } else if (event->type() == QEvent::FocusOut) {
    stopManual();
  }
  return rviz_common::Panel::eventFilter(object, event);
}
}  // namespace auto_parking

PLUGINLIB_EXPORT_CLASS(auto_parking::ParkingPanel, rviz_common::Panel)
