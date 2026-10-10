#include <chrono>
#include <thread>
#include <QApplication>
#include <QComboBox>
#include <QDockWidget>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <auto_parking/msg/parking_status.hpp>
#include <prius_msgs/msg/control.hpp>
#include <std_msgs/msg/string.hpp>

// Panel 不需要渲染器；用真实 ROS 节点和最小 DisplayContext 验证 Qt/ROS 接线。
class Context : public rviz_common::DisplayContext
{
public:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("panel_test");
  Ogre::SceneManager * getSceneManager() const override {return nullptr;}
  rviz_common::WindowManagerInterface * getWindowManager() const override {return nullptr;}
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface> getSelectionManager() const override {return {};}
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface> getHandlerManager() const override {return {};}
  std::shared_ptr<rviz_common::interaction::ViewPickerIface> getViewPicker() const override {return {};}
  rviz_common::FrameManagerIface * getFrameManager() const override {return nullptr;}
  QString getFixedFrame() const override {return "odom";}
  uint64_t getFrameCount() const override {return 0;}
  rviz_common::DisplayFactory * getDisplayFactory() const override {return nullptr;}
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr getRosNodeAbstraction() const override {return ros;}
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}
  rviz_common::ToolManager * getToolManager() const override {return nullptr;}
  rviz_common::ViewManager * getViewManager() const override {return nullptr;}
  rviz_common::transformation::TransformationManager * getTransformationManager() override {return nullptr;}
  rviz_common::DisplayGroup * getRootDisplayGroup() const override {return nullptr;}
  uint32_t getDefaultVisibilityBit() const override {return 0;}
  rviz_common::BitAllocator * visibilityBits() override {return nullptr;}
  void setStatus(const QString &) override {}
  QString getHelpPath() const override {return {};}
  std::shared_ptr<rclcpp::Clock> getClock() override {return ros->get_raw_node()->get_clock();}
  void lockRender() override {}
  void unlockRender() override {}
  void queueRender() override {}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("auto_parking/ParkingPanel");
  QMainWindow window;
  auto * dock = new QDockWidget("ParkingPanel", &window);
  dock->setWidget(panel.get());
  window.addDockWidget(Qt::RightDockWidgetArea, dock);
  Context context;
  panel->initialize(&context);
  auto node = context.ros->get_raw_node();
  auto status_pub = node->create_publisher<auto_parking::msg::ParkingStatus>(
    "/parking/status", rclcpp::QoS(1).transient_local());
  prius_msgs::msg::Control manual;
  std::string request;
  auto manual_sub = node->create_subscription<prius_msgs::msg::Control>(
    "/parking/manual_control", 10, [&manual](prius_msgs::msg::Control::ConstSharedPtr msg) {manual = *msg;});
  auto command_sub = node->create_subscription<std_msgs::msg::String>(
    "/parking/command", 10, [&request](std_msgs::msg::String::ConstSharedPtr msg) {request = msg->data;});
  auto pump = [&]() {
      for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node);
        app.processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    };
  auto * mode = panel->findChild<QComboBox *>("parking_mode");
  auto * start = panel->findChild<QPushButton *>("parking_start");
  bool ok = mode && mode->count() == 2 && start && !start->isEnabled();
  auto_parking::msg::ParkingStatus status;
  status.mode = "MANUAL";
  status.state = "MANUAL_APPROACH";
  status_pub->publish(status);
  pump();
  ok = ok && window.dockWidgetArea(dock) == Qt::LeftDockWidgetArea;
  QKeyEvent press(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier);
  QApplication::sendEvent(panel.get(), &press);
  pump();
  ok = ok && manual.throttle == 0.35 && manual.shift_gears == prius_msgs::msg::Control::FORWARD;
  QKeyEvent release(QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier);
  QApplication::sendEvent(panel.get(), &release);
  pump();
  ok = ok && manual.brake == 1 && manual.shift_gears == prius_msgs::msg::Control::NEUTRAL;
  status.mode = "AUTO";
  status.state = "MANUAL_APPROACH";
  status.ready_to_park = false;
  status_pub->publish(status);
  pump();
  ok = ok && !start->isEnabled();
  status.state = "READY_TO_PARK";
  status.mode = "MANUAL";
  status.entry_distance = 4.8;
  status.activation_radius = 6.0;
  status.ready_to_park = true;
  status_pub->publish(status);
  pump();
  auto * distance = panel->findChild<QLabel *>("parking_distance");
  ok = ok && start->isEnabled() && mode->currentIndex() == 0 && distance &&
    distance->text().contains("后轴到车位边界：4.80 m");
  start->click();
  ok = ok && !start->isEnabled();
  pump();
  ok = ok && request == "start";
  manual.throttle = 0;
  QApplication::sendEvent(panel.get(), &press);
  pump();
  ok = ok && manual.throttle == 0;
  status.mode = "AUTO";
  status.state = "AUTO_PARKING";
  status.ready_to_park = false;
  status_pub->publish(status);
  pump();
  ok = ok && mode->currentIndex() == 1 && !start->isEnabled();
  status.state = "HOLD_FOR_MANUAL";
  status.ready_to_park = false;
  status_pub->publish(status);
  pump();
  ok = ok && !start->isEnabled();
  QMetaObject::invokeMethod(mode, "activated", Q_ARG(int, 0));
  pump();
  ok = ok && request == "manual";
  panel->setParent(nullptr);  // pluginlib shared_ptr 管理 Panel，避免 dock 再次删除。
  panel.reset();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
