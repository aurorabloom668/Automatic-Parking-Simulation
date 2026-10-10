#ifndef AUTO_PARKING__PARKING_PANEL_HPP_
#define AUTO_PARKING__PARKING_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <prius_msgs/msg/control.hpp>
#include <std_msgs/msg/string.hpp>
#include <auto_parking/msg/parking_status.hpp>

class QComboBox;
class QLabel;
class QPushButton;
class QTimer;

namespace auto_parking
{
class ParkingPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ParkingPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  bool eventFilter(QObject * object, QEvent * event) override;

private:
  void sendRequest(const std::string & command);
  void setDrive(double throttle, double steer, uint8_t gear, double brake);
  void stopManual();
  void publishManual();
  void updateStatus(const msg::ParkingStatus & status);
  bool keyCommand(int key);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<prius_msgs::msg::Control>::SharedPtr manual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<msg::ParkingStatus>::SharedPtr status_sub_;
  prius_msgs::msg::Control manual_;
  QComboBox * mode_;
  QWidget * manual_controls_;
  QLabel * slot_;
  QLabel * distance_;
  QLabel * stage_;
  QLabel * reason_;
  QLabel * metrics_;
  QPushButton * start_;
  QTimer * timer_;
  bool manual_enabled_ = false;
  double throttle_ = 0.35;
  double turn_ = 0.7;
  int active_key_ = 0;
};
}  // namespace auto_parking
#endif
