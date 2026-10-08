#include "chassis_control/chassis_control.h"


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<ChassisControlNode>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}