#include "basalt_wrapper/basalt_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<BasaltNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("basalt_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
