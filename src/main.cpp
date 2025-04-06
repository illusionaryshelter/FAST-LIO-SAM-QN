#include "fast_lio_sam_qn.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastLioSamQn>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  
  return 0;
}
