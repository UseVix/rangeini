#include "rangeini/Ouster_compressor_config.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ouster_Decompressor>(config_presets::MakeOusterOs0128Config());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}