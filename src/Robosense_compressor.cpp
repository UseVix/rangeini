#include "rangeini/compressor.hpp"
#include "rangeini/Robosense_compressor_config.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Robosense_Compressor>(config_presets::MakeRobosenseAiryConfig());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}