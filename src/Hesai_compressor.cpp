#include "rangeini/compressor.hpp"
#include "rangeini/Hesai_compressor_config.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hesai_Compressor>(config_presets::MakeHesaiXT32Config());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}