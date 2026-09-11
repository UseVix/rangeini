#pragma once

#include "rangeini/compressor_config.hpp"
#include "rangeini/compressor.hpp"
#include "rangeini/decompressor.hpp"
#include "hesai_ros_driver/msg/udp_frame.hpp"
namespace config_presets {
CompressorConfig MakeHesaiXT32Config();
CompressorConfig MakeHesaiJT128Config();
}
class Hesai_Compressor : public Compressor<hesai_ros_driver::msg::UdpFrame> {
 public:
  using Compressor<hesai_ros_driver::msg::UdpFrame>::Compressor;

  void lidar_msg_callback(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg) override;

};
class Hesai_Decompressor : public Decompressor<hesai_ros_driver::msg::UdpFrame> {
 public:
  using Decompressor<hesai_ros_driver::msg::UdpFrame>::Decompressor;
  void PacketsDecompression(std::vector<uint8_t>& compressed_packets_buffer, hesai_ros_driver::msg::UdpFrame& msg_ref, const uint32_t& packet_count, const std_msgs::msg::Header& published_header);
  void DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) override;
};
