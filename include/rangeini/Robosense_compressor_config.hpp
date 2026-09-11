#pragma once

#include "rangeini/compressor_config.hpp"
#include "rangeini/compressor.hpp"
#include "rangeini/decompressor.hpp"
#include "rslidar_msg/msg/rslidar_packet.hpp"
namespace config_presets {
CompressorConfig MakeRobosenseAiryConfig();
}
class Robosense_Compressor : public Compressor<rslidar_msg::msg::RslidarPacket> {
 public:
  using Compressor<rslidar_msg::msg::RslidarPacket>::Compressor;

  void lidar_msg_callback(const rslidar_msg::msg::RslidarPacket::SharedPtr msg) override;

};
class Robosense_Decompressor : public Decompressor<rslidar_msg::msg::RslidarPacket> {
 public:
  using Decompressor<rslidar_msg::msg::RslidarPacket>::Decompressor;
  void DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) override;
};
