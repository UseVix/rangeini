#pragma once

#include "rangeini/compressor_config.hpp"
#include "rangeini/compressor.hpp"
#include "rangeini/decompressor.hpp"
#include "ouster_sensor_msgs/msg/packet_msg.hpp"
namespace config_presets {
CompressorConfig MakeOusterOs0128Config();
}
class Ouster_Compressor : public Compressor<ouster_sensor_msgs::msg::PacketMsg> {
 public:
  using Compressor<ouster_sensor_msgs::msg::PacketMsg>::Compressor;

  void lidar_msg_callback(const ouster_sensor_msgs::msg::PacketMsg::SharedPtr msg) override;

};
class Ouster_Decompressor : public Decompressor<ouster_sensor_msgs::msg::PacketMsg> {
 public:
  using Decompressor<ouster_sensor_msgs::msg::PacketMsg>::Decompressor;
  void DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) override;
};
