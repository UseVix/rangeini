#include "rangeini/Hesai_compressor_config.hpp"
namespace config_presets {
  CompressorConfig MakeHesaiXT32Config() {
    CompressorConfig config{};
    config.initial_offset = 12;
    config.per_block_offset = 2;
    config.point_size = 4;
    config.channel_count = 32;
    config.block_count = 8;
    config.packet_size = 1080;
    config.tail_data_size = 28;
    config.packets_in_chunk = 500;
    config.chunk_point_size = config.channel_count * config.block_count * config.packets_in_chunk;
    config.max_buffer_size = config.packets_in_chunk * config.packet_size * 3 / 2;
    config.info_.compression_opt = Cloudini::CompressionOption::LZ4;
    config.info_.use_threads = true;
    config.scan_fields.push_back({Cloudini::FieldType::UINT16, 0});
    config.point_fields.push_back({Cloudini::FieldType::UINT16, 0});
    config.point_fields.push_back({Cloudini::FieldType::UINT8, 2});
    return config;
  }
  CompressorConfig MakeHesaiJT128Config() {
    CompressorConfig config{};
    config.initial_offset = 12;
    config.per_block_offset = 2;
    config.point_size = 4;
    config.channel_count = 128;
    config.block_count = 2;
    config.packet_size = 1100;
    config.tail_data_size = 56;
    config.packets_in_chunk = 500;
    config.chunk_point_size = config.channel_count * config.block_count * config.packets_in_chunk;
    config.max_buffer_size = config.packets_in_chunk * config.packet_size * 3 / 2;
    config.info_.compression_opt = Cloudini::CompressionOption::LZ4;
    config.info_.use_threads = true;
    config.scan_fields.push_back({Cloudini::FieldType::UINT16, 0});
    config.point_fields.push_back({Cloudini::FieldType::UINT16, 0});
    config.point_fields.push_back({Cloudini::FieldType::UINT8, 2});
    config.point_fields.push_back({Cloudini::FieldType::UINT8, 3});
    return config;
  }
}

void Hesai_Compressor::lidar_msg_callback(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg) {
  for (size_t p = 0; p < msg->packets.size(); p++) {
    const auto& packet = msg->packets[p];
    RCLCPP_DEBUG(this->get_logger(),
      "Before packet %zu/%zu: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, packet.data.size()=%zu",
      p, msg->packets.size(), buffer_view.size(), buffer_size, packet_count, point_count, packet.data.size());
    try {
      ConstBufferView input_view(packet.data.data(), packet.data.size());
      PacketCompression(input_view);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "PacketCompression FAILED at packet %zu/%zu: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, packet.data.size()=%zu, error: %s",
        p, msg->packets.size(), buffer_view.size(), buffer_size, packet_count, point_count, packet.data.size(), e.what());
      throw;
    }
  }
  packet_count += msg->packets.size();
  CompressAndPublishSwitching(msg->header);
}

void Hesai_Decompressor::PacketsDecompression(std::vector<uint8_t>& compressed_packets_buffer, hesai_ros_driver::msg::UdpFrame& msg_ref, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) {
  // This is lidar dependent
  msg_ref.header = published_header;
  msg_ref.packets.resize(packet_count);
  // dependency ends here
  for (auto& decoder : scan_decoders) {
    decoder->reset();
  }
  for (auto& decoder : point_decoders) {
    decoder->reset();
  }
  ConstBufferView input_view(compressed_packets_buffer);
  for (auto& packet : msg_ref.packets) {
    // This is lidar dependent
    packet.data.resize(config_.packet_size);
    BufferView output_view(packet.data);
    // dependency ends here
    PacketDecompression(input_view,output_view);
    // This is lidar dependent
    packet.size = config_.packet_size;
    packet.stamp = this->get_clock()->now();
    // dependency ends here
  }
}
void Hesai_Decompressor::DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) {
  if (is_loaning_available) {
    auto loaned_msg = publisher_->borrow_loaned_message();
    PacketsDecompression(compressed_packets_buffer, loaned_msg.get(),packet_count,published_header);
    publisher_->publish(std::move(loaned_msg));
  } else {
    preallocated_msg.resize(1);
    PacketsDecompression(compressed_packets_buffer, preallocated_msg[0],packet_count,published_header);
    publisher_->publish(preallocated_msg[0]);
  }
}

