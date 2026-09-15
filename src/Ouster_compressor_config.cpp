#include "rangeini/Ouster_compressor_config.hpp"
#include <chrono>
namespace config_presets {
  CompressorConfig MakeOusterOs0128Config() {
    CompressorConfig config{};
    config.initial_offset = 0;
    config.per_block_offset = 16;
    config.point_size = 12;
    config.channel_count = 128;
    config.block_count = 16;
    config.packet_size = 24896;
    config.tail_data_size = 0;
    config.packets_in_chunk = 64;
    config.chunk_point_size = config.channel_count * config.block_count * config.packets_in_chunk;
    config.max_buffer_size = config.packets_in_chunk * config.packet_size * 3 / 2;
    config.per_block_offset_end = 4;
    config.info_.compression_opt = Cloudini::CompressionOption::ZSTD;
    config.info_.use_threads = false;
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT32, .offset = 0});
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT32, .offset = 4});
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT16, .offset = 8});
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT16, .offset = 10});
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT32, .offset = 12, .mask = 16777215});
    config.point_fields.push_back({.type = Cloudini::FieldType::UINT32, .offset = 0, .mask = 1048575});
    config.point_fields.push_back({.type = Cloudini::FieldType::UINT8, .offset = 4, .mask = 268435456, .merge = true});
    config.point_fields.push_back({.type = Cloudini::FieldType::UINT16, .offset = 6});
    config.point_fields.push_back({.type = Cloudini::FieldType::UINT16, .offset = 8});
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT32, .offset = 28});
    return config;
  }
}

void Ouster_Compressor::lidar_msg_callback(const ouster_sensor_msgs::msg::PacketMsg::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(),
      "PacketCompression DEBUG LOG: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, buf.size()=%zu",
      buffer_view.size(), buffer_size, packet_count, point_count, msg->buf.size());
  try {
    ConstBufferView input_view(msg->buf.data(), msg->buf.size());
    auto t1 = std::chrono::high_resolution_clock::now();
    PacketCompression(input_view);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    RCLCPP_DEBUG(this->get_logger(), "PacketCompression time: %ld ns", total_time);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "PacketCompression FAILED: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, buf.size()=%zu, error: %s",
      buffer_view.size(), buffer_size, packet_count, point_count, msg->buf.size(), e.what());
    throw;
  }
  packet_count += 1;
  if (point_count >= config_.chunk_point_size) {
    std_msgs::msg::Header header;
    header.stamp = this->now();
    auto t1 = std::chrono::high_resolution_clock::now();
    CompressAndPublishSwitching(header);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    RCLCPP_DEBUG(this->get_logger(), "CompressAndPublishSwitching time: %ld ns", total_time);
  }
}


void Ouster_Decompressor::DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) {
  auto t1 = std::chrono::high_resolution_clock::now();
  preallocated_msg.resize(packet_count);
  for (auto& decoder : scan_decoders) {
      decoder->reset();
  }
  for (auto& decoder : point_decoders) {
    decoder->reset();
  }
  ConstBufferView input_view(compressed_packets_buffer);
  if (is_loaning_available) {
    for (uint32_t i = 0; i < packet_count; ++i) {
      auto loaned_msg = publisher_->borrow_loaned_message();
      loaned_msg.get().buf.resize(config_.packet_size);
      BufferView output_view(loaned_msg.get().buf);
      PacketDecompression(input_view,output_view);
      publisher_->publish(std::move(loaned_msg));
    }
    
  } else {
    for (uint32_t i = 0; i < packet_count; ++i) {
      preallocated_msg[i].buf.resize(config_.packet_size);
      BufferView output_view(preallocated_msg[i].buf);
      PacketDecompression(input_view,output_view);
      publisher_->publish(preallocated_msg[i]);
    }
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  auto total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  RCLCPP_DEBUG(this->get_logger(), "DecompressAndPublish time: %ld ns", total_time);
}

