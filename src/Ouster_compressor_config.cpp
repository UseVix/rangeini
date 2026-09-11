#include "rangeini/Ouster_compressor_config.hpp"
namespace config_presets {
  CompressorConfig MakeOusterOs0128Config() {
    CompressorConfig config{};
    config.initial_offset = 16;
    config.per_block_offset = 2;
    config.point_size = 4;
    config.channel_count = 128;
    config.block_count = 16;
    config.packet_size = 2096;
    config.tail_data_size = 32;
    config.packets_in_chunk = 64;
    config.chunk_point_size = config.channel_count * config.block_count * config.packets_in_chunk;
    config.max_buffer_size = config.packets_in_chunk * config.packet_size * 3 / 2;
    config.info_.compression_opt = Cloudini::CompressionOption::LZ4;
    config.info_.use_threads = true;
    config.point_fields.push_back({Cloudini::FieldType::UINT32, 0});
    config.point_fields.push_back({Cloudini::FieldType::UINT32, 4});
    config.point_fields.push_back({Cloudini::FieldType::UINT16, 8});
    config.point_fields.push_back({Cloudini::FieldType::UINT16, 10});
    config.point_fields.push_back({Cloudini::FieldType::UINT32, 12});
    return config;
  }
}

void Ouster_Compressor::lidar_msg_callback(const ouster_sensor_msgs::msg::PacketMsg::SharedPtr msg) {
  try {
    ConstBufferView input_view(msg->buf.data(), msg->buf.size());
    PacketCompression(input_view);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "PacketCompression FAILED: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, buf.size()=%zu, error: %s",
      buffer_view.size(), buffer_size, packet_count, point_count, msg->buf.size(), e.what());
    throw;
  }
  packet_count += 1;
  std_msgs::msg::Header header;
  header.stamp = this->now();
  CompressAndPublishSwitching(header);
}


void Ouster_Decompressor::DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) {
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
}

