#include "rangeini/Robosense_compressor_config.hpp"
#include <chrono>
namespace config_presets {
  CompressorConfig MakeRobosenseAiryConfig() {
    CompressorConfig config{};
    config.initial_offset = 42;
    config.per_block_offset = 4;
    config.point_size = 3;
    config.channel_count = 48;
    config.block_count = 8;
    config.packet_size = 1248;
    config.tail_data_size = 22;
    config.packets_in_chunk = 225;
    config.chunk_point_size = config.channel_count * config.block_count * config.packets_in_chunk;
    config.max_buffer_size = config.packets_in_chunk * config.packet_size * 10 / 2;
    config.info_.compression_opt = Cloudini::CompressionOption::ZSTD;
    config.info_.use_threads = false;
    config.scan_fields.push_back({.type = Cloudini::FieldType::UINT32, .offset = 0, .big_endian = true});
    config.point_fields.push_back({.type = Cloudini::FieldType::UINT16, .offset = 0, .big_endian = true});
    config.point_fields.push_back({.type = Cloudini::FieldType::UINT8, .offset = 2, .big_endian = true});
    return config;
  }

}
void Robosense_Compressor::lidar_msg_callback(const rslidar_msg::msg::RslidarPacket::SharedPtr msg) {
  if (std::equal(msg->data.begin(), msg->data.begin() + 4, std::initializer_list<uint8_t>{0x55, 0xAA, 0x05, 0x5A}.begin())) {
    if (msg->is_frame_begin==1 && packet_count > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Frame begin detected, compressing and publishing:buffer_size=%zu, packet_count=%u, point_count=%d",
        buffer_size, packet_count, point_count);
      auto t1 = std::chrono::high_resolution_clock::now();
      CompressAndPublishSwitching(msg->header);
      auto t2 = std::chrono::high_resolution_clock::now();
      auto total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      RCLCPP_DEBUG(this->get_logger(), "CompressAndPublishSwitching time: %ld ns", total_time);
    }
    //RCLCPP_DEBUG(this->get_logger(),
    //    "PacketCompression DEBUG LOG: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, data.size()=%zu, is_frame_begin=%d",
    //    buffer_view.size(), buffer_size, packet_count, point_count, msg->data.size(), msg->is_frame_begin);
    try {
      ConstBufferView input_view(msg->data.data(), msg->data.size());
      auto t1 = std::chrono::high_resolution_clock::now();
      PacketCompression(input_view);
      auto t2 = std::chrono::high_resolution_clock::now();
      auto total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      RCLCPP_DEBUG(this->get_logger(), "PacketCompression time: %ld ns", total_time);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
          "PacketCompression FAILED: buffer_view.size()=%zu, buffer_size=%zu, packet_count=%u, point_count=%d, data.size()=%zu, error: %s",
          buffer_view.size(), buffer_size, packet_count, point_count, msg->data.size(), e.what());
      throw;
    }
    packet_count += 1;
  }
}

void Robosense_Decompressor::DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) {
  auto t1 = std::chrono::high_resolution_clock::now();
  if (packet_count > preallocated_msg.size()) {
    preallocated_msg.resize(packet_count);
  }
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
      loaned_msg.get().data.resize(config_.packet_size);
      if (i==0){
        loaned_msg.get().is_frame_begin = 1;
      } else {
        loaned_msg.get().is_frame_begin = 0;
      }
      loaned_msg.get().header = published_header;
      BufferView output_view(loaned_msg.get().data);
      PacketDecompression(input_view,output_view);
      loaned_msg.get().is_difop = 0;
      publisher_->publish(std::move(loaned_msg));
    }
  } else {
    for (uint32_t i = 0; i < packet_count; ++i){
      if (i==0){
        preallocated_msg[i].is_frame_begin = 1;
      } else {
        preallocated_msg[i].is_frame_begin = 0;
      }
      preallocated_msg[i].header = published_header;
      preallocated_msg[i].data.resize(config_.packet_size);
      BufferView output_view(preallocated_msg[i].data);
      PacketDecompression(input_view,output_view);
      preallocated_msg[i].is_difop = 0;
      publisher_->publish(preallocated_msg[i]);
    }
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  auto total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  RCLCPP_DEBUG(this->get_logger(), "DecompressAndPublish time: %ld ns", total_time);
}



