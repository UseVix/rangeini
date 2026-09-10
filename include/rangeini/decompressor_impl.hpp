#include "rangeini/decompressor.hpp"



using BufferView = Span<uint8_t>;
using ConstBufferView = Span<const uint8_t>;
namespace {
  std::unique_ptr<Cloudini::FieldDecoder> CreateDecoder(Cloudini::FieldType type, size_t offset) {
    switch (type) {
      case Cloudini::FieldType::INT8: return std::make_unique<Cloudini::FieldDecoderInt<int8_t>>(offset);
      case Cloudini::FieldType::UINT8: return std::make_unique<Cloudini::FieldDecoderInt<uint8_t>>(offset);
      case Cloudini::FieldType::INT16: return std::make_unique<Cloudini::FieldDecoderInt<int16_t>>(offset);
      case Cloudini::FieldType::UINT16: return std::make_unique<Cloudini::FieldDecoderInt<uint16_t>>(offset);
      case Cloudini::FieldType::INT32: return std::make_unique<Cloudini::FieldDecoderInt<int32_t>>(offset);
      case Cloudini::FieldType::UINT32: return std::make_unique<Cloudini::FieldDecoderInt<uint32_t>>(offset);
      case Cloudini::FieldType::INT64: return std::make_unique<Cloudini::FieldDecoderInt<int64_t>>(offset);
      case Cloudini::FieldType::UINT64: return std::make_unique<Cloudini::FieldDecoderInt<uint64_t>>(offset);
      default: throw std::runtime_error("Unsupported field type for decoder");
    }
  }
}
template <typename MsgT>
Decompressor<MsgT>::Decompressor(CompressorConfig config)
: Node("decompressor"),
  config_(std::move(config))
{
  for (const auto& field : config_.scan_fields) {
    scan_decoders.push_back(CreateDecoder(field.first, field.second));
  }
  for (const auto& field : config_.point_fields) {
    point_decoders.push_back(CreateDecoder(field.first, field.second));
  }

  publisher_ = this->create_publisher<MsgT>("/lidar_packets_decompressed", 10);
  
  is_loaning_available = publisher_->can_loan_messages();
  if (is_loaning_available) {
    RCLCPP_INFO(this->get_logger(), "Success: Message is plain. Zero-copy enabled!");
  } else {
    RCLCPP_WARN(this->get_logger(), "Message is NOT plain (or middleware lacks support). Falling back to standard copy.");
  }
  
  subscription_ = this->create_subscription<rangeini::msg::CompressedLidarPackets>(
    "/compressed_lidar_packets", 10, std::bind(&Decompressor::compressed_msg_callback, this, std::placeholders::_1));
    
  compressed_packets_buffer.resize(config_.max_buffer_size);
  compressed_packets_buffer_swap.resize(config_.max_buffer_size);
  if (config_.info_.use_threads) {
    decompressing_thread_ = std::thread(&Decompressor::DecompressAndPublishParallelised, this);
  }
}

template <typename MsgT>
Decompressor<MsgT>::~Decompressor() {
  if (decompressing_thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      should_exit_ = true;
    }
    cv_ready_to_decompress_.notify_one();
    decompressing_thread_.join();
  }
}
template <typename MsgT>
void Decompressor<MsgT>::PacketDecompression(ConstBufferView &input_view, BufferView &output_view)
{
  memcpy(output_view.data(), input_view.data(), config_.initial_offset);
  output_view.trim_front(config_.initial_offset);
  input_view.trim_front(config_.initial_offset);

  for (int block_idx = 0; block_idx < config_.block_count; ++block_idx) {
    for (auto& decoder : scan_decoders) {
      decoder->decode(input_view, output_view);
    }

    output_view.trim_front(config_.per_block_offset);

    for (int i = 0 ; i < config_.channel_count ; i++){
      for (auto& decoder : point_decoders) {
        decoder->decode(input_view, output_view);
      }
      output_view.trim_front(config_.point_size);
    }
  }

  memcpy(output_view.data(), input_view.data(), config_.tail_data_size);
  output_view.trim_front(config_.tail_data_size);
  input_view.trim_front(config_.tail_data_size);
}
template <typename MsgT>
void Decompressor<MsgT>::MessageDecompression(const rangeini::msg::CompressedLidarPackets& msg) {
  const uint8_t* chunk_size_ptr = &(msg.data[0]);
  uint32_t chunk_size = 0;
  memcpy(&chunk_size, chunk_size_ptr, sizeof(uint32_t));

  const char* src = reinterpret_cast<const char*>(chunk_size_ptr + sizeof(uint32_t));
  char* dst = reinterpret_cast<char*>(compressed_packets_buffer.data());

  if (config_.info_.compression_opt == Cloudini::CompressionOption::LZ4) {
    int decompressed_size = LZ4_decompress_safe(src, dst, chunk_size, config_.max_buffer_size);
    if (decompressed_size < 0) {
      throw std::runtime_error("LZ4 decompression failed");
    }
  } else {
        size_t ds = ZSTD_decompress(dst, config_.max_buffer_size, src, chunk_size);
    if (ZSTD_isError(ds)) {
      throw std::runtime_error("ZSTD decompression failed");
    }
  }

}
template <typename MsgT>
void Decompressor<MsgT>::DecompressAndPublishParallelised() {
  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_ready_to_decompress_.wait(lock, [this] { return has_data_to_decompress_ || should_exit_; });

        if (should_exit_) {
          break;
        }
        has_data_to_decompress_ = false;
      }
      
      DecompressAndPublish(compressed_packets_buffer_swap, packet_count_swap, header_swap);
      
      {
        std::lock_guard<std::mutex> lock(mutex_);
        decompression_done_ = true;
      }

      cv_done_decompressing_.notify_one();
    }
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_failed_ = true;
      worker_exception_ = std::current_exception();
    }
    cv_done_decompressing_.notify_all();
  }
}

template <typename MsgT>
void Decompressor<MsgT>::waitForDecompressionComplete() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_done_decompressing_.wait(lock, [this] { return decompression_done_ || worker_failed_; });
  if (worker_failed_) {
    std::rethrow_exception(worker_exception_);
  }
}
template <typename MsgT>
void Decompressor<MsgT>::DecompressAndPublishSwitching(const uint32_t& packet_count, const std_msgs::msg::Header& published_header) {
        if (config_.info_.use_threads) {
    waitForDecompressionComplete();
    {
      std::unique_lock<std::mutex> lock(mutex_);
      std::swap(compressed_packets_buffer, compressed_packets_buffer_swap);
      header_swap = published_header;
      packet_count_swap = packet_count;
      decompression_done_ = false;
      has_data_to_decompress_ = true;
    }
    cv_ready_to_decompress_.notify_one();
  } else {
    DecompressAndPublish(compressed_packets_buffer, packet_count, published_header);
  }
}
template <typename MsgT>
void Decompressor<MsgT>::compressed_msg_callback(const rangeini::msg::CompressedLidarPackets::SharedPtr msg) {
  MessageDecompression(*msg);
  DecompressAndPublishSwitching(msg->packet_count, msg->header);
}






