#include "rangeini/compressor.hpp"


namespace {
  std::unique_ptr<Cloudini::FieldEncoder> CreateEncoder(const FieldEncodeConfig& field_config) {
    const auto type = field_config.type;
    const auto offset = field_config.offset;
    const auto mask = field_config.mask;
    if (field_config.copy) {
      return std::make_unique<Cloudini::FieldEncoderCopy>(offset, type);
    }
    if (field_config.mask != std::numeric_limits<int64_t>::max()) {
      switch (type) {
        case Cloudini::FieldType::INT8: return std::make_unique<FieldEncoderIntMasked<int8_t>>(offset, mask);
        case Cloudini::FieldType::UINT8: return std::make_unique<FieldEncoderIntMasked<uint8_t>>(offset, mask);
        case Cloudini::FieldType::INT16: return std::make_unique<FieldEncoderIntMasked<int16_t>>(offset, mask);
        case Cloudini::FieldType::UINT16: return std::make_unique<FieldEncoderIntMasked<uint16_t>>(offset, mask);
        case Cloudini::FieldType::INT32: return std::make_unique<FieldEncoderIntMasked<int32_t>>(offset, mask);
        case Cloudini::FieldType::UINT32: return std::make_unique<FieldEncoderIntMasked<uint32_t>>(offset, mask);
        case Cloudini::FieldType::INT64: return std::make_unique<FieldEncoderIntMasked<int64_t>>(offset, mask);
        case Cloudini::FieldType::UINT64: return std::make_unique<FieldEncoderIntMasked<uint64_t>>(offset, mask);
      }
    }
    if (field_config.big_endian) {
      switch (type) {
        case Cloudini::FieldType::INT8: return std::make_unique<FieldEncoderIntBigEndian<int8_t>>(offset);
        case Cloudini::FieldType::UINT8: return std::make_unique<FieldEncoderIntBigEndian<uint8_t>>(offset);
        case Cloudini::FieldType::INT16: return std::make_unique<FieldEncoderIntBigEndian<int16_t>>(offset);
        case Cloudini::FieldType::UINT16: return std::make_unique<FieldEncoderIntBigEndian<uint16_t>>(offset);
        case Cloudini::FieldType::INT32: return std::make_unique<FieldEncoderIntBigEndian<int32_t>>(offset);
        case Cloudini::FieldType::UINT32: return std::make_unique<FieldEncoderIntBigEndian<uint32_t>>(offset);
        case Cloudini::FieldType::INT64: return std::make_unique<FieldEncoderIntBigEndian<int64_t>>(offset);
        case Cloudini::FieldType::UINT64: return std::make_unique<FieldEncoderIntBigEndian<uint64_t>>(offset);
      }
    }
    switch (type) {
      case Cloudini::FieldType::INT8: return std::make_unique<Cloudini::FieldEncoderInt<int8_t>>(offset);
      case Cloudini::FieldType::UINT8: return std::make_unique<Cloudini::FieldEncoderInt<uint8_t>>(offset);
      case Cloudini::FieldType::INT16: return std::make_unique<Cloudini::FieldEncoderInt<int16_t>>(offset);
      case Cloudini::FieldType::UINT16: return std::make_unique<Cloudini::FieldEncoderInt<uint16_t>>(offset);
      case Cloudini::FieldType::INT32: return std::make_unique<Cloudini::FieldEncoderInt<int32_t>>(offset);
      case Cloudini::FieldType::UINT32: return std::make_unique<Cloudini::FieldEncoderInt<uint32_t>>(offset);
      case Cloudini::FieldType::INT64: return std::make_unique<Cloudini::FieldEncoderInt<int64_t>>(offset);
      case Cloudini::FieldType::UINT64: return std::make_unique<Cloudini::FieldEncoderInt<uint64_t>>(offset);
      default: throw std::runtime_error("Unsupported field type for encoder");
    }
  }
}
template <typename MsgT>
Compressor<MsgT>::Compressor(CompressorConfig config)
: Node("compressor"),
  config_(std::move(config))
{
  for (const auto& field : config_.scan_fields) {
    scan_encoders.push_back(CreateEncoder(field));
  }
  for (const auto& field : config_.point_fields) {
    point_encoders.push_back(CreateEncoder(field));
  }
  publisher_ = this->create_publisher<rangeini::msg::CompressedLidarPackets>("/compressed_lidar_packets", 10);
  is_loaning_available = publisher_->can_loan_messages();
  if (is_loaning_available) {
    RCLCPP_INFO(this->get_logger(), "Success: Message is plain. Zero-copy enabled!");
  } else {
    RCLCPP_WARN(this->get_logger(), "Message is NOT plain (or middleware lacks support). Falling back to standard copy.");
    preallocated_msg.data.reserve(config_.max_buffer_size);
  } 
  subscription_ = this->create_subscription<MsgT>(
    "/lidar_packets", 10, std::bind(&Compressor::lidar_msg_callback, this, std::placeholders::_1));
  buffer.resize(config_.max_buffer_size);
  buffer_compressing.resize(config_.max_buffer_size);
  buffer_view = BufferView(buffer);
  if (config_.info_.use_threads) {
    compressing_thread_ = std::thread(&Compressor::CompressAndPublishParallelised, this);
  }
}

template <typename MsgT>
Compressor<MsgT>::~Compressor() {
  if (compressing_thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      should_exit_ = true;
    }
    cv_ready_to_compress_.notify_one();
    compressing_thread_.join();
  }
}
template <typename MsgT>
void Compressor<MsgT>::PacketCompression(ConstBufferView &input_view)
{
  memcpy(buffer_view.data(), input_view.data(), config_.initial_offset);
  buffer_view.trim_front(config_.initial_offset);
  buffer_size += config_.initial_offset;
  input_view.trim_front(config_.initial_offset);

  for (int block_idx = 0; block_idx < config_.block_count; ++block_idx) {
    for (auto& encoder : scan_encoders) {
      
      buffer_size += encoder->encode(input_view, buffer_view);
    }

    input_view.trim_front(config_.per_block_offset);

    for (int i =0 ; i< config_.channel_count ; i++){
      for (auto& encoder : point_encoders) {
        buffer_size += encoder->encode(input_view, buffer_view);
      }
      input_view.trim_front(config_.point_size);
    }
    input_view.trim_front(config_.per_block_offset_end);
  }
  memcpy(buffer_view.data(), input_view.data(), config_.tail_data_size);
  buffer_view.trim_front(config_.tail_data_size);
  input_view.trim_front(config_.tail_data_size);
  buffer_size += config_.tail_data_size;
  point_count += config_.channel_count * config_.block_count;
}

template <typename MsgT>
void Compressor<MsgT>::MessageCompression(rangeini::msg::CompressedLidarPackets& target_msg, std::vector<uint8_t>& buffer_to_compress) {
  target_msg.data.resize(config_.max_buffer_size);
  uint8_t* chunk_size_ptr = &target_msg.data[0];
  BufferView output_view(&target_msg.data[0], target_msg.data.size());
  output_view.trim_front(4);
  const char* src = reinterpret_cast<const char*>(buffer_to_compress.data());
  char* dst = reinterpret_cast<char*>(output_view.data());
  uint32_t chunk_size = 0;
  if (config_.info_.compression_opt == Cloudini::CompressionOption::LZ4) {
    int cs = LZ4_compress_default(src, dst, buffer_to_compress_size, output_view.size());
    if (cs <= 0) {
      throw std::runtime_error("LZ4 compression failed");
    }
    chunk_size = static_cast<uint32_t>(cs);
  } else {
    size_t cs = ZSTD_compress(dst, output_view.size(), src, buffer_to_compress_size, 1);
    if (ZSTD_isError(cs)) {
      throw std::runtime_error("ZSTD compression failed");
    }
    chunk_size = static_cast<uint32_t>(cs);
  }

  memcpy(chunk_size_ptr, &chunk_size, sizeof(uint32_t));

  // Resize output to the logical size
  target_msg.data.resize(chunk_size + sizeof(uint32_t));
  float percentage = (target_msg.data.size() * 100.0) / (target_msg.packet_count * config_.packet_size);
  RCLCPP_INFO(this->get_logger(),
    "Message Compression Ratio: %zu/%u (%.2f%%)",
    target_msg.data.size(), 
    target_msg.packet_count * config_.packet_size,
    percentage);
    
  if (percentage > 100.0) {
    throw std::runtime_error("Too high compression ratio detected: " + std::to_string(percentage) + "%");
  }
}
template <typename MsgT>
void Compressor<MsgT>::CompressAndPublish(std::vector<uint8_t>& buffer_to_compress) {
  if (is_loaning_available) {
    // Ask ROS 2 for a loaned message
    auto loaned_msg = publisher_->borrow_loaned_message();
    loaned_msg.get().header = preallocated_msg.header;
    loaned_msg.get().packet_count = preallocated_msg.packet_count;
    MessageCompression(loaned_msg.get(), buffer_to_compress);
    publisher_->publish(std::move(loaned_msg));
  } else {
    // Use standard heap-allocated message (header already set)
    MessageCompression(preallocated_msg, buffer_to_compress);
    publisher_->publish(preallocated_msg);
  }
}
template <typename MsgT>
void Compressor<MsgT>::waitForCompressionComplete() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_done_compressing_.wait(lock, [this] { return compression_done_ || worker_failed_; });
  if (worker_failed_) {
    std::rethrow_exception(worker_exception_);
  }
}
template <typename MsgT>
void Compressor<MsgT>::ResetCompressorState() {
  buffer_size = 0;
  point_count = 0;
  packet_count = 0;
  buffer_view = BufferView(buffer);
  for (auto& encoder : scan_encoders) {
    encoder->reset();
  }
  for (auto& encoder : point_encoders) {
    encoder->reset();
  }
}

template <typename MsgT>
void Compressor<MsgT>::CompressAndPublishParallelised() {
  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_ready_to_compress_.wait(lock, [this] { return has_data_to_compress_ || should_exit_; });

        if (should_exit_) {
          break;
        }
        has_data_to_compress_ = false;
      }
      CompressAndPublish(buffer_compressing);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        compression_done_ = true;
      }

      cv_done_compressing_.notify_one();
    }
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_failed_ = true;
      worker_exception_ = std::current_exception();
    }
    cv_done_compressing_.notify_all();
  }
}
template <typename MsgT>
void Compressor<MsgT>::CompressAndPublishSwitching(std_msgs::msg::Header& header) {
  
    if (config_.info_.use_threads) {
      waitForCompressionComplete();
      // swap buffers and start compressing in the other thread
      {
        std::unique_lock<std::mutex> lock(mutex_);
        std::swap(buffer, buffer_compressing);
        buffer_to_compress_size = buffer_size;
        preallocated_msg.header = header;
        preallocated_msg.packet_count = packet_count;
        compression_done_ = false;
        has_data_to_compress_ = true;
      }
      cv_ready_to_compress_.notify_one();
    } else {
      buffer_to_compress_size = buffer_size;
      preallocated_msg.header = header;
      preallocated_msg.packet_count = packet_count;
      CompressAndPublish(buffer);
    }
    ResetCompressorState();
}





