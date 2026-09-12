#pragma once

#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "cloudini_lib/cloudini.hpp"
#include "rangeini/msg/compressed_lidar_packets.hpp"
#include "lz4.h"
#include "zstd.h"
#include "std_msgs/msg/header.hpp"
#include "cloudini_lib/contrib/span.hpp"
#include "rangeini/compressor_config.hpp"
#include "cloudini_lib/basic_types.hpp"


using BufferView = Span<uint8_t>;
using ConstBufferView = Span<const uint8_t>;

template <typename IntType>
int64_t ReadBigEndian(const uint8_t* data) {
  using UnsignedType = std::make_unsigned_t<IntType>;
  UnsignedType value = 0;
  for (size_t i = 0; i < sizeof(IntType); ++i) {
    value = static_cast<UnsignedType>((value << 8) | data[i]);
  }
  return static_cast<int64_t>(value);
}


template <typename IntType>
class FieldEncoderIntMasked : public Cloudini::FieldEncoder {
 public:
  FieldEncoderIntMasked(size_t field_offset, uint64_t mask) : offset_(field_offset), mask_(mask) {
    static_assert(std::is_integral<IntType>::value, "FieldEncoderIntMasked requires an integral type");
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override {
    int64_t value = Cloudini::ToInt64<IntType>(point_view.data() + offset_);
    value &= mask_;
    int64_t diff = value - prev_value_;
    prev_value_ = value;
    int64_t var_size = Cloudini::encodeVarint64(diff, output.data());
    output.trim_front(var_size);
    return var_size;
  }

  void reset() override {
    prev_value_ = 0;
  }

 private:
  int64_t prev_value_ = 0;
  size_t offset_ = 0;
  uint64_t mask_ =std::numeric_limits<int64_t>::max();
};
template <typename IntType>
class FieldEncoderIntBigEndian : public Cloudini::FieldEncoder {
 public:
  explicit FieldEncoderIntBigEndian(size_t field_offset) : offset_(field_offset) {
    static_assert(std::is_integral<IntType>::value, "FieldEncoderInt requires an integral type");
  }

  size_t encode(const ConstBufferView& point_view, BufferView& output) override {
    int64_t value = ReadBigEndian<IntType>(point_view.data() + offset_);
    int64_t diff = value - prev_value_;
    prev_value_ = value;
    int64_t var_size = Cloudini::encodeVarint64(diff, output.data());
    output.trim_front(var_size);
    return var_size;
  }

  void reset() override {
    prev_value_ = 0;
  }

 private:
  int64_t prev_value_ = 0;
  size_t offset_ = 0;
  uint64_t mask_ =std::numeric_limits<int64_t>::max();
};
template <typename MsgT>
class Compressor : public rclcpp::Node
{
  public:
    explicit Compressor(CompressorConfig config);
    Compressor(const Compressor&) = delete;
    Compressor& operator=(const Compressor&) = delete;
    ~Compressor();

    rclcpp::Publisher<rangeini::msg::CompressedLidarPackets>::SharedPtr publisher_;
    rclcpp::Subscription<MsgT>::SharedPtr subscription_;
    std::vector<uint8_t> buffer;
    std::vector<uint8_t> buffer_compressing;
    size_t buffer_size = 0;
    size_t buffer_to_compress_size = 0;
    int point_count = 0;
    BufferView buffer_view;
    rangeini::msg::CompressedLidarPackets preallocated_msg;
    bool is_loaning_available;
    uint32_t packet_count = 0;

    // Thread synchronization
    std::mutex mutex_;
    std::thread compressing_thread_;
    std::condition_variable cv_ready_to_compress_;
    std::condition_variable cv_done_compressing_;

    // Thread state management
    bool has_data_to_compress_ = false;
    bool compression_done_ = true;
    bool should_exit_ = false;
    bool worker_failed_ = false;
    std::exception_ptr worker_exception_;

    void PacketCompression(ConstBufferView &input_view);
    

    void MessageCompression(rangeini::msg::CompressedLidarPackets& target_msg, std::vector<uint8_t>& buffer_to_compress);

    void CompressAndPublish(std::vector<uint8_t>& buffer_to_compress);

    void ResetCompressorState();

    void CompressAndPublishParallelised();

    void CompressAndPublishSwitching(std_msgs::msg::Header& header);

    void waitForCompressionComplete();

    virtual void lidar_msg_callback(const typename MsgT::SharedPtr msg) = 0;
    CompressorConfig config_;
    std::vector<std::unique_ptr<Cloudini::FieldEncoder>> scan_encoders;
    std::vector<std::unique_ptr<Cloudini::FieldEncoder>> point_encoders;
};
#include "rangeini/compressor_impl.hpp"




