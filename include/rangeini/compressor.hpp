#pragma once

#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "cloudini_lib/field_encoder.hpp"
#include "cloudini_lib/cloudini.hpp"
#include "hesai_ros_driver/msg/udp_frame.hpp"
#include "rangeini/msg/compressed_lidar_packets.hpp"
#include "lz4.h"
#include "zstd.h"
#include "std_msgs/msg/header.hpp"
#include "cloudini_lib/contrib/span.hpp"
#include "rangeini/compressor_config.hpp"
#include "cloudini_lib/basic_types.hpp"


using BufferView = Span<uint8_t>;
using ConstBufferView = Span<const uint8_t>;



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




