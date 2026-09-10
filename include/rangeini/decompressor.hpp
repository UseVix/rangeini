#pragma once


#include <cstring>
#include <stdexcept>

#include "lz4.h"
#include "zstd.h"

#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>
#include <thread>
#include <condition_variable>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "cloudini_lib/field_decoder.hpp"
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
class Decompressor : public rclcpp::Node
{
  public:
    explicit Decompressor(CompressorConfig config);
    
    Decompressor(const Decompressor&) = delete;
    Decompressor& operator=(const Decompressor&) = delete;
    ~Decompressor();

    rclcpp::Publisher<MsgT>::SharedPtr publisher_;
    rclcpp::Subscription<rangeini::msg::CompressedLidarPackets>::SharedPtr subscription_;
    
    std::vector<uint8_t> compressed_packets_buffer;
    std::vector<uint8_t> compressed_packets_buffer_swap;
    std_msgs::msg::Header header_swap;
    uint32_t packet_count_swap;

    std::mutex mutex_;
    std::thread decompressing_thread_;
    std::condition_variable cv_ready_to_decompress_;
    std::condition_variable cv_done_decompressing_;

    bool has_data_to_decompress_ = false;
    bool decompression_done_ = true;
    bool should_exit_ = false;
    bool worker_failed_ = false;
    std::exception_ptr worker_exception_;

    std::vector<MsgT> preallocated_msg;
    bool is_loaning_available;

    CompressorConfig config_;
    std::vector<std::unique_ptr<Cloudini::FieldDecoder>> scan_decoders;
    std::vector<std::unique_ptr<Cloudini::FieldDecoder>> point_decoders;

    void PacketDecompression(ConstBufferView &input_view, BufferView &output_view);

    void MessageDecompression(const rangeini::msg::CompressedLidarPackets& msg);
    void DecompressAndPublishParallelised();

    virtual void DecompressAndPublish(std::vector<uint8_t>& compressed_packets_buffer, const uint32_t& packet_count, const std_msgs::msg::Header& published_header) = 0;

    void waitForDecompressionComplete();
    void DecompressAndPublishSwitching(const uint32_t& packet_count, const std_msgs::msg::Header& published_header);
    void compressed_msg_callback(const rangeini::msg::CompressedLidarPackets::SharedPtr msg);
};
#include "rangeini/decompressor_impl.hpp"





