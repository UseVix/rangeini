#pragma once

#include <vector>
#include <utility>
#include <limits>
#include "cloudini_lib/basic_types.hpp"
#include "cloudini_lib/cloudini.hpp"
struct FieldEncodeConfig {
  Cloudini::FieldType type;
  size_t offset;
  bool copy = false;  // If true, use FieldEncoderCopy instead of FieldEncoderInt
  int64_t mask = std::numeric_limits<int64_t>::max();  // Mask for FieldEncoderInt
  bool merge = false;  // If true, use FieldEncoderIntMerging instead of FieldEncoderInt
  bool big_endian = false;  // If true, use big-endian encoding
};
struct CompressorConfig {
  int initial_offset;
  int per_block_offset;
  int point_size;
  int channel_count;
  int block_count;
  int packet_size;
  int tail_data_size;
  int packets_in_chunk;
  int chunk_point_size;
  int max_buffer_size;
  int per_block_offset_end=0;
  Cloudini::EncodingInfo info_;
  std::vector<FieldEncodeConfig> scan_fields;
  std::vector<FieldEncodeConfig> point_fields;
};


