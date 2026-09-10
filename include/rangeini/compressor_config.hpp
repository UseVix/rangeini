#pragma once

#include <vector>
#include <utility>
#include <limits>
#include "cloudini_lib/basic_types.hpp"
#include "cloudini_lib/cloudini.hpp"

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
  Cloudini::EncodingInfo info_;
  std::vector<std::pair<Cloudini::FieldType, size_t>> scan_fields;
  std::vector<std::pair<Cloudini::FieldType, size_t>> point_fields;
};


