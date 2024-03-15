
#include "compressed_trace.h"


#include <assert.h>
#include <iostream>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DEBUG

char compressed_trace_t::KIND;

compressed_trace_t::compressed_trace_t(simif_t &sim,
            StreamEngine &stream,
            const COMPTRACEBRIDGEMODULE_struct &mmio_addrs,
            int comp_trace_bridge_idx,
            std::vector<std::string> &args,
            uint32_t iaddr_width,
            uint32_t insn_width,
            uint32_t cause_width,
            uint32_t wdata_width,
            uint32_t num_commit_insts,
            const char *isa,
            uint32_t vlen,
            const char *priv,
            uint32_t pmp_regions,
            uint64_t mem0_base,
            uint64_t mem0_size,
            uint32_t nharts,
            const char *bootrom,
            uint32_t hartid,
            uint32_t stream_idx,
            uint32_t stream_depth)
  : streaming_bridge_driver_t(sim, stream, &KIND),
    stream_depth(stream_depth),
    stream_idx(stream_idx),
    mmio_addrs(mmio_addrs)
{
  stream_bytes = stream_depth * STREAM_WIDTH_BYTES;
  cbuf = new circular_buffer_t(stream_bytes, 16);
}

void compressed_trace_t::init() {
  write(mmio_addrs.snappy_target_cycles_per_file, stream_depth * 10);
  write(mmio_addrs.snappy_target_cycles_per_file_valid, true);
}

void compressed_trace_t::tick() {
  bool val = read(mmio_addrs.compressed_bytes_valid);

  if (val) {
    uint64_t cb_lo = read(mmio_addrs.compressed_bytes_data_lo);
    uint64_t cb_hi = read(mmio_addrs.compressed_bytes_data_hi);
    write(mmio_addrs.compressed_bytes_ready, 1);

    uint64_t cb = cb_lo | (cb_hi << 32UL);
#ifdef DEBUG
    printf("MMIO received compBytes: %lu\n", cb);
#endif
    comp_bytes.push(cb);
  }

  if (!comp_bytes.empty() && (comp_bytes.front() >= cbuf->valid_bytes())) {
    uint64_t cb = comp_bytes.front();
    comp_bytes.pop();

    output_file_idx++;
    std::string filename = "trace-" + std::to_string(output_file_idx) + ".snappy";
    FILE* output_file = fopen(filename.c_str(), "w");
    if (output_file == NULL) {
      printf("Failed to open file %s\n", filename.c_str());
      exit(1);
    }

    char* comp_data = cbuf->get_head_ptr();
    fwrite(comp_data, sizeof(char), cb, output_file);
    fclose(output_file);
    cbuf->free_bytes(cb);
  }

  if (cbuf->is_full()) {
    return;
  }

  char* empty_buffer = cbuf->get_empty_entry();
  if (empty_buffer == nullptr) {
    return;
  }

  auto bytes_received = pull(stream_idx, empty_buffer, stream_bytes, stream_bytes);
#ifdef DEBUG
  printf("Bytes received: %lu\n", bytes_received);
#endif
  if (bytes_received == 0) {
    return;
  }
}

void compressed_trace_t::finish() {
}

//////////////////////////////////////////////////////////////////////////////


circular_buffer_t::circular_buffer_t(int per_entry_bytes, int num_entries) 
  : per_entry_bytes(per_entry_bytes), num_entries(num_entries)
{
  char* buf_ptr = (char*)malloc(sizeof(char) * per_entry_bytes * num_entries);
  for (int i = 0; i < num_entries; i++) {
    buffers.push_back(buf_ptr + per_entry_bytes);
  }
  head = 0;
  tail = 0;
  full = false;
  cur_buf_offset = 0;
}

circular_buffer_t::~circular_buffer_t() {
  for (int i = 0; i < (int)buffers.size(); i++) {
    free(buffers[i]);
  }
  buffers.clear();
}

bool circular_buffer_t::is_full() {
  return full;
}

char* circular_buffer_t::get_empty_entry() {
  if (full)
    return nullptr;

  char* ret = buffers[tail];
  tail = (tail + 1) % num_entries;
  if (tail == head)
    full = true;
  return ret;
}

char* circular_buffer_t::get_head_ptr() {
  return (buffers[head] + cur_buf_offset);
}

void circular_buffer_t::free_bytes(int bytes) {
  int head_incr_amount = (cur_buf_offset + bytes) / per_entry_bytes;
  head = head + head_incr_amount;
  cur_buf_offset = (cur_buf_offset + bytes) % per_entry_bytes;
}

uint64_t circular_buffer_t::valid_bytes() {
  uint64_t entries = (tail + num_entries - head) % num_entries;
  return (entries * per_entry_bytes) - cur_buf_offset;
}
