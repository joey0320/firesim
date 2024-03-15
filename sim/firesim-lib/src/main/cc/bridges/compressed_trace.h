#ifndef __COMPRESSED_TRACE_H
#define __COMPRESSED_TRACE_H

#include "core/bridge_driver.h"
#include <queue>
#include <inttypes.h>


class circular_buffer_t {
public:
  circular_buffer_t(int per_entry_bytes, int num_entries);
  ~circular_buffer_t();

  bool is_full();
  char* get_empty_entry();
  char* get_head_ptr();
  void free_bytes(int bytes);
  uint64_t valid_bytes();

private:
  std::vector<char*> buffers;
  int num_entries;
  int per_entry_bytes;
  int head;
  int tail;
  bool full;
  uint64_t cur_buf_offset;
};

//////////////////////////////////////////////////////////////////////////////

struct COMPTRACEBRIDGEMODULE_struct {
  uint64_t snappy_target_cycles_per_file;
  uint64_t snappy_target_cycles_per_file_valid;
  uint64_t compressed_bytes_valid;
  uint64_t compressed_bytes_data_lo;
  uint64_t compressed_bytes_data_hi;
  uint64_t compressed_bytes_ready;
};

class compressed_trace_t : public streaming_bridge_driver_t {
public:
  /// The identifier for the bridge type used for casts.
  static char KIND;

  compressed_trace_t(simif_t &sim,
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
            uint32_t stream_depth);

  ~compressed_trace_t() override = default;

  void init() override;
  void tick() override;
  void finish() override;

private:
  const COMPTRACEBRIDGEMODULE_struct mmio_addrs;
  std::queue<int> compressed_bytes;

  // stream config
  int stream_idx;
  int stream_depth;
  int stream_bytes;

  circular_buffer_t* cbuf;
  uint64_t output_file_idx = 0;
  std::queue<uint64_t> comp_bytes;
};

#endif // __COMPRESSED_TRACE_H
