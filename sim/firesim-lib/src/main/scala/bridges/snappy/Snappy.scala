package firesim.bridges.snappy

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util.DecoupledHelper

object CompressAccelLogger {
  def logInfo(format: String, args: Bits*)(implicit p: Parameters) {
    val loginfo_cycles = RegInit(0.U(64.W))
    loginfo_cycles := loginfo_cycles + 1.U

    printf("cy: %d, ", loginfo_cycles)
    printf(Printable.pack(format, args:_*))
  }
}

case class CompressorParams(
  BUS_BYTES: Int = 32
) {
  val BUS_BITS : Int = BUS_BYTES * 8
}

class MemLoaderConsumerBundle(cp: CompressorParams) extends Bundle {
  val user_consumed_bytes    = Input (UInt(log2Up(cp.BUS_BYTES+1).W))
  val available_output_bytes = Output(UInt(log2Up(cp.BUS_BYTES+1).W))
  val output_valid           = Output(Bool())
  val output_ready           = Input (Bool())
  val output_data            = Output(UInt(cp.BUS_BITS.W))
  val output_last_chunk      = Output(Bool())
}

class CompressWriterBundle(val cp: CompressorParams) extends Bundle {
  val data            = UInt(cp.BUS_BITS.W)
  val validbytes      = UInt(log2Up(cp.BUS_BYTES+1).W)
  val end_of_message = Bool()
  val is_copy        = Bool()
  val length_header  = Bool()
  val is_dummy       = Bool()
}

class StreamInfo extends Bundle {
  val ip = UInt(64.W)
  val isize = UInt(64.W)
}

class SnappyCompressor(
  val iBits: Int,
  val oBits: Int,
  val cp: CompressorParams
)(
  implicit p: Parameters
) extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(UInt(iBits.W)))
    val src_info   = Flipped(Decoupled(new StreamInfo))
    val src_info_2 = Flipped(Decoupled(new StreamInfo))
    val out = Decoupled(UInt(oBits.W))
    val compressed_bytes = Decoupled(UInt(64.W))
  })
  when (io.in.fire()) {
    CompressAccelLogger.logInfo("SNPY-INPUTFIRE\n")
  }

  val memloader = Module(new NarrowToWideMemLoader(iBits, cp))
  memloader.io.input <> io.in
  memloader.io.input_bytes.valid := io.src_info.valid
  io.src_info.ready := memloader.io.input_bytes.ready
  memloader.io.input_bytes.bits := io.src_info.bits.isize

  when (memloader.io.consumer.output_valid && memloader.io.consumer.output_ready) {
    CompressAccelLogger.logInfo("ML-CONSUMERFIRE avail: %d user_consumed: %d\n",
      memloader.io.consumer.available_output_bytes,
      memloader.io.consumer.user_consumed_bytes)
  }

  val lz77hashmatcher = Module(new LZ77HashMatcher(cp))
  lz77hashmatcher.io.MAX_OFFSET_ALLOWED := (64 << 10).U
  lz77hashmatcher.io.RUNTIME_HT_NUM_ENTRIES_LOG2 := 14.U
  lz77hashmatcher.io.write_snappy_header := true.B
  lz77hashmatcher.io.src_info <> io.src_info_2

  lz77hashmatcher.io.memloader_in <> memloader.io.consumer
  lz77hashmatcher.io.memloader_optional_hbsram_in <> memloader.io.optional_hbsram_write


  when (lz77hashmatcher.io.memwrites_out.fire) {
    CompressAccelLogger.logInfo("LZ77-MEMWRITEFIRE: data: 0x%x, validbytes: %d, EOM: %d, is_copy: %d, length_header: %d\n",
      lz77hashmatcher.io.memwrites_out.bits.data,
      lz77hashmatcher.io.memwrites_out.bits.validbytes,
      lz77hashmatcher.io.memwrites_out.bits.end_of_message,
      lz77hashmatcher.io.memwrites_out.bits.is_copy,
      lz77hashmatcher.io.memwrites_out.bits.length_header
      )
  }

  val compress_copy_expander = Module(new SnappyCompressCopyExpander(cp))
  compress_copy_expander.io.memwrites_in <> lz77hashmatcher.io.memwrites_out

  when (compress_copy_expander.io.memwrites_out.fire) {
    CompressAccelLogger.logInfo("CEXP-MEMWRITEFIRE: data: 0x%x, validbytes: %d, EOM: %d, is_copy: %d, length_header: %d\n",
      compress_copy_expander.io.memwrites_out.bits.data,
      compress_copy_expander.io.memwrites_out.bits.validbytes,
      compress_copy_expander.io.memwrites_out.bits.end_of_message,
      compress_copy_expander.io.memwrites_out.bits.is_copy,
      compress_copy_expander.io.memwrites_out.bits.length_header
      )
  }

  val compress_litlen_injector = Module(new SnappyCompressLitLenInjector(cp))
  compress_litlen_injector.io.memwrites_in <> compress_copy_expander.io.memwrites_out

  when (compress_litlen_injector.io.memwrites_out.fire) {
    CompressAccelLogger.logInfo("CLLI-MEMWRITEFIRE: data: 0x%x, validbytes: %d, EOM: %d, is_copy: %d, length_header: %d\n",
      compress_litlen_injector.io.memwrites_out.bits.data,
      compress_litlen_injector.io.memwrites_out.bits.validbytes,
      compress_litlen_injector.io.memwrites_out.bits.end_of_message,
      compress_litlen_injector.io.memwrites_out.bits.is_copy,
      compress_litlen_injector.io.memwrites_out.bits.length_header
      )
  }

  val memwriter = Module(new NarrowToWideMemWriter(oBits, cp))

  val fire_memwrites = DecoupledHelper(
    memwriter.io.input.ready,
    compress_litlen_injector.io.memwrites_out.valid,
    io.compressed_bytes.ready)

  memwriter.io.input.bits <> compress_litlen_injector.io.memwrites_out.bits
  memwriter.io.input.valid := fire_memwrites.fire(memwriter.io.input.ready)
  compress_litlen_injector.io.memwrites_out.ready := fire_memwrites.fire(compress_litlen_injector.io.memwrites_out.valid)

  io.out <> memwriter.io.output

  val eom = compress_litlen_injector.io.memwrites_out.bits.end_of_message
  val vbs = compress_litlen_injector.io.memwrites_out.bits.validbytes
  val compressed_bytes = RegInit(0.U(64.W))
  when (fire_memwrites.fire()) {
    compressed_bytes := Mux(eom, 0.U, compressed_bytes + vbs)
    CompressAccelLogger.logInfo("MW-INPUTFIRE validbytes: %d eom: %d\n",
      memwriter.io.input.bits.validbytes,
      memwriter.io.input.bits.end_of_message)
  }

  when (io.out.fire()) {
    CompressAccelLogger.logInfo("SNPY-OUTFIRE 0x%x\n", io.out.bits)
  }

  io.compressed_bytes.valid := fire_memwrites.fire(io.compressed_bytes.ready,
                                 compress_litlen_injector.io.memwrites_out.bits.end_of_message)
  io.compressed_bytes.bits  := compressed_bytes + vbs

  when (io.compressed_bytes.fire()) {
    CompressAccelLogger.logInfo("BUF-COMPRESSED %d bytes\n", io.compressed_bytes.bits)
  }
}
