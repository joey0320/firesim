package firesim.bridges.snappy

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util.DecoupledHelper


class NarrowToWideMemLoader(
    iBits: Int,
    cp: CompressorParams,
    dataQueueDepth: Int = 32
  )(
    implicit p: Parameters
  ) extends Module {
  val io = IO(new Bundle {
    val input_bytes = Flipped(Decoupled(UInt(64.W)))
    val input = Flipped(Decoupled(UInt(iBits.W)))
    val optional_hbsram_write = Valid(new HBSRAMWrite)
    val consumer = new MemLoaderConsumerBundle(cp)
  })
  require(iBits <= cp.BUS_BITS, f"input bits ${iBits} > ${cp.BUS_BITS}")
  require(iBits % 8 == 0, f"iBits (${iBits}) %% 8 != 0")

  val NUM_QUEUES = cp.BUS_BYTES
  val data_queues = Seq.fill(NUM_QUEUES)(Module(new Queue(UInt(8.W), dataQueueDepth)))
  val sidx = RegInit(0.U(log2Up(NUM_QUEUES + 1).W))

  val input_queue = Module(new Queue(UInt(iBits.W), 4))
  input_queue.io.enq <> io.input

  val input_bytes = Module(new Queue(UInt(64.W), 4))
  input_bytes.io.enq <> io.input_bytes

  val iBytes = iBits >> 3
  val input_data_by_bytes = Seq.fill(iBytes)(Wire(UInt(8.W)))
  for (i <- 0 until iBytes) {
    input_data_by_bytes(i) := input_queue.io.deq.bits >> (i * 8)
  }

  val all_data_queues_enq_ready = data_queues.map(_.io.enq.ready).reduce(_ && _)
  val input_data_fire = DecoupledHelper(
    input_queue.io.deq.valid,
    all_data_queues_enq_ready)

  input_queue.io.deq.ready := input_data_fire.fire(input_queue.io.deq.valid)

  /*
   * data_queue((j + sidx) % NUM_QUEUES) := input_data_by_bytes(j)
   */
  val should_enqueue = Seq.fill(NUM_QUEUES)(Wire(Bool()))
  for (i <- 0 until NUM_QUEUES) {
    // Tie off enq bits by default
    data_queues(i).io.enq.bits  := 0.U
    data_queues(i).io.enq.valid := false.B
    data_queues(i).io.deq.ready := false.B
    should_enqueue(i) := false.B

    // Enq data
    data_queues(i).io.enq.valid := input_data_fire.fire(all_data_queues_enq_ready) &&
                                    should_enqueue(i)

    // Swizzle input
    for (j <- 0 until iBytes) {
      when (i.U === (j.U + sidx) % NUM_QUEUES.U) {
        data_queues(i).io.enq.bits := input_data_by_bytes(j)
        should_enqueue(i) := true.B
      }
    }
  }

  when (input_data_fire.fire()) {
    val nxt_sidx = (sidx +& iBytes.U) % NUM_QUEUES.U
    sidx := nxt_sidx

    CompressAccelLogger.logInfo("ML input_data_fiire sid %d -> %d\n", sidx, nxt_sidx)
    for (i <- 0 until NUM_QUEUES) {
      CompressAccelLogger.logInfo("q(%d) enq: %d data: %d\n",
        i.U, data_queues(i).io.enq.fire(), data_queues(i).io.enq.bits)
    }
  }

  io.optional_hbsram_write.valid := io.input.fire()
  io.optional_hbsram_write.bits.valid_bytes := iBytes.U
  io.optional_hbsram_write.bits.data := io.input.bits

  // io.consumer logic
  val read_start_index = RegInit(0.U(log2Up(NUM_QUEUES+1).W))
  val len_already_consumed = RegInit(0.U(64.W))

  val remapVecData = Wire(Vec(NUM_QUEUES, UInt(8.W)))
  val remapVecValids = Wire(Vec(NUM_QUEUES, Bool()))
  val remapVecReadys = Wire(Vec(NUM_QUEUES, Bool()))

  for (i <- 0 until NUM_QUEUES) {
    // Tie off default values
    remapVecData(i)   := 0.U
    remapVecValids(i) := false.B
    remapVecReadys(i) := false.B

    for (j <- 0 until NUM_QUEUES) {
      when ((i.U +& read_start_index) % NUM_QUEUES.U === j.U) {
        remapVecData(i)   := data_queues(j).io.deq.bits
        remapVecValids(i) := data_queues(j).io.deq.valid
        data_queues(j).io.deq.ready := remapVecReadys(i)
      }
    }
  }

  io.consumer.output_data := Cat(remapVecData.reverse)

  val tot_len_consumed = len_already_consumed + io.consumer.user_consumed_bytes
  val buf_last = tot_len_consumed === input_bytes.io.deq.bits
  val count_valids = remapVecValids.map(_.asUInt).reduce(_ +& _)
  val unconsumed_bytes_so_far = input_bytes.io.deq.bits - len_already_consumed

  val enough_data = Mux(unconsumed_bytes_so_far >= NUM_QUEUES.U,
                        count_valids === NUM_QUEUES.U,
                        count_valids >= unconsumed_bytes_so_far)

  io.consumer.available_output_bytes := Mux(unconsumed_bytes_so_far >= NUM_QUEUES.U,
                                    NUM_QUEUES.U,
                                    unconsumed_bytes_so_far)

  io.consumer.output_last_chunk := (unconsumed_bytes_so_far <= NUM_QUEUES.U)

  val read_fire = DecoupledHelper(
    io.consumer.output_ready,
    input_bytes.io.deq.valid,
    enough_data)

  io.consumer.output_valid := read_fire.fire(io.consumer.output_ready)

  for (queueno <- 0 until NUM_QUEUES) {
    remapVecReadys(queueno) := (queueno.U < io.consumer.user_consumed_bytes) && read_fire.fire()
  }

  when (read_fire.fire()) {
    read_start_index := (read_start_index +& io.consumer.user_consumed_bytes) % NUM_QUEUES.U
  }

  input_bytes.io.deq.ready := read_fire.fire(input_bytes.io.deq.valid) && buf_last

  when (read_fire.fire()) {
    when (buf_last) {
      len_already_consumed := 0.U
    } .otherwise {
      len_already_consumed := len_already_consumed + io.consumer.user_consumed_bytes
    }
  }
}

class NarrowToWideMemWriter(
  oBits: Int,
  cp: CompressorParams,
  dataQueueDepth: Int = 32
  )(
    implicit p: Parameters
  ) extends Module {
  val io = IO(new Bundle {
    val input  = Flipped(Decoupled(new CompressWriterBundle(cp)))
    val output = Decoupled(UInt(oBits.W))
  })
  val oBytes = oBits >> 3
  require(oBytes >= cp.BUS_BYTES, f"oBytes (${oBytes}) < BUS_BYTES (${cp.BUS_BYTES})")
  require(oBits % 8 == 0, f"oBits is not a multiple of 8 ${oBits}")

  val NUM_QUEUES = oBytes
  val data_queues = Seq.fill(NUM_QUEUES)(Module(new Queue(UInt(8.W), dataQueueDepth)))
  val sidx = RegInit(0.U(log2Up(NUM_QUEUES + 1).W))

  val input_queue = Module(new Queue(new CompressWriterBundle(cp), 4))
  input_queue.io.enq <> io.input

  val input_data_by_bytes = Seq.fill(cp.BUS_BYTES)(Wire(UInt(8.W)))
  for (i <- 0 until cp.BUS_BYTES) {
    input_data_by_bytes(i) := input_queue.io.deq.bits.data >> (i * 8)
  }

  val all_data_queues_enq_ready = data_queues.map(_.io.enq.ready).reduce(_ && _)
  val input_data_fire = DecoupledHelper(
    input_queue.io.deq.valid,
    all_data_queues_enq_ready)

  input_queue.io.deq.ready := input_data_fire.fire(input_queue.io.deq.valid)

  /*
   * data_queues((sid + j) % NUM_QUEUES) := input_data_by_bytes(j)
   */
  val should_enqueue = Seq.fill(NUM_QUEUES)(Wire(Bool()))
  for (i <- 0 until NUM_QUEUES) {
    // Tie off default signals
    data_queues(i).io.enq.bits  := 0.U
    data_queues(i).io.enq.valid := false.B
    should_enqueue(i) := false.B

    // Enq data
    data_queues(i).io.enq.valid := input_data_fire.fire(all_data_queues_enq_ready) &&
                                   should_enqueue(i)

    // Swizzle input
    for (j <- 0 until cp.BUS_BYTES) {
      when (i.U === (sidx + j.U) % NUM_QUEUES.U) {
        data_queues(i).io.enq.bits := input_data_by_bytes(j)
        should_enqueue(i) := Mux(j.U < input_queue.io.deq.bits.validbytes, true.B, false.B)
      }
    }
  }

  when (input_data_fire.fire()) {
    val nxt_sidx = (sidx + input_queue.io.deq.bits.validbytes) % NUM_QUEUES.U
    sidx := nxt_sidx

    CompressAccelLogger.logInfo("MW input_data_fire sidx %d -> %d\n", sidx, nxt_sidx)
    for (i <- 0 until NUM_QUEUES) {
      CompressAccelLogger.logInfo("q(%d) enq: %d data: %d\n",
        i.U, data_queues(i).io.enq.fire(), data_queues(i).io.enq.bits)
    }
  }

  val all_data_queues_deq_valid = data_queues.map(_.io.deq.valid).reduce(_ && _)

  val output_fire = DecoupledHelper(
    io.output.ready,
    all_data_queues_deq_valid)

  io.output.valid := output_fire.fire(io.output.ready)
  for (i <- 0 until NUM_QUEUES) {
    data_queues(i).io.deq.ready := output_fire.fire()
  }
  io.output.bits := Cat(data_queues.map(_.io.deq.bits).reverse)
}
