//See LICENSE for license details
package firesim.bridges
import firesim.bridges.snappy._

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util.DecoupledHelper

import testchipip.cosim.{SerializableTileTraceIO, SpikeCosimConfig, TileTraceIO, TraceBundleWidths}

import midas.widgets._

object Logger {
  def logInfo(format: String, args: Bits*)(implicit p: Parameters) {
    val loginfo_cycles = RegInit(0.U(64.W))
    loginfo_cycles := loginfo_cycles + 1.U

    printf("cy: %d, ", loginfo_cycles)
    printf(Printable.pack(format, args:_*))
  }
}

class CompTraceTargetIO(widths: TraceBundleWidths) extends Bundle {
  val trace = Input(new SerializableTileTraceIO(widths))
}

/** Blackbox that is instantiated in the target
  */
class CompTraceBridge(params: CospikeBridgeParams)
    extends BlackBox
    with Bridge[HostPortIO[CompTraceTargetIO], CompTraceBridgeModule] {
  val io       = IO(new CompTraceTargetIO(params.widths))
  val bridgeIO = HostPort(io)

  val constructorArg = Some(params)
  generateAnnotations()
}

/** Helper function to connect blackbox
  */
object CompTraceBridge {
  def apply(trace: TileTraceIO, hartid: Int, cfg: SpikeCosimConfig) = {
    val params = new CospikeBridgeParams(trace.traceBundleWidths, hartid, cfg)
    val tb  = withClockAndReset(trace.clock, trace.reset) {
      Module(new CompTraceBridge(params))
    }
    tb.io.trace.trace.insns.map(t => {
      t       := DontCare
      t.valid := false.B
    })
    tb.io.trace := trace.asSerializableTileTrace
    tb
  }
}

class CompTraceBridgeModule(params: CospikeBridgeParams)(implicit p: Parameters)
    extends BridgeModule[HostPortIO[CompTraceTargetIO]]()(p)
    with StreamToHostCPU {
  // CONSTANTS: DMA Parameters
  val toHostCPUQueueDepth = 6144

  lazy val module = new BridgeModuleImp(this) {

    // setup io
    val io    = IO(new WidgetIO)
    val hPort = IO(HostPort(new CompTraceTargetIO(params.widths)))

    // helper to get number to round up to nearest multiple
    def roundUp(num: Int, mult: Int): Int = { (num.toFloat / mult).ceil.toInt * mult }

    // get the traces
    val traces = hPort.hBits.trace.trace.insns.map({ unmasked =>
      val masked = WireDefault(unmasked)
      masked.valid := unmasked.valid && !hPort.hBits.trace.reset
      masked
    })
    private val iaddrWidth = roundUp(traces.map(_.iaddr.getWidth).max, 8)
    private val insnWidth  = roundUp(traces.map(_.insn.getWidth).max, 8)
    private val causeWidth = roundUp(traces.map(_.cause.getWidth).max, 8)
    private val wdataWidth = roundUp(traces.map(t => if (t.wdata.isDefined) t.wdata.get.getWidth else 0).max, 8)

    // hack since for some reason padding a bool doesn't work...
    def boolPad(in: Bool, size: Int): UInt = {
      val temp = Wire(UInt(size.W))
      temp := in.asUInt
      temp
    }

    // matches order of TracedInstruction in CSR.scala
    val paddedTraces = traces.map { trace =>
      val pre_cat = Cat(
        trace.cause.pad(causeWidth),
        boolPad(trace.interrupt, 8),
        boolPad(trace.exception, 8),
        trace.priv.asUInt.pad(8),
        trace.insn.pad(insnWidth),
        trace.iaddr.pad(iaddrWidth),
        boolPad(trace.valid, 8),
      )

      if (wdataWidth == 0) {
        pre_cat
      } else {
        Cat(trace.wdata.get.pad(wdataWidth), pre_cat)
      }
    }

    ///////////////////////////////////////////////////////////////////////

    /*
     * NOTE
     * - Kind of ignores the last batch of instructions, but that is fine TBH
     * - Need to parameterize the compressor's width in order to not backpressure the target from advancing
     *   - Basically, the compressor has to be able to receive all the instructions that the SoC generates per cycle
     */
    val maxTraceSize = paddedTraces.map(t => t.getWidth).max
    val iBits = maxTraceSize * paddedTraces.length
    val oBits = streamEnq.bits.getWidth

    val compBytes = Module(new Queue(UInt(64.W), 8))
    val snappyParams = CompressorParams(BUS_BYTES = 32)
    val snappy = Module(new SnappyCompressor(iBits, oBits, snappyParams))

    val defaultTargetCyclesPerFile = (toHostCPUQueueDepth * 10).U
    val defaultTargetCyclesPerFileValid = 0.U
    val targetCyclesPerFile = genWORegInit(Wire(UInt(32.W)), "snappy_target_cycles_per_file", defaultTargetCyclesPerFile)
    val targetCyclesPerFileValid = genWORegInit(Wire(UInt(1.W)), "snappy_target_cycles_per_file_valid", defaultTargetCyclesPerFile)
    val prevTargetCyclesPerFile = RegNext(targetCyclesPerFile)
    when (targetCyclesPerFile =/= prevTargetCyclesPerFile) {
      Logger.logInfo("targetCyclesPerFile %d to %d\n",
        prevTargetCyclesPerFile, targetCyclesPerFile)
    }

    genROReg(compBytes.io.deq.valid, "compressed_bytes_valid")
    genROReg(compBytes.io.deq.bits(31,  0),  "compressed_bytes_data_lo")
    genROReg(compBytes.io.deq.bits(63, 32),  "compressed_bytes_data_hi")
    Pulsify(genWORegInit(compBytes.io.deq.ready, "compressed_bytes_ready", false.B), pulseLength = 1)

    when (compBytes.io.enq.fire()) {
      Logger.logInfo("enq compBytes %d\n", compBytes.io.enq.bits)
    }
    when (compBytes.io.deq.fire()) {
      Logger.logInfo("deq compBytes %d\n", compBytes.io.deq.bits)
    }

    val enqTrace = DecoupledHelper(
     hPort.toHost.hValid,
     snappy.io.in.ready)

    val srcInfoQ  = Module(new Queue(new StreamInfo, 4))
    val srcInfoQ2 = Module(new Queue(new StreamInfo, 4))
    val pushedInputs = RegInit(0.U(64.W))
    val srcInfoFired  = RegInit(false.B)
    when (enqTrace.fire()) {
      pushedInputs := Mux(pushedInputs === targetCyclesPerFile - 1.U,
                           0.U,
                           pushedInputs + 1.U)
      srcInfoFired := false.B
    }

    val enqSrcFire = DecoupledHelper(
      srcInfoQ.io.enq.ready,
      srcInfoQ2.io.enq.ready,
      pushedInputs === 0.U,
      !srcInfoFired,
      (targetCyclesPerFileValid =/= 0.U))

    srcInfoQ.io.enq.valid  := enqSrcFire.fire(srcInfoQ.io.enq.ready)
    srcInfoQ2.io.enq.valid := enqSrcFire.fire(srcInfoQ2.io.enq.ready)
    srcInfoQ.io.enq.bits.ip  := DontCare
    srcInfoQ2.io.enq.bits.ip := DontCare
    srcInfoQ.io.enq.bits.isize   := (targetCyclesPerFile * iBits.U) >> 3.U
    srcInfoQ2.io.enq.bits.isize  := (targetCyclesPerFile * iBits.U) >> 3.U
    when (enqSrcFire.fire()) {
      srcInfoFired := true.B
    }

    hPort.toHost.hReady := enqTrace.fire(hPort.toHost.hValid)
    snappy.io.in.valid  := enqTrace.fire(snappy.io.in.ready)
    snappy.io.in.bits   := Cat(paddedTraces)
    snappy.io.src_info   <> srcInfoQ.io.deq
    snappy.io.src_info_2 <> srcInfoQ2.io.deq

    streamEnq <> snappy.io.out
    compBytes.io.enq <> snappy.io.compressed_bytes

    hPort.fromHost.hValid := true.B // this is uni-directional. we don't drive tokens back to target
    genCRFile()

    // modify the output header file
    override def genHeader(base: BigInt, memoryRegions: Map[String, BigInt], sb: StringBuilder): Unit = {
      genConstructor(
        base,
        sb,
        "compressed_trace_t",
        "compressed_trace",
        Seq(
          UInt32(iaddrWidth),
          UInt32(insnWidth),
          UInt32(causeWidth),
          UInt32(wdataWidth),
          UInt32(traces.length),
          CStrLit(params.cfg.isa),
          UInt32(params.cfg.vlen),
          CStrLit(params.cfg.priv),
          UInt32(params.cfg.pmpregions),
          UInt64(params.cfg.mem0_base),
          UInt64(params.cfg.mem0_size),
          UInt32(params.cfg.nharts),
          CStrLit(params.cfg.bootrom),
          UInt32(params.hartid),
          UInt32(toHostStreamIdx),
          UInt32(toHostCPUQueueDepth),
        ),
        hasStreams = true,
      )
    }

    println(s" CompressedTrace Bridge Information")
    println(s"  Total Inst. Traces (i.e. Commit Width): ${traces.length}")
    println(s"  Trace bits per instruction ${maxTraceSize}")
  }
}
