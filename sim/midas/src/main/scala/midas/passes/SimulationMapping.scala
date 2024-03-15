// See LICENSE for license details.

package midas
package passes

import firrtl._
import firrtl.annotations.{CircuitName, ModuleTarget, InstanceTarget}
import firrtl.options.Dependency
import firrtl.stage.Forms
import firrtl.stage.transforms.Compiler
import firrtl.ir._
import firrtl.passes._
import firrtl.Mappers._
import firrtl.WrappedExpression._
import firrtl.Utils.module_type
import midas.passes.Utils._
import freechips.rocketchip.diplomacy.LazyModule

import midas.core._
import midas.platform.PlatformShim
import midas.stage.{OutputFileBuilder, GoldenGateOutputFileAnnotation}

import annotation.tailrec
import collection.mutable

private[passes] class SimulationMapping(targetName: String) extends firrtl.Transform {
  def inputForm = LowForm
  def outputForm = HighForm
  override def name = "[Golden Gate] Simulation Mapping"

  private def generateHeaderAnnos(c: PlatformShim): Seq[GoldenGateOutputFileAnnotation] = {
    val csb = new OutputFileBuilder(
      """// Golden Gate-generated Driver Header
        |// This contains target-specific preprocessor macro definitions,
        |// and encodes all required bridge metadata to instantiate bridge drivers.
        |""".stripMargin,
      fileSuffix = ".const.h")
    c.genHeader(csb.getBuilder, targetName)

    val vsb = new OutputFileBuilder(
      """// Golden Gate-generated Verilog Header
        |// This file encodes variable width fields used in MIDAS-level simulation
        |// and is not used in FPGA compilation flows.
        |""".stripMargin,
      fileSuffix = ".const.vh")

    vsb append "`ifndef __%s_H\n".format(targetName.toUpperCase)
    vsb append "`define __%s_H\n".format(targetName.toUpperCase)
    c.genVHeader(vsb.getBuilder, targetName)
    vsb append "`endif  // __%s_H\n".format(targetName.toUpperCase)
    Seq(csb.toAnnotation, vsb.toAnnotation)
  }

  // Note: this only runs on the SimulationWrapper Module
  private def initStmt(targetModuleName: String, targetInstName: String)(s: Statement): Statement =
    s match {
      case s @ WDefInstance(_, name, _, _) if name == targetInstName =>
        Block(Seq(
          s copy (module = targetModuleName), // replace TargetBox with the actual target module
          IsInvalid(NoInfo, WRef(name))
        ))
      case s => s map initStmt(targetModuleName, targetInstName)
    }

  private def init(info: Info, target: String, tpe: Type, targetBoxParent: String, targetBoxInst: String)(m: DefModule) = m match {
    case m: Module if m.name == targetBoxParent =>
      val body = initStmt(target, targetBoxInst)(m.body)
      Some(m.copy(info = info, body = body))
    case o => Some(o)
  }

  def execute(innerState: CircuitState) = {
    // Generate a port map to look up the types of the IO of the channels
    implicit val p = innerState.annotations.collectFirst({ case midas.stage.phases.ConfigParametersAnnotation(p)  => p }).get
    val circuit = innerState.circuit
    val portTypeMap = circuit.modules.filter(_.name == circuit.main).flatMap({m =>
      val mTarget = ModuleTarget(circuit.main, m.name)
      m.ports.map({ p => mTarget.ref(p.name) ->  p })
    }).toMap

    // Lower the inner circuit in preparation for linking
    // This prevents having to worry about matching aggregate structure in the wrapper IO
    val loweredInnerState = new Compiler(Forms.LowForm, Forms.HighForm).execute(innerState)
    val innerCircuit = loweredInnerState.circuit

    // Generate the encapsulating simulator RTL
    lazy val shim = PlatformShim(SimWrapperConfig(innerState.annotations, portTypeMap))
    val (chirrtl, elaboratedAnnos) = midas.targetutils.ElaborateChiselSubCircuit(LazyModule(shim).module)

    val outerAnnos = PreLinkRenamingAnnotation(Namespace(innerCircuit)) +: elaboratedAnnos
    val outerStateHigh = new Compiler(Forms.HighForm ++ Seq(Dependency(PreLinkRenaming)))
      .execute(CircuitState(chirrtl, ChirrtlForm, outerAnnos))

    println("Chirrtl to High firrtl lowering done")

    val outerStateMid = new Compiler(Seq(
      Dependency(firrtl.passes.PullMuxes),
      Dependency(firrtl.passes.ReplaceAccesses),
      Dependency(firrtl.passes.ExpandConnects),
      Dependency(firrtl.passes.RemoveAccesses),
      Dependency(firrtl.passes.ZeroLengthVecs),
      Dependency(MyExpandWhens)
      )).execute(outerStateHigh)

    println("High to Mid firrtl lowering done")

    val outerState = new Compiler(Forms.LowForm ++ Seq(Dependency(PreLinkRenaming)))
      .execute(CircuitState(chirrtl, ChirrtlForm, outerAnnos))

    val outerCircuit = outerState.circuit
    val targetType = module_type((innerCircuit.modules find (_.name == innerCircuit.main)).get)
    val targetBoxInstTarget = outerState.annotations.collectFirst({
      case TargetBoxAnnotation(it: InstanceTarget) => it
    }).getOrElse(throw new Exception("TargetBoxAnnotation not found or annotated top module!"))
    val targetBoxParent = targetBoxInstTarget.encapsulatingModule
    val targetBoxInst = targetBoxInstTarget.instance
    val modules = (outerCircuit.modules flatMap
      init(innerCircuit.info, innerCircuit.main, targetType, targetBoxParent, targetBoxInst)) ++
      innerCircuit.modules
    // Rename the annotations from the inner module, which are using an obsolete CircuitName
    val renameMap = RenameMap(
      Map(CircuitName(innerCircuit.main) -> Seq(CircuitName(outerCircuit.main))))

    val innerAnnos = loweredInnerState.annotations.filter(_ match {
      case _: midas.targetutils.FAMEAnnotation => false
      case _ => true
    })

    val linkedState = CircuitState(
      circuit     = Circuit(outerCircuit.info, modules, outerCircuit.main),
      form        = HighForm,
      annotations = innerAnnos ++ outerState.annotations,
      renames     = Some(renameMap)
    )
    writeState(linkedState, "post-sim-mapping.fir")
    linkedState.copy(annotations = linkedState.annotations ++ generateHeaderAnnos(shim))
  }
}

case class InfoExpr(info: firrtl.ir.Info, expr: firrtl.ir.Expression) extends firrtl.ir.Expression {
  def foreachExpr(f:  Expression => Unit):       Unit = f(expr)
  def foreachType(f:  Type => Unit):             Unit = ()
  def foreachWidth(f: Width => Unit):            Unit = ()
  def mapExpr(f:      Expression => Expression): Expression = this.copy(expr = f(this.expr))
  def mapType(f:      Type => Type):             Expression = this
  def mapWidth(f:     Width => Width):           Expression = this
  def tpe: Type = expr.tpe

  // Members declared in firrtl.ir.FirrtlNode
  override def serialize: String = s"(${expr.serialize}: ${info.serialize})"
}


object InfoExpr {
  def wrap(info: Info, expr: Expression): Expression =
    if (info == NoInfo) expr else InfoExpr(info, expr)

  def unwrap(expr: Expression): (Info, Expression) = expr match {
    case InfoExpr(i, e) => (i, e)
    case other          => (NoInfo, other)
  }

  def orElse(info: Info, alt: => Info): Info = if (info == NoInfo) alt else info

  // TODO this the right name?
  def map(expr: Expression)(f: Expression => Expression): Expression = expr match {
    case ie: InfoExpr => ie.mapExpr(f)
    case e => f(e)
  }
}
object MyExpandWhens extends Pass {

  override def prerequisites =
    Seq(
      Dependency(firrtl.passes.PullMuxes),
      Dependency(firrtl.passes.ReplaceAccesses),
      Dependency(firrtl.passes.ExpandConnects),
      Dependency(firrtl.passes.RemoveAccesses)
    ) ++ firrtl.stage.Forms.Resolved

  override def invalidates(a: Transform): Boolean = a match {
    case CheckInitialization | ResolveKinds | InferTypes => true
    case _                                               => false
  }

  /** Returns circuit with when and last connection semantics resolved */
  def run(c: Circuit): Circuit = {
    println("running MyExpandWhens")
    val modulesx = c.modules.map {
      case m: ExtModule => m
      case m: Module    => onModule(m)
    }
    Circuit(c.info, modulesx, c.main)
  }

  /** Maps an expression to a declared node name. Used to memoize predicates */
  @deprecated("This will be removed in FIRRTL 1.4.0", "FIRRTL 1.3.2")
  type NodeMap = mutable.HashMap[firrtl.MemoizedHash[Expression], String]

  private type NodeLookup = mutable.HashMap[firrtl.WrappedExpression, String]

  /** Maps a reference to whatever connects to it. Used to resolve last connect semantics */
  type Netlist = mutable.LinkedHashMap[firrtl.WrappedExpression, firrtl.ir.Expression]

  /** Contains all simulation constructs */
  type Simlist = mutable.ArrayBuffer[firrtl.ir.Statement]

  /** List of all netlists of each declared scope, ordered from closest to farthest
    * @note Note immutable.Map because conversion from mutable.LinkedHashMap to mutable.Map is VERY slow
    */
  type Defaults = Seq[mutable.Map[firrtl.WrappedExpression, firrtl.ir.Expression]]

  /** Expands a module's when statements */
  private def onModule(m: Module): Module = {
    val namespace = firrtl.Namespace(m)
    val simlist = new Simlist

    // Memoizes if an expression contains any WVoids inserted in this pass
    val memoizedVoid = new mutable.HashSet[firrtl.WrappedExpression] += WrappedExpression(firrtl.WVoid)

    // Does an expression contain WVoid inserted in this pass?
    def containsVoid(e: Expression): Boolean = e match {
      case firrtl.WVoid                => true
      case firrtl.ir.ValidIf(_, value, _) => memoizedVoid(WrappedExpression(value))
      case firrtl.ir.Mux(_, tv, fv, _)    => memoizedVoid(WrappedExpression(tv)) || memoizedVoid(WrappedExpression(fv))
      case _                    => false
    }

    // Memoizes the node that holds a particular expression, if any
    val nodes = new NodeLookup

    // Seq of attaches in order
    lazy val attaches = mutable.ArrayBuffer.empty[firrtl.ir.Attach]

    /* Removes connections/attaches from the statement
     * Mutates namespace, simlist, nodes, attaches
     * Mutates input netlist
     * @param netlist maps references to their values for a given immediate scope
     * @param defaults sequence of netlists of surrouding scopes, ordered closest to farthest
     * @param p predicate so far, used to update simulation constructs
     * @param s statement to expand
     */
    def expandWhens(netlist: Netlist, defaults: Defaults, p: firrtl.ir.Expression)(s: firrtl.ir.Statement): Statement = s match {
      // For each non-register declaration, update netlist with value WVoid for each sink reference
      // Return self, unchanged
      case stmt @ (_: DefNode | EmptyStmt) => stmt
      case w: DefWire =>
        netlist ++= (getSinkRefs(w.name, w.tpe, DuplexFlow).map(ref => we(ref) -> WVoid))
        w
      case w: DefMemory =>
        netlist ++= (getSinkRefs(w.name, MemPortUtils.memType(w), SourceFlow).map(ref => we(ref) -> WVoid))
        w
      case w: WDefInstance =>
        netlist ++= (getSinkRefs(w.name, w.tpe, SourceFlow).map(ref => we(ref) -> WVoid))
        w
      case r: DefRegister =>
        // Update netlist with self reference for each sink reference
        netlist ++= getSinkRefs(r.name, r.tpe, DuplexFlow).map(ref => we(ref) -> InfoExpr(r.info, ref))
        r
      // For value assignments, update netlist/attaches and return EmptyStmt
      case c: Connect =>
        netlist(firrtl.WrappedExpression(c.loc)) = InfoExpr(c.info, c.expr)
        EmptyStmt
      case c: IsInvalid =>
        netlist(firrtl.WrappedExpression(c.expr)) = WInvalid
        EmptyStmt
      case a: Attach =>
        attaches += a
        EmptyStmt
      // For simulation constructs, update simlist with predicated statement and return EmptyStmt
      case sx: Print =>
        simlist += (if (weq(p, firrtl.Utils.one)) sx else sx.withEn(firrtl.PrimOps.And(p, sx.en)))
        EmptyStmt
      case sx: Stop =>
        simlist += (if (weq(p, firrtl.Utils.one)) sx else sx.withEn(firrtl.PrimOps.And(p, sx.en)))
        EmptyStmt
      case sx: Verification =>
        simlist += (if (weq(p, firrtl.Utils.one)) sx else sx.withEn(firrtl.PrimOps.And(p, sx.en)))
        EmptyStmt
      // Expand conditionally, see comments below
      case sx: Conditionally =>
        /* 1) Recurse into conseq and alt with empty netlist, updated defaults, updated predicate
         * 2) For each assigned reference (lvalue) in either conseq or alt, get merged value
         *   a) Find default value from defaults
         *   b) Create Mux, ValidIf or WInvalid, depending which (or both) conseq/alt assigned lvalue
         * 3) If a merged value has been memoized, update netlist. Otherwise, memoize then update netlist.
         * 4) Return conseq and alt declarations, followed by memoized nodes
         */
        val conseqNetlist = new Netlist
        val altNetlist = new Netlist
        val conseqStmt = expandWhens(conseqNetlist, netlist +: defaults, AND(p, sx.pred))(sx.conseq)
        val altStmt = expandWhens(altNetlist, netlist +: defaults, AND(p, NOT(sx.pred)))(sx.alt)

        // Process combined maps because we only want to create 1 mux for each node
        //   present in the conseq and/or alt
        val memos = (conseqNetlist ++ altNetlist).map {
          case (lvalue, _) =>
            // Defaults in netlist get priority over those in defaults
            val default = netlist.get(lvalue) match {
              case Some(v) => Some(v)
              case None    => getDefault(lvalue, defaults)
            }
            // info0 and info1 correspond to Mux infos, use info0 only if ValidIf
            val (res, info0, info1) = default match {
              case Some(defaultValue) =>
                val (tinfo, trueValue) = InfoExpr.unwrap(conseqNetlist.getOrElse(lvalue, defaultValue))
                val (finfo, falseValue) = InfoExpr.unwrap(altNetlist.getOrElse(lvalue, defaultValue))
                (trueValue, falseValue) match {
                  case (WInvalid, WInvalid) => (WInvalid, NoInfo, NoInfo)
                  case (WInvalid, fv)       => (ValidIf(NOT(sx.pred), fv, fv.tpe), finfo, NoInfo)
                  case (tv, WInvalid)       => (ValidIf(sx.pred, tv, tv.tpe), tinfo, NoInfo)
                  case (tv, fv)             => (Mux(sx.pred, tv, fv, firrtl.Utils.mux_type_and_widths(tv, fv)), tinfo, finfo)
                }
              case None =>
                // Since not in netlist, lvalue must be declared in EXACTLY one of conseq or alt
                (conseqNetlist.getOrElse(lvalue, altNetlist(lvalue)), NoInfo, NoInfo)
            }

            res match {
              // Don't create a node to hold mux trees with void values
              // "Idiomatic" emission of these muxes isn't a concern because they represent bad code (latches)
              case e if containsVoid(e) =>
                netlist(lvalue) = e
                memoizedVoid += WrappedExpression(e) // remember that this was void
                EmptyStmt
              case _: ValidIf | _: Mux | _: DoPrim =>
                nodes.get(WrappedExpression(res)) match {
                  case Some(name) =>
                    netlist(lvalue) = WRef(name, res.tpe, NodeKind, SourceFlow)
                    EmptyStmt
                  case None =>
                    val name = namespace.newTemp
                    nodes(WrappedExpression(res)) = name
                    netlist(lvalue) = WRef(name, res.tpe, NodeKind, SourceFlow)
                    // Use MultiInfo constructor to preserve NoInfos
                    val info = new MultiInfo(List(sx.info, info0, info1))
                    DefNode(info, name, res)
                }
              case _ =>
                netlist(lvalue) = res
                EmptyStmt
            }
        }
        Block(Seq(conseqStmt, altStmt) ++ memos)
      case block: Block => block.map(expandWhens(netlist, defaults, p))
      case _ => firrtl.Utils.throwInternalError()
    }
    val netlist = new Netlist
    // Add ports to netlist
    netlist ++= (m.ports.flatMap {
      case Port(_, name, dir, tpe) =>
        getSinkRefs(name, tpe, firrtl.Utils.to_flow(dir)).map(ref => we(ref) -> WVoid)
    })
    // Do traversal and construct mutable datastructures
    val bodyx = expandWhens(netlist, Seq(netlist), firrtl.Utils.one)(m.body)

    val attachedAnalogs = attaches.flatMap(_.exprs.map(we)).toSet
    val newBody = Block(
      Seq(firrtl.Utils.squashEmpty(bodyx)) ++ expandNetlist(netlist, attachedAnalogs) ++
        combineAttaches(attaches.toSeq) ++ simlist
    )
    Module(m.info, m.name, m.ports, newBody)
  }

  /** Returns all references to all sink leaf subcomponents of a reference */
  private def getSinkRefs(n: String, t: Type, g: Flow): Seq[Expression] = {
    val exps = firrtl.Utils.create_exps(WRef(n, t, ExpKind, g))
    exps.flatMap {
      case exp =>
        exp.tpe match {
          case AnalogType(w) => None
          case _ =>
            firrtl.Utils.flow(exp) match {
              case (DuplexFlow | SinkFlow) => Some(exp)
              case _                       => None
            }
        }
    }
  }

  /** Returns all connections/invalidations in the circuit
    * @note Remove IsInvalids on attached Analog-typed components
    */
  private def expandNetlist(netlist: Netlist, attached: Set[WrappedExpression]) = {
    // Remove IsInvalids on attached Analog types
    def handleInvalid(k: WrappedExpression, info: Info): Statement =
      if (attached.contains(k)) EmptyStmt else IsInvalid(info, k.e1)
    netlist.map {
      case (k, WInvalid)                 => handleInvalid(k, NoInfo)
      case (k, InfoExpr(info, WInvalid)) => handleInvalid(k, info)
      case (k, v) =>
        val (info, expr) = InfoExpr.unwrap(v)
        Connect(info, k.e1, expr)
    }
  }

  /** Returns new sequence of combined Attaches
    * @todo Preserve Info
    */
  private def combineAttaches(attaches: Seq[Attach]): Seq[Attach] = {
    // Helper type to add an ordering index to attached Expressions
    case class AttachAcc(exprs: Seq[WrappedExpression], idx: Int)
    // Map from every attached expression to its corresponding AttachAcc
    //   (many keys will point to same value)
    val attachMap = mutable.LinkedHashMap.empty[WrappedExpression, AttachAcc]
    for (Attach(_, es) <- attaches) {
      val exprs = es.map(we(_))
      val acc = exprs.map(attachMap.get(_)).flatten match {
        case Seq() => // None of these expressions is present in the attachMap
          AttachAcc(exprs, attachMap.size)
        case accs => // At least one expression present in the attachMap
          val sorted = accs.sortBy(_.idx)
          AttachAcc((sorted.map(_.exprs) :+ exprs).flatten.distinct, sorted.head.idx)
      }
      attachMap ++= acc.exprs.map(_ -> acc)
    }
    attachMap.values.toList.distinct.map(acc => Attach(NoInfo, acc.exprs.map(_.e1)))
  }
  // Searches nested scopes of defaults for lvalue
  // defaults uses mutable Map because we are searching LinkedHashMaps and conversion to immutable is VERY slow
  @tailrec
  private def getDefault(lvalue: WrappedExpression, defaults: Defaults): Option[Expression] = {
    defaults match {
      case Nil => None
      case head :: tail =>
        head.get(lvalue) match {
          case Some(p) => Some(p)
          case None    => getDefault(lvalue, tail)
        }
    }
  }

  private def AND(e1: Expression, e2: Expression) =
    DoPrim(firrtl.PrimOps.And, Seq(e1, e2), Nil, firrtl.Utils.BoolType)
  private def NOT(e: Expression) =
    DoPrim(firrtl.PrimOps.Eq, Seq(e, firrtl.Utils.zero), Nil, firrtl.Utils.BoolType)
}

class MyExpandWhensAndCheck extends Transform with DependencyAPIMigration {

  override def prerequisites =
    Seq(
      Dependency(firrtl.passes.PullMuxes),
      Dependency(firrtl.passes.ReplaceAccesses),
      Dependency(firrtl.passes.ExpandConnects),
      Dependency(firrtl.passes.RemoveAccesses)
    ) ++ firrtl.stage.Forms.Deduped

  override def invalidates(a: Transform): Boolean = a match {
    case ResolveKinds | InferTypes | ResolveFlows | _: InferWidths => true
    case _ => false
  }

  override def execute(a: CircuitState): CircuitState =
    Seq(MyExpandWhens, CheckInitialization).foldLeft(a) { case (acc, tx) => tx.transform(acc) }

}
