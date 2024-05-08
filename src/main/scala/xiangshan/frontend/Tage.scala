/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils._
import xs.utils.mbist.MBISTPipeline
import xs.utils.sram.{FoldedSRAMTemplate, SRAMTemplate}

import scala.math.min
import scala.util.matching.Regex
import scala.{Tuple2 => &}
import os.followLink
import xs.utils.perf.HasPerfLogging

trait TageParams extends HasBPUConst with HasXSParameter {
  // println(BankTageTableInfos)
  val TageNTables = TageTableInfos.size
  // val BankTageNTables = BankTageTableInfos.map(_.size) // Number of tage tables
  // val UBitPeriod = 256
  val TageCtrBits = 3
  val TickWidth = 7

  val USE_ALT_ON_NA_WIDTH = 4
  val NUM_USE_ALT_ON_NA = 128
  def use_alt_idx(pc: UInt) = (pc >> instOffsetBits)(log2Ceil(NUM_USE_ALT_ON_NA)-1, 0)

  val altCtrsNum = 128
  val alterCtrBits = 4

  val TotalBits = TageTableInfos.map {
    case (s, h, t) => {
      s * (1+t+TageCtrBits+1)
    }
  }.reduce(_+_)

  def posUnconf(ctr: UInt) = ctr === (1 << (ctr.getWidth - 1)).U
  def negUnconf(ctr: UInt) = ctr === ((1 << (ctr.getWidth - 1)) - 1).U

  def unconf(ctr: UInt) = posUnconf(ctr) || negUnconf(ctr)

  // val unshuffleBitWidth = log2Ceil(numBr)
  // def get_unshuffle_bits(idx: UInt) = idx(unshuffleBitWidth-1, 0)
  // xor hashes are reversable
  // def get_phy_br_idx(unhashed_idx: UInt, br_lidx: Int)  = get_unshuffle_bits(unhashed_idx) ^ br_lidx.U(log2Ceil(numBr).W)
  // def get_lgc_br_idx(unhashed_idx: UInt, br_pidx: UInt) = get_unshuffle_bits(unhashed_idx) ^ br_pidx

  def get_phy_br_idx(unhashed_idx: UInt, br_lidx: Int)  = 0.U
  def get_lgc_br_idx(unhashed_idx: UInt, br_pidx: UInt) = 0.U
}

trait HasFoldedHistory {
  val histLen: Int
  def compute_folded_hist(hist: UInt, l: Int)(histLen: Int) = {
    if (histLen > 0) {
      val nChunks = (histLen + l - 1) / l
      val hist_chunks = (0 until nChunks) map {i =>
        hist(min((i+1)*l, histLen)-1, i*l)
      }
      ParallelXOR(hist_chunks)
    }
    else 0.U
  }
  val compute_folded_ghist = compute_folded_hist(_: UInt, _: Int)(histLen)
}

abstract class TageBundle(implicit p: Parameters)
  extends XSBundle with TageParams with BPUUtils

abstract class TageModule(implicit p: Parameters)
  extends XSModule with TageParams with BPUUtils
  {}



class TageReq(implicit p: Parameters) extends TageBundle {
  val pc         = UInt(VAddrBits.W)
  val ghist      = UInt(HistoryLength.W)
  val foldedHist = new AllFoldedHistories(foldedGHistInfos)
}

class TageResp(implicit p: Parameters) extends TageBundle {
  val ctr    = UInt(TageCtrBits.W)
  val u      = Bool()
  val unconf = Bool()
  val wayIdx = UInt(2.W)
}

class TageUpdate(implicit p: Parameters) extends TageBundle {
  // val pc = UInt(VAddrBits.W)
  // val folded_hist = new AllFoldedHistories(foldedGHistInfos)
  // val ghist = UInt(HistoryLength.W)
  // // update tag and ctr
  // val mask = Vec(numBr, Bool())
  // val takens = Vec(numBr, Bool())
  // val alloc = Vec(numBr, Bool())
  // val oldCtrs = Vec(numBr, UInt(TageCtrBits.W))
  // // update u
  // val uMask = Vec(numBr, Bool())
  // val us = Vec(numBr, Bool())
  // val reset_u = Vec(numBr, Bool())
  val pc         = UInt(VAddrBits.W)
  val foldedHist = new AllFoldedHistories(foldedGHistInfos)
  val ghist      = UInt(HistoryLength.W)
  val mask       = Vec(numBr, Bool())
  val takens     = Vec(numBr, Bool())
  val alloc      = Vec(numBr, Bool())
  val oldCtrs    = Vec(numBr, UInt(TageCtrBits.W))
  val wayIdx     = UInt(2.W)
  val uMask      = Vec(numBr, Bool())
  val us         = Vec(numBr, Bool())
  val reset_u    = Vec(numBr, Bool())
}

class TageMeta(implicit p: Parameters)
  extends TageBundle with HasSCParameter
{
  val providers = Vec(numBr, ValidUndirectioned(UInt(log2Ceil(TageNTables).W)))
  val providerResps = Vec(numBr, new TageResp)
  // val altProviders = Vec(numBr, ValidUndirectioned(UInt(log2Ceil(TageNTables).W)))
  // val altProviderResps = Vec(numBr, new TageResp)
  val altUsed = Vec(numBr, Bool())
  val altDiffers = Vec(numBr, Bool())
  val basecnts = Vec(numBr, UInt(2.W))
  val allocates = Vec(numBr, UInt(TageNTables.W))
  val takens = Vec(numBr, Bool())
  val scMeta = if (EnableSC) Some(new SCMeta(SCNTables)) else None
  val pred_cycle = if (!env.FPGAPlatform) Some(UInt(64.W)) else None
  val use_alt_on_na = if (!env.FPGAPlatform) Some(Vec(numBr, Bool())) else None

  def altPreds = basecnts.map(_(1))
  def allocateValid = allocates.map(_.orR)
}

trait TBTParams extends HasXSParameter with TageParams {
  val BtSize = 2048
  val bypassEntries = 8
}


// class TageBTable(parentName:String = "Unknown")(implicit p: Parameters) extends XSModule with TBTParams{
//   val io = IO(new Bundle {
//     //#2410
//     // val s0_fire = Input(Bool())
//     // val s0_pc   = Input(UInt(VAddrBits.W))
//     //#2410
//     val req = Flipped(DecoupledIO(UInt(VAddrBits.W))) // s0_pc
//     val s1_cnt     = Output(Vec(numBr,UInt(2.W)))
//     val update_mask = Input(Vec(TageBanks, Bool()))
//     val update_pc = Input(UInt(VAddrBits.W))
//     val update_cnt  = Input(Vec(numBr,UInt(2.W)))
//     val update_takens = Input(Vec(TageBanks, Bool()))
//    // val update  = Input(new TageUpdate)
//   })

//   val bimAddr = new TableAddr(log2Up(BtSize), instOffsetBits)

//   val bt = Module(new SRAMTemplate(UInt(2.W), set = BtSize, way=numBr, shouldReset = true, holdRead = true, bypassWrite = true,
//     hasMbist = coreParams.hasMbist,
//     hasShareBus = coreParams.hasShareBus,
//     parentName = parentName
//   ))
//   val mbistPipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
//     MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistPipe", true)
//   } else {
//     None
//   }

//   val doing_reset = RegInit(true.B)
//   val resetRow = RegInit(0.U(log2Ceil(BtSize).W))
//   resetRow := resetRow + doing_reset
//   when (resetRow === (BtSize-1).U) { doing_reset := false.B }

//   //#2410
//   io.req.ready := !doing_reset
//   // val s0_idx = bimAddr.getIdx(io.s0_pc)
//   // bt.io.r.req.valid := io.s0_fire
//   val s0_pc = io.req.bits
//   val s0_fire = io.req.valid
//   val s0_idx = bimAddr.getIdx(s0_pc)
//   bt.io.r.req.valid := s0_fire
//   bt.io.r.req.bits.setIdx := s0_idx

//   val s1_read = bt.io.r.resp.data

//   //#2410
//   // val s1_idx = RegEnable(s0_idx, io.s0_fire)
//   val s1_idx = RegEnable(s0_idx, s0_fire)


//   val per_br_ctr = VecInit((0 until numBr).map(i => Mux1H(UIntToOH(get_phy_br_idx(s1_idx, i), numBr), s1_read)))
//   io.s1_cnt := per_br_ctr

//   // Update logic

//   val u_idx = bimAddr.getIdx(io.update_pc)

//   val newCtrs = Wire(Vec(numBr, UInt(2.W))) // physical bridx

//   val wrbypass = Module(new WrBypass(UInt(2.W), bypassEntries, log2Up(BtSize), numWays = numBr)) // logical bridx
//   wrbypass.io.wen := io.update_mask.reduce(_||_)
//   wrbypass.io.write_idx := u_idx
//   wrbypass.io.write_way_mask.map(_ := io.update_mask)
//   for (li <- 0 until numBr) {
//     val br_pidx = get_phy_br_idx(u_idx, li)
//     wrbypass.io.write_data(li) := newCtrs(br_pidx)
//   }


//   val oldCtrs =
//     VecInit((0 until numBr).map(pi => {
//       // val br_lidx = get_lgc_br_idx(u_idx, pi.U(log2Ceil(numBr).W))
//       val br_lidx = get_lgc_br_idx(u_idx, pi.U(1.W))
//       Mux(wrbypass.io.hit && wrbypass.io.hit_data(br_lidx).valid,
//         wrbypass.io.hit_data(br_lidx).bits,
//         io.update_cnt(br_lidx))
//     }))

//   def satUpdate(old: UInt, len: Int, taken: Bool): UInt = {
//     val oldSatTaken = old === ((1 << len)-1).U
//     val oldSatNotTaken = old === 0.U
//     Mux(oldSatTaken && taken, ((1 << len)-1).U,
//       Mux(oldSatNotTaken && !taken, 0.U,
//         Mux(taken, old + 1.U, old - 1.U)))
//   }

//   val newTakens = io.update_takens
//   newCtrs := VecInit((0 until numBr).map(pi => {
//     // val br_lidx = get_lgc_br_idx(u_idx, pi.U(log2Ceil(numBr).W))
//     val br_lidx = get_lgc_br_idx(u_idx, pi.U(1.W))
//     satUpdate(oldCtrs(pi), 2, newTakens(br_lidx))
//   }))

//   val updateWayMask = VecInit((0 until numBr).map(pi =>
//     (0 until numBr).map(li =>
//       io.update_mask(li) && get_phy_br_idx(u_idx, li) === pi.U  
//     ).reduce(_||_)
//   )).asUInt

//   bt.io.w.apply(
//     valid = io.update_mask.reduce(_||_) || doing_reset,
//     data = Mux(doing_reset, VecInit(Seq.fill(numBr)(2.U(2.W))), newCtrs),
//     setIdx = Mux(doing_reset, resetRow, u_idx),
//     waymask = Mux(doing_reset, Fill(numBr, 1.U(1.W)).asUInt, updateWayMask)
//   )

// }

class TageBTable(parentName:String = "Unknown")(implicit p: Parameters) extends XSModule with TBTParams{
  val io = IO(new Bundle {
    val req          = Flipped(DecoupledIO(UInt(VAddrBits.W)))
    val cnt          = Output(Vec(numBr,UInt(2.W)))
    val updateMask   = Input(Vec(TageBanks, Bool()))
    val updatePC     = Input(UInt(VAddrBits.W))
    val updateCnt    = Input(Vec(numBr,UInt(2.W)))
    val updateTakens = Input(Vec(TageBanks, Bool()))
  })

  def satUpdate(old: UInt, len: Int, taken: Bool): UInt = {
    val oldSatTaken = old === ((1 << len)-1).U
    val oldSatNotTaken = old === 0.U
    Mux(oldSatTaken && taken, ((1 << len)-1).U,
      Mux(oldSatNotTaken && !taken, 0.U,
        Mux(taken, old + 1.U, old - 1.U)))
  }

  val bt = Module(new SRAMTemplate(UInt(2.W), set = BtSize, way=numBr, shouldReset = true,
    holdRead = true, bypassWrite = true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName
  ))
  val mbistPipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistPipe", true)
  } else {
    None
  }

  // reset
  val btReset = RegInit(true.B)
  val resetRow = RegInit(0.U(log2Ceil(BtSize).W))
  resetRow := resetRow + btReset
  when (resetRow === (BtSize-1).U) { btReset := false.B }

  // read bt
  val addr = new TableAddr(log2Up(BtSize), instOffsetBits)
  io.req.ready := !btReset
  val s0PC   = io.req.bits
  val s0Fire = io.req.valid
  val s0Idx  = addr.getIdx(s0PC)
  bt.io.r.req.valid       := s0Fire
  bt.io.r.req.bits.setIdx := s0Idx
  val s1ReadData = bt.io.r.resp.data
  io.cnt := s1ReadData

  // Update logic
  val updtIdx = addr.getIdx(io.updatePC)
  val newCtrs = Wire(Vec(numBr, UInt(2.W)))

  val wrbypass = Module(new WrBypass(UInt(2.W), bypassEntries, log2Up(BtSize), numWays = numBr)) // logical bridx
  wrbypass.io.wen        := io.updateMask.reduce(_||_)
  wrbypass.io.write_idx  := updtIdx
  wrbypass.io.write_way_mask.map(_ := io.updateMask)
  wrbypass.io.write_data := newCtrs
  val oldCtrs = Mux(wrbypass.io.hit && wrbypass.io.hit_data(0).valid, wrbypass.io.hit_data(0).bits,
                                                                      io.updateCnt(0))
  newCtrs(0) := satUpdate(oldCtrs, 2, io.updateTakens(0))

  bt.io.w.apply(
    valid   = io.updateMask.reduce(_||_) || btReset,
    data    = Mux(btReset, VecInit(Seq.fill(numBr)(2.U(2.W))), newCtrs),
    setIdx  = Mux(btReset, resetRow, updtIdx),
    waymask = Mux(btReset, Fill(numBr, 1.U(1.W)).asUInt, io.updateMask.asUInt)
  )
}




// class TageTable
// (
//   val nRows: Int, val histLen: Int, val tagLen: Int, val tableIdx: Int, parentName:String = "Unknown"
// )(implicit p: Parameters)
//   extends TageModule with HasFoldedHistory with HasPerfLogging {
//   val io = IO(new Bundle() {
//     val req = Flipped(DecoupledIO(new TageReq))
//     val resps = Output(Vec(numBr, Valid(new TageResp)))
//     val update = Input(new TageUpdate)
//   })

//   class TageEntry() extends TageBundle {
//     val valid = Bool()
//     val tag = UInt(tagLen.W)
//     val ctr = UInt(TageCtrBits.W)
//   }


//   val SRAM_SIZE = 256 // physical size
//   require(nRows % SRAM_SIZE == 0)
//   require(isPow2(numBr))
//   val nRowsPerBr = nRows / numBr
//   val nBanks = 8
//   val bankSize = nRowsPerBr / nBanks
//   val bankFoldWidth = if (bankSize >= SRAM_SIZE) bankSize / SRAM_SIZE else 1
//   val uFoldedWidth = nRowsPerBr / SRAM_SIZE
//   if (bankSize < SRAM_SIZE) {
//     println(f"warning: tage table $tableIdx has small sram depth of $bankSize")
//   }
//   val bankIdxWidth = log2Ceil(nBanks)
//   def get_bank_mask(idx: UInt) = VecInit((0 until nBanks).map(idx(bankIdxWidth-1, 0) === _.U))
//   def get_bank_idx(idx: UInt) = idx >> bankIdxWidth
//   def get_way_in_bank(idx: UInt) =
//     if (log2Ceil(bankFoldWidth) > 0)
//       (idx >> bankIdxWidth)(log2Ceil(bankFoldWidth)-1, 0)
//     else
//       0.U(1.W)



//   // bypass entries for tage update
//   val perBankWrbypassEntries = 8

//   val idxFhInfo = (histLen, min(log2Ceil(nRowsPerBr), histLen))
//   val tagFhInfo = (histLen, min(histLen, tagLen))
//   val altTagFhInfo = (histLen, min(histLen, tagLen-1))
//   val allFhInfos = Seq(idxFhInfo, tagFhInfo, altTagFhInfo)

//   def getFoldedHistoryInfo = allFhInfos.filter(_._1 >0).toSet
//   def compute_tag_and_hash(unhashed_idx: UInt, allFh: AllFoldedHistories) = {
//     val idx_fh = allFh.getHistWithInfo(idxFhInfo).folded_hist
//     val tag_fh = allFh.getHistWithInfo(tagFhInfo).folded_hist
//     val alt_tag_fh = allFh.getHistWithInfo(altTagFhInfo).folded_hist
//     // require(idx_fh.getWidth == log2Ceil(nRows))
//     val idx = (unhashed_idx ^ idx_fh)(log2Ceil(nRowsPerBr)-1, 0)
//     val tag = (unhashed_idx ^ tag_fh ^ (alt_tag_fh << 1)) (tagLen - 1, 0)
//     (idx, tag)
//   }

//   def inc_ctr(ctr: UInt, taken: Bool): UInt = satUpdate(ctr, TageCtrBits, taken)
  
//   if (EnableGHistDiff) {
//     val idx_history = compute_folded_ghist(io.req.bits.ghist, log2Ceil(nRowsPerBr))
//     val idx_fh = io.req.bits.folded_hist.getHistWithInfo(idxFhInfo)
//     XSError(idx_history =/= idx_fh.folded_hist, p"tage table $tableIdx has different fh," +
//       p" ghist: ${Binary(idx_history)}, fh: ${Binary(idx_fh.folded_hist)}\n")
//   }
//   // pc is start address of basic block, most 2 branch inst in block
//   // def getUnhashedIdx(pc: UInt) = pc >> (instOffsetBits+log2Ceil(TageBanks))
//   def getUnhashedIdx(pc: UInt): UInt = pc >> instOffsetBits

//   // val s1_pc = io.req.bits.pc
//   val req_unhashed_idx = getUnhashedIdx(io.req.bits.pc)

//   val us = Module(new FoldedSRAMTemplate(Bool(), set=nRowsPerBr, width=uFoldedWidth, way=numBr, shouldReset=true, extraReset=true, holdRead=true, singlePort=true,
//     hasMbist = coreParams.hasMbist,
//     hasShareBus = coreParams.hasShareBus,
//     parentName = parentName + "us_"
//   ))
//   us.extra_reset.get := io.update.reset_u.reduce(_||_)


//   val table_banks = Seq.tabulate(nBanks)(idx =>
//     Module(new FoldedSRAMTemplate(new TageEntry, set=bankSize, width=bankFoldWidth, way=numBr, shouldReset=true, holdRead=true, singlePort=true,
//       hasMbist = coreParams.hasMbist,
//       hasShareBus = coreParams.hasShareBus,
//       parentName = parentName + s"table${idx}_"
//     )))

//   val mbistTablePipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
//     MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistTablePipe")
//   } else {
//     None
//   }


//   val (s0_idx, s0_tag) = compute_tag_and_hash(req_unhashed_idx, io.req.bits.folded_hist)
//   val s0_bank_req_1h = get_bank_mask(s0_idx)

//     for (b <- 0 until nBanks) {
//       table_banks(b).io.r.req.valid := io.req.fire && s0_bank_req_1h(b)
//       table_banks(b).io.r.req.bits.setIdx := get_bank_idx(s0_idx)
//     }

//   us.io.r.req.valid := io.req.fire
//   us.io.r.req.bits.setIdx := s0_idx


//   val s1_unhashed_idx = RegEnable(req_unhashed_idx, io.req.fire)
//   val s1_idx = RegEnable(s0_idx, io.req.fire)
//   val s1_tag = RegEnable(s0_tag, io.req.fire)
//   val s1_pc  = RegEnable(io.req.bits.pc, io.req.fire)
//   val s1_bank_req_1h = RegEnable(s0_bank_req_1h, io.req.fire)
//   val s1_bank_has_write_on_this_req = RegEnable(VecInit(table_banks.map(_.io.w.req.valid)), io.req.valid)

//   val resp_invalid_by_write = Wire(Bool())
  
//   val tables_r = table_banks.map(_.io.r.resp.data) // s1
//   val unconfs = tables_r.map(r => VecInit(r.map(e => WireInit(unconf(e.ctr))))) // do unconf cal in parallel
//   val hits = tables_r.map(r => VecInit(r.map(e => e.tag === s1_tag && e.valid && !resp_invalid_by_write))) // do tag compare in parallel
  
//   val resp_selected = Mux1H(s1_bank_req_1h, tables_r)
//   val unconf_selected = Mux1H(s1_bank_req_1h, unconfs)
//   val hit_selected = Mux1H(s1_bank_req_1h, hits)
//   resp_invalid_by_write := Mux1H(s1_bank_req_1h, s1_bank_has_write_on_this_req)


//   val per_br_resp = VecInit((0 until numBr).map(i => Mux1H(UIntToOH(get_phy_br_idx(s1_unhashed_idx, i), numBr), resp_selected)))
//   val per_br_unconf = VecInit((0 until numBr).map(i => Mux1H(UIntToOH(get_phy_br_idx(s1_unhashed_idx, i), numBr), unconf_selected)))
//   val per_br_hit = VecInit((0 until numBr).map(i => Mux1H(UIntToOH(get_phy_br_idx(s1_unhashed_idx, i), numBr), hit_selected)))
//   val per_br_u    = VecInit((0 until numBr).map(i => Mux1H(UIntToOH(get_phy_br_idx(s1_unhashed_idx, i), numBr), us.io.r.resp.data)))

//   for (i <- 0 until numBr) {
//     io.resps(i).valid := per_br_hit(i)
//     io.resps(i).bits.ctr := per_br_resp(i).ctr
//     io.resps(i).bits.u := per_br_u(i)
//     io.resps(i).bits.unconf := per_br_unconf(i)
//   }

//   if (EnableGHistDiff) {
//     val update_idx_history = compute_folded_ghist(io.update.ghist, log2Ceil(nRowsPerBr))
//     val update_idx_fh = io.update.folded_hist.getHistWithInfo(idxFhInfo)
//     XSError(update_idx_history =/= update_idx_fh.folded_hist && io.update.mask.reduce(_||_),
//       p"tage table $tableIdx has different fh when update," +
//       p" ghist: ${Binary(update_idx_history)}, fh: ${Binary(update_idx_fh.folded_hist)}\n")
//   }
//   // Use fetchpc to compute hash
//   val per_bank_update_wdata = Wire(Vec(nBanks, Vec(numBr, new TageEntry))) // corresponds to physical branches

//   val update_unhashed_idx = getUnhashedIdx(io.update.pc)
//   val (update_idx, update_tag) = compute_tag_and_hash(update_unhashed_idx, io.update.folded_hist)
//   val update_req_bank_1h = get_bank_mask(update_idx)
//   val update_idx_in_bank = get_bank_idx(update_idx)
  
//   val per_bank_not_silent_update = Wire(Vec(nBanks, Vec(numBr, Bool()))) // corresponds to physical branches
//   val per_bank_update_way_mask =
//     VecInit((0 until nBanks).map(b =>
//       VecInit((0 until numBr).map(pi => {
//         // whether any of the logical branches updates on each slot
//         Seq.tabulate(numBr)(li =>
//           get_phy_br_idx(update_unhashed_idx, li) === pi.U &&
//           io.update.mask(li)).reduce(_||_) && per_bank_not_silent_update(b)(pi)
//       })).asUInt
//     ))

//   // val silent_update_from_wrbypass = Wire(Bool())

//   for (b <- 0 until nBanks) {
//     table_banks(b).io.w.apply(
//       valid   = per_bank_update_way_mask(b).orR && update_req_bank_1h(b),
//       data    = per_bank_update_wdata(b),
//       setIdx  = update_idx_in_bank,
//       waymask = per_bank_update_way_mask(b)
//     )
//   }

//   //#2410
//   // Power-on reset
//   val powerOnResetState = RegInit(true.B)
//   when(us.io.r.req.ready && table_banks.map(_.io.r.req.ready).reduce(_ && _)) {
//     // When all the SRAM first reach ready state, we consider power-on reset is done
//     powerOnResetState := false.B
//   }
//   // Do not use table banks io.r.req.ready directly
//   // All the us & table_banks are single port SRAM, ready := !wen
//   // We do not want write request block the whole BPU pipeline
//   io.req.ready := !powerOnResetState
//   val bank_conflict = (0 until nBanks).map(b => table_banks(b).io.w.req.valid && s0_bank_req_1h(b)).reduce(_||_)
//   // io.req.ready := true.B //#2410
//   // io.req.ready := !(io.update.mask && not_silent_update)
//   // io.req.ready := !bank_conflict
//   XSPerfAccumulate(f"tage_table_bank_conflict", bank_conflict)

//   val update_u_idx = update_idx
//   val update_u_way_mask = VecInit((0 until numBr).map(pi => {
//     Seq.tabulate(numBr)(li =>
//       get_phy_br_idx(update_unhashed_idx, li) === pi.U &&
//       io.update.uMask(li)
//     ).reduce(_||_)
//   })).asUInt

//   val update_u_wdata = VecInit((0 until numBr).map(pi =>
//     Mux1H(Seq.tabulate(numBr)(li =>
//       (get_phy_br_idx(update_unhashed_idx, li) === pi.U, io.update.us(li))
//     ))
//   ))

//   us.io.w.apply(io.update.uMask.reduce(_||_), update_u_wdata, update_u_idx, update_u_way_mask)
  
//   // remove silent updates
//   def silentUpdate(ctr: UInt, taken: Bool) = {
//     ctr.andR && taken || !ctr.orR && !taken
//   }

//   val bank_wrbypasses = Seq.fill(nBanks)(Seq.fill(numBr)(
//     Module(new WrBypass(UInt(TageCtrBits.W), perBankWrbypassEntries, 1, tagWidth=tagLen))
//   )) // let it corresponds to logical brIdx

//   for (b <- 0 until nBanks) {
//     val not_silent_update = per_bank_not_silent_update(b)
//     for (pi <- 0 until numBr) { // physical brIdx 
//       val update_wdata = per_bank_update_wdata(b)(pi)
//       // val br_lidx = get_lgc_br_idx(update_unhashed_idx, pi.U(log2Ceil(numBr).W))
//       val br_lidx = get_lgc_br_idx(update_unhashed_idx, pi.U(1.W))
//       // this 
//       val wrbypass_io = Mux1H(UIntToOH(br_lidx, numBr), bank_wrbypasses(b).map(_.io))
//       val wrbypass_hit = wrbypass_io.hit
//       val wrbypass_ctr = wrbypass_io.hit_data(0).bits
//       val wrbypass_data_valid = wrbypass_hit && wrbypass_io.hit_data(0).valid
//       update_wdata.ctr :=
//         Mux(io.update.alloc(br_lidx),
//           Mux(io.update.takens(br_lidx), 4.U, 3.U),
//           Mux(wrbypass_data_valid,
//             inc_ctr(wrbypass_ctr,               io.update.takens(br_lidx)),
//             inc_ctr(io.update.oldCtrs(br_lidx), io.update.takens(br_lidx))
//           )
//         )
//       not_silent_update(pi) :=
//         Mux(wrbypass_data_valid,
//           !silentUpdate(wrbypass_ctr,               io.update.takens(br_lidx)),
//           !silentUpdate(io.update.oldCtrs(br_lidx), io.update.takens(br_lidx))) ||
//         io.update.alloc(br_lidx)

//       update_wdata.valid := true.B
//       update_wdata.tag   := update_tag
//     }

//     for (li <- 0 until numBr) {
//       val wrbypass = bank_wrbypasses(b)(li)
//       val br_pidx = get_phy_br_idx(update_unhashed_idx, li)
//       wrbypass.io.wen := io.update.mask(li) && update_req_bank_1h(b)
//       wrbypass.io.write_idx := get_bank_idx(update_idx)
//       wrbypass.io.write_tag.map(_ := update_tag)
//       wrbypass.io.write_data(0) := Mux1H(UIntToOH(br_pidx, numBr), per_bank_update_wdata(b)).ctr
//     }
//   }

//   for (i <- 0 until numBr) {
//     for (b <- 0 until nBanks) {
//       val wrbypass = bank_wrbypasses(b)(i)
//       XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_enq_$i", io.update.mask(i) && update_req_bank_1h(b) && !wrbypass.io.hit)
//       XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_hit_$i", io.update.mask(i) && update_req_bank_1h(b) &&  wrbypass.io.hit)
//     }
//   }

//   for (b <- 0 until nBanks) {
//     val not_silent_update = per_bank_not_silent_update(b)
//     XSPerfAccumulate(f"tage_table_bank_${b}_real_updates",
//       io.update.mask.reduce(_||_) && update_req_bank_1h(b) && not_silent_update.reduce(_||_))
//     XSPerfAccumulate(f"tage_table_bank_${b}_silent_updates_eliminated",
//       io.update.mask.reduce(_||_) && update_req_bank_1h(b) && !not_silent_update.reduce(_||_))
//   }

//   XSPerfAccumulate("tage_table_hits", PopCount(io.resps.map(_.valid)))
  
//   for (b <- 0 until nBanks) {
//     XSPerfAccumulate(f"tage_table_bank_${b}_update_req", io.update.mask.reduce(_||_) && update_req_bank_1h(b))
//     for (i <- 0 until numBr) {
//       val li = i
//       val pidx = get_phy_br_idx(update_unhashed_idx, li)
//       //XSPerfAccumulate(f"tage_table_bank_${b}_br_li_${li}_updated", table_banks(b).io.w.req.valid && table_banks(b).io.w.req.bits.waymask.get(pidx))
//       val pi = i
//       //XSPerfAccumulate(f"tage_table_bank_${b}_br_pi_${pi}_updated", table_banks(b).io.w.req.valid && table_banks(b).io.w.req.bits.waymask.get(pi))
//     }
//   }

//   val u = io.update
//   val b = PriorityEncoder(u.mask)
//   val ub = PriorityEncoder(u.uMask)
//   XSDebug(io.req.fire,
//     p"tableReq: pc=0x${Hexadecimal(io.req.bits.pc)}, " +
//     p"idx=$s0_idx, tag=$s0_tag\n")
//   for (i <- 0 until numBr) {
//     XSDebug(RegNext(io.req.fire) && per_br_hit(i),
//       p"TageTableResp_br_$i: idx=$s1_idx, hit:${per_br_hit(i)}, " +
//       p"ctr:${io.resps(i).bits.ctr}, u:${io.resps(i).bits.u}\n")
//     XSDebug(io.update.mask(i),
//       p"update Table_br_$i: pc:${Hexadecimal(u.pc)}}, " +
//       p"taken:${u.takens(i)}, alloc:${u.alloc(i)}, oldCtrs:${u.oldCtrs(i)}\n")
//     val bank = OHToUInt(update_req_bank_1h.asUInt, nBanks)
//     val pi = get_phy_br_idx(update_unhashed_idx, i)
//     XSDebug(io.update.mask(i),
//       p"update Table_$i: writing tag:$update_tag, " +
//       p"ctr: ${per_bank_update_wdata(bank)(pi).ctr} in idx ${update_idx}\n")
//     XSDebug(RegNext(io.req.fire) && !per_br_hit(i), p"TageTableResp_$i: not hit!\n")
//   }

//   // ------------------------------Debug-------------------------------------
//   val valids = RegInit(VecInit(Seq.fill(nRows)(false.B)))
//   when (io.update.mask.reduce(_||_)) { valids(update_idx) := true.B }
//   XSDebug("Table usage:------------------------\n")
//   XSDebug("%d out of %d rows are valid\n", PopCount(valids), nRows.U)

// }

class TageTable
(
  val nRows: Int, val histLen: Int, val tagLen: Int, val tableIdx: Int, parentName:String = "Unknown"
)(implicit p: Parameters)
  extends TageModule with HasFoldedHistory with HasPerfLogging {
  val io = IO(new Bundle() {
    val req = Flipped(DecoupledIO(new TageReq))
    val resp = Output(Valid(new TageResp))
    val update = Input(new TageUpdate)
  })

  class TageEntry() extends TageBundle {
    val valid = Bool()
    val tag = UInt(tagLen.W)
    val ctr = UInt(TageCtrBits.W)
  }

  val SRAM_SIZE = 256
  require(nRows % SRAM_SIZE == 0)
  require(isPow2(numBr))
  val nBanks = 8
  val bankIdxWidth = log2Ceil(nBanks)
  val nRowsPerBr = nRows / numBr
  val bankSize = nRowsPerBr / nBanks
  val uFoldedWidth = nRowsPerBr / SRAM_SIZE
  val bankFoldWidth = if (bankSize >= SRAM_SIZE) bankSize / SRAM_SIZE else 1
  val perBankWrbypassEntries = 8
  if (bankSize < SRAM_SIZE) {
    println(f"warning: tage table $tableIdx has small sram depth of $bankSize")
  }
  
  def getBankMask(idx: UInt) = VecInit((0 until nBanks).map(idx(bankIdxWidth-1, 0) === _.U))
  def getBankIdx(idx: UInt) = idx >> bankIdxWidth

  val idxFhInfo = (histLen, min(log2Ceil(nRowsPerBr), histLen))
  val tagFhInfo = (histLen, min(histLen, tagLen))
  val altTagFhInfo = (histLen, min(histLen, tagLen-1))
  val allFhInfos = Seq(idxFhInfo, tagFhInfo, altTagFhInfo)

  def getFoldedHistoryInfo = allFhInfos.filter(_._1 >0).toSet
  def getHashedIdxTag(unhashed_idx: UInt, allFh: AllFoldedHistories) = {
    val idx_fh = allFh.getHistWithInfo(idxFhInfo).folded_hist
    val tag_fh = allFh.getHistWithInfo(tagFhInfo).folded_hist
    val alt_tag_fh = allFh.getHistWithInfo(altTagFhInfo).folded_hist
    val idx = (unhashed_idx ^ idx_fh)(log2Ceil(nRowsPerBr)-1, 0)
    val tag = (unhashed_idx ^ tag_fh ^ (alt_tag_fh << 1)) (tagLen - 1, 0)
    (idx, tag)
  }
  def incCtr(ctr: UInt, taken: Bool): UInt = satUpdate(ctr, TageCtrBits, taken)
  def getUnhashedIdx(pc: UInt): UInt = pc >> instOffsetBits
  // def getAllocWayIdx(valids: UInt, setIdx: UInt) = {
  //   val allocWayIdx = WireDefault(0.U(log2Ceil(associating).W))
  //   if(associating > 1) {
  //     val isValid = valids.andR
  //     allocWayIdx := Mux(isValid, replacer.way(setIdx), PriorityEncoder(~valids))
  //   }
  //   allocWayIdx
  // }
  def silentUpdate(ctr: UInt, taken: Bool) = {
    ctr.andR && taken || !ctr.orR && !taken
  }
  
  if (EnableGHistDiff) {
    val idx_history = compute_folded_ghist(io.req.bits.ghist, log2Ceil(nRowsPerBr))
    val idx_fh = io.req.bits.foldedHist.getHistWithInfo(idxFhInfo)
    XSError(idx_history =/= idx_fh.folded_hist, p"tage table $tableIdx has different fh," +
      p" ghist: ${Binary(idx_history)}, fh: ${Binary(idx_fh.folded_hist)}\n")
  }
 
  // val associating = 4
  // val associSetNum = nRowsPerBr / associating
  // val associSetNum = bankSize / associating
  // val us = Module(new SRAMTemplate(Bool(),
  //   set=associSetNum, way=associating,
  //   shouldReset=true, extraReset=true, holdRead=true, singlePort=true,
  //   hasMbist = coreParams.hasMbist,
  //   hasShareBus = coreParams.hasShareBus,
  //   parentName = parentName + "us_"
  // ))
  val us = Module(new FoldedSRAMTemplate(Bool(), set=nRowsPerBr, width=uFoldedWidth, way=numBr,
    shouldReset=true, extraReset=true, holdRead=true, singlePort=true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName + "us_"
  ))
  us.extra_reset.get := io.update.reset_u.reduce(_||_)

  // val tableBanks = Seq.tabulate(nBanks)(idx =>
  //   Module(new SRAMTemplate(new TageEntry,
  //     set=associSetNum, way=associating,
  //     shouldReset=true, holdRead=true, singlePort=true,
  //     hasMbist = coreParams.hasMbist,
  //     hasShareBus = coreParams.hasShareBus,
  //     parentName = parentName + s"table${idx}_"
  //   )))
   val tableBanks = Seq.tabulate(nBanks)(idx =>
    Module(new FoldedSRAMTemplate(new TageEntry, set=bankSize, width=bankFoldWidth, way=numBr,
      shouldReset=true, holdRead=true, singlePort=true,
      hasMbist = coreParams.hasMbist,
      hasShareBus = coreParams.hasShareBus,
      parentName = parentName + s"table${idx}_"
    )))

  val mbistTablePipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistTablePipe")
  } else {
    None
  }

  // val replacer = ReplacementPolicy.fromString(Some("setplru"), 4, associSetNum)

  // read
  val reqUnhashedIdx       = getUnhashedIdx(io.req.bits.pc)
  val (reqS0Idx, reqS0Tag) = getHashedIdxTag(reqUnhashedIdx, io.req.bits.foldedHist)
  val reqS0Bank1h          = getBankMask(reqS0Idx)
  for (b <- 0 until nBanks) {
    tableBanks(b).io.r.req.valid := (io.req.fire && reqS0Bank1h(b))
    tableBanks(b).io.r.req.bits.setIdx := getBankIdx(reqS0Idx)
  }
  us.io.r.req.valid := io.req.fire
  us.io.r.req.bits.setIdx := reqS0Idx

  val s1Idx       = RegEnable(reqS0Idx, io.req.fire)
  val s1Tag       = RegEnable(reqS0Tag, io.req.fire)
  val s1Bank1h    = RegEnable(reqS0Bank1h, io.req.fire)
  val s1ReadWrite = RegEnable(VecInit(tableBanks.map(_.io.w.req.valid)), io.req.valid)
  val respInvalidByWrite = Wire(Bool())
  respInvalidByWrite := Mux1H(s1Bank1h, s1ReadWrite)
  
  val reqTageData      = tableBanks.map(_.io.r.resp.data)
  val tablesBankValids = reqTageData.map(bank => VecInit(bank.map(_.valid)))
  val unconfs          = reqTageData.map(r => VecInit(r.map(e => WireInit(unconf(e.ctr)))))
  val hits             = reqTageData.map(r =>
    VecInit(r.map(e => e.tag === s1Tag && e.valid && !respInvalidByWrite)))
  
  val setRespVec   = Mux1H(s1Bank1h, reqTageData)
  val setUnconfVec = Mux1H(s1Bank1h, unconfs)
  val setHitsVec   = Mux1H(s1Bank1h, hits)
  val isS1Hit      = setHitsVec.reduce(_||_)
  val s1HitWayIdx  = PriorityEncoder(setHitsVec)

  // io.resp.valid       := isS1Hit
  // io.resp.bits.ctr    := setRespVec(s1HitWayIdx).ctr
  // io.resp.bits.u      := us.io.r.resp.data(s1HitWayIdx)
  // io.resp.bits.unconf := setUnconfVec(s1HitWayIdx)
  // io.resp.bits.wayIdx := s1HitWayIdx
  io.resp.valid       := isS1Hit
  io.resp.bits.ctr    := setRespVec(0).ctr
  io.resp.bits.u      := us.io.r.resp.data(0)
  io.resp.bits.unconf := setUnconfVec(0)
  io.resp.bits.wayIdx := 0.U

  if (EnableGHistDiff) {
    val update_idx_history = compute_folded_ghist(io.update.ghist, log2Ceil(nRowsPerBr))
    val update_idx_fh = io.update.foldedHist.getHistWithInfo(idxFhInfo)
    XSError(update_idx_history =/= update_idx_fh.folded_hist && io.update.mask.reduce(_||_),
      p"tage table $tableIdx has different fh when update," +
      p" ghist: ${Binary(update_idx_history)}, fh: ${Binary(update_idx_fh.folded_hist)}\n")
  }

  // update
  val updtBanksWdata     = Wire(Vec(nBanks,  new TageEntry))
  val updtUnhashedIdx    = getUnhashedIdx(io.update.pc)
  val (updtIdx, updtTag) = getHashedIdxTag(updtUnhashedIdx, io.update.foldedHist)
  val updtBank1h         = getBankMask(updtIdx)
  val updtBankIdx        = getBankIdx(updtIdx)
  val writeWayIdx  = 0.U //Mux(io.update.alloc(0), replacer.way(updtIdx), io.update.wayIdx)
  // val writeWayMask = UIntToOH(writeWayIdx, associating)
  // val touchSetIdx  = Seq.fill(1)(Wire(UInt(log2Ceil(associSetNum).W)))
  // val touchWayIdx  = Seq.fill(1)(Wire(Valid(UInt(2.W))))
  // touchSetIdx(0)       := Mux(io.update.mask(0), updtIdx, s1Idx)
  // touchWayIdx(0).valid := io.update.mask(0) || (RegNext(io.req.fire) && isS1Hit) || io.update.alloc(0)
  // touchWayIdx(0).bits  := Mux(io.update.alloc(0) || io.update.mask(0), writeWayIdx, s1HitWayIdx)
  // replacer.access(touchSetIdx, touchWayIdx)
  
  val notSilentUpdate = Wire(Vec(nBanks, Bool()))
  val updteWayMask = VecInit((0 until nBanks).map(a =>
    io.update.mask(0) && notSilentUpdate(a)))
  val wrBypasses = Seq.fill(nBanks)(
    Module(new WrBypass(UInt(TageCtrBits.W), perBankWrbypassEntries, 1, tagWidth=tagLen)))

  for(a <- 0 until nBanks) {
    val wrBypassCtr = wrBypasses(a).io.hit_data(0).bits
    val wrBypassDataValid = wrBypasses(a).io.hit && wrBypasses(a).io.hit_data(0).valid        
    updtBanksWdata(a).valid := true.B
    updtBanksWdata(a).tag   := updtTag
    updtBanksWdata(a).ctr   := Mux(io.update.alloc(0), Mux(io.update.takens(0), 4.U, 3.U),
      Mux(wrBypassDataValid, incCtr(wrBypassCtr,          io.update.takens(0)),
                             incCtr(io.update.oldCtrs(0), io.update.takens(0))))
    notSilentUpdate(a) := Mux(wrBypassDataValid,
      !silentUpdate(wrBypassCtr,          io.update.takens(0)),
      !silentUpdate(io.update.oldCtrs(0), io.update.takens(0))) || io.update.alloc(0)
    wrBypasses(a).io.wen := io.update.mask(0) && updtBank1h(a)
    wrBypasses(a).io.write_idx := getBankIdx(updtIdx)
    wrBypasses(a).io.write_tag.map(_ := updtTag)
    wrBypasses(a).io.write_data(0) := updtBanksWdata(a).ctr
  }

  // write
  for (b <- 0 until nBanks) {
    tableBanks(b).io.w.apply(
      // valid   = updteWayMask(b) && updtBank1h(b),
      // data    = updtBanksWdata(b),
      // setIdx  = updtBankIdx,
      // waymask = writeWayMask
      valid   = updteWayMask(b) && updtBank1h(b),
      data    = updtBanksWdata(b),
      setIdx  = updtBankIdx,
      waymask = 1.U
    )
  }
  us.io.w.apply(io.update.uMask(0), io.update.us(0), updtIdx, 1.U) //writeWayMask)

  val powerOnResetState = RegInit(true.B)
  when(us.io.r.req.ready && tableBanks.map(_.io.r.req.ready).reduce(_ && _)) {
    powerOnResetState := false.B
  }
 
  io.req.ready := !powerOnResetState
  val bank_conflict = (0 until nBanks).map(b => tableBanks(b).io.w.req.valid && reqS0Bank1h(b)).reduce(_||_)
  XSPerfAccumulate(f"tage_table_bank_conflict", bank_conflict)

  for (i <- 0 until numBr) {
    for (b <- 0 until nBanks) {
      val wrbypass = wrBypasses(b)
      XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_enq_$i", io.update.mask(i) && updtBank1h(b) && !wrbypass.io.hit)
      XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_hit_$i", io.update.mask(i) && updtBank1h(b) &&  wrbypass.io.hit)
    }
  }

  for (b <- 0 until nBanks) {
    val not_silent_update = notSilentUpdate(b)
    XSPerfAccumulate(f"tage_table_bank_${b}_real_updates",
      io.update.mask.reduce(_||_) && updtBank1h(b) && not_silent_update(0))
    XSPerfAccumulate(f"tage_table_bank_${b}_silent_updates_eliminated",
      io.update.mask.reduce(_||_) && updtBank1h(b) && !not_silent_update(0))
  }

  XSPerfAccumulate("tage_table_hits", PopCount(io.resp.valid))
  
  for (b <- 0 until nBanks) {
    XSPerfAccumulate(f"tage_table_bank_${b}_update_req", io.update.mask.reduce(_||_) && updtBank1h(b))
    for (i <- 0 until numBr) {
      val li = i
      val pidx = 0.U //get_phy_br_idx(updtUnhashedIdx, li)
      val pi = i
    }
  }

  val u = io.update
  val b = PriorityEncoder(u.mask)
  val ub = PriorityEncoder(u.uMask)
  XSDebug(io.req.fire,
    p"tableReq: pc=0x${Hexadecimal(io.req.bits.pc)}, " +
    p"idx=$reqS0Idx, tag=$reqS0Tag\n")
  for (i <- 0 until numBr) {
    XSDebug(RegNext(io.req.fire) && isS1Hit,
      p"TageTableResp_br_$i: idx=$s1Idx, hit:${isS1Hit}, " +
      p"ctr:${io.resp.bits.ctr}, u:${io.resp.bits.u}\n")
    XSDebug(io.update.mask(i),
      p"update Table_br_$i: pc:${Hexadecimal(u.pc)}}, " +
      p"taken:${u.takens(i)}, alloc:${u.alloc(i)}, oldCtrs:${u.oldCtrs(i)}\n")
    val bank = OHToUInt(updtBank1h.asUInt, nBanks)
    val pi = 0.U // get_phy_br_idx(updtUnhashedIdx, i)
    XSDebug(io.update.mask(i),
      p"update Table_$i: writing tag:$updtTag, " +
      p"ctr: ${updtBanksWdata(bank).ctr} in idx ${updtIdx}\n")
    XSDebug(RegNext(io.req.fire) && !isS1Hit, p"TageTableResp_$i: not hit!\n")
  }

  // ------------------------------Debug-------------------------------------
  val valids = RegInit(VecInit(Seq.fill(nRows)(false.B)))
  when (io.update.mask.reduce(_||_)) { valids(updtIdx) := true.B }
  XSDebug("Table usage:------------------------\n")
  XSDebug("%d out of %d rows are valid\n", PopCount(valids), nRows.U)

}




abstract class BaseTage(implicit p: Parameters)
extends BasePredictor with TageParams with BPUUtils {}

class FakeTage(implicit p: Parameters) extends BaseTage {
  io.out <> 0.U.asTypeOf(DecoupledIO(new BasePredictorOutput))

  // io.s0_ready := true.B
  io.s1_ready := true.B
  io.s2_ready := true.B
}


class Tage(val parentName:String = "Unknown")(implicit p: Parameters) extends BaseTage {

  val resp_meta = Wire(new TageMeta)
  override val meta_size = resp_meta.getWidth

  val tables = TageTableInfos.zipWithIndex.map {
    case ((nRows, histLen, tagLen), i) => {
      val t = Module(new TageTable(nRows, histLen, tagLen, i, parentName = parentName + s"tagtable${i}_"))
      t.io.req.valid := io.s0_fire(1)
      t.io.req.bits.pc := s0_pc_dup(1)
      t.io.req.bits.foldedHist := io.in.bits.folded_hist(1)
      t.io.req.bits.ghist := io.in.bits.ghist
      t
    }
  }
  val bt = Module (new TageBTable(parentName = parentName + "bttable_"))
  // bt.io.s0_fire := io.s0_fire(1)
  // bt.io.s0_pc   := s0_pc_dup(1)
  //#2410
  bt.io.req.valid := io.s0_fire(1)
  bt.io.req.bits := s0_pc_dup(1)

  // #2462
  // val bankTickCtrDistanceToTops = Seq.fill(numBr)(RegInit((1 << (TickWidth-1)).U(TickWidth.W)))
  val bankTickCtrDistanceToTops = Seq.fill(numBr)(RegInit(((1 << TickWidth) - 1).U(TickWidth.W)))  
  val bankTickCtrs = Seq.fill(numBr)(RegInit(0.U(TickWidth.W)))
  val useAltOnNaCtrs = RegInit(
    VecInit(Seq.fill(numBr)(
      VecInit(Seq.fill(NUM_USE_ALT_ON_NA)((1 << (USE_ALT_ON_NA_WIDTH-1)).U(USE_ALT_ON_NA_WIDTH.W)))
    ))
  )

  val tage_fh_info = tables.map(_.getFoldedHistoryInfo).reduce(_++_).toSet
  override def getFoldedHistoryInfo = Some(tage_fh_info)

  val s1_resps = VecInit(tables.map(_.io.resp))

  //val s1_bim = io.in.bits.resp_in(0).s1.full_pred
  // val s2_bim = RegEnable(s1_bim, io.s1_fire)

  val debug_pc_s0 = s0_pc_dup(1)
  val debug_pc_s1 = RegEnable(s0_pc_dup(1), io.s0_fire(1))
  val debug_pc_s2 = RegEnable(debug_pc_s1, io.s1_fire(1))

  val s1_provideds        = Wire(Vec(numBr, Bool()))
  val s1_providers        = Wire(Vec(numBr, UInt(log2Ceil(TageNTables).W)))
  val s1_providerResps    = Wire(Vec(numBr, new TageResp))
  // val s1_altProvideds     = Wire(Vec(numBr, Bool()))
  // val s1_altProviders     = Wire(Vec(numBr, UInt(log2Ceil(TageNTables).W)))
  // val s1_altProviderResps = Wire(Vec(numBr, new TageResp))
  val s1_altUsed          = Wire(Vec(numBr, Bool()))
  val s1_tageTakens       = Wire(Vec(numBr, Bool()))
  val s1_finalAltPreds    = Wire(Vec(numBr, Bool()))
  val s1_basecnts         = Wire(Vec(numBr, UInt(2.W)))
  val s1_useAltOnNa       = Wire(Vec(numBr, Bool()))

  val s2_provideds        = RegEnable(s1_provideds, io.s1_fire(1))
  val s2_providers        = RegEnable(s1_providers, io.s1_fire(1))
  val s2_providerResps    = RegEnable(s1_providerResps, io.s1_fire(1))
  // val s2_altProvideds     = RegEnable(s1_altProvideds, io.s1_fire)
  // val s2_altProviders     = RegEnable(s1_altProviders, io.s1_fire)
  // val s2_altProviderResps = RegEnable(s1_altProviderResps, io.s1_fire)  
  val s2_altUsed          = RegEnable(s1_altUsed, io.s1_fire(1))
  val s2_tageTakens_dup   = io.s1_fire.map(f => RegEnable(s1_tageTakens, f))
  val s2_finalAltPreds    = RegEnable(s1_finalAltPreds, io.s1_fire(1))
  val s2_basecnts         = RegEnable(s1_basecnts, io.s1_fire(1))
  val s2_useAltOnNa       = RegEnable(s1_useAltOnNa, io.s1_fire(1))

  val s3_tageTakens_dup = RegEnable(VecInit(s2_tageTakens_dup), io.s2_fire(1))

  io.out := io.in.bits.resp_in(0)
  io.out.last_stage_meta := resp_meta.asUInt

  val resp_s2 = io.out.s2
  val resp_s3 = io.out.s3

  // Update logic
  val u_valid = io.update(dupForTageSC).valid
  val update = io.update(dupForTageSC).bits
  val updateValids = VecInit((0 until TageBanks).map(w =>
      update.ftb_entry.brValids(w) && u_valid && !update.ftb_entry.always_taken(w) &&
      !(PriorityEncoder(update.br_taken_mask) < w.U)))
  val updateFHist = update.spec_info.folded_hist

  val updateMeta = update.meta.asTypeOf(new TageMeta)

  val updateMask    = WireInit(0.U.asTypeOf(Vec(numBr, Vec(TageNTables, Bool()))))
  val updateUMask   = WireInit(0.U.asTypeOf(Vec(numBr, Vec(TageNTables, Bool()))))
  val updateResetU  = WireInit(0.U.asTypeOf(Vec(numBr, Bool()))) // per predictor
  val updateTakens  = Wire(Vec(numBr, Vec(TageNTables, Bool())))
  val updateAlloc   = WireInit(0.U.asTypeOf(Vec(numBr, Vec(TageNTables, Bool()))))
  val updateOldCtrs  = Wire(Vec(numBr, Vec(TageNTables, UInt(TageCtrBits.W))))
  val updateU       = Wire(Vec(numBr, Vec(TageNTables, Bool())))
  val updatebcnt    = Wire(Vec(TageBanks, UInt(2.W)))
  val baseupdate    = WireInit(0.U.asTypeOf(Vec(TageBanks, Bool())))
  val bUpdateTakens = Wire(Vec(TageBanks, Bool()))
  updateTakens  := DontCare
  updateOldCtrs  := DontCare
  updateU       := DontCare

  val updateMisPreds = update.mispred_mask

  class TageTableInfo(implicit p: Parameters) extends XSBundle {
    val resp = new TageResp
    val tableIdx = UInt(log2Ceil(TageNTables).W)
    val use_alt_on_unconf = Bool()
  }
  // access tag tables and output meta info

  for (i <- 0 until numBr) {
    val useAltCtr = Mux1H(UIntToOH(use_alt_idx(s1_pc_dup(1)), NUM_USE_ALT_ON_NA), useAltOnNaCtrs(i))
    val useAltOnNa = useAltCtr(USE_ALT_ON_NA_WIDTH-1) // highest bit

    val s1_per_br_resp = s1_resps //VecInit(s1_resps.map(_(i)))
    val inputRes = s1_per_br_resp.zipWithIndex.map{case (r, idx) => {
      val tableInfo = Wire(new TageTableInfo)
      tableInfo.resp := r.bits
      tableInfo.use_alt_on_unconf := r.bits.unconf && useAltOnNa
      tableInfo.tableIdx := idx.U(log2Ceil(TageNTables).W)
      (r.valid, tableInfo)
    }}
    val providerInfo = ParallelPriorityMux(inputRes.reverse)
    val provided = inputRes.map(_._1).reduce(_||_)
    // val altProvided = selectedInfo.hasTwo
    // val providerInfo = selectedInfo
    // val altProviderInfo = selectedInfo.second
    s1_provideds(i)      := provided
    s1_providers(i)      := providerInfo.tableIdx
    s1_providerResps(i)  := providerInfo.resp
    // s1_altProvideds(i)   := altProvided
    // s1_altProviders(i)   := altProviderInfo.tableIdx
    // s1_altProviderResps(i) := altProviderInfo.resp

    resp_meta.providers(i).valid    := RegEnable(s2_provideds(i), io.s2_fire(1))
    resp_meta.providers(i).bits     := RegEnable(s2_providers(i), io.s2_fire(1))
    resp_meta.providerResps(i)      := RegEnable(s2_providerResps(i), io.s2_fire(1))
    // resp_meta.altProviders(i).valid := RegEnable(s2_altProvideds(i), io.s2_fire)
    // resp_meta.altProviders(i).bits  := RegEnable(s2_altProviders(i), io.s2_fire)
    // resp_meta.altProviderResps(i)   := RegEnable(s2_altProviderResps(i), io.s2_fire)
    resp_meta.pred_cycle.map(_ := RegEnable(GTimer(), io.s2_fire(1)))
    resp_meta.use_alt_on_na.map(_(i) := RegEnable(s2_useAltOnNa(i), io.s2_fire(1)))

    // Create a mask fo tables which did not hit our query, and also contain useless entries
    // and also uses a longer history than the provider
    val allocatableSlots =
      RegEnable(
        VecInit(s1_per_br_resp.map(r => !r.valid && !r.bits.u)).asUInt &
          ~(LowerMask(UIntToOH(s1_providers(i)), TageNTables) &
            Fill(TageNTables, s1_provideds(i).asUInt)),
        io.s1_fire(1)
      )
    
    resp_meta.allocates(i) := RegEnable(allocatableSlots, io.s2_fire(1))

    s1_altUsed(i)       := !provided || providerInfo.use_alt_on_unconf
    val s1_bimCtr = bt.io.cnt(i)
    s1_tageTakens(i) := 
      Mux(s1_altUsed(i) ,
        s1_bimCtr(1),
        providerInfo.resp.ctr(TageCtrBits-1)
      )    
    s1_finalAltPreds(i) := s1_bimCtr(1)
    s1_basecnts(i)      := s1_bimCtr
    s1_useAltOnNa(i)    := providerInfo.use_alt_on_unconf

    resp_meta.altUsed(i)    := RegEnable(s2_altUsed(i), io.s2_fire(1))
      // #2462
    // resp_meta.altDiffers(i) := RegEnable(s2_finalAltPreds(i) =/= s2_tageTakens_dup(0)(i), io.s2_fire(1))
    resp_meta.altDiffers(i) := RegEnable(
      s2_finalAltPreds(i) =/= s2_providerResps(i).ctr(TageCtrBits - 1), io.s2_fire(1)) // alt != provider
    resp_meta.takens(i)     := RegEnable(s2_tageTakens_dup(0)(i), io.s2_fire(1))
    resp_meta.basecnts(i)   := RegEnable(s2_basecnts(i), io.s2_fire(1))

    val tage_enable_dup = RegNext(dup(io.ctrl.tage_enable))
    for (tage_enable & fp & s2_tageTakens <- tage_enable_dup zip resp_s2.full_pred zip s2_tageTakens_dup) {
      when (tage_enable) {
        fp.br_taken_mask(i) := s2_tageTakens(i)
      }
      dontTouch(tage_enable)
    }
    for (tage_enable & fp & s3_tageTakens <- tage_enable_dup zip resp_s3.full_pred zip s3_tageTakens_dup) {
      when (tage_enable) {
        fp.br_taken_mask(i) := s3_tageTakens(i)
      }
    }

    //---------------- update logics below ------------------//
    val hasUpdate = updateValids(i)
    val updateMispred = updateMisPreds(i)
    val updateTaken = hasUpdate && update.br_taken_mask(i)

    val updateProvided     = updateMeta.providers(i).valid
    val updateProvider     = updateMeta.providers(i).bits
    val updateProviderResp = updateMeta.providerResps(i)
    val updateProviderCorrect = updateProviderResp.ctr(TageCtrBits-1) === updateTaken
    val updateUseAlt = updateMeta.altUsed(i)
    val updateAltDiffers = updateMeta.altDiffers(i)
    val updateAltIdx = use_alt_idx(update.pc)
    val updateUseAltCtr = Mux1H(UIntToOH(updateAltIdx, NUM_USE_ALT_ON_NA), useAltOnNaCtrs(i))
    val updateAltPred = updateMeta.altPreds(i)
    val updateAltCorrect = updateAltPred === updateTaken


    val updateProviderWeakTaken = posUnconf(updateProviderResp.ctr)
    val updateProviderWeaknotTaken = negUnconf(updateProviderResp.ctr)
    val updateProviderWeak = unconf(updateProviderResp.ctr)

    when (hasUpdate) {
      when (updateProvided && updateProviderWeak && updateAltDiffers) {
        val newCtr = satUpdate(updateUseAltCtr, USE_ALT_ON_NA_WIDTH, updateAltCorrect)
        useAltOnNaCtrs(i)(updateAltIdx) := newCtr
      }
    }

    XSPerfAccumulate(f"tage_bank_${i}_use_alt_pred", hasUpdate && updateUseAlt)
    XSPerfAccumulate(f"tage_bank_${i}_alt_correct", hasUpdate && updateUseAlt && updateAltCorrect)
    XSPerfAccumulate(f"tage_bank_${i}_alt_wrong", hasUpdate && updateUseAlt && !updateAltCorrect)
    XSPerfAccumulate(f"tage_bank_${i}_alt_differs", hasUpdate && updateAltDiffers)
    XSPerfAccumulate(f"tage_bank_${i}_use_alt_on_na_ctr_updated", hasUpdate && updateAltDiffers && updateProvided && updateProviderWeak)
    XSPerfAccumulate(f"tage_bank_${i}_use_alt_on_na_ctr_inc", hasUpdate && updateAltDiffers && updateProvided && updateProviderWeak &&  updateAltCorrect)
    XSPerfAccumulate(f"tage_bank_${i}_use_alt_on_na_ctr_dec", hasUpdate && updateAltDiffers && updateProvided && updateProviderWeak && !updateAltCorrect)
    
    XSPerfAccumulate(f"tage_bank_${i}_na", hasUpdate && updateProvided && updateProviderWeak)
    XSPerfAccumulate(f"tage_bank_${i}_use_na_correct", hasUpdate && updateProvided && updateProviderWeak && !updateUseAlt && !updateMispred)
    XSPerfAccumulate(f"tage_bank_${i}_use_na_wrong",   hasUpdate && updateProvided && updateProviderWeak && !updateUseAlt &&  updateMispred)

    updateMeta.use_alt_on_na.map(uaon => XSPerfAccumulate(f"tage_bank_${i}_use_alt_on_na", hasUpdate && uaon(i)))

    when (hasUpdate) {
      when (updateProvided) {
        updateMask(i)(updateProvider) := true.B
        updateUMask(i)(updateProvider) := updateAltDiffers
        updateU(i)(updateProvider) := updateProviderCorrect
        updateTakens(i)(updateProvider) := updateTaken
        updateOldCtrs(i)(updateProvider) := updateProviderResp.ctr
        updateAlloc(i)(updateProvider) := false.B
      }
    }

    // update base table if used base table to predict
    baseupdate(i) := hasUpdate && updateUseAlt
    updatebcnt(i) := updateMeta.basecnts(i)
    bUpdateTakens(i) := updateTaken

    val needToAllocate = hasUpdate && updateMispred && !(updateUseAlt && updateProviderCorrect && updateProvided)
    val allocatableMask = updateMeta.allocates(i)
    val canAllocate = updateMeta.allocateValid(i)

    val allocLFSR = LFSR64()(TageNTables - 1, 0)
    val longerHistoryTableMask = ~(LowerMask(UIntToOH(updateProvider), TageNTables) & Fill(TageNTables, updateProvided.asUInt))
    val canAllocMask = allocatableMask & longerHistoryTableMask
    val allocFailureMask = ~allocatableMask & longerHistoryTableMask
    val tickInc = PopCount(allocFailureMask) > PopCount(canAllocMask)
    val tickDec = PopCount(canAllocMask) > PopCount(allocFailureMask)
    val tickIncVal = PopCount(allocFailureMask) - PopCount(canAllocMask)
    val tickDecVal = PopCount(canAllocMask) - PopCount(allocFailureMask)
    val tickToPosSat = tickIncVal >= bankTickCtrDistanceToTops(i) && tickInc
    val tickToNegSat = tickDecVal >= bankTickCtrs(i) && tickDec

    val firstEntry = PriorityEncoder(canAllocMask)
    val maskedEntry = PriorityEncoder(canAllocMask & allocLFSR)
    val allocate = Mux(canAllocMask(maskedEntry), maskedEntry, firstEntry)


    when (needToAllocate) {
      // val allocate = updateMeta.allocates(i).bits
      when (tickInc) {
        when (tickToPosSat) {
          bankTickCtrs(i) := ((1 << TickWidth) - 1).U
          bankTickCtrDistanceToTops(i) := 0.U
        }.otherwise {
          bankTickCtrs(i) := bankTickCtrs(i) + tickIncVal
          bankTickCtrDistanceToTops(i) := bankTickCtrDistanceToTops(i) - tickIncVal
        }
      }.elsewhen (tickDec) {
        when (tickToNegSat) {
          bankTickCtrs(i) := 0.U
          bankTickCtrDistanceToTops(i) := ((1 << TickWidth) - 1).U
        }.otherwise {
          bankTickCtrs(i) := bankTickCtrs(i) - tickDecVal
          bankTickCtrDistanceToTops(i) := bankTickCtrDistanceToTops(i) + tickDecVal
        }
      }
      when (canAllocate) {
        updateMask(i)(allocate) := true.B
        updateTakens(i)(allocate) := updateTaken
        updateAlloc(i)(allocate) := true.B
        updateUMask(i)(allocate) := true.B
        updateU(i)(allocate) := false.B
      }
      when (bankTickCtrs(i) === ((1 << TickWidth) - 1).U) {
        bankTickCtrs(i) := 0.U
        bankTickCtrDistanceToTops(i) := ((1 << TickWidth) - 1).U
        updateResetU(i) := true.B
      }
    }
    XSPerfAccumulate(f"tage_bank_${i}_update_allocate_failure", needToAllocate && !canAllocate)
    XSPerfAccumulate(f"tage_bank_${i}_update_allocate_success", needToAllocate &&  canAllocate)
    XSPerfAccumulate(s"tage_bank_${i}_mispred", hasUpdate && updateMispred)
    XSPerfAccumulate(s"tage_bank_${i}_reset_u", updateResetU(i))
    for (t <- 0 to TageNTables) {
      XSPerfAccumulate(f"tage_bank_${i}_tick_inc_${t}", needToAllocate && tickInc && tickIncVal === t.U)
      XSPerfAccumulate(f"tage_bank_${i}_tick_dec_${t}", needToAllocate && tickDec && tickDecVal === t.U)
    }
  }

  for (w <- 0 until TageBanks) {
    for (i <- 0 until TageNTables) {
      tables(i).io.update.mask(w)    := RegNext(updateMask(w)(i), false.B)
      tables(i).io.update.takens(w)  := RegEnable(updateTakens(w)(i), false.B, updateValids(w))
      tables(i).io.update.alloc(w)   := RegEnable(updateAlloc(w)(i), false.B, updateValids(w))
      tables(i).io.update.oldCtrs(w) := RegEnable(updateOldCtrs(w)(i), 0.U, updateValids(w))

      tables(i).io.update.uMask(w)   := RegNext(updateUMask(w)(i), false.B)
      tables(i).io.update.us(w)      := RegNext(updateU(w)(i), false.B)
      tables(i).io.update.reset_u(w) := RegNext(updateResetU(w), false.B)
      // use fetch pc instead of instruction pc
      tables(i).io.update.pc       := RegEnable(update.pc, 0.U, updateValids(w))
      tables(i).io.update.foldedHist := RegEnable(updateFHist, updateValids(w))
      tables(i).io.update.ghist := RegEnable(io.update(dupForTageSC).bits.ghist, 0.U, updateValids(w))
      tables(i).io.update.wayIdx := DontCare
    }
  }
  bt.io.updateMask   := RegNext(baseupdate)
  bt.io.updateCnt    := RegEnable(updatebcnt, updateValids(0))
  bt.io.updatePC     := RegEnable(update.pc, updateValids(0))
  bt.io.updateTakens := RegEnable(bUpdateTakens, updateValids(0))

  // all should be ready for req
  // io.s1_ready := tables.map(_.io.req.ready).reduce(_&&_)
  //#2410
  io.s1_ready := tables.map(_.io.req.ready).reduce(_ && _) && bt.io.req.ready  
  
  XSPerfAccumulate(f"tage_write_blocks_read", !io.s1_ready)

  def pred_perf(name: String, cnt: UInt)   = XSPerfAccumulate(s"${name}_at_pred", cnt)
  def commit_perf(name: String, cnt: UInt) = XSPerfAccumulate(s"${name}_at_commit", cnt)
  def tage_perf(name: String, pred_cnt: UInt, commit_cnt: UInt) = {
    pred_perf(name, pred_cnt)
    commit_perf(name, commit_cnt)
  }

  // Debug and perf info
  for (b <- 0 until TageBanks) {
    val updateProvided = updateMeta.providers(b).valid
    val updateProvider = updateMeta.providers(b).bits
    for (i <- 0 until TageNTables) {
      val pred_i_provided =
        s2_provideds(b) && s2_providers(b) === i.U
      val commit_i_provided =
        updateProvided && updateProvider === i.U && updateValids(b)
      tage_perf(
        s"bank_${b}_tage_table_${i}_provided",
        PopCount(pred_i_provided),
        PopCount(commit_i_provided)
      )
    }
    tage_perf(
      s"bank_${b}_tage_use_bim",
      PopCount(!s2_provideds(b)),
      PopCount(!updateProvided && updateValids(b))
    )
    def unconf(providerCtr: UInt) = providerCtr === 3.U || providerCtr === 4.U
    tage_perf(
      s"bank_${b}_tage_use_altpred",
      PopCount(s2_provideds(b) && unconf(s2_providerResps(b).ctr)),
      PopCount(updateProvided &&
        unconf(updateMeta.providerResps(b).ctr) && updateValids(b))
    )
    tage_perf(
      s"bank_${b}_tage_provided",
      PopCount(s2_provideds(b)),
      PopCount(updateProvided && updateValids(b))
    )
  }

  for (b <- 0 until TageBanks) {
    val m = updateMeta
    // val bri = u.metas(b)
    XSDebug(updateValids(b), "update(%d): pc=%x, cycle=%d, taken:%b, misPred:%d, bimctr:%d, pvdr(%d):%d, altDiff:%d, pvdrU:%d, pvdrCtr:%d, alloc:%b\n",
      b.U, update.pc, 0.U, update.br_taken_mask(b), update.mispred_mask(b),
      0.U, m.providers(b).valid, m.providers(b).bits, m.altDiffers(b), m.providerResps(b).u,
      m.providerResps(b).ctr, m.allocates(b)
    )
  }
  val s2_resps = RegEnable(s1_resps, io.s1_fire(1))
  XSDebug("req: v=%d, pc=0x%x\n", io.s0_fire(1), s0_pc_dup(1))
  XSDebug("s1_fire:%d, resp: pc=%x\n", io.s1_fire(1), debug_pc_s1)
  XSDebug("s2_fireOnLastCycle: resp: pc=%x, target=%x, hits=%b, takens=%b\n",
    debug_pc_s2, io.out.s2.target(1), s2_provideds.asUInt, s2_tageTakens_dup(0).asUInt)

  for (b <- 0 until TageBanks) {
    for (i <- 0 until TageNTables) {
      XSDebug("bank(%d)_tage_table(%d): valid:%b, resp_ctr:%d, resp_us:%d\n",
        b.U, i.U, s2_resps(i).valid, s2_resps(i).bits.ctr, s2_resps(i).bits.u)
    }
  }
    // XSDebug(io.update.valid && updateIsBr, p"update: sc: ${updateSCMeta}\n")
    // XSDebug(true.B, p"scThres: use(${useThreshold}), update(${updateThreshold})\n")



  // new tage
  val TickMax = ((1 << TickWidth) - 1).U

  val tageMeta = WireDefault(0.U.asTypeOf(new TageMeta))
  // override val meta_size = tageMeta.getWidth

  // val tageTable = TageTableInfos.zipWithIndex.map {
  //   case ((nRows, histLen, tagLen), i) => {
  //     val t = Module(new TageTable(nRows, histLen, tagLen, i, parentName = parentName + s"tagtable${i}_"))
  //     t.io.req.valid           := io.s0_fire(1)
  //     t.io.req.bits.pc         := s0_pc_dup(1)
  //     t.io.req.bits.foldedHist := io.in.bits.folded_hist(1)
  //     t.io.req.bits.ghist      := io.in.bits.ghist
  //     t
  //   }
  // }

  // val bt = Module (new TageBTable(parentName = parentName + "bttable_"))
  // bt.io.req.valid := io.s0_fire(1)
  // bt.io.req.bits  := s0_pc_dup(1)

  val altCounters = RegInit(VecInit(
    Seq.fill(altCtrsNum)((1 << (alterCtrBits-1)).U(alterCtrBits.W))))

  // val s1DebugPC = RegEnable(s0_pc_dup(1), io.s0_fire(1))
  // val s2DebugPC = RegEnable(s1DebugPC, io.s1_fire(1))
  // val tage_fh_info = tageTable.map(_.getFoldedHistoryInfo).reduce(_++_).toSet
  // override def getFoldedHistoryInfo = Some(tage_fh_info)

  // predict
  val s1RespVec      = tables.map(_.io.resp) // tageTable.map(_.io.resp)
  val s1RespBitsVec  = s1RespVec.map(_.bits)
  val s1RespValidVec = s1RespVec.map(_.valid)
  val s1Provide      = s1RespValidVec.reduce(_||_)
  val s1ProIdxVec    = VecInit((0 until TageNTables).map(i => i.U))
  val s1ProvideIdx   = ParallelPriorityMux(s1RespValidVec.reverse, s1ProIdxVec.reverse)
  val s1Resp         = ParallelPriorityMux(s1RespValidVec.reverse, s1RespBitsVec.reverse)
  val predAltCtrIdx  = use_alt_idx(s1_pc_dup(1)) //useAltIdx(s1_pc_dup(1))
  val predAltCtr     = Mux1H( UIntToOH(predAltCtrIdx, altCtrsNum), altCounters ) //useAltOnNaCtrs(0)) // 
  val isUseAltCtr    = (predAltCtr(alterCtrBits - 1) && s1Resp.unconf) || !s1Provide
  val s1BaseCtr      = bt.io.cnt(0)
  val s1PredTaken    = Mux(isUseAltCtr, s1BaseCtr(1), s1Resp.ctr(TageCtrBits - 1))
  val s2PredTaken    = RegEnable(s1PredTaken, false.B, io.s1_fire(1))
  val s3PredTaken    = RegEnable(s2PredTaken, false.B, io.s2_fire(1))

  val s2TageEna  = RegEnable(RegEnable(io.ctrl.tage_enable, io.s0_fire(1)), io.s1_fire(1))
  val s3TageEna  = RegEnable(s2TageEna, io.s2_fire(1))
 
  when(s2TageEna) {
    io.out.s2.full_pred.map(_.br_taken_mask(0) := s2PredTaken)
  }
  when(s3TageEna) {
    io.out.s3.full_pred.map(_.br_taken_mask(0) := s3PredTaken)
  }

  // meta
  val s1AllocMask  = VecInit(s1RespVec.map(resp => !resp.valid && !resp.bits.u)).asUInt &
    ~(LowerMask(UIntToOH(s1ProvideIdx, TageNTables)) & Fill(TageNTables, s1Provide.asUInt))
  val s1altDiffer  = s1BaseCtr(1) =/= s1Resp.ctr(TageCtrBits - 1) // s1PredTaken //
  val s1UseAltOnNa = predAltCtr(alterCtrBits - 1) && s1Resp.unconf
  val s1HitWayIdx  = s1Resp.wayIdx

  val s2altUsed     = RegEnable(isUseAltCtr, false.B, io.s1_fire(1))
  val s2Provide     = RegEnable(s1Provide, false.B, io.s1_fire(1))
  val s2ProvideIdx  = RegEnable(s1ProvideIdx, 0.U.asTypeOf(s1ProvideIdx), io.s1_fire(1))
  val s2Resp        = RegEnable(s1Resp, 0.U.asTypeOf(s1Resp), io.s1_fire(1))
  val s2altDiffer   = RegEnable(s1altDiffer, false.B, io.s1_fire(1))
  val s2AllocMask   = RegEnable(s1AllocMask, 0.U.asTypeOf(s1AllocMask), io.s1_fire(1))
  val s2PredCycle   = GTimer()
  val s2UseAltOnNa  = RegEnable(s1UseAltOnNa, 0.U.asTypeOf(s1UseAltOnNa), io.s1_fire(1))
  val s2BaseCtr     = RegEnable(s1BaseCtr, 0.U, io.s1_fire(1))
  val s2HitWayIdx   = RegEnable(s1HitWayIdx, io.s1_fire(1))

  val s3Provide    = RegEnable(s2Provide, false.B, io.s2_fire(1))
  val s3ProvideIdx = RegEnable(s2ProvideIdx, 0.U.asTypeOf(s1ProvideIdx), io.s2_fire(1))
  val s3Resp       = RegEnable(s2Resp, 0.U.asTypeOf(s1Resp), io.s2_fire(1))
  val s3altUsed    = RegEnable(s2altUsed, false.B, io.s2_fire(1))
  val s3altDiffer  = RegEnable(s2altDiffer, false.B, io.s2_fire(1))
  val s3baseCtr    = RegEnable(s2BaseCtr, 0.U, io.s2_fire(1))
  val s3AllocMask  = RegEnable(s2AllocMask, 0.U.asTypeOf(s2AllocMask), io.s2_fire(1))
  val s3PredCycle  = RegEnable(s2PredCycle, 0.U.asTypeOf(s2PredCycle), io.s2_fire(1))
  val s3UseAltOnNa = RegEnable(s2UseAltOnNa, 0.U.asTypeOf(s2UseAltOnNa), io.s2_fire(1))
  val s3HitWayIdx  = RegEnable(s2HitWayIdx, io.s2_fire(1))

  // delete
  // val allocatableSlots =
  //     RegEnable(
  //       VecInit(s1_resps.map(r => !r.valid && !r.bits.u)).asUInt &
  //         ~(LowerMask(UIntToOH(s1_providers(0)), TageNTables) &
  //           Fill(TageNTables, s1_provideds(0).asUInt)),
  //       io.s1_fire(1)
  //     )

  tageMeta.providers(0).valid := s3Provide
  tageMeta.providers(0).bits  := s3ProvideIdx
  tageMeta.providerResps(0)   := s3Resp
  tageMeta.altUsed(0)         := s3altUsed //RegEnable(s2_altUsed(0), io.s2_fire(1)) // 
  tageMeta.altDiffers(0)      := s3altDiffer // RegEnable(s2_finalAltPreds(0) =/= s2_providerResps(0).ctr(TageCtrBits - 1), io.s2_fire(1)) // 
  tageMeta.basecnts(0)        := s3baseCtr // RegEnable(s2_basecnts(0), io.s2_fire(1)) // 
  tageMeta.allocates(0)       := s3AllocMask // RegEnable(allocatableSlots, io.s2_fire(1)) //
  tageMeta.takens(0)          := s3PredTaken //RegEnable(s2_tageTakens_dup(0)(0), io.s2_fire(1)) // 
  tageMeta.pred_cycle.map(_ := s3PredCycle)
  tageMeta.use_alt_on_na.map(_(0) := s3UseAltOnNa)
  // tageMeta.wayIdx             := s3HitWayIdx

  resp_meta := tageMeta
  io.out.last_stage_meta := tageMeta.asUInt

  // update
  val xupdateValid   = io.update(0).valid
  val xupdateIn      = io.update(0).bits
  val xupdateMeta    = (io.update(0).bits.meta).asTypeOf(new TageMeta)
  val xupdateMispred = xupdateIn.mispred_mask(0)
  val xupdateBrJmpValid = xupdateValid && xupdateIn.ftb_entry.brValids(0) &&
                         !xupdateIn.ftb_entry.always_taken(0)
  val xupdateTaken      = xupdateBrJmpValid && xupdateIn.br_taken_mask(0)
  val xupdateGHhis      = xupdateIn.spec_info.folded_hist
  val xupdateProvide    = xupdateMeta.providers(0).valid
  val xupdateProvideIdx = xupdateMeta.providers(0).bits

  // update altCounters
  val updateProvideWeak = unconf(xupdateMeta.providerResps(0).ctr)
  val updateAltDiff     = xupdateMeta.altDiffers(0)
  val xupdateAltIdx      = use_alt_idx(xupdateIn.pc) // useAltIdx(updateIn.pc)
  val updateOldAltCtr   = Mux1H( UIntToOH(xupdateAltIdx, altCtrsNum), altCounters )
  val xupdateAltPred     = xupdateMeta.altPreds(0)
  val xupdateAltCorrect  = (xupdateAltPred === xupdateTaken)
  when(xupdateBrJmpValid && xupdateProvide && updateProvideWeak && updateAltDiff) {
    // val newCnt = updateCtr(updateOldAltCtr, alterCtrBits, updateAltCorrect)
    val newCnt = satUpdate(updateOldAltCtr, alterCtrBits, xupdateAltCorrect)
    altCounters(xupdateAltIdx) := newCnt
  }

  // update
  val updateTagePredCorrect = xupdateMeta.providerResps(0).ctr(TageCtrBits - 1) === xupdateTaken
  val updateOldCtrIn   = WireDefault(VecInit(Seq.fill(TageNTables)(0.U(TageCtrBits.W))))
  val updateIsUsIn     = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  val updateTageTaken  = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  val xupdateMask       = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  val xupdateUMask      = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  // allocate
  val isTageAllocate = xupdateBrJmpValid && xupdateMispred &&
                       !(xupdateProvide && updateTagePredCorrect && xupdateMeta.altUsed(0))
  // val canAllocate = updateMeta.allocates(0)
  val allocRandomMask = LFSR64()(TageNTables - 1, 0)
  val xlongerHistoryTableMask = ~(LowerMask(UIntToOH(xupdateProvideIdx), TageNTables) &
                                 Fill(TageNTables, xupdateProvide.asUInt))
  val xcanAllocate = xupdateMeta.allocates(0).orR //& xlongerHistoryTableMask
  val xcanAllocMask = xupdateMeta.allocates(0) & xlongerHistoryTableMask
  val allocTableMask = xcanAllocMask & allocRandomMask
  val allocateIdx = Mux(xcanAllocMask(PriorityEncoder(allocTableMask)),
                    PriorityEncoder(allocTableMask), PriorityEncoder(xcanAllocMask))
  val updateAllocMaskIn = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  // tick
  val tickCnt = RegInit(0.U(TickWidth.W))
  val tickTopDistance = RegInit(TickMax)
  val notAllocate = ~xcanAllocate & xlongerHistoryTableMask
  val tickIsInc = PopCount(xcanAllocate) < PopCount(notAllocate)
  val tickIsDec = PopCount(xcanAllocate) > PopCount(notAllocate)
  val xtickIncVal = PopCount(notAllocate) - PopCount(xcanAllocate)
  val xtickDecVal = PopCount(xcanAllocate) - PopCount(notAllocate)
  val tickSetToMax = tickIsInc && (xtickIncVal >= tickTopDistance)
  val tickSetToMin = tickIsDec && (xtickDecVal >= tickCnt)
  val tickCntReset = WireDefault(false.B)

  // update
  when(xupdateBrJmpValid && xupdateProvide) {
    xupdateMask(xupdateProvideIdx)        := true.B
    updateOldCtrIn(xupdateProvideIdx)    := xupdateMeta.providerResps(0).ctr
    updateIsUsIn(xupdateProvideIdx)      := updateTagePredCorrect
    updateTageTaken(xupdateProvideIdx)   := xupdateTaken
    xupdateUMask(xupdateProvideIdx)       := xupdateMeta.altDiffers(0)
    updateAllocMaskIn(xupdateProvideIdx) := false.B
  }
  
  // allocate
  when(isTageAllocate && xcanAllocate) {
    xupdateMask(allocateIdx)        := true.B
    updateAllocMaskIn(allocateIdx) := true.B
    updateTageTaken(allocateIdx)   := xupdateTaken
    xupdateUMask(allocateIdx)       := true.B
    updateIsUsIn(allocateIdx)      := false.B
  }

  // tick
  when(isTageAllocate) {
    when(tickCnt === TickMax) {
      tickCnt := 0.U
      tickTopDistance := TickMax
      tickCntReset := true.B
    }.elsewhen(tickIsInc) {
      when(tickSetToMax) {
        tickCnt := TickMax
        tickTopDistance := 0.U
      }.otherwise {
        tickCnt := tickCnt + xtickIncVal
        tickTopDistance := tickTopDistance - xtickIncVal
      }
    }.elsewhen(tickIsDec) {
      when(tickSetToMin) {
        tickCnt := 0.U
        tickTopDistance := TickMax
      }.otherwise {
        tickCnt := tickCnt - xtickDecVal
        tickTopDistance := tickTopDistance + xtickDecVal
      }
    }
  }
  for(a <- 0 until TageNTables) {
    tables(a).io.update.pc          := RegNext(xupdateIn.pc)
    tables(a).io.update.foldedHist  := RegNext(xupdateGHhis)
    tables(a).io.update.ghist       := RegNext(xupdateIn.ghist)

    tables(a).io.update.mask(0)     := RegNext(xupdateMask(a)) // RegNext(updateMask(0)(a), false.B) // 
    tables(a).io.update.takens(0)   := RegNext(updateTageTaken(a)) // dui RegEnable(updateTakens(0)(a), false.B, updateValids(0)) //   
    tables(a).io.update.alloc(0)    := RegEnable(updateAlloc(0)(a), false.B, updateValids(0)) //RegNext(updateAllocMaskIn(a)) // 

    tables(a).io.update.oldCtrs(0)  := RegNext(updateOldCtrIn(a))// dui
    tables(a).io.update.uMask(0)    := RegNext(xupdateUMask(a)) // RegNext(updateUMask(0)(a), false.B) //
    tables(a).io.update.us(0)       := RegNext(updateIsUsIn(a)) // RegNext(updateU(0)(a), false.B) //
    tables(a).io.update.reset_u(0)  := RegNext(tickCntReset) //RegNext(updateResetU(0), false.B) // 
    tables(a).io.update.wayIdx      := DontCare //RegNext(xupdateMeta.wayIdx)
  }
  bt.io.updateMask(0)   := RegNext(xupdateBrJmpValid && xupdateMeta.altUsed(0)) // RegNext(baseupdate(0)) // 
  bt.io.updateCnt(0)    := RegNext(xupdateMeta.basecnts(0)) // RegEnable(updatebcnt(0), updateValids(0)) // 
  bt.io.updatePC        := RegNext(xupdateIn.pc) // RegEnable(update.pc, updateValids(0)) // 
  bt.io.updateTakens(0) := RegNext(xupdateTaken) // RegEnable(bUpdateTakens(0), updateValids(0)) // 

  io.s1_ready := tables.map(_.io.req.ready).reduce(_&&_) && bt.io.req.ready 



}


class Tage_SC(parentName:String = "Unknown")(implicit p: Parameters) extends Tage(parentName) with HasSC {}