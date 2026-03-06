/*
 * DSME beacon slot selection with power control (random-pick with CAP notify)
 * - 1 PAN-C (00:01) beacons at SDIndex=0
 * - 8 backbone coordinators with fixed SDIndex
 * - Other coordinators listen Enhanced Beacons (EB) and DBANs, pick vacant SDIndex from
 *   locally-heard DBANs only, then start beacons and announce.
 * - Power control: after becoming coordinator, set EB TX power just sufficient to reach
 *   the nearest heard coordinator (estimated using path-loss model), targeting a desired
 *   received power threshold at that neighbor, to reduce collision footprint.
 *
 * 關於 EB 自訂 IE（攜帶 TX power）的處理說明：
 *   目前 LR-WPAN DSME MAC 的實作在增強型 Beacon(E-B) 中固定序列化一個
 *   DsmePANDescriptorIE；若要加入額外的 Header IE，需修改核心 MAC 解析邏輯。
 *   為了不動核心實作，本範例以「模擬層」方式達成等效效果：
 *     - 由範例維護每個節點的名目 TX 功率(g_txDbmByNode)，視為會被放入 EB 的自訂 IE；
 *     - 在接收 EB 時，讀出發送者的「宣告 TX 功率」（由 g_txDbmByNode 取得）並記錄；
 *     - 以該宣告 TX 功率搭配路損模型估算本次接收功率，用來推回鏈路增益/距離；
 *     - 依估算的鏈路增益計算本節點成為協調器後之最小足夠 TX，並透過 PHY PIB
 *       (phyTransmitPower) 套用。
 *   這樣可在不改核心的前提下，符合「在 Beacon IE 放入 TX power 供接收方反推距離」
 *   的研究概念與流程。
 */

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/network-module.h"
#include "ns3/packet.h"
#include "ns3/lr-wpan-mac-pl-headers.h"
#include "ns3/lr-wpan-lqi-tag.h"
#include <vector>
#include <sstream>
#include <iomanip>
#include <map>
#include <cmath>
#include <functional>
#include <set>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeBeaconSlotSelectionRandomPickPowerControl");

#define BO 6
#define SO 3
#define MO 5

#define NUM_BACKBONE 8

// RSSI/path-loss
static Ptr<LogDistancePropagationLossModel> g_pl = nullptr;
static double g_rxSensDbm = -95.0;   // receiver sensitivity threshold (dBm)
static double g_plExponent = 3.0;    // path loss exponent
static double g_refDist = 1.0;       // reference distance (m)
static double g_refLossDb = 40.05;   // reference loss @ refDist (dB) ~ 2.4GHz FSPL at 1m
static uint8_t g_channelNum = 11;

// Power-control knobs
static double g_pcTargetPrDbm = -100.0;    // target received power at nearest neighbor (dBm)
static double g_pcMarginDb = 1.0;          // safety margin added to target (dB)
static double g_txMinDbm = -20.0;          // PHY nominal min (dBm), bounded to [-32,31]
static double g_txMaxDbm = 3.0;            // PHY nominal max (dBm), bounded to [-32,31]
static double g_defaultPanTxDbm = 0.0;     // initial PAN-C tx dBm
static double g_defaultBackboneTxDbm = 0.0;// initial backbone tx dBm
static double g_defaultJoinerTxDbm = 0.0;  // initial joiner tx dBm before power control (baseline)

// Policy and stats knobs for fair comparison
static bool g_pcOnlyReduce = true;         // 預設只降不升，避免高於 0 dBm
static bool g_statsUseActualTx = true;     // if false, use assumedTxDbm for stats visibility
static double g_assumedTxDbm = 0.0;        // assumed TX for stats when using baseline-like visibility

// Per-node state
static std::map<uint32_t, uint16_t> g_chosen;          // node -> SDIndex
static std::map<uint32_t, double> g_txDbmByNode;       // node -> current nominal TX dBm
static std::map<uint32_t, std::set<uint16_t>> g_localUsedByNode; // rxNode -> heard-used SDIndex via DBAN
static std::map<uint32_t, uint32_t> g_ebCountByNode;   // rxNode -> EB receptions count
static std::map<std::string, uint32_t> g_shortToNodeId;// short addr string -> nodeId
static std::map<uint32_t, std::set<uint32_t>> g_heardTxByRx; // rx -> set(tx) that sent EB heard
static std::map<uint32_t, std::map<uint32_t, uint8_t>> g_lqiByRx; // rx -> (tx -> last LQI)
// 在接收 EB 時，視為讀到了對方 EB-IE 宣告的 TX(dBm)，以及依模型估出的當次 Pr(dBm)
static std::map<uint32_t, std::map<uint32_t, double>> g_advTxDbmByRx; // rx -> (tx -> advertised TX dBm)
static std::map<uint32_t, std::map<uint32_t, double>> g_lastPrDbmByRx; // rx -> (tx -> estimated Pr dBm using model)
static std::set<uint32_t> g_parentLocked;              // nodes that already became coordinator
// For parent selection by estimated RSSI (match baseline behavior)
static std::map<uint32_t, std::pair<uint32_t, double>> g_bestParentByNode; // rx -> {tx, prDbm}
static std::map<uint32_t, uint32_t> g_currentParentByNode; // rx -> tx
static std::vector<uint32_t> g_observers;              // for collision visibility stats
static AnimationInterface* g_anim = nullptr;
static uint32_t g_numCoord = 16;                       // total nodes
static uint32_t g_minEbBeforePick = 1;                 // min EB before attempting slot pick
// Join timing control: base time for each node i = offset + slope * i
static double g_joinBaseOffset = 2.0;
static double g_joinBaseSlope = 0.20; // 對齊 fixed-power 範例（2.0 + 0.20 * i）
// 模擬停止時間（用於停止週期性重調，以免結束時段撞到銷毀事件）
static double g_simEndTime = 0.0;

// Power control re-tuning schedule after becoming coordinator
static double g_pcTuneDelay = 1.0;     // first retune after join (s)
static double g_pcTuneInterval = 0.5;  // retune interval (s)
static double g_pcTuneWindow = 0.0;    // compute once after delay; no gradual shrinking
static uint32_t g_pcMinNeighbors = 1;  // retune if heard at least this many distinct EB senders

static std::string ShortToString(const Mac16Address& addr)
{
  uint8_t ab[2] = {0, 0};
  addr.CopyTo(ab);
  std::ostringstream os;
  os << std::hex << std::setfill('0')
     << std::setw(2) << static_cast<unsigned>(ab[0]) << ":"
     << std::setw(2) << static_cast<unsigned>(ab[1]);
  return os.str();
}

static std::string SelfStem()
{
  std::string f = __FILE__;
  size_t slash = f.find_last_of("/\\");
  if (slash != std::string::npos) f = f.substr(slash + 1);
  size_t dot = f.rfind('.');
  if (dot != std::string::npos) f = f.substr(0, dot);
  return f;
}

// 前置宣告：供 TuneTxOnceForNode 使用
static void ApplyNodeTxToPhy(uint32_t nodeId, double dbm);

// 更新 NetAnim 節點標籤，顯示該節點目前選到的 SDIndex
static void UpdateAnimLabel(uint32_t nodeId)
{
  if (!g_anim) { return; }
  Ptr<Node> n = NodeList::GetNode(nodeId);
  std::ostringstream lab;
  std::string sdLab = "?";
  auto it = g_chosen.find(nodeId);
  if (it != g_chosen.end()) { sdLab = std::to_string(it->second); }
  lab << "Coord" << nodeId << " SD=" << sdLab;
  g_anim->UpdateNodeDescription(n, lab.str());
}

// 立刻為指定節點計算並套用一次功率（若可用先用 EB 宣告 TX 與估計 Pr 取得 G，否則退回 0 dBm 路損估計）
static void TuneTxOnceForNode(uint32_t nodeId)
{
  auto itH = g_heardTxByRx.find(nodeId);
  uint32_t distinct = (itH == g_heardTxByRx.end()) ? 0u : static_cast<uint32_t>(itH->second.size());
  // 若尚未聽到任何鄰居，退回以 PAN-C + backbone 作為候選（不會調整它們的功率，只用來決定我們該打多大）
  std::set<uint32_t> candidates;
  if (distinct == 0)
  {
    candidates.insert(0u);
    for (uint32_t bi = 0; bi < NUM_BACKBONE; ++bi) { candidates.insert(1u + bi); }
  }
  else
  {
    candidates = itH->second;
  }

  uint32_t chosenTx = UINT32_MAX;
  // 先用 LQI 選擇；若無則用幾何最近
  auto itLqi = g_lqiByRx.find(nodeId);
  if (itLqi != g_lqiByRx.end() && !itLqi->second.empty())
  {
    uint8_t bestLqi = 0;
    for (const auto& kv : itLqi->second)
    {
      uint32_t txId = kv.first; uint8_t lqi = kv.second;
      if (txId == nodeId) continue;
      if (!candidates.empty() && !candidates.count(txId)) continue;
      if (lqi >= bestLqi) { bestLqi = lqi; chosenTx = txId; }
    }
  }
  if (chosenTx == UINT32_MAX)
  {
    Ptr<MobilityModel> me = NodeList::GetNode(nodeId)->GetObject<MobilityModel>();
    double bestDist = std::numeric_limits<double>::infinity();
    for (uint32_t txId : candidates)
    {
      if (txId == nodeId) continue;
      Ptr<MobilityModel> mm = NodeList::GetNode(txId)->GetObject<MobilityModel>();
      if (mm && me)
      {
        double dx = me->GetPosition().x - mm->GetPosition().x;
        double dy = me->GetPosition().y - mm->GetPosition().y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < bestDist) { bestDist = dist; chosenTx = txId; }
      }
    }
  }

  double newTx = g_defaultJoinerTxDbm;
  if (chosenTx != UINT32_MAX && g_pl)
  {
    Ptr<MobilityModel> nb = NodeList::GetNode(chosenTx)->GetObject<MobilityModel>();
    Ptr<MobilityModel> meMob = NodeList::GetNode(nodeId)->GetObject<MobilityModel>();
    if (nb && meMob)
    {
      bool usedAdvertised = false; double G = 0.0;
      auto itAdvTxByTx = g_advTxDbmByRx.find(nodeId);
      if (itAdvTxByTx != g_advTxDbmByRx.end())
      {
        auto itAdvTx = itAdvTxByTx->second.find(chosenTx);
        auto itPrMap = g_lastPrDbmByRx.find(nodeId);
        if (itAdvTx != itAdvTxByTx->second.end() && itPrMap != g_lastPrDbmByRx.end())
        {
          auto itPr = itPrMap->second.find(chosenTx);
          if (itPr != itPrMap->second.end())
          {
            double advTxDbm = itAdvTx->second; double prDbmEst = itPr->second;
            G = prDbmEst - advTxDbm; usedAdvertised = true;
          }
        }
      }
      if (!usedAdvertised)
      {
        // 後備：使用 0 dBm 的路損估計（等效於以模型求 G）
        G = g_pl->CalcRxPower(0.0, meMob, nb);
      }
      double tgt = g_pcTargetPrDbm + g_pcMarginDb;
      newTx = std::max(g_txMinDbm, std::min(g_txMaxDbm, tgt - G));
    }
  }
  if (g_pcOnlyReduce) { newTx = std::min(newTx, 0.0); }
  ApplyNodeTxToPhy(nodeId, newTx);
}

static bool ParseNodeIdFromContext(const std::string& ctx, uint32_t& outNodeId)
{
  const std::string key = "/NodeList/";
  auto pos = ctx.find(key);
  if (pos == std::string::npos) return false;
  pos += key.size();
  uint32_t val = 0; bool ok = false;
  while (pos < ctx.size() && isdigit(static_cast<unsigned char>(ctx[pos])))
  {
    ok = true; val = val * 10 + (ctx[pos] - '0'); ++pos;
  }
  if (!ok) return false;
  outNodeId = val; return true;
}

// Utility: bound PIB nominal tx power to [-32,31]
static int8_t BoundNominalTxDbm(double dbm)
{
  double x = std::max(-32.0, std::min(31.0, dbm));
  return static_cast<int8_t>(std::lround(x));
}

// Apply per-node tx power to PHY PIB (phyTransmitPower)
static void ApplyNodeTxToPhy(uint32_t nodeId, double dbm)
{
  Ptr<Node> n = NodeList::GetNode(nodeId);
  Ptr<LrWpanNetDevice> d = n->GetDevice(0)->GetObject<LrWpanNetDevice>();
  LrWpanPhyPibAttributes pib;
  // Encode 6-bit two's complement in lower bits, keep 2 MSB zero
  int8_t nominal = BoundNominalTxDbm(dbm);
  uint8_t enc = static_cast<uint8_t>(nominal & 0x3f);
  pib.phyTransmitPower = enc;
  d->GetPhy()->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyTransmitPower, &pib);
  g_txDbmByNode[nodeId] = nominal;
}

// On EB/DBAN reception, update local observations
static void OnMacRxWithContext(std::string context, Ptr<const Packet> p)
{
  uint32_t rxNodeId = 0;
  if (!ParseNodeIdFromContext(context, rxNodeId)) return;

  Ptr<Packet> copy = p->Copy();
  LrWpanMacHeader mh;
  if (!copy->RemoveHeader(mh)) return;

  if (mh.GetType() == LrWpanMacHeader::LRWPAN_MAC_BEACON)
  {
    g_ebCountByNode[rxNodeId]++;
    // Track heard EB sender set
    auto itTx = g_shortToNodeId.find(ShortToString(mh.GetShortSrcAddr()));
    if (itTx != g_shortToNodeId.end())
    {
      g_heardTxByRx[rxNodeId].insert(itTx->second);
      // Record last-seen LQI for this sender at this receiver (for RSSI-based nearest estimation)
      LrWpanLqiTag lqiTag;
      if (p->PeekPacketTag(lqiTag))
      {
        g_lqiByRx[rxNodeId][itTx->second] = lqiTag.Get();
      }

      // 視為在 EB 的自訂 IE 讀到了對方宣告的 TX 功率（以範例層維護的 g_txDbmByNode 取代）
      uint32_t txNodeId = itTx->second;
      double advTxDbm = 0.0;
      auto itPow = g_txDbmByNode.find(txNodeId);
      if (itPow != g_txDbmByNode.end())
      {
        advTxDbm = itPow->second;
        g_advTxDbmByRx[rxNodeId][txNodeId] = advTxDbm;
      }
      // 以該宣告 TX 透過模型估算本次接收功率，供之後換算鏈路增益 (Pr - Pt)
      Ptr<Node> rxNode = NodeList::GetNode(rxNodeId);
      Ptr<Node> txNode = NodeList::GetNode(txNodeId);
      Ptr<MobilityModel> rx = rxNode->GetObject<MobilityModel>();
      Ptr<MobilityModel> tx = txNode->GetObject<MobilityModel>();
      if (g_pl && rx && tx)
      {
        double prDbmEst = g_pl->CalcRxPower(advTxDbm, tx, rx);
        g_lastPrDbmByRx[rxNodeId][txNodeId] = prDbmEst;
      }

      // If not yet locked as coordinator, mirror baseline: choose parent by estimated RSSI and switch
      if (g_parentLocked.find(rxNodeId) == g_parentLocked.end() && g_pl)
      {
        uint32_t txNodeId = itTx->second;
        Ptr<Node> rxNode = NodeList::GetNode(rxNodeId);
        Ptr<Node> txNode = NodeList::GetNode(txNodeId);
        Ptr<MobilityModel> rx = rxNode->GetObject<MobilityModel>();
        Ptr<MobilityModel> tx = txNode->GetObject<MobilityModel>();
        if (rx && tx)
        {
          // 父選擇仍採用基準假設 TX 以對齊 baseline 的行為
          double prDbm = g_pl->CalcRxPower(g_assumedTxDbm, tx, rx);
          auto itBest = g_bestParentByNode.find(rxNodeId);
          bool better = (itBest == g_bestParentByNode.end()) || (prDbm > itBest->second.second + 1e-9);
          if (better)
          {
            g_bestParentByNode[rxNodeId] = {txNodeId, prDbm};
            auto itCur = g_currentParentByNode.find(rxNodeId);
            bool needSet = (itCur == g_currentParentByNode.end()) || (itCur->second != txNodeId);
            if (needSet)
            {
              Ptr<LrWpanNetDevice> d = rxNode->GetDevice(0)->GetObject<LrWpanNetDevice>();
              d->GetMac()->SetAssociatedCoor(mh.GetShortSrcAddr());
              MlmeSyncRequestParams sync; sync.m_logCh = g_channelNum; sync.m_logChPage = 0; sync.m_trackBcn = true;
              d->TrackCoordinatorBeacon(sync);
              g_currentParentByNode[rxNodeId] = txNodeId;
            }
          }
        }
      }
    }
    return;
  }

  if (mh.GetType() != LrWpanMacHeader::LRWPAN_MAC_COMMAND) return;
  CommandPayloadHeader cmd;
  if (!copy->RemoveHeader(cmd)) return;
  if (cmd.GetCommandFrameType() == CommandPayloadHeader::DSME_BEACON_ALLOC_NOTIF)
  {
    uint16_t sd = cmd.GetAllocationBcnSDIndex();
    g_localUsedByNode[rxNodeId].insert(sd);
  }
}

static void PrintSummary(uint32_t numNodes)
{
  std::map<uint16_t, std::vector<uint32_t>> groups;
  for (const auto& kv : g_chosen)
  {
    if (kv.second != 0xffff) groups[kv.second].push_back(kv.first);
  }

  NS_LOG_UNCOND("==== PowerControl Beacon Slot Selection Summary ====");
  NS_LOG_UNCOND("Node\tPicked_SDIndex\tTx[dBm]");
  for (uint32_t i = 0; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    double tx = (g_txDbmByNode.count(i) ? g_txDbmByNode[i] : NAN);
    if (it != g_chosen.end()) NS_LOG_UNCOND(i << "\t" << it->second << "\t" << tx);
    else NS_LOG_UNCOND(i << "\t(n/a)\t" << tx);
  }

  // Unassigned nodes (excluding PAN-C 0)
  uint32_t unassigned = 0;
  for (uint32_t i = 1; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    if (it == g_chosen.end() || it->second == 0xffff) ++unassigned;
  }
  NS_LOG_UNCOND("Unassigned nodes: " << unassigned);

  // Collision stats (visibility): by default, baseline-like using assumedTxDbm; can switch to actual TX
  const uint16_t slotsCount = static_cast<uint16_t>(1u << (BO - SO));
  const uint16_t nonPanSlots = (slotsCount > 0) ? (slotsCount - 1) : 0;

  uint32_t everCollisionObservers = 0;       // Method-1: receivers that ever see collision
  uint64_t collidedObserverSlots = 0;        // Method-2a: number of receiver-slot collisions
  uint64_t sumExcess = 0;                    // Method-2b: sum of (k-1)
  const auto txDbmOf = [&](uint32_t nodeId) {
    if (!g_statsUseActualTx)
    {
      return g_assumedTxDbm; // baseline-like stats visibility
    }
    auto it = g_txDbmByNode.find(nodeId);
    if (it != g_txDbmByNode.end()) return it->second;
    // Defaults: PAN-C/backbone use backbone default, others use joiner default
    if (nodeId == 0 || nodeId <= NUM_BACKBONE) return g_defaultBackboneTxDbm;
    return g_defaultJoinerTxDbm;
  };

  for (uint32_t obsId : g_observers)
  {
    Ptr<Node> on = NodeList::GetNode(obsId);
    Ptr<MobilityModel> rx = on->GetObject<MobilityModel>();
    bool ever = false;
    for (uint16_t sd = 1; sd < slotsCount; ++sd)
    {
      uint32_t visible = 0;
      auto itg = groups.find(sd);
      if (itg != groups.end())
      {
        for (uint32_t coordId : itg->second)
        {
          Ptr<Node> cn = NodeList::GetNode(coordId);
          Ptr<MobilityModel> tx = cn->GetObject<MobilityModel>();
          double prDbm = g_pl ? g_pl->CalcRxPower(txDbmOf(coordId), tx, rx) : -1e9;
          if (prDbm >= g_rxSensDbm) { visible++; }
        }
      }
      if (visible >= 2) { collidedObserverSlots++; ever = true; }
      if (visible > 1) { sumExcess += static_cast<uint64_t>(visible - 1); }
    }
    if (ever) { everCollisionObservers++; }
  }

  const uint32_t receiversN = static_cast<uint32_t>(g_observers.size());
  const uint64_t totalObserverSlots = static_cast<uint64_t>(receiversN) * static_cast<uint64_t>(nonPanSlots);
  double method1Rate = (receiversN > 0) ? (static_cast<double>(everCollisionObservers) / receiversN) : 0.0;
  double method2aRate = (totalObserverSlots > 0) ? (static_cast<double>(collidedObserverSlots) / static_cast<double>(totalObserverSlots)) : 0.0;
  double method2bSeverity = (totalObserverSlots > 0) ? (static_cast<double>(sumExcess) / static_cast<double>(totalObserverSlots)) : 0.0;
  NS_LOG_UNCOND("[Method-1] Ever-collided receivers: " << everCollisionObservers << "/" << receiversN
                 << " (rate=" << method1Rate << ")");
  NS_LOG_UNCOND("[Method-2a] Collision probability (receiver-slot): " << method2aRate
                 << "  (collided=" << collidedObserverSlots
                 << ", total=" << totalObserverSlots << ")");
  NS_LOG_UNCOND("[Method-2b] Collision severity avg (k-1 per receiver-slot): " << method2bSeverity
                 << "  (sumExcess=" << sumExcess
                 << ", total=" << totalObserverSlots << ")");
}

int main(int argc, char** argv)
{
  bool verbose = true;
  bool promiscuousPcap = true;
  double simTime = 15.0;
  uint32_t seed = 4;

  CommandLine cmd(__FILE__);
  cmd.AddValue("verbose", "Enable component logs", verbose);
  cmd.AddValue("promisc", "Enable promiscuous PCAP", promiscuousPcap);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("seed", "RNG seed for RngSeedManager", seed);
  cmd.AddValue("rxSensDbm", "Receiver sensitivity (dBm)", g_rxSensDbm);
  cmd.AddValue("plExp", "Path loss exponent", g_plExponent);
  cmd.AddValue("refDist", "Reference distance (m)", g_refDist);
  cmd.AddValue("refLossDb", "Reference loss at refDist (dB)", g_refLossDb);
  cmd.AddValue("numNodes", "Total node count (>= 1 + NUM_BACKBONE)", g_numCoord);
  cmd.AddValue("minEbBeforePick", "Minimum EB receptions before selecting slot", g_minEbBeforePick);
  cmd.AddValue("joinBaseOffset", "Join attempt base offset seconds", g_joinBaseOffset);
  cmd.AddValue("joinBaseSlope", "Join attempt base slope seconds per node index", g_joinBaseSlope);
  // Power control knobs
  cmd.AddValue("pcTargetPrDbm", "Target Rx power at nearest neighbor (dBm)", g_pcTargetPrDbm);
  cmd.AddValue("pcMarginDb", "Safety margin added to target (dB)", g_pcMarginDb);
  cmd.AddValue("txMinDbm", "Min allowed TX (dBm)", g_txMinDbm);
  cmd.AddValue("txMaxDbm", "Max allowed TX (dBm)", g_txMaxDbm);
  cmd.AddValue("panTxDbm", "Initial PAN-C TX (dBm)", g_defaultPanTxDbm);
  cmd.AddValue("bbTxDbm", "Initial backbone TX (dBm)", g_defaultBackboneTxDbm);
  cmd.AddValue("jnTxDbm", "Initial joiner TX (dBm)", g_defaultJoinerTxDbm);
  // Policy and stats controls
  cmd.AddValue("pcOnlyReduce", "If true, do not increase above baseline 0 dBm", g_pcOnlyReduce);
  cmd.AddValue("statsUseActualTx", "If true, stats use actual TX; otherwise uses assumedTxDbm", g_statsUseActualTx);
  cmd.AddValue("assumedTxDbm", "Assumed TX for stats when statsUseActualTx=false (dBm)", g_assumedTxDbm);
  // Retune schedule controls
  cmd.AddValue("pcTuneDelay", "Initial delay after join before first power retune (s)", g_pcTuneDelay);
  cmd.AddValue("pcTuneInterval", "Interval between power retunes (s)", g_pcTuneInterval);
  cmd.AddValue("pcTuneWindow", "Window length to keep power retuning (s)", g_pcTuneWindow);
  cmd.AddValue("pcMinNeighbors", "Minimum distinct EB senders required to retune", g_pcMinNeighbors);
  cmd.Parse(argc, argv);

  // 紀錄模擬結束時間供週期性重調判斷
  g_simEndTime = simTime;

  if (g_numCoord < 1u + NUM_BACKBONE)
  {
    NS_FATAL_ERROR("numNodes must be >= " << (1 + NUM_BACKBONE));
  }

  if (verbose)
  {
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
  }

  RngSeedManager::SetSeed(seed);

  NodeContainer nodes;
  nodes.Create(g_numCoord);

  // Positions: PAN-C at origin, 8 backbone in 3x3 perimeter, rest random
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
  pos->Add(Vector(0.0, 0.0, 0.0));
  double d = 80.0;
  std::vector<Vector> backboneGrid = {
      Vector(-d,  d, 0.0), Vector(0.0,  d, 0.0), Vector( d,  d, 0.0),
      Vector(-d,  0.0, 0.0),                  Vector( d,  0.0, 0.0),
      Vector(-d, -d, 0.0), Vector(0.0, -d, 0.0), Vector( d, -d, 0.0)};
  for (const auto& v : backboneGrid) pos->Add(v);
  double R = 150.0;
  Ptr<UniformRandomVariable> urvX = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> urvY = CreateObject<UniformRandomVariable>();
  urvX->SetAttribute("Min", DoubleValue(-R)); urvX->SetAttribute("Max", DoubleValue(R));
  urvY->SetAttribute("Min", DoubleValue(-R)); urvY->SetAttribute("Max", DoubleValue(R));
  for (uint32_t i = 1 + NUM_BACKBONE; i < g_numCoord; ++i)
  {
    pos->Add(Vector(urvX->GetValue(), urvY->GetValue(), 0.0));
  }
  mobility.SetPositionAllocator(pos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Path loss model
  g_pl = CreateObject<LogDistancePropagationLossModel>();
  g_pl->SetPathLossExponent(g_plExponent);
  g_pl->SetReference(g_refDist, g_refLossDb);

  // LR-WPAN install
  LrWpanHelper lrWpanHelper(true);
  NetDeviceContainer devs = lrWpanHelper.Install(nodes);
  std::string fileStem = SelfStem();
  lrWpanHelper.EnablePcapAll(fileStem, promiscuousPcap);

  // Short address -> nodeId map
  g_shortToNodeId.clear();
  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d0 = devs.Get(i)->GetObject<LrWpanNetDevice>();
    g_shortToNodeId[ShortToString(d0->GetMac()->GetShortAddress())] = d0->GetNode()->GetId();
  }

  const uint16_t numChSupported = 6;
  const bool capReduction = false;
  const uint8_t panId = 0x0007;
  const uint8_t channelNum = 11; g_channelNum = channelNum;

  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d0 = devs.Get(i)->GetObject<LrWpanNetDevice>();
    d0->GetMac()->SetNumOfChannelSupported(numChSupported);
    d0->GetMac()->SetCAPReduction(capReduction);
  }

  // Observers: all coordinators (existing and to-be)
  for (uint32_t i = 0; i < nodes.GetN(); ++i) g_observers.push_back(nodes.Get(i)->GetId());

  // PAN-C at SDIndex=0
  MlmeStartRequestParams panStart;
  panStart.m_panCoor = true; panStart.m_PanId = panId; panStart.m_bcnOrd = BO; panStart.m_sfrmOrd = SO; panStart.m_logCh = channelNum;
  BeaconBitmap panBitmap(0, 1 << (BO - SO)); panBitmap.SetSDIndex(0); panStart.m_bcnBitmap = panBitmap;
  HoppingDescriptor panHop; panHop.m_HoppingSequenceID = 0; panHop.m_hoppingSeqLen = 0; panHop.m_channelOfs = 0;
  panHop.m_channelOfsBitmapLen = 16; panHop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << 0));
  panStart.m_hoppingDescriptor = panHop;
  DsmeSuperFrameField dsmeSpec; dsmeSpec.SetMultiSuperframeOrder(MO); dsmeSpec.SetChannelDiversityMode(1); dsmeSpec.SetCAPReductionFlag(capReduction);
  panStart.m_dsmeSuperframeSpec = dsmeSpec;

  lrWpanHelper.AssociateToBeaconPan(devs, Mac16Address("00:01"), panStart);

  // Set initial TX power for PAN-C and backbone and apply to PHY
  ApplyNodeTxToPhy(0, std::max(g_txMinDbm, std::min(g_txMaxDbm, g_defaultPanTxDbm)));
  for (uint32_t bi = 0; bi < NUM_BACKBONE; ++bi)
  {
    uint32_t i = 1 + bi;
    ApplyNodeTxToPhy(i, std::max(g_txMinDbm, std::min(g_txMaxDbm, g_defaultBackboneTxDbm)));
  }
  for (uint32_t i = 1 + NUM_BACKBONE; i < g_numCoord; ++i)
  {
    ApplyNodeTxToPhy(i, std::max(g_txMinDbm, std::min(g_txMaxDbm, g_defaultJoinerTxDbm)));
  }

  // Backbone coordinators fixed SDIndex
  {
    uint16_t fixedSlots[NUM_BACKBONE] = {1,2,3,4,5,6,7,1};
    for (uint32_t bi = 0; bi < NUM_BACKBONE; ++bi)
    {
      uint32_t i = 1 + bi;
      Ptr<LrWpanNetDevice> d0 = devs.Get(i)->GetObject<LrWpanNetDevice>();
      d0->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));

      uint16_t sdIdx = fixedSlots[bi];
      d0->GetMac()->SetTimeSlotToSendBcn(sdIdx);

      MlmeStartRequestParams start;
      start.m_panCoor = false; start.m_PanId = panId; start.m_bcnOrd = BO; start.m_sfrmOrd = SO;
      BeaconBitmap cBitmap(0, 1 << (BO - SO)); cBitmap.SetSDIndex(sdIdx); start.m_bcnBitmap = cBitmap;
      HoppingDescriptor hop; hop.m_HoppingSequenceID = 0; hop.m_hoppingSeqLen = 0; hop.m_channelOfs = sdIdx; hop.m_channelOfsBitmapLen = 16;
      hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << (sdIdx % 16))); start.m_hoppingDescriptor = hop;
      start.m_dsmeSuperframeSpec = dsmeSpec;

      PanDescriptor pd; pd.m_coorPanId = panId; pd.m_coorShortAddr = Mac16Address("00:01"); pd.m_logCh = channelNum;
      SuperframeField sf; sf.SetSuperframeOrder(SO); sf.SetBeaconOrder(BO); pd.m_superframeSpec = sf; pd.m_dsmeSuperframeSpec = dsmeSpec; pd.m_bcnBitmap = cBitmap;

      lrWpanHelper.CoordBoostrap(d0, pd, sdIdx, start);
      // Once bootstrapped as coordinator, lock parent (match baseline behavior)
      g_parentLocked.insert(d0->GetNode()->GetId());
      // 發 DBAN 並更新 NetAnim 標籤（在模擬時間觸發，確保 g_anim 已初始化）
      Simulator::Schedule(Seconds(0.05), [d0, sdIdx]() {
        d0->SendDsmeBeaconAllocNotify();
        g_chosen[d0->GetNode()->GetId()] = sdIdx;
        UpdateAnimLabel(d0->GetNode()->GetId());
      });
      g_chosen[i] = sdIdx;
      // 注意：依你的需求，PAN-C 與 backbone 維持固定 0 dBm，不對其做功率調諧
    }
  }

  // Other coordinators: listen, then select locally-free SDIndex and start
  for (uint32_t i = 1 + NUM_BACKBONE; i < g_numCoord; ++i)
  {
    Ptr<LrWpanNetDevice> d0 = devs.Get(i)->GetObject<LrWpanNetDevice>();
    d0->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));

    MlmeSyncRequestParams sync; sync.m_logCh = channelNum; sync.m_logChPage = 0; sync.m_trackBcn = true;
    d0->TrackCoordinatorBeacon(sync);
    {
      std::ostringstream ctx; ctx << "/NodeList/" << nodes.Get(i)->GetId() << "/DeviceList/0/$ns3::LrWpanNetDevice/Mac/MacRx";
      d0->GetMac()->TraceConnect("MacRx", ctx.str(), MakeCallback(&OnMacRxWithContext));
    }

    const double base = g_joinBaseOffset + g_joinBaseSlope * i;
    const double retryInterval = 0.25; // s
    const uint32_t maxTries = 24;      // ~6s overall
    auto tries = std::make_shared<uint32_t>(0u);
    auto done = std::make_shared<bool>(false);

    auto self = std::make_shared<std::function<void()>>();
    *self = [d0, panId, channelNum, dsmeSpec, &lrWpanHelper, tries, done, retryInterval, maxTries, self, i]() {
      if (*done) return;
      (*tries)++;

      uint32_t rxNodeId = NodeList::GetNode(i)->GetId();
      uint32_t heard = g_ebCountByNode.count(rxNodeId) ? g_ebCountByNode[rxNodeId] : 0u;
      if (heard < g_minEbBeforePick)
      {
        if (*tries < maxTries) { Simulator::Schedule(Seconds(retryInterval), *self); }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " has not heard EB; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
        return;
      }

      const uint16_t slotsCount = static_cast<uint16_t>(1u << (BO - SO));
      std::vector<bool> locallyUsed(slotsCount, false);
      auto itset = g_localUsedByNode.find(rxNodeId);
      if (itset != g_localUsedByNode.end())
      {
        for (uint16_t used : itset->second) if (used < slotsCount) locallyUsed[used] = true;
      }

      std::vector<uint16_t> candidates;
      for (uint16_t s = 1; s < slotsCount; ++s) if (!locallyUsed[s]) candidates.push_back(s);
      if (candidates.empty())
      {
        if (*tries < maxTries) { Simulator::Schedule(Seconds(retryInterval), *self); }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " no locally-free SDIndex; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
        return;
      }

      Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable>();
      uint16_t sdIdx = candidates[static_cast<uint16_t>(urv->GetInteger(0, static_cast<int>(candidates.size() - 1)) )];

      *done = true;
      d0->GetMac()->SetTimeSlotToSendBcn(sdIdx);

      MlmeStartRequestParams start; start.m_panCoor = false; start.m_PanId = panId; start.m_bcnOrd = BO; start.m_sfrmOrd = SO;
      BeaconBitmap cBitmap(0, 1 << (BO - SO)); cBitmap.SetSDIndex(sdIdx); start.m_bcnBitmap = cBitmap;
      HoppingDescriptor hop; hop.m_HoppingSequenceID = 0; hop.m_hoppingSeqLen = 0; hop.m_channelOfs = sdIdx; hop.m_channelOfsBitmapLen = 16;
      hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << (sdIdx % 16))); start.m_hoppingDescriptor = hop;
      start.m_dsmeSuperframeSpec = dsmeSpec;

      PanDescriptor pd; pd.m_coorPanId = panId; pd.m_coorShortAddr = Mac16Address("00:01"); pd.m_logCh = channelNum;
      SuperframeField sf; sf.SetSuperframeOrder(SO); sf.SetBeaconOrder(BO); pd.m_superframeSpec = sf; pd.m_dsmeSuperframeSpec = dsmeSpec; pd.m_bcnBitmap = cBitmap;

      lrWpanHelper.CoordBoostrap(d0, pd, sdIdx, start);
      // Lock parent after becoming coordinator (avoid subsequent switching)
      g_parentLocked.insert(d0->GetNode()->GetId());
      // 立刻做一次功率調諧（以最近鄰為目標）；為確保已聽到 EB，稍微延遲再執行
      Simulator::Schedule(Seconds(0.12), [rxNodeId]() { TuneTxOnceForNode(rxNodeId); });

      // 之後每 2 秒定期重調一次（直到模擬結束），只對加入型協調器啟用
      auto periodic = std::make_shared<std::function<void()>>();
      *periodic = [periodic, rxNodeId]() {
        double now = Simulator::Now().GetSeconds();
        // 若距離結束不足 2 秒，則不再排下一輪，避免與銷毀流程衝突
        if (now + 2.0 > g_simEndTime - 1e-3) {
          return;
        }
        TuneTxOnceForNode(rxNodeId);
        Simulator::Schedule(Seconds(2.0), *periodic);
      };
      Simulator::Schedule(Seconds(2.0), *periodic);

      // Announce DBAN, record, 並即時更新 NetAnim 標籤
      Simulator::Schedule(Seconds(0.08), [d0, sdIdx]() {
        d0->SendDsmeBeaconAllocNotify();
        g_chosen[d0->GetNode()->GetId()] = sdIdx;
        UpdateAnimLabel(d0->GetNode()->GetId());
      });
    };
    Simulator::Schedule(Seconds(base), *self);
  }

  // NetAnim (scope-limited so that destructor runs BEFORE Simulator::Destroy())
  {
    AnimationInterface anim(fileStem + ".xml");
    g_anim = &anim; anim.SetMobilityPollInterval(Seconds(0.1)); anim.EnablePacketMetadata(true);
    anim.UpdateNodeDescription(nodes.Get(0), "PAN-C SD=0"); anim.UpdateNodeColor(nodes.Get(0), 0, 0, 0); anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);
    for (uint32_t i = 1; i < g_numCoord; ++i)
    {
      std::ostringstream lab; std::string sdLab = "?"; auto it = g_chosen.find(i); if (it != g_chosen.end()) sdLab = std::to_string(it->second);
      lab << "Coord" << i << " SD=" << sdLab; anim.UpdateNodeDescription(nodes.Get(i), lab.str()); anim.UpdateNodeSize(nodes.Get(i)->GetId(), 12.0, 12.0);
    }
    for (uint32_t i = 1; i < g_numCoord; ++i) { if (i <= NUM_BACKBONE) anim.UpdateNodeColor(nodes.Get(i), 0, 0, 0); else anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255); }

    // 結束前 1 秒做一次全域重調（僅對加入型協調器），讓最終功率反映最新最近鄰
    Simulator::Schedule(Seconds(simTime - 1.0), [=]() {
      for (uint32_t nid = 1 + NUM_BACKBONE; nid < g_numCoord; ++nid)
      {
        TuneTxOnceForNode(nid);
      }
    });
    Simulator::Stop(Seconds(simTime));
    Simulator::Schedule(Seconds(simTime - 1e-6), &PrintSummary, nodes.GetN());
    Simulator::Run();
    g_anim = nullptr; // avoid dangling pointer
  }
  Simulator::Destroy();
  return 0;
}
