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

// Path-loss (for visibility/collision stats); TX is fixed at 0 dBm
static Ptr<LogDistancePropagationLossModel> g_pl = nullptr;
static double g_rxSensDbm = -95.0;   // receiver sensitivity threshold (dBm)
static double g_plExponent = 3.0;    // path loss exponent
static double g_refDist = 1.0;       // reference distance (m)
static double g_refLossDb = 40.05;   // reference loss @ refDist (dB) ~ 2.4GHz FSPL at 1m
static uint8_t g_channelNum = 11;

// Per-node state
static std::map<uint32_t, uint16_t> g_chosen;          // node -> SDIndex
static std::map<uint32_t, std::set<uint16_t>> g_localUsedByNode; // rxNode -> used SDIndex observed via EB
static std::map<uint32_t, uint32_t> g_ebCountByNode;   // rxNode -> EB receptions count
static std::map<std::string, uint32_t> g_shortToNodeId;// short addr string -> nodeId
static std::vector<uint32_t> g_observers;              // for collision visibility stats
static AnimationInterface* g_anim = nullptr;
static uint32_t g_numCoord = 16;                       // total nodes (PAN-C + joiners)
static uint32_t g_minEbBeforePick = 1;                 // min EB before attempting slot pick
// Join timing control: base time for each node i = offset + slope * i
static double g_joinBaseOffset = 2.0;
static double g_joinBaseSlope = 0.20; // 2.0 + 0.20 * i
// Join retry/timeout controls
static double g_joinRetryInterval = 0.25; // s
static double g_joinTimeout = 6.0;        // s
// Listen-window tracking: must listen at least 1 multi-superframe before choosing
static std::map<uint32_t, double> g_listenStartSecByNode; // nodeId -> sim seconds when listening window started

static inline double MultiSuperframeSeconds()
{
  // aBaseSuperframeDuration = 960 symbols; 1 symbol = 16 us @ 2.4GHz OQPSK
  const double baseSfSec = 960.0 * 16e-6; // 0.01536 s
  return baseSfSec * static_cast<double>(1u << MO); // 2^MO superframe duration
}

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

// 前置宣告
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

// (Removed) Power-control retune: not used in fixed 0 dBm design

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
    auto itTx = g_shortToNodeId.find(ShortToString(mh.GetShortSrcAddr()));
    if (itTx != g_shortToNodeId.end())
    {
      uint32_t txNodeId = itTx->second;
      auto itChosen = g_chosen.find(txNodeId);
      if (itChosen != g_chosen.end())
      {
        uint16_t sd = itChosen->second;
        g_localUsedByNode[rxNodeId].insert(sd);
      }
    }
    return;
  }
  // Also parse DBAN to learn used SDIndex from notifications
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

  NS_LOG_UNCOND("==== Beacon Slot Selection Summary (0 dBm fixed) ====");
  NS_LOG_UNCOND("Node\tPicked_SDIndex\tTx[dBm]");
  for (uint32_t i = 0; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    double tx = 0.0;
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
  // Also report join-failure rate over joiner population (exclude 1 PAN-C)
  const uint32_t baseline = 1u;
  uint32_t joinerPopulation = (numNodes > baseline) ? (numNodes - baseline) : 0u;
  double unassignedRate = (joinerPopulation > 0) ? (static_cast<double>(unassigned) / static_cast<double>(joinerPopulation)) : 0.0;
  NS_LOG_UNCOND("Not-joined count / joiners (numNodes-" << baseline << "): "
                << unassigned << "/" << joinerPopulation << "  (rate=" << unassignedRate << ")");

  // Average TX power among successfully joined joiners (in mW)
  uint32_t joinedJoiners = 0;
  double sumMw = 0.0;
  if (joinerPopulation > 0)
  {
    for (uint32_t nid = baseline; nid < numNodes; ++nid)
    {
      auto it = g_chosen.find(nid);
      if (it != g_chosen.end() && it->second != 0xffff)
      {
        double mw = 1.0; // 0 dBm
        sumMw += mw;
        joinedJoiners++;
      }
    }
  }
  double avgMw = (joinedJoiners > 0) ? 1.0 : 0.0;
  double avgDbm = (joinedJoiners > 0) ? 0.0 : 0.0;
  if (joinedJoiners > 0)
  {
    NS_LOG_UNCOND("Avg TX power of joined joiners: " << avgMw << " mW (" << avgDbm << " dBm)"
                   << "  (joinedJoiners=" << joinedJoiners << ")");
  }
  else
  {
    NS_LOG_UNCOND("Avg TX power of joined joiners: 0 mW (n/a dBm)"
                   << "  (joinedJoiners=" << joinedJoiners << ")");
  }

  // Collision stats (visibility) computed using fixed 0 dBm
  const uint16_t slotsCount = static_cast<uint16_t>(1u << (BO - SO));
  const uint16_t nonPanSlots = (slotsCount > 0) ? (slotsCount - 1) : 0;

  uint32_t everCollisionObservers = 0;       // Method-1: receivers that ever see collision
  uint64_t collidedObserverSlots = 0;        // Method-2a: number of receiver-slot collisions
  uint64_t sumExcess = 0;                    // Method-2b: sum of (k-1)
  const auto txDbmOf = [&](uint32_t) { return 0.0; };

  // Build joined-observer list: include PAN-C (0) and any node that successfully picked an SDIndex
  std::vector<uint32_t> joinedObservers;
  joinedObservers.reserve(g_observers.size());
  for (uint32_t obsId : g_observers)
  {
    if (obsId == 0)
    {
      joinedObservers.push_back(obsId);
      continue;
    }
    auto itc = g_chosen.find(obsId);
    if (itc != g_chosen.end() && itc->second != 0xffff)
    {
      joinedObservers.push_back(obsId);
    }
  }

  for (uint32_t obsId : joinedObservers)
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

  const uint32_t receiversN = static_cast<uint32_t>(joinedObservers.size());
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
  // Topology and joining controls
  uint32_t joiners = (g_numCoord > 0) ? (g_numCoord - 1) : 15;
  cmd.AddValue("joiners", "Number of joiners (total nodes = 1 + joiners)", joiners);
  cmd.AddValue("minEbBeforePick", "Minimum EB receptions before selecting slot", g_minEbBeforePick);
  cmd.AddValue("joinBaseOffset", "Join attempt base offset seconds", g_joinBaseOffset);
  cmd.AddValue("joinBaseSlope", "Join attempt base slope seconds per node index", g_joinBaseSlope);
  cmd.AddValue("joinRetryInterval", "Interval between join retries (s)", g_joinRetryInterval);
  cmd.AddValue("joinTimeout", "Timeout for joining attempts (s)", g_joinTimeout);
  cmd.Parse(argc, argv);
  g_numCoord = 1 + joiners;

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

  // Positions: PAN-C at origin, all joiners random around
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
  pos->Add(Vector(0.0, 0.0, 0.0));
  double R = 150.0;
  Ptr<UniformRandomVariable> urvX = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> urvY = CreateObject<UniformRandomVariable>();
  urvX->SetAttribute("Min", DoubleValue(-R)); urvX->SetAttribute("Max", DoubleValue(R));
  urvY->SetAttribute("Min", DoubleValue(-R)); urvY->SetAttribute("Max", DoubleValue(R));
  for (uint32_t i = 1; i < g_numCoord; ++i)
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
  // Fixed TX power at 0 dBm for all nodes
  for (uint32_t i = 0; i < g_numCoord; ++i) { ApplyNodeTxToPhy(i, 0.0); }
  // Record PAN-C SDIndex=0
  g_chosen[0] = 0;

  // Joiners: listen, then select locally-free SDIndex and start
  for (uint32_t i = 1; i < g_numCoord; ++i)
  {
    Ptr<LrWpanNetDevice> d0 = devs.Get(i)->GetObject<LrWpanNetDevice>();
    d0->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));

  MlmeSyncRequestParams sync; sync.m_logCh = channelNum; sync.m_logChPage = 0; sync.m_trackBcn = true;
  d0->TrackCoordinatorBeacon(sync);
  {
    std::ostringstream ctx; ctx << "/NodeList/" << nodes.Get(i)->GetId() << "/DeviceList/0/$ns3::LrWpanNetDevice/Mac/MacRx";
    d0->GetMac()->TraceConnect("MacRx", ctx.str(), MakeCallback(&OnMacRxWithContext));
  }
    // mark listen-window start for this node
    g_listenStartSecByNode[nodes.Get(i)->GetId()] = Simulator::Now().GetSeconds();

    const double base = g_joinBaseOffset + g_joinBaseSlope * i;
    const double retryInterval = g_joinRetryInterval; // s
    const uint32_t maxTries = static_cast<uint32_t>(std::ceil(g_joinTimeout / retryInterval));
    auto tries = std::make_shared<uint32_t>(0u);
    auto done = std::make_shared<bool>(false);

    auto self = std::make_shared<std::function<void()>>();
    *self = [d0, panId, channelNum, dsmeSpec, &lrWpanHelper, tries, done, retryInterval, maxTries, self, i]() {
      if (*done) return;
      (*tries)++;

      uint32_t rxNodeId = NodeList::GetNode(i)->GetId();
      // Ensure at least one full multi-superframe of listening
      double now = Simulator::Now().GetSeconds();
      double listenStart = 0.0;
      auto its = g_listenStartSecByNode.find(rxNodeId);
      if (its != g_listenStartSecByNode.end()) listenStart = its->second; else { g_listenStartSecByNode[rxNodeId] = now; listenStart = now; }
      double need = MultiSuperframeSeconds();
      if (now - listenStart < need - 1e-9)
      {
        // Not enough listening yet; try again after the remaining time or retryInterval
        double rem = std::max(0.0, need - (now - listenStart));
        double delay = std::min(retryInterval, rem);
        if (*tries < maxTries) { Simulator::Schedule(Seconds(delay), *self); }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " insufficient listening; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
        return;
      }
      // After a full listen window, if heard no beacons at all, postpone and restart a new window
      uint32_t heard = g_ebCountByNode.count(rxNodeId) ? g_ebCountByNode[rxNodeId] : 0u;
      if (heard == 0u)
      {
        if (*tries < maxTries)
        {
          g_listenStartSecByNode[rxNodeId] = now; // restart window
          Simulator::Schedule(Seconds(retryInterval), *self);
        }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " heard no EB in a full multi-superframe; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
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
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " no locally-free SDIndex from heard beacons; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
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
      // Announce via DBAN for notification only, and record/update label
      Simulator::Schedule(Seconds(0.05), [d0, sdIdx]() {
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
    for (uint32_t i = 1; i < g_numCoord; ++i) { anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255); }
    Simulator::Stop(Seconds(simTime));
    Simulator::Schedule(Seconds(simTime - 1e-6), &PrintSummary, nodes.GetN());
    Simulator::Run();
    g_anim = nullptr; // avoid dangling pointer
  }
  Simulator::Destroy();
  return 0;
}
