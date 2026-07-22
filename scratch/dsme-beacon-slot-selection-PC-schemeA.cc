/*
 * DSME beacon slot selection — Power Control, scheme A (real Custom IE in Enhanced Beacon)
 *
 * 與 scheme B（模擬層 g_gpsTable + 因果閘門）的差異：
 *  - GPS 座標透過核心新增的 `GpsCoordIE` (HEADERIE_GPS_COORD = 0x18, IEEE Unmanaged ID 區段) 真正序列化進 EB wire。
 *  - 每個節點只知道：a) 自己的 GPS（由本 scratch 透過 LrWpanMac::SetSelfGpsCoord 灌入）；
 *                   b) 鄰居的 GPS ←← 只從收到的 EB 的 GpsCoordIE 解析而得（MAC 的 trace
 *                   source `GpsFromBeacon` 回呼 scratch 更新 g_neighborGps）。
 *  - 沒有任何「模擬器上帝視角」的 g_gpsTable / 因果閘門，徹底消除方案 B 被 reviewer 質疑的點。
 *  - 指標層面與方案 B 等價（EB 多 10 bytes overhead 影響幾可忽略）。
 *
 * 與 baseline 的差異（同 scheme B）：
 *  1. Joiner 在挑到 SDIndex 後，從「已收過 GpsCoordIE 的鄰居集合」找最近鄰居 d_nearest，
 *     以 log-distance path loss 反推最小 TX power：
 *         P_tx = (RX_sens + margin) + L(d_nearest),  L(d) = L_ref + 10 n log10(d / d_ref)
 *     並 clamp 到 [g_txMinDbm, g_txMaxDbm]。
 *  2. PAN-C 固定 0 dBm，不做 power control（提供穩定的 bootstrap 種子）。
 *  3. PrintSummary 的 p_coll / s_coll 改為「以各節點實際 TX power」套 path loss 算可見性；
 *     P_tx_avg 改為對 T 集合實際平均（mW 平均再轉 dBm）。
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
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeBeaconSlotSelectionPCSchemeA");

#define BO 6
#define SO 3
#define MO 5

// Path-loss model parameters (same as baseline for fair comparison)
static Ptr<LogDistancePropagationLossModel> g_pl = nullptr;
static double g_rxSensDbm = -95.0;   // receiver sensitivity threshold (dBm)
static double g_plExponent = 2.7;    // path loss exponent
static double g_refDist = 1.0;       // reference distance (m)
static double g_refLossDb = 40.05;   // reference loss @ refDist (dB)
static uint8_t g_channelNum = 11;

// Per-node state (mirrors baseline)
static std::map<uint32_t, uint16_t> g_chosen;
static std::map<uint32_t, std::set<uint16_t>> g_localUsedByNode;
static std::map<uint32_t, uint32_t> g_ebCountByNode;
static std::map<std::string, uint32_t> g_shortToNodeId;
static std::vector<uint32_t> g_observers;
static AnimationInterface* g_anim = nullptr;
static uint32_t g_numCoord = 16;
static uint32_t g_minEbBeforePick = 1;
static double g_joinBaseOffset = 2.0;
static double g_joinBaseSlope = 0.20;
static double g_joinRetryInterval = 0.25;
static double g_joinTimeout = 6.0;
static std::map<uint32_t, double> g_listenStartSecByNode;

// ==== 2-hop beacon scheduling (DSME SD-Bitmap relay) ====
// g_hopScope: 1 = direct 1-hop occupancy; 2 = spec-faithful 2-hop via neighbour bitmap relay.
static uint32_t g_hopScope = 2;
// rxNode -> set of neighbour nodeIds whose beacon/DBAN was received directly (1-hop neighbour set).
static std::map<uint32_t, std::set<uint32_t>> g_heardNeighbors;

// ==== PC-specific state (scheme A: all neighbor GPS comes from received IEs) ====
struct NodeGps { double x; double y; };
// Neighbor GPS learned from incoming GpsCoordIE (one entry per (myNodeId, neighborNodeId)).
// Populated ONLY by OnGpsFromBeacon trace callback — no global ground-truth table.
static std::map<uint32_t, std::map<uint32_t, NodeGps>> g_neighborGps;
// Self GPS kept locally so ComputePcTxDbm can compute distance without re-querying mobility
// (still not cheating: a real node knows its own GPS via its own receiver).
static std::map<uint32_t, NodeGps> g_selfGps;
static std::map<uint32_t, double> g_txDbmByNode;                 // actual TX power per node

// PC parameters (tunable from CLI)
static double g_pcMarginDb = 3.0;
static double g_txMinDbm   = -32.0;
static double g_txMaxDbm   = 0.0;
static double g_panCoordTxDbm = 0.0;

static inline double MultiSuperframeSeconds()
{
  const double baseSfSec = 960.0 * 16e-6;
  return baseSfSec * static_cast<double>(1u << MO);
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

static void UpdateAnimLabel(uint32_t nodeId)
{
  if (!g_anim) { return; }
  Ptr<Node> n = NodeList::GetNode(nodeId);
  std::ostringstream lab;
  std::string sdLab = "?";
  auto it = g_chosen.find(nodeId);
  if (it != g_chosen.end()) { sdLab = std::to_string(it->second); }
  double tx = 0.0;
  auto itTx = g_txDbmByNode.find(nodeId);
  if (itTx != g_txDbmByNode.end()) tx = itTx->second;
  lab << "Coord" << nodeId << " SD=" << sdLab << " Tx=" << tx;
  g_anim->UpdateNodeDescription(n, lab.str());
}

// LR-WPAN joiners get a new short address after association, so the cache built
// at setup time goes stale. On cache miss, rescan live devices and rebuild.
static uint32_t LookupNodeIdByShort(const std::string& shortStr)
{
  auto it = g_shortToNodeId.find(shortStr);
  if (it != g_shortToNodeId.end()) return it->second;
  for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
  {
    Ptr<Node> n = NodeList::GetNode(i);
    Ptr<NetDevice> nd = n->GetDevice(0);
    if (!nd) continue;
    Ptr<LrWpanNetDevice> d = nd->GetObject<LrWpanNetDevice>();
    if (!d) continue;
    std::string s = ShortToString(d->GetMac()->GetShortAddress());
    g_shortToNodeId[s] = n->GetId();
  }
  it = g_shortToNodeId.find(shortStr);
  return (it != g_shortToNodeId.end()) ? it->second : UINT32_MAX;
}

// Build the SDIndex occupancy view used when a joiner picks its slot.
//   g_hopScope == 1 : only slots this node heard directly (1-hop).
//   g_hopScope >= 2 : additionally OR in every 1-hop neighbour's advertised bitmap
//                     (its own directly-heard slots + its own allocation). This mirrors
//                     the standard DSME SD-Bitmap relay, which covers the 2-hop
//                     neighbourhood and prevents hidden-node slot collisions.
static std::set<uint16_t> BuildUsedView(uint32_t nodeId)
{
  std::set<uint16_t> used;
  auto itSelf = g_localUsedByNode.find(nodeId);
  if (itSelf != g_localUsedByNode.end())
    used.insert(itSelf->second.begin(), itSelf->second.end());
  if (g_hopScope >= 2)
  {
    auto itNb = g_heardNeighbors.find(nodeId);
    if (itNb != g_heardNeighbors.end())
    {
      for (uint32_t nb : itNb->second)
      {
        auto itNbUsed = g_localUsedByNode.find(nb);
        if (itNbUsed != g_localUsedByNode.end())
          used.insert(itNbUsed->second.begin(), itNbUsed->second.end());
        auto itNbChosen = g_chosen.find(nb);
        if (itNbChosen != g_chosen.end() && itNbChosen->second != 0xffff)
          used.insert(itNbChosen->second);
      }
    }
  }
  return used;
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

// Bound dBm to 6-bit two's complement nominal range
static int8_t BoundNominalTxDbm(double dbm)
{
  double x = std::max(-32.0, std::min(31.0, dbm));
  return static_cast<int8_t>(std::lround(x));
}

// Apply per-node TX power to PHY PIB and record into g_txDbmByNode
static void ApplyNodeTxDbm(uint32_t nodeId, double dbm)
{
  Ptr<Node> n = NodeList::GetNode(nodeId);
  Ptr<LrWpanNetDevice> d = n->GetDevice(0)->GetObject<LrWpanNetDevice>();
  LrWpanPhyPibAttributes pib;
  int8_t nominal = BoundNominalTxDbm(dbm);
  uint8_t enc = static_cast<uint8_t>(nominal & 0x3f);
  pib.phyTransmitPower = enc;
  d->GetPhy()->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyTransmitPower, &pib);
  g_txDbmByNode[nodeId] = static_cast<double>(nominal);
}

// ==== PC core (scheme A): compute minimum TX power using only IE-learned GPS ====
//
// Rules:
//  - Candidates = g_neighborGps[nodeId] (populated strictly from received GpsCoordIE).
//  - d_nearest  = min euclidean distance between g_selfGps[nodeId] and each neighbor entry.
//  - P_tx = (RX_sens + margin) + L(d_nearest), clamped to [g_txMinDbm, g_txMaxDbm].
//  - If no neighbor GPS learned yet, fall back to g_txMaxDbm.
static double ComputePcTxDbm(uint32_t nodeId)
{
  auto itNbrs = g_neighborGps.find(nodeId);
  if (itNbrs == g_neighborGps.end() || itNbrs->second.empty())
  {
    return g_txMaxDbm;
  }
  auto itMe = g_selfGps.find(nodeId);
  if (itMe == g_selfGps.end()) return g_txMaxDbm;

  double bestDist = std::numeric_limits<double>::infinity();
  for (const auto& kv : itNbrs->second)
  {
    double dx = itMe->second.x - kv.second.x;
    double dy = itMe->second.y - kv.second.y;
    double d = std::sqrt(dx*dx + dy*dy);
    if (d < bestDist) bestDist = d;
  }
  if (!std::isfinite(bestDist) || bestDist <= 0.0)
  {
    return g_txMaxDbm;
  }

  // log-distance path loss: L(d) = L_ref + 10 n log10(d / d_ref)
  double dRatio = bestDist / g_refDist;
  if (dRatio < 1.0) dRatio = 1.0;   // don't extrapolate below reference distance
  double Ld = g_refLossDb + 10.0 * g_plExponent * std::log10(dRatio);

  double target = g_rxSensDbm + g_pcMarginDb;
  double pTx = target + Ld;
  if (pTx < g_txMinDbm) pTx = g_txMinDbm;
  if (pTx > g_txMaxDbm) pTx = g_txMaxDbm;
  return pTx;
}

// On EB/DBAN reception, update local SD-index observations (same as baseline — scheme A no
// longer needs a causality gate here because neighbor GPS is populated via a separate MAC
// trace source, see OnGpsFromBeacon below).
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
    uint32_t txNodeId = LookupNodeIdByShort(ShortToString(mh.GetShortSrcAddr()));
    if (txNodeId != UINT32_MAX)
    {
      g_heardNeighbors[rxNodeId].insert(txNodeId);   // 1-hop neighbour (for 2-hop relay)
      auto itChosen = g_chosen.find(txNodeId);
      if (itChosen != g_chosen.end())
      {
        uint16_t sd = itChosen->second;
        g_localUsedByNode[rxNodeId].insert(sd);
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
    uint32_t txNodeId = LookupNodeIdByShort(ShortToString(mh.GetShortSrcAddr()));
    if (txNodeId != UINT32_MAX)
      g_heardNeighbors[rxNodeId].insert(txNodeId);   // DBAN sender is a 1-hop neighbour
  }
}

// Scheme A: MAC fires this every time a GpsCoordIE is extracted from a received EB.
// Context string is "/NodeList/<id>/DeviceList/0/...", so rxNodeId is parsed out.
// Populates g_neighborGps[rxNodeId][txNodeId] = {x, y}; this is the ONLY path to
// learn a neighbor's position — no god-mode table.
static void OnGpsFromBeacon(std::string context,
                            Mac16Address senderShort,
                            int32_t latE6,
                            int32_t lonE6)
{
  uint32_t rxNodeId = 0;
  if (!ParseNodeIdFromContext(context, rxNodeId)) return;

  uint32_t txNodeId = LookupNodeIdByShort(ShortToString(senderShort));
  if (txNodeId == UINT32_MAX) return;   // unknown sender, skip
  if (txNodeId == rxNodeId) return;     // ignore echo of self beacon (should not happen)

  NodeGps g;
  g.x = static_cast<double>(latE6) / 1e6;
  g.y = static_cast<double>(lonE6) / 1e6;
  g_neighborGps[rxNodeId][txNodeId] = g;
}

static void PrintSummary(uint32_t numNodes)
{
  std::map<uint16_t, std::vector<uint32_t>> groups;
  for (const auto& kv : g_chosen)
  {
    if (kv.second != 0xffff) groups[kv.second].push_back(kv.first);
  }

  NS_LOG_UNCOND("==== Beacon Slot Selection Summary (PC, scheme A: real GpsCoordIE in EB) ====");
  NS_LOG_UNCOND("pcMarginDb=" << g_pcMarginDb << " txMin=" << g_txMinDbm
                 << " txMax=" << g_txMaxDbm << " plExp=" << g_plExponent);
  NS_LOG_UNCOND("Node\tSDIndex\tTx[dBm]");
  for (uint32_t i = 0; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    double tx = 0.0;
    auto itTx = g_txDbmByNode.find(i);
    if (itTx != g_txDbmByNode.end()) tx = itTx->second;
    if (it != g_chosen.end() && it->second != 0xffff)
      NS_LOG_UNCOND(i << "\t" << it->second << "\t" << tx);
    else
      NS_LOG_UNCOND(i << "\t(n/a)\t" << tx);
  }

  // R: set of successfully upgraded coordinators (PAN-C + joined joiners)
  std::vector<uint32_t> R;
  R.push_back(0);
  for (uint32_t i = 1; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    if (it != g_chosen.end() && it->second != 0xffff)
      R.push_back(i);
  }
  const auto& T = R;

  // Metric C: eta (exclude PAN-C from numerator and denominator)
  uint32_t joinedJoiners = (R.size() > 1) ? static_cast<uint32_t>(R.size() - 1) : 0;
  uint32_t totalJoiners = (numNodes > 1) ? (numNodes - 1) : 0;
  double eta = (totalJoiners > 0) ? static_cast<double>(joinedJoiners) / static_cast<double>(totalJoiners) : 0.0;

  // Metric D: real P_tx average over T (mW average, then convert to dBm)
  double sumMw = 0.0;
  uint32_t txCount = 0;
  for (uint32_t t : T)
  {
    double dbm = 0.0;
    auto itTx = g_txDbmByNode.find(t);
    if (itTx != g_txDbmByNode.end()) dbm = itTx->second;
    sumMw += std::pow(10.0, dbm / 10.0);
    txCount++;
  }
  double avgMw = (txCount > 0) ? (sumMw / txCount) : 0.0;
  double avgDbm = (avgMw > 0.0) ? (10.0 * std::log10(avgMw))
                                : -std::numeric_limits<double>::infinity();

  // Metric A/B: p_coll & s_coll (use each transmitter's actual TX power)
  const uint16_t slotsCount = static_cast<uint16_t>(1u << (BO - SO));
  const uint16_t nonPanSlots = (slotsCount > 0) ? (slotsCount - 1) : 0;

  uint64_t collidedSlots = 0;
  uint64_t sumExcess = 0;

  for (uint32_t obsId : R)
  {
    Ptr<Node> on = NodeList::GetNode(obsId);
    Ptr<MobilityModel> rxMob = on->GetObject<MobilityModel>();

    for (uint16_t sd = 1; sd < slotsCount; ++sd)
    {
      uint32_t k = 0;
      auto itg = groups.find(sd);
      if (itg != groups.end())
      {
        for (uint32_t coordId : itg->second)
        {
          Ptr<Node> cn = NodeList::GetNode(coordId);
          Ptr<MobilityModel> txMob = cn->GetObject<MobilityModel>();
          double txDbm = 0.0;
          auto itTx = g_txDbmByNode.find(coordId);
          if (itTx != g_txDbmByNode.end()) txDbm = itTx->second;
          double prDbm = g_pl ? g_pl->CalcRxPower(txDbm, txMob, rxMob) : -1e9;
          if (prDbm >= g_rxSensDbm) { k++; }
        }
      }
      if (k >= 2) { collidedSlots++; }
      if (k > 1)  { sumExcess += static_cast<uint64_t>(k - 1); }
    }
  }

  const uint64_t totalRS = static_cast<uint64_t>(R.size()) * static_cast<uint64_t>(nonPanSlots);
  double p_coll = (totalRS > 0) ? (static_cast<double>(collidedSlots) / static_cast<double>(totalRS)) : 0.0;
  double s_coll = (totalRS > 0) ? (static_cast<double>(sumExcess)     / static_cast<double>(totalRS)) : 0.0;

  NS_LOG_UNCOND("---- Metrics ----");
  NS_LOG_UNCOND("[p_coll]    Collision probability:  " << p_coll
                 << "  (collided_RS=" << collidedSlots << ", total_RS=" << totalRS << ")");
  NS_LOG_UNCOND("[s_coll]    Collision severity:     " << s_coll
                 << "  (sum_excess=" << sumExcess << ", total_RS=" << totalRS << ")");
  NS_LOG_UNCOND("[eta]       Join success rate:      " << eta
                 << "  (joined_joiners=" << joinedJoiners << ", total_joiners=" << totalJoiners << ")");
  NS_LOG_UNCOND("[P_tx_avg]  Avg TX power:           " << avgDbm << " dBm  (" << avgMw << " mW)"
                 << "  (|T|=" << T.size() << ")");
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
  uint32_t joiners = (g_numCoord > 0) ? (g_numCoord - 1) : 15;
  cmd.AddValue("joiners", "Number of joiners (total nodes = 1 + joiners)", joiners);
  cmd.AddValue("minEbBeforePick", "Minimum EB receptions before selecting slot", g_minEbBeforePick);
  cmd.AddValue("joinBaseOffset", "Join attempt base offset seconds", g_joinBaseOffset);
  cmd.AddValue("joinBaseSlope", "Join attempt base slope seconds per node index", g_joinBaseSlope);
  cmd.AddValue("joinRetryInterval", "Interval between join retries (s)", g_joinRetryInterval);
  cmd.AddValue("joinTimeout", "Timeout for joining attempts (s)", g_joinTimeout);
  cmd.AddValue("hopScope", "Beacon occupancy scope: 1=1-hop direct, 2=2-hop SD-bitmap relay (spec)", g_hopScope);
  // PC-specific knobs
  cmd.AddValue("pcMarginDb", "Fade margin above RX sensitivity (dB)", g_pcMarginDb);
  cmd.AddValue("txMinDbm", "Minimum allowed TX power (dBm)", g_txMinDbm);
  cmd.AddValue("txMaxDbm", "Maximum allowed TX power (dBm)", g_txMaxDbm);
  cmd.AddValue("panCoordTxDbm", "Fixed PAN-C TX power (dBm, no PC)", g_panCoordTxDbm);
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

  // Positions: PAN-C at origin, joiners random in +/-R square (same as baseline)
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

  // Record self GPS per node locally (used in ComputePcTxDbm for the d_nearest calc).
  // Note: this is NOT a god-mode neighbor table — each node only learns its own coord,
  // which in a real deployment would come from its own GPS receiver. Neighbor GPS is
  // learned strictly via incoming GpsCoordIE (see OnGpsFromBeacon below).
  g_selfGps.clear();
  g_neighborGps.clear();
  for (uint32_t i = 0; i < g_numCoord; ++i)
  {
    Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
    Vector v = mm->GetPosition();
    g_selfGps[nodes.Get(i)->GetId()] = NodeGps{v.x, v.y};
  }

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

  // Scheme A: (1) inject each node's self GPS into its MAC so it serializes GpsCoordIE into EB;
  //           (2) hook MAC's GpsFromBeacon trace to populate g_neighborGps from received IEs.
  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d0 = devs.Get(i)->GetObject<LrWpanNetDevice>();
    uint32_t nodeId = d0->GetNode()->GetId();
    auto itSelf = g_selfGps.find(nodeId);
    if (itSelf != g_selfGps.end())
    {
      d0->GetMac()->SetSelfGpsCoord(itSelf->second.x, itSelf->second.y);
    }
    std::ostringstream ctx; ctx << "/NodeList/" << nodeId
                                << "/DeviceList/0/$ns3::LrWpanNetDevice/Mac/GpsFromBeacon";
    d0->GetMac()->TraceConnect("GpsFromBeacon", ctx.str(), MakeCallback(&OnGpsFromBeacon));
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

  // Initial TX power:
  //   - PAN-C pinned at g_panCoordTxDbm (default 0 dBm), will NOT be updated.
  //   - Joiners get g_txMaxDbm as a placeholder; they don't transmit anything
  //     until CoordBoostrap, at which point ComputePcTxDbm() overrides them.
  ApplyNodeTxDbm(0, g_panCoordTxDbm);
  for (uint32_t i = 1; i < g_numCoord; ++i) { ApplyNodeTxDbm(i, g_txMaxDbm); }
  g_chosen[0] = 0;

  // Joiners: listen, then select locally-free SDIndex, compute PC power, then start
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
    g_listenStartSecByNode[nodes.Get(i)->GetId()] = Simulator::Now().GetSeconds();

    const double base = g_joinBaseOffset + g_joinBaseSlope * i;
    const double retryInterval = g_joinRetryInterval;
    const uint32_t maxTries = static_cast<uint32_t>(std::ceil(g_joinTimeout / retryInterval));
    auto tries = std::make_shared<uint32_t>(0u);
    auto done = std::make_shared<bool>(false);

    auto self = std::make_shared<std::function<void()>>();
    *self = [d0, panId, channelNum, dsmeSpec, &lrWpanHelper, tries, done, retryInterval, maxTries, self, i]() {
      if (*done) return;
      (*tries)++;

      uint32_t rxNodeId = NodeList::GetNode(i)->GetId();
      double now = Simulator::Now().GetSeconds();
      double listenStart = 0.0;
      auto its = g_listenStartSecByNode.find(rxNodeId);
      if (its != g_listenStartSecByNode.end()) listenStart = its->second;
      else { g_listenStartSecByNode[rxNodeId] = now; listenStart = now; }
      double need = MultiSuperframeSeconds();
      if (now - listenStart < need - 1e-9)
      {
        double rem = std::max(0.0, need - (now - listenStart));
        double delay = std::min(retryInterval, rem);
        if (*tries < maxTries) { Simulator::Schedule(Seconds(delay), *self); }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " insufficient listening; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
        return;
      }
      uint32_t heard = g_ebCountByNode.count(rxNodeId) ? g_ebCountByNode[rxNodeId] : 0u;
      if (heard == 0u)
      {
        if (*tries < maxTries)
        {
          g_listenStartSecByNode[rxNodeId] = now;
          Simulator::Schedule(Seconds(retryInterval), *self);
        }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " heard no EB in a full multi-superframe; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
        return;
      }

      const uint16_t slotsCount = static_cast<uint16_t>(1u << (BO - SO));
      std::vector<bool> locallyUsed(slotsCount, false);
      std::set<uint16_t> usedView = BuildUsedView(rxNodeId); // 1-hop or 2-hop per g_hopScope
      for (uint16_t used : usedView) if (used < slotsCount) locallyUsed[used] = true;

      std::vector<uint16_t> candidates;
      for (uint16_t s = 0; s < slotsCount; ++s) if (!locallyUsed[s]) candidates.push_back(s);
      if (candidates.empty())
      {
        if (*tries < maxTries) { Simulator::Schedule(Seconds(retryInterval), *self); }
        else { NS_LOG_UNCOND("Node " << d0->GetNode()->GetId() << " no locally-free SDIndex from heard beacons; give up"); g_chosen[d0->GetNode()->GetId()] = 0xffff; }
        return;
      }

      Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable>();
      uint16_t sdIdx = candidates[static_cast<uint16_t>(urv->GetInteger(0, static_cast<int>(candidates.size() - 1)))];

      *done = true;

      // ==== PC: compute TX power from nearest IE-learned neighbor, apply BEFORE first beacon ====
      double pcTx = ComputePcTxDbm(rxNodeId);
      ApplyNodeTxDbm(rxNodeId, pcTx);
      NS_LOG_UNCOND("Node " << rxNodeId << " PC TX = " << pcTx << " dBm  (learned GPS from "
                     << (g_neighborGps.count(rxNodeId) ? g_neighborGps[rxNodeId].size() : 0u)
                     << " neighbors via GpsCoordIE)");

      d0->GetMac()->SetTimeSlotToSendBcn(sdIdx);

      MlmeStartRequestParams start; start.m_panCoor = false; start.m_PanId = panId; start.m_bcnOrd = BO; start.m_sfrmOrd = SO;
      BeaconBitmap cBitmap(0, 1 << (BO - SO)); cBitmap.SetSDIndex(sdIdx); start.m_bcnBitmap = cBitmap;
      HoppingDescriptor hop; hop.m_HoppingSequenceID = 0; hop.m_hoppingSeqLen = 0; hop.m_channelOfs = sdIdx; hop.m_channelOfsBitmapLen = 16;
      hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << (sdIdx % 16))); start.m_hoppingDescriptor = hop;
      start.m_dsmeSuperframeSpec = dsmeSpec;

      PanDescriptor pd; pd.m_coorPanId = panId; pd.m_coorShortAddr = Mac16Address("00:01"); pd.m_logCh = channelNum;
      SuperframeField sf; sf.SetSuperframeOrder(SO); sf.SetBeaconOrder(BO); pd.m_superframeSpec = sf; pd.m_dsmeSuperframeSpec = dsmeSpec; pd.m_bcnBitmap = cBitmap;

      lrWpanHelper.CoordBoostrap(d0, pd, sdIdx, start);
      Simulator::Schedule(Seconds(0.05), [d0, sdIdx]() {
        d0->SendDsmeBeaconAllocNotify();
        g_chosen[d0->GetNode()->GetId()] = sdIdx;
        UpdateAnimLabel(d0->GetNode()->GetId());
      });
    };
    Simulator::Schedule(Seconds(base), *self);
  }

  // NetAnim
  {
    AnimationInterface anim(fileStem + ".xml");
    g_anim = &anim; anim.SetMobilityPollInterval(Seconds(0.1)); anim.EnablePacketMetadata(true);
    anim.UpdateNodeDescription(nodes.Get(0), "PAN-C SD=0 Tx=0");
    anim.UpdateNodeColor(nodes.Get(0), 0, 0, 0);
    anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);
    for (uint32_t i = 1; i < g_numCoord; ++i)
    {
      std::ostringstream lab; std::string sdLab = "?"; auto it = g_chosen.find(i); if (it != g_chosen.end()) sdLab = std::to_string(it->second);
      lab << "Coord" << i << " SD=" << sdLab; anim.UpdateNodeDescription(nodes.Get(i), lab.str()); anim.UpdateNodeSize(nodes.Get(i)->GetId(), 12.0, 12.0);
    }
    for (uint32_t i = 1; i < g_numCoord; ++i) { anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255); }
    Simulator::Stop(Seconds(simTime));
    Simulator::Schedule(Seconds(simTime - 1e-6), &PrintSummary, nodes.GetN());
    Simulator::Run();
    g_anim = nullptr;
  }
  Simulator::Destroy();
  return 0;
}
