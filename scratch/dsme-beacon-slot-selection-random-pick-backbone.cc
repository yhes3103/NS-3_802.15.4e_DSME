/*
 * DSME beacon slot selection (random-pick with CAP notify)
 * - 1 PAN-C (00:01) beacons at SDIndex=0
 * - 4 coordinators listen EBs, pick vacant SDIndex from aggregated bitmap,
 *   send DSME Beacon Allocation Notification in CAP, then start beacons
 */

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"
#include <vector>
#include <sstream>
#include <iomanip>
#include <map>
#include <cmath>
#include <functional>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeBeaconSlotSelectionRandomPick");

#define BO 6
#define SO 3
#define MO 5

#define NUM_COORD 16 // 1 PAN-C + 8 backbone coordinators + 7 random joining coordinators
#define NUM_BACKBONE 8 // 以原本觀察者位置改為已入網 backbone 協調器

// 幾何近似的覆蓋半徑（公尺），用於判斷觀察者是否同時位於多個衝突協調器的覆蓋範圍內
static double g_coverageRadius = 80.0;

// RSSI-based collision model parameters (for scheme B)
static Ptr<LogDistancePropagationLossModel> g_pl = nullptr;
static double g_rxSensDbm = -95.0;   // receiver sensitivity threshold (dBm)
static double g_plExponent = 3.0;    // path loss exponent
static double g_refDist = 1.0;       // reference distance (m)
static double g_refLossDb = 40.05;   // reference loss @ refDist (dB) ~ 2.4GHz FSPL at 1m
static double g_assumedTxDbm = 0.0;  // assumed TX power for coordinators (dBm)

static void LogPickedSlot(uint32_t nodeIdx, uint16_t sdIdx)
{
  NS_LOG_UNCOND("Node " << nodeIdx << " picked Beacon SDIndex=" << sdIdx);
}

// Record chosen SDIndex per node for final summary
static std::map<uint32_t, uint16_t> g_chosen;
static std::vector<uint32_t> g_observers; // 用於RSSI檢查的觀察者：這裡改為所有協調器自身
static AnimationInterface* g_anim = nullptr; // 用於在總表階段更新顏色

static double Dist(const Vector& a, const Vector& b)
{
  double dx = a.x - b.x, dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

// Derive short address (0x0001-based) from node index in this program
static Mac16Address ShortFromNodeIndex(uint32_t nodeIdx)
{
  uint16_t shortVal = static_cast<uint16_t>(nodeIdx + 1u);
  uint8_t ab[2] = {static_cast<uint8_t>((shortVal >> 8) & 0xff), static_cast<uint8_t>(shortVal & 0xff)};
  Mac16Address sa; sa.CopyFrom(ab);
  return sa;
}

// Derive this source file's stem (filename without extension)
static std::string SelfStem()
{
  std::string f = __FILE__;
  size_t slash = f.find_last_of("/\\");
  if (slash != std::string::npos)
  {
    f = f.substr(slash + 1);
  }
  size_t dot = f.rfind('.');
  if (dot != std::string::npos)
  {
    f = f.substr(0, dot);
  }
  return f;
}

static void PrintSummary(uint32_t numNodes)
{
  // Compute collisions: nodes choosing same SDIndex (>1) excluding SDIndex 0
  std::map<uint16_t, std::vector<uint32_t>> groups;
  for (const auto& kv : g_chosen)
  {
    if (kv.second != 0xffff)
      groups[kv.second].push_back(kv.first);
  }

  uint32_t collisionNodes = 0;
  for (const auto& kv : groups)
  {
    if (kv.first == 0) continue; // PAN-C reserved
    if (kv.second.size() > 1)
      collisionNodes += static_cast<uint32_t>(kv.second.size() - 1);
  }

  NS_LOG_UNCOND("==== Beacon Slot Selection Summary ====");
  NS_LOG_UNCOND("Node\tPicked_SDIndex");
  for (uint32_t i = 0; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    if (it != g_chosen.end())
    {
      NS_LOG_UNCOND(i << "\t" << it->second);
    }
    else
    {
      NS_LOG_UNCOND(i << "\t(n/a)");
    }
  }
  // 未完成分配：沒有紀錄或紀錄為 0xffff（不含 PAN-C 節點 0）
  uint32_t unassigned = 0;
  for (uint32_t i = 1; i < numNodes; ++i)
  {
    auto it = g_chosen.find(i);
    if (it == g_chosen.end() || it->second == 0xffff)
      ++unassigned;
  }

  NS_LOG_UNCOND("Unassigned nodes: " << unassigned);

  // 幾何近似的「觀察者可見碰撞」檢查：
  // 改為 RSSI 近似：對每個觀察者、每個 SDIndex 群組，
  // 計算以 LogDistance 模型預估之接收功率 Pr(dBm)；若可見 (Pr>=g_rxSensDbm) 的發送者數量 >=2，
  // 則視為該觀察者在該 SDIndex 可能遭遇 beacon 碰撞。
  uint32_t obsCollisions = 0;
  for (uint32_t obsId : g_observers)
  {
    Ptr<Node> on = NodeList::GetNode(obsId);
    // RSSI-based: position fetched inside model; no direct use here
    bool hasCollision = false;
    for (const auto& kv : groups)
    {
      if (kv.first == 0 || kv.second.size() < 2) continue;
      uint32_t visible = 0;
      Ptr<MobilityModel> rx = on->GetObject<MobilityModel>();
      for (uint32_t coordId : kv.second)
      {
        Ptr<Node> cn = NodeList::GetNode(coordId);
        Ptr<MobilityModel> tx = cn->GetObject<MobilityModel>();
        double prDbm = g_pl ? g_pl->CalcRxPower(g_assumedTxDbm, tx, rx)
                            : -1e9; // if not initialized, treat as not visible
        if (prDbm >= g_rxSensDbm) visible++;
        if (visible >= 2) { hasCollision = true; break; }
      }
      if (hasCollision) break;
    }
    if (hasCollision) obsCollisions++;
    // 觀察者顏色：發生碰撞者金色，否則黑色
    if (g_anim)
    {
      if (hasCollision) { g_anim->UpdateNodeColor(on, 255, 215, 0); }
      else { g_anim->UpdateNodeColor(on, 0, 0, 0); }
    }
  }
  NS_LOG_UNCOND("Observers potentially experiencing beacon collision: " << obsCollisions << "/" << g_observers.size());
}

// 於模擬進行中依現有 g_chosen 週期性更新觀察者顏色，
// 讓可能的碰撞更早在 NetAnim 呈現（金色=可能觀察到碰撞，黑色=否）。
static void UpdateObserverCollisionColors()
{
  if (!g_anim)
  {
    return;
  }

  // 依當前挑選結果建立 SDIndex 群組（忽略未分配 0xffff）
  std::map<uint16_t, std::vector<uint32_t>> groups;
  for (const auto& kv : g_chosen)
  {
    if (kv.second != 0xffff)
    {
      groups[kv.second].push_back(kv.first);
    }
  }

  // 對每個觀察者（此處即所有協調器），依 RSSI 推估可見的同 SDIndex 發送者數是否 >= 2
  for (uint32_t obsId : g_observers)
  {
    Ptr<Node> on = NodeList::GetNode(obsId);
    // RSSI-based: position fetched inside model; no direct use here
    bool hasCollision = false;

    for (const auto& kv : groups)
    {
      if (kv.first == 0 || kv.second.size() < 2) continue;
      uint32_t visible = 0;
      Ptr<MobilityModel> rx = on->GetObject<MobilityModel>();
      for (uint32_t coordId : kv.second)
      {
        Ptr<Node> cn = NodeList::GetNode(coordId);
        Ptr<MobilityModel> tx = cn->GetObject<MobilityModel>();
        double prDbm = g_pl ? g_pl->CalcRxPower(g_assumedTxDbm, tx, rx)
                            : -1e9;
        if (prDbm >= g_rxSensDbm) visible++;
        if (visible >= 2) { hasCollision = true; break; }
      }
      if (hasCollision) break;
    }

    if (hasCollision) { g_anim->UpdateNodeColor(on, 255, 215, 0); } // 金色
    // 若無碰撞則保持既有顏色（藍色或紅色），不強制改為黑色
  }

  // 依加入結果把「未加入/未分配」的協調器標示為紅色；已加入者維持原色（避免覆蓋碰撞的金色）
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    auto it = g_chosen.find(i);
    Ptr<Node> cn = NodeList::GetNode(i);
    // 更新協調器的簡潔描述：只顯示序號與 SDIndex
    std::string sdLab = "?";
    if (it != g_chosen.end())
    {
      if (it->second == 0xffff) sdLab = "NA"; else sdLab = std::to_string(it->second);
    }
    {
      std::ostringstream lab;
      lab << "Coord" << i << " SD=" << sdLab;
      g_anim->UpdateNodeDescription(cn, lab.str());
    }
    if (it != g_chosen.end() && it->second == 0xffff)
    {
      g_anim->UpdateNodeColor(cn, 255, 0, 0); // 紅色：未加入
    }
  }
}

int main(int argc, char** argv)
{
  bool verbose = true;
  bool promiscuousPcap = true;
  double simTime = 29.0; // 拉長預設模擬時間，讓各節點有餘裕完成挑選與公告
  uint32_t seed = 4; // deterministic seed

  CommandLine cmd(__FILE__);
  cmd.AddValue("verbose", "Enable component logs", verbose);
  cmd.AddValue("promisc", "Enable promiscuous PCAP", promiscuousPcap);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("seed", "RNG seed for RngSeedManager", seed);
  // RSSI-based collision knobs
  cmd.AddValue("rxSensDbm", "Receiver sensitivity for collision check (dBm)", g_rxSensDbm);
  cmd.AddValue("plExp", "Path loss exponent", g_plExponent);
  cmd.AddValue("refDist", "Reference distance (m)", g_refDist);
  cmd.AddValue("refLossDb", "Reference loss at refDist (dB)", g_refLossDb);
  cmd.AddValue("assumedTxDbm", "Assumed coordinator TX power (dBm)", g_assumedTxDbm);
  cmd.Parse(argc, argv);

  if (verbose)
  {
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
  }

  RngSeedManager::SetSeed(seed);

  NodeContainer nodes;
  nodes.Create(NUM_COORD);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
  // PAN-C 放在正中央 (0,0)
  pos->Add(Vector(0.0, 0.0, 0.0));
  // 設定 8 個 backbone 協調器位置：以 PAN-C 為中心之 3x3 外圈，自左上到右下
  double d = 80.0; // 格距
  std::vector<Vector> backboneGrid = {
      Vector(-d,  d, 0.0), Vector(0.0,  d, 0.0), Vector( d,  d, 0.0),
      Vector(-d,  0.0, 0.0),                  Vector( d,  0.0, 0.0),
      Vector(-d, -d, 0.0), Vector(0.0, -d, 0.0), Vector( d, -d, 0.0)};
  for (const auto& v : backboneGrid)
  {
    pos->Add(v);
  }
  // 其餘協調器隨機擺放於正方區域內（可透過 seed 重現）
  double R = 150.0; // 區域半邊長（座標範圍為 [-R, R]）
  Ptr<UniformRandomVariable> urvX = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> urvY = CreateObject<UniformRandomVariable>();
  urvX->SetAttribute("Min", DoubleValue(-R));
  urvX->SetAttribute("Max", DoubleValue(R));
  urvY->SetAttribute("Min", DoubleValue(-R));
  urvY->SetAttribute("Max", DoubleValue(R));
  for (uint32_t i = 1 + NUM_BACKBONE; i < NUM_COORD; ++i)
  {
    double x = urvX->GetValue();
    double y = urvY->GetValue();
    pos->Add(Vector(x, y, 0.0));
  }
  mobility.SetPositionAllocator(pos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Initialize path loss model for RSSI-based collision checks
  g_pl = CreateObject<LogDistancePropagationLossModel>();
  g_pl->SetPathLossExponent(g_plExponent);
  g_pl->SetReference(g_refDist, g_refLossDb);

  LrWpanHelper lrWpanHelper(true);
  NetDeviceContainer devs = lrWpanHelper.Install(nodes);
  // Use source filename stem for PCAP prefix
  std::string fileStem = SelfStem();
  lrWpanHelper.EnablePcapAll(fileStem, promiscuousPcap);

  const uint16_t numChSupported = 6;
  const bool capReduction = false;
  const uint8_t panId = 0x0007;
  const uint8_t channelNum = 11;

  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();
    d->GetMac()->SetNumOfChannelSupported(numChSupported);
    d->GetMac()->SetCAPReduction(capReduction);
  }

  // 將所有協調器節點加入觀察者集合（用於RSSI可見碰撞統計與著色）
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
  {
    g_observers.push_back(nodes.Get(i)->GetId());
  }

  // PAN-C setup at SDIndex=0
  MlmeStartRequestParams panStart;
  panStart.m_panCoor = true;
  panStart.m_PanId = panId;
  panStart.m_bcnOrd = BO;
  panStart.m_sfrmOrd = SO;
  panStart.m_logCh = channelNum;

  BeaconBitmap panBitmap(0, 1 << (BO - SO));
  panBitmap.SetSDIndex(0);
  panStart.m_bcnBitmap = panBitmap;

  HoppingDescriptor panHop;
  panHop.m_HoppingSequenceID = 0;
  panHop.m_hoppingSeqLen = 0;
  panHop.m_channelOfs = 0;
  panHop.m_channelOfsBitmapLen = 16;
  panHop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << 0));
  panStart.m_hoppingDescriptor = panHop;

  DsmeSuperFrameField dsmeSpec;
  dsmeSpec.SetMultiSuperframeOrder(MO);
  dsmeSpec.SetChannelDiversityMode(1 /*CHANNEL_HOPPING*/);
  dsmeSpec.SetCAPReductionFlag(capReduction);
  panStart.m_dsmeSuperframeSpec = dsmeSpec;

  lrWpanHelper.AssociateToBeaconPan(devs, Mac16Address("00:01"), panStart);

  // 設定 8 個 backbone 協調器（節點 1..8）使用固定 SDIndex：依左上到右下順序指定 1,2,3,4,5,6,7,1
  {
    uint16_t fixedSlots[NUM_BACKBONE] = {1,2,3,4,5,6,7,1};
    for (uint32_t bi = 0; bi < NUM_BACKBONE; ++bi)
    {
      uint32_t i = 1 + bi;
      Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();
      // 與 PAN 同步
      d->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));
      MlmeSyncRequestParams sync;
      sync.m_logCh = channelNum;
      sync.m_logChPage = 0;
      sync.m_trackBcn = true;
      d->TrackCoordinatorBeacon(sync);

      uint16_t sdIdx = fixedSlots[bi];
      d->GetMac()->SetTimeSlotToSendBcn(sdIdx);

      MlmeStartRequestParams start;
      start.m_panCoor = false;
      start.m_PanId = panId;
      start.m_bcnOrd = BO;
      start.m_sfrmOrd = SO;
      BeaconBitmap cBitmap(0, 1 << (BO - SO));
      cBitmap.SetSDIndex(sdIdx);
      start.m_bcnBitmap = cBitmap;
      HoppingDescriptor hop;
      hop.m_HoppingSequenceID = 0;
      hop.m_hoppingSeqLen = 0;
      hop.m_channelOfs = sdIdx;
      hop.m_channelOfsBitmapLen = 16;
      hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << (sdIdx % 16)));
      start.m_hoppingDescriptor = hop;
      start.m_dsmeSuperframeSpec = dsmeSpec;

      PanDescriptor pd;
      pd.m_coorPanId = panId;
      pd.m_coorShortAddr = Mac16Address("00:01");
      pd.m_logCh = channelNum;
      SuperframeField sf;
      sf.SetSuperframeOrder(SO);
      sf.SetBeaconOrder(BO);
      pd.m_superframeSpec = sf;
      pd.m_dsmeSuperframeSpec = dsmeSpec;
      pd.m_bcnBitmap = cBitmap;

      lrWpanHelper.CoordBoostrap(d, pd, sdIdx, start);
      Simulator::Schedule(Seconds(0.05), [d, sdIdx]() {
        d->SendDsmeBeaconAllocNotify();
      });
      g_chosen[i] = sdIdx;
    }
  }

  // 其餘隨機加入協調器（節點 1+NUM_BACKBONE .. NUM_COORD-1）
  for (uint32_t i = 1 + NUM_BACKBONE; i < NUM_COORD; ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();

    // 不強制找 PAN-C；改為選擇「最鄰近」的 backbone 協調器作為 parent
    // 這符合「聽到任一個 beacon 就能入網」的預期，便於建立本地 EB 位圖
    uint32_t bestParentIdx = 1; // default to first backbone
    double bestDist = 1e100;
    Ptr<MobilityModel> mmSelf = nodes.Get(i)->GetObject<MobilityModel>();
    Vector posSelf = mmSelf ? mmSelf->GetPosition() : Vector();
    for (uint32_t bi = 0; bi < NUM_BACKBONE; ++bi)
    {
      uint32_t cand = 1 + bi;
      Ptr<MobilityModel> mm = nodes.Get(cand)->GetObject<MobilityModel>();
      if (!mm) continue;
      double dxy = Dist(posSelf, mm->GetPosition());
      if (dxy < bestDist)
      {
        bestDist = dxy;
        bestParentIdx = cand;
      }
    }
    Mac16Address parent = ShortFromNodeIndex(bestParentIdx);
    d->GetMac()->SetAssociatedCoor(parent);

    MlmeSyncRequestParams sync;
    sync.m_logCh = channelNum;
    sync.m_logChPage = 0;
    sync.m_trackBcn = true;
    d->TrackCoordinatorBeacon(sync);

    // After listening some EBs, pick a vacant slot from local bitmap with retries
    const double base = 2.0 + 0.20 * i;
    const double retryInterval = 0.25; // s
    const uint32_t maxTries = 24;      // total ~6s retry window，給非 PAN 關聯多一點時間收EB
    auto tries = std::make_shared<uint32_t>(0u);
    auto done = std::make_shared<bool>(false);

    auto self = std::make_shared<std::function<void()>>();
    *self = [d, panId, channelNum, dsmeSpec, &lrWpanHelper, tries, done, retryInterval, maxTries, self, i]() {
      if (*done) return;
      (*tries)++;

      // 只看本地可見的協調器所使用的 SDIndex，從剩餘的中挑選
      const uint16_t slotsCount = static_cast<uint16_t>(1u << (BO - SO));
      std::vector<bool> locallyUsed(slotsCount, false);
      Ptr<MobilityModel> rx = NodeList::GetNode(i)->GetObject<MobilityModel>();
      if (rx)
      {
        for (uint32_t cid = 1; cid < NUM_COORD; ++cid)
        {
          auto it = g_chosen.find(cid);
          if (it == g_chosen.end()) continue;
          uint16_t usedIdx = it->second;
          if (usedIdx == 0xffff || usedIdx >= slotsCount) continue;
          Ptr<Node> cn = NodeList::GetNode(cid);
          if (!cn) continue;
          Ptr<MobilityModel> tx = cn->GetObject<MobilityModel>();
          if (!tx) continue;
          double prDbm = g_pl ? g_pl->CalcRxPower(g_assumedTxDbm, tx, rx) : -1e9;
          if (prDbm >= g_rxSensDbm)
          {
            locallyUsed[usedIdx] = true;
          }
        }
      }

      // 從 1..(slotsCount-1) 中挑選未被本地可見者使用的 SDIndex
      std::vector<uint16_t> candidates;
      for (uint16_t s = 1; s < slotsCount; ++s)
      {
        if (!locallyUsed[s]) candidates.push_back(s);
      }

      if (candidates.empty())
      {
        if (*tries < maxTries)
        {
          Simulator::Schedule(Seconds(retryInterval), *self);
        }
        else
        {
          NS_LOG_UNCOND("Node " << d->GetNode()->GetId() << " no locally-free SDIndex; give up selecting");
          g_chosen[d->GetNode()->GetId()] = 0xffff;
        }
        return;
      }

      Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable>();
      uint16_t sdIdx = candidates[static_cast<uint16_t>(urv->GetInteger(0, static_cast<int>(candidates.size() - 1)) )];

      *done = true;
      d->GetMac()->SetTimeSlotToSendBcn(sdIdx);

      MlmeStartRequestParams start;
      start.m_panCoor = false;
      start.m_PanId = panId;
      start.m_bcnOrd = BO;
      start.m_sfrmOrd = SO;

      BeaconBitmap cBitmap(0, 1 << (BO - SO));
      cBitmap.SetSDIndex(sdIdx);
      start.m_bcnBitmap = cBitmap;

      HoppingDescriptor hop;
      hop.m_HoppingSequenceID = 0;
      hop.m_hoppingSeqLen = 0;
      hop.m_channelOfs = sdIdx; // illustrative
      hop.m_channelOfsBitmapLen = 16;
      hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << (sdIdx % 16)));
      start.m_hoppingDescriptor = hop;

      start.m_dsmeSuperframeSpec = dsmeSpec;

      // Minimal PanDescriptor to bootstrap as coordinator
      PanDescriptor pd;
      pd.m_coorPanId = panId;
      pd.m_coorShortAddr = Mac16Address("00:01");
      pd.m_logCh = channelNum;
      SuperframeField sf;
      sf.SetSuperframeOrder(SO);
      sf.SetBeaconOrder(BO);
      pd.m_superframeSpec = sf;
      pd.m_dsmeSuperframeSpec = dsmeSpec;
      pd.m_bcnBitmap = cBitmap; // local view for label

      // Start as coordinator then notify in CAP
      lrWpanHelper.CoordBoostrap(d, pd, sdIdx, start);
      Simulator::Schedule(Seconds(0.08), [d, sdIdx]() {
        d->SendDsmeBeaconAllocNotify();
        LogPickedSlot(d->GetNode()->GetId(), sdIdx);
        g_chosen[d->GetNode()->GetId()] = sdIdx;
        // 立即更新該協調器在 NetAnim 的簡潔描述
        if (g_anim)
        {
          std::ostringstream lab;
          lab << "Coord" << d->GetNode()->GetId() << " SD=" << sdIdx;
          g_anim->UpdateNodeDescription(d->GetNode(), lab.str());
        }
      });
    };

    Simulator::Schedule(Seconds(base), *self);
  }

  // NetAnim
  AnimationInterface anim(fileStem + ".xml");
  g_anim = &anim;
  anim.SetMobilityPollInterval(Seconds(0.1));
  anim.EnablePacketMetadata(true);

  anim.UpdateNodeDescription(nodes.Get(0), "PAN-C SD=0");
  anim.UpdateNodeColor(nodes.Get(0), 0, 0, 0); // 改為黑色
  anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);

  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    std::ostringstream lab;
    std::string sdLab = "?";
    auto it = g_chosen.find(i);
    if (it != g_chosen.end()) sdLab = std::to_string(it->second);
    lab << "Coord" << i << " SD=" << sdLab;
    anim.UpdateNodeDescription(nodes.Get(i), lab.str());
    anim.UpdateNodeSize(nodes.Get(i)->GetId(), 12.0, 12.0);
  }
  // Coordinators 一律藍色
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255);
  }

  // 週期性提前刷新觀察者顏色（從 3 秒開始，每 0.5 秒，直到結束前 0.5 秒）
  const double refreshInterval = 0.5;
  double refreshStart = 0.5; // 更早開始刷新，盡快反映紅色/金色
  double refreshEnd = simTime - 0.5;
  if (refreshEnd < 0.0) refreshEnd = 0.0;

  auto refresher = std::make_shared<std::function<void()>>();
  *refresher = [refresher, refreshInterval, refreshEnd]() {
    UpdateObserverCollisionColors();
    if (Simulator::Now().GetSeconds() + refreshInterval <= refreshEnd)
    {
      Simulator::Schedule(Seconds(refreshInterval), *refresher);
    }
  };
  Simulator::Schedule(Seconds(refreshStart), *refresher);

  Simulator::Stop(Seconds(simTime));
  // Print summary just before stop
  Simulator::Schedule(Seconds(simTime - 1e-6), &PrintSummary, nodes.GetN());
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
