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

#define NUM_COORD 16 // 1 PAN-C + 15 joining coordinators（增加協調器以提高碰撞機率）
#define NUM_OBSERVERS 8 // 觀察者改為 8 個，形成 3x3 格（PAN-C 在中心）

// 幾何近似的覆蓋半徑（公尺），用於判斷觀察者是否同時位於多個衝突協調器的覆蓋範圍內
static double g_coverageRadius = 80.0;

static void LogPickedSlot(uint32_t nodeIdx, uint16_t sdIdx)
{
  NS_LOG_UNCOND("Node " << nodeIdx << " picked Beacon SDIndex=" << sdIdx);
}

// Record chosen SDIndex per node for final summary
static std::map<uint32_t, uint16_t> g_chosen;
static std::vector<uint32_t> g_observers; // 儲存旁觀端的 nodeId
static AnimationInterface* g_anim = nullptr; // 用於在總表階段更新顏色

static double Dist(const Vector& a, const Vector& b)
{
  double dx = a.x - b.x, dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
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
  // 若存在同一 SDIndex 的多個協調器，且某觀察者同時位於其中兩個以上的覆蓋範圍內，
  // 則此觀察者被視為可能遭遇 beacon 碰撞。
  uint32_t obsCollisions = 0;
  for (uint32_t obsId : g_observers)
  {
    Ptr<Node> on = NodeList::GetNode(obsId);
    Vector op = on->GetObject<MobilityModel>()->GetPosition();
    bool hasCollision = false;
    for (const auto& kv : groups)
    {
      if (kv.first == 0 || kv.second.size() < 2) continue;
      uint32_t within = 0;
      for (uint32_t coordId : kv.second)
      {
        Ptr<Node> cn = NodeList::GetNode(coordId);
        Vector cp = cn->GetObject<MobilityModel>()->GetPosition();
        if (Dist(op, cp) <= g_coverageRadius) within++;
        if (within >= 2) { hasCollision = true; break; }
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

  // 對每個觀察者，檢查是否同時落在同一 SDIndex 的兩個以上協調器覆蓋範圍
  for (uint32_t obsId : g_observers)
  {
    Ptr<Node> on = NodeList::GetNode(obsId);
    Vector op = on->GetObject<MobilityModel>()->GetPosition();
    bool hasCollision = false;

    for (const auto& kv : groups)
    {
      if (kv.first == 0 || kv.second.size() < 2) continue;
      uint32_t within = 0;
      for (uint32_t coordId : kv.second)
      {
        Ptr<Node> cn = NodeList::GetNode(coordId);
        Vector cp = cn->GetObject<MobilityModel>()->GetPosition();
        if (Dist(op, cp) <= g_coverageRadius) within++;
        if (within >= 2) { hasCollision = true; break; }
      }
      if (hasCollision) break;
    }

    if (hasCollision)
    {
      g_anim->UpdateNodeColor(on, 255, 215, 0); // 金色
    }
    else
    {
      g_anim->UpdateNodeColor(on, 0, 0, 0); // 黑色
    }
  }

  // 依加入結果把「未加入/未分配」的協調器標示為紅色，其餘協調器維持藍色
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
    else
    {
      g_anim->UpdateNodeColor(cn, 0, 0, 255); // 藍色：已加入（即使可能碰撞）
    }
  }
}

int main(int argc, char** argv)
{
  bool verbose = true;
  bool promiscuousPcap = true;
  double simTime = 8.0; // 拉長預設模擬時間，讓各節點有餘裕完成挑選與公告
  uint32_t seed = 4; // deterministic seed

  CommandLine cmd(__FILE__);
  cmd.AddValue("verbose", "Enable component logs", verbose);
  cmd.AddValue("promisc", "Enable promiscuous PCAP", promiscuousPcap);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("seed", "RNG seed for RngSeedManager", seed);
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

  // 旁觀端裝置（只接收）
  NodeContainer observers;
  observers.Create(NUM_OBSERVERS);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
  // PAN-C 放在正中央 (0,0)
  pos->Add(Vector(0.0, 0.0, 0.0));
  // 其餘協調器隨機擺放於正方區域內（可透過 seed 重現）
  double R = 150.0; // 區域半邊長（座標範圍為 [-R, R]）
  Ptr<UniformRandomVariable> urvX = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> urvY = CreateObject<UniformRandomVariable>();
  urvX->SetAttribute("Min", DoubleValue(-R));
  urvX->SetAttribute("Max", DoubleValue(R));
  urvY->SetAttribute("Min", DoubleValue(-R));
  urvY->SetAttribute("Max", DoubleValue(R));
  for (uint32_t i = 1; i < NUM_COORD; ++i) {
    double x = urvX->GetValue();
    double y = urvY->GetValue();
    pos->Add(Vector(x, y, 0.0));
  }
  mobility.SetPositionAllocator(pos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // 安排觀察者位置（選擇容易同時覆蓋的區域）
  Ptr<ListPositionAllocator> posObs = CreateObject<ListPositionAllocator>();
  // 以 PAN-C 為中心的 3x3 格子，觀察者取外圈 8 個點
  double d = 80.0; // 格距
  std::vector<Vector> grid = {
    Vector(-d, -d, 0.0), Vector(0.0, -d, 0.0), Vector(d, -d, 0.0),
    Vector(-d,  0.0, 0.0),                   /* center (0,0) 給 PAN-C */ Vector(d,  0.0, 0.0),
    Vector(-d,  d, 0.0),  Vector(0.0,  d, 0.0),  Vector(d,  d, 0.0)
  };
  for (uint32_t i = 0; i < NUM_OBSERVERS && i < grid.size(); ++i) {
    posObs->Add(grid[i]);
  }
  MobilityHelper mobilityObs;
  mobilityObs.SetPositionAllocator(posObs);
  mobilityObs.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityObs.Install(observers);

  LrWpanHelper lrWpanHelper(true);
  NetDeviceContainer devs = lrWpanHelper.Install(nodes);
  NetDeviceContainer obsDevs = lrWpanHelper.Install(observers);
  lrWpanHelper.EnablePcapAll(std::string("dsme-beacon-slot-selection-random-pick"), promiscuousPcap);

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

  // 設定觀察者為同 PAN，只接收不成為協調器
  for (uint32_t i = 0; i < obsDevs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d = obsDevs.Get(i)->GetObject<LrWpanNetDevice>();
    // 指派不重複短址：從 0x20 起
    uint8_t ab[2] = {0x00, static_cast<uint8_t>(0x20 + i)};
    Mac16Address sa; sa.CopyFrom(ab);
    d->GetMac()->SetShortAddress(sa);
    d->GetMac()->SetPanId(panId);
    d->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));
    MlmeSyncRequestParams sync;
    sync.m_logCh = channelNum;
    sync.m_trackBcn = true;
    d->TrackCoordinatorBeacon(sync);
    g_observers.push_back(d->GetNode()->GetId());
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

  // Joining coordinators
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();

    // 隨機拓樸下，統一由 PAN-C 作為 parent 以利同步
    Mac16Address parent = Mac16Address("00:01");

    d->GetMac()->SetAssociatedCoor(parent);

    MlmeSyncRequestParams sync;
    sync.m_logCh = channelNum;
    sync.m_logChPage = 0;
    sync.m_trackBcn = true;
    d->TrackCoordinatorBeacon(sync);

    // After listening some EBs, pick a vacant slot from local bitmap with retries
    const double base = 2.0 + 0.20 * i;
    const double retryInterval = 0.25; // s
    const uint32_t maxTries = 20;      // total ~5s retry window
    auto tries = std::make_shared<uint32_t>(0u);
    auto done = std::make_shared<bool>(false);

    auto self = std::make_shared<std::function<void()>>();
    *self = [d, panId, channelNum, dsmeSpec, &lrWpanHelper, tries, done, retryInterval, maxTries, self]() {
      if (*done) return;
      (*tries)++;

      // 若尚未收到任何 EB，位圖長度為 0，先延後重試
      auto agg = d->GetMac()->GetAggregatedBeaconBitmap();
      if (agg.GetSDBitmapLength() == 0) {
        if (*tries < maxTries) {
          Simulator::Schedule(Seconds(retryInterval), *self);
        } else {
          NS_LOG_UNCOND("Node " << d->GetNode()->GetId() << " no EB observed; give up selecting");
          g_chosen[d->GetNode()->GetId()] = 0xffff;
        }
        return;
      }

      uint16_t sdIdx = d->FindVacantBeaconSlot(true);
      // 保持純隨機選位（不強制重疊），以真實結果為準
      if (sdIdx == 0xffff) {
        if (*tries < maxTries) {
          Simulator::Schedule(Seconds(retryInterval), *self);
        } else {
          NS_LOG_UNCOND("Node " << d->GetNode()->GetId() << " no vacant SDIndex; give up selecting");
          g_chosen[d->GetNode()->GetId()] = 0xffff;
        }
        return;
      }

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
  AnimationInterface anim("dsme-beacon-slot-selection-random-pick.xml");
  g_anim = &anim;
  anim.SetMobilityPollInterval(Seconds(0.1));
  anim.EnablePacketMetadata(true);

  anim.UpdateNodeDescription(nodes.Get(0), "PAN-C SD=0");
  anim.UpdateNodeColor(nodes.Get(0), 0, 0, 0); // 改為黑色
  anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);

  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    std::ostringstream lab;
    lab << "Coord" << i << " SD=?";
    anim.UpdateNodeDescription(nodes.Get(i), lab.str());
    anim.UpdateNodeSize(nodes.Get(i)->GetId(), 12.0, 12.0);
  }
  // Coordinators 一律藍色
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255);
  }

  // 標示觀察者（預設黑色，如偵測碰撞則在總表階段改為金色）
  for (uint32_t i = 0; i < observers.GetN(); ++i)
  {
    Ptr<Node> on = observers.Get(i);
    std::ostringstream lab;
    lab << "O" << i;
    anim.UpdateNodeDescription(on, lab.str());
    anim.UpdateNodeColor(on, 0, 0, 0); // 黑色（無碰撞）
    anim.UpdateNodeSize(on->GetId(), 11.0, 11.0);
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
