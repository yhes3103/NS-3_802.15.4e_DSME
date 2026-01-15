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
#include <functional>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeBeaconSlotSelectionRandomPick");

#define BO 6
#define SO 3
#define MO 5

#define NUM_COORD 5 // 1 PAN-C + 4 joining coordinators

static void LogPickedSlot(uint32_t nodeIdx, uint16_t sdIdx)
{
  NS_LOG_UNCOND("Node " << nodeIdx << " picked Beacon SDIndex=" << sdIdx);
}

// Record chosen SDIndex per node for final summary
static std::map<uint32_t, uint16_t> g_chosen;

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

  NS_LOG_UNCOND("Collisions (nodes beyond unique per SD): " << collisionNodes);
  NS_LOG_UNCOND("Unassigned nodes: " << unassigned);
}

int main(int argc, char** argv)
{
  bool verbose = true;
  bool promiscuousPcap = true;
  double simTime = 20.0; // 拉長預設模擬時間，讓各節點有餘裕完成挑選與公告
  uint32_t seed = 3; // deterministic seed

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

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
  pos->Add(Vector(0.0, 100.0, 0.0));   // Node 0: PAN-C (00:01)
  pos->Add(Vector(-70.0, 50.0, 0.0));  // Node 1: Coord1 (00:02)
  pos->Add(Vector(70.0, 50.0, 0.0));   // Node 2: Coord2 (00:03)
  pos->Add(Vector(-100.0, 0.0, 0.0));  // Node 3: Coord3 (00:04)
  pos->Add(Vector(100.0, 0.0, 0.0));   // Node 4: Coord4 (00:05)
  mobility.SetPositionAllocator(pos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  LrWpanHelper lrWpanHelper(true);
  NetDeviceContainer devs = lrWpanHelper.Install(nodes);
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

    // Choose a parent based on layout for reliable sync
    Mac16Address parent = Mac16Address("00:01");
    if (i == 3) parent = Mac16Address("00:02");
    if (i == 4) parent = Mac16Address("00:03");

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
      });
    };

    Simulator::Schedule(Seconds(base), *self);
  }

  // NetAnim
  AnimationInterface anim("dsme-beacon-slot-selection-random-pick.xml");
  anim.SetMobilityPollInterval(Seconds(0.1));
  anim.EnablePacketMetadata(true);

  anim.UpdateNodeDescription(nodes.Get(0), "PAN-C [00:01] SD=0");
  anim.UpdateNodeColor(nodes.Get(0), 255, 0, 0);
  anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);

  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    Mac16Address parent = Mac16Address("00:01");
    if (i == 3) parent = Mac16Address("00:02");
    if (i == 4) parent = Mac16Address("00:03");

    std::ostringstream lab;
    lab << "Coord" << i << " [00:" << std::setfill('0') << std::setw(2) << (i + 1)
        << "] SD=chosen-in-sim P=";
    uint8_t buf[2];
    parent.CopyTo(buf);
    lab << std::setfill('0') << std::setw(2) << unsigned(buf[0])
        << ":" << std::setfill('0') << std::setw(2) << unsigned(buf[1]);
    anim.UpdateNodeDescription(nodes.Get(i), lab.str());
    anim.UpdateNodeSize(nodes.Get(i)->GetId(), 12.0, 12.0);
  }
  anim.UpdateNodeColor(nodes.Get(1), 0, 128, 255);
  anim.UpdateNodeColor(nodes.Get(2), 0, 200, 0);
  anim.UpdateNodeColor(nodes.Get(3), 200, 100, 0);
  anim.UpdateNodeColor(nodes.Get(4), 128, 0, 128);

  Simulator::Stop(Seconds(simTime));
  // Print summary just before stop
  Simulator::Schedule(Seconds(simTime - 1e-6), &PrintSummary, nodes.GetN());
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
