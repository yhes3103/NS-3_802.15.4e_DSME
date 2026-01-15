/*
 * DSME beacon slot selection (random, conflict-free)
 * - 1 PAN-C (00:01) beacons at SDIndex=0
 * - 5 coordinators randomly pick unique SDIndex from 1..(2^(BO-SO)-1)
 * - Emits PCAPs and NetAnim with clear labels and larger nodes
 */

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include <vector>
#include <sstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeBeaconSlotSelectionRandom");

#define BO 6
#define SO 3
#define MO 5

#define NUM_COORD 6 // 1 PAN-C + 5 joining coordinators

#define BIT(X) (1u << (X))

static void LogPickedSlot(uint32_t nodeIdx, uint16_t sdIdx)
{
  NS_LOG_UNCOND("Node " << nodeIdx << " picked Beacon SDIndex=" << sdIdx);
}

int main(int argc, char** argv)
{
  bool verbose = true;
  bool promiscuousPcap = true;
  double simTime = 6.0;
  uint32_t seed = 2; // default deterministic seed

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
  pos->Add(Vector(0.0, -20.0, 0.0));   // Node 4: Coord4 (00:05)
  pos->Add(Vector(100.0, 0.0, 0.0));   // Node 5: Coord5 (00:06)
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
  panHop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(BIT(0)));
  panStart.m_hoppingDescriptor = panHop;

  DsmeSuperFrameField dsmeSpec;
  dsmeSpec.SetMultiSuperframeOrder(MO);
  dsmeSpec.SetChannelDiversityMode(1 /*CHANNEL_HOPPING*/);
  dsmeSpec.SetCAPReductionFlag(capReduction);
  panStart.m_dsmeSuperframeSpec = dsmeSpec;

  lrWpanHelper.AssociateToBeaconPan(devs, Mac16Address("00:01"), panStart);

  // Prepare candidate SD slots (exclude 0 reserved for PAN-C)
  const uint16_t totalSlots = (1u << (BO - SO));
  std::vector<uint16_t> freeSlots;
  for (uint16_t s = 1; s < totalSlots; ++s)
  {
    freeSlots.push_back(s);
  }
  Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable>();

  // Joining coordinators: randomly pick unique SDIndex
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();

    // Choose a parent based on layout for reliable sync
    Mac16Address parent = Mac16Address("00:01");
    if (i == 3) parent = Mac16Address("00:02");
    if (i == 4) parent = Mac16Address("00:02");
    if (i == 5) parent = Mac16Address("00:03");

    d->GetMac()->SetAssociatedCoor(parent);

    MlmeSyncRequestParams sync;
    sync.m_logChPage = 0;
    sync.m_trackBcn = true;
    d->TrackCoordinatorBeacon(sync);

    // Randomly pick one available SDIndex (unique)
    uint32_t idx = urv->GetInteger(0, (int)freeSlots.size() - 1);
    uint16_t sdIdx = freeSlots[idx];
    freeSlots.erase(freeSlots.begin() + idx);

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
    hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(BIT(sdIdx)));
    start.m_hoppingDescriptor = hop;

    start.m_dsmeSuperframeSpec = dsmeSpec;

    PanDescriptor pd;
    pd.m_coorPanId = panId;
    pd.m_coorShortAddr = parent;
    pd.m_logCh = channelNum;
    SuperframeField sf;
    sf.SetSuperframeOrder(SO);
    sf.SetBeaconOrder(BO);
    pd.m_superframeSpec = sf;
    pd.m_dsmeSuperframeSpec = dsmeSpec;
    pd.m_bcnBitmap = panBitmap;

    // Delay bootstrap to ensure sync is established
    Simulator::Schedule(Seconds(2.20 + 0.20 * i), &LrWpanHelper::CoordBoostrap,
                        &lrWpanHelper, d, pd, i /* short addr low byte */, start);

    Simulator::ScheduleNow(&LogPickedSlot, i, sdIdx);
  }

  // NetAnim
  AnimationInterface anim("dsme-beacon-slot-selection-random-pick.xml");
  anim.SetMobilityPollInterval(Seconds(0.1));
  anim.EnablePacketMetadata(true);

  anim.UpdateNodeDescription(nodes.Get(0), "PAN-C [00:01] SD=0");
  anim.UpdateNodeColor(nodes.Get(0), 255, 0, 0);
  anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);

  // Recompute selected SD label (deterministic here due to scheduleNow print; for label we recompute similarly)
  // For clarity, rebuild the labels using the same parent mapping and indicate that SD is random.
  // Note: This label does not recompute the actual random choice; it is for display.
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    Mac16Address parent = Mac16Address("00:01");
    if (i == 3) parent = Mac16Address("00:02");
    if (i == 4) parent = Mac16Address("00:02");
    if (i == 5) parent = Mac16Address("00:03");

    std::ostringstream lab;
    lab << "Coord" << i << " [00:" << std::setfill('0') << std::setw(2) << (i + 1)
        << "] SD=random P=";
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
  anim.UpdateNodeColor(nodes.Get(4), 150, 150, 150);
  anim.UpdateNodeColor(nodes.Get(5), 128, 0, 128);

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}

