/*
 * One PAN-C starts DSME; five coordinators join and select beacon slots.
 * Generates PCAPs and NetAnim XML to visualize EB slot selection.
 */

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include <vector>
#include <sstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeBeaconSlotSelection");

// DSME timing (fits many beacon SD slots)
#define BO 6
#define SO 3
#define MO 5

// 1 PAN-C + 5 joining coordinators
#define NUM_COORD 6

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

  CommandLine cmd(__FILE__);
  cmd.AddValue("verbose", "Enable component logs", verbose);
  cmd.AddValue("promisc", "Enable promiscuous PCAP", promiscuousPcap);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.Parse(argc, argv);

  if (verbose)
  {
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
  }

  // Fixed seed so behavior is repeatable (change if you want randomness)
  RngSeedManager::SetSeed(2);

  // Create nodes (all coordinators). Node 0 is PAN-C.
  NodeContainer nodes;
  nodes.Create(NUM_COORD);

  // Arrange nodes in a hex/star around the origin for visibility
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

  // Install LR-WPAN (DSME-enabled) devices
  LrWpanHelper lrWpanHelper(true);
  NetDeviceContainer devs = lrWpanHelper.Install(nodes);

  // Enable PCAP
  lrWpanHelper.EnablePcapAll(std::string("dsme-beacon-slot-selection"), promiscuousPcap);

  // Common MAC parameters
  const uint16_t numChSupported = 6;    // logical channels supported
  const bool capReduction = false;
  const uint8_t panId = 0x0007;
  const uint8_t channelNum = 11;        // PHY channel

  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();
    d->GetMac()->SetNumOfChannelSupported(numChSupported);
    d->GetMac()->SetCAPReduction(capReduction);
    // coordinators by default; PAN-C is started via StartRequest
  }

  // PAN-C setup
  MlmeStartRequestParams panStart;
  panStart.m_panCoor = true;
  panStart.m_PanId = panId;
  panStart.m_bcnOrd = BO;
  panStart.m_sfrmOrd = SO;
  panStart.m_logCh = channelNum;

  // PAN-C beacons at SDIndex=0
  BeaconBitmap panBitmap(0, 1 << (BO - SO));
  panBitmap.SetSDIndex(0);
  panStart.m_bcnBitmap = panBitmap;

  // Hopping descriptor (single channel offset 0)
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

  // Start PAN-C and set its short address to 00:01
  lrWpanHelper.AssociateToBeaconPan(devs, Mac16Address("00:01"), panStart);

  // Joining coordinators: 5 nodes choose distinct SDIndex 1..5
  // 並為每個節點指定靠近的父協調器以避免未同步導致負延遲排程。
  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();

    // 指定父協調器（根據拓撲距離挑較近者）
    // Node 1(00:02)→PAN-C(00:01); Node 2(00:03)→PAN-C(00:01)
    // Node 3(00:04)→Coord1(00:02); Node 4(00:05)→PAN-C(00:01); Node 5(00:06)→Coord2(00:03)
    Mac16Address parent = Mac16Address("00:01");
    if (i == 3) parent = Mac16Address("00:02");
    if (i == 4) parent = Mac16Address("00:02");
    if (i == 5) parent = Mac16Address("00:03");

    // 確認在追蹤前先設置父協調器，使同步時間基準正確
    d->GetMac()->SetAssociatedCoor(parent);

    MlmeSyncRequestParams sync;
    sync.m_logChPage = 0;
    sync.m_trackBcn = true; // 持續追蹤父協調器的 beacon
    d->TrackCoordinatorBeacon(sync);

    // 建立子協調器的 Start 參數
    MlmeStartRequestParams start;
    start.m_panCoor = false;
    start.m_PanId = panId;
    start.m_bcnOrd = BO;
    start.m_sfrmOrd = SO;

    // 指派不衝突的 beacon SDIndex（1..5）
    BeaconBitmap cBitmap(0, 1 << (BO - SO));
    uint16_t sdIdx = static_cast<uint16_t>(i); // 1..5
    cBitmap.SetSDIndex(sdIdx);
    start.m_bcnBitmap = cBitmap;

    HoppingDescriptor hop;
    hop.m_HoppingSequenceID = 0;
    hop.m_hoppingSeqLen = 0;
    hop.m_channelOfs = sdIdx; // 示意用
    hop.m_channelOfsBitmapLen = 16;
    hop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(BIT(sdIdx)));
    start.m_hoppingDescriptor = hop;

    start.m_dsmeSuperframeSpec = dsmeSpec;

    // 以掃描結果構造對應父節點的 PAN 描述符
    PanDescriptor pd;
    pd.m_coorPanId = panId;
    pd.m_coorShortAddr = parent;
    pd.m_logCh = channelNum;
    SuperframeField sf;
    sf.SetSuperframeOrder(SO);
    sf.SetBeaconOrder(BO);
    pd.m_superframeSpec = sf;
    pd.m_dsmeSuperframeSpec = dsmeSpec;
    // 參考 PAN 的 beacon 位置（即可由父節點傳遞而知）
    pd.m_bcnBitmap = panBitmap;

    // 以指定父協調器進行引導，短址設為 00:(i+1)
    // 將引導動作延後到 2.2s 之後（跨過多個 PAN-C beacon 週期），避免觸發負延遲
    Simulator::Schedule(Seconds(2.20 + 0.20 * i), &LrWpanHelper::CoordBoostrap,
                        &lrWpanHelper, d, pd, i /* short addr low byte */, start);

    // 記錄選槽
    Simulator::ScheduleNow(&LogPickedSlot, i, sdIdx);
  }

  // NetAnim visualization
  AnimationInterface anim("dsme-beacon-slot-selection.xml");
  anim.SetMobilityPollInterval(Seconds(0.1));
  anim.EnablePacketMetadata(true);

  // Clearer labels and larger node sizes
  anim.UpdateNodeDescription(nodes.Get(0), "PAN-C [00:01] SD=0");
  anim.UpdateNodeColor(nodes.Get(0), 255, 0, 0);
  anim.UpdateNodeSize(nodes.Get(0)->GetId(), 14.0, 14.0);

  for (uint32_t i = 1; i < NUM_COORD; ++i)
  {
    uint16_t sdIdx = static_cast<uint16_t>(i);
    Mac16Address parent = Mac16Address("00:01");
    if (i == 3) parent = Mac16Address("00:02");
    if (i == 4) parent = Mac16Address("00:02");
    if (i == 5) parent = Mac16Address("00:03");

    std::ostringstream lab;
    lab << "Coord" << i
        << " [00:" << std::setfill('0') << std::setw(2) << (i + 1)
        << "] SD=" << sdIdx
        << " P=";
    // print parent short address
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

  // Run
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
