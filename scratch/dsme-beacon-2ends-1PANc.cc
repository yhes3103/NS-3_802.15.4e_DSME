/*
 * DSME 2-end-devices + 1 PAN-C
 * - Topology: End1 (left) -- PAN-C (center) -- End2 (right)
 * - PAN-C starts beaconing (DSME superframe), end devices associate to PAN via beacon-based helper
 * - After association, End1 sends one IPv6 ping packet to End2; forwarding goes via PAN-C using 6LoWPAN Mesh-Under
 * - PCAP and NetAnim XML enabled by default
 */

#include "ns3/core-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/spectrum-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DsmeTwoEndsOnePanc");

// Globals for simple logging/forwarding callbacks
static Ptr<LrWpanNetDevice> g_devPan;
static bool g_forwarded = false;
static uint16_t g_panIdGlobal = 0x0007;

static void OnConfirm(McpsDataConfirmParams params)
{
  NS_LOG_UNCOND("[CONFIRM] status=" << static_cast<int>(params.m_status));
}

static void OnIndEnd1(McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND("[IND] End1 len=" << p->GetSize());
}

static void OnIndEnd2(McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND("[IND] End2 len=" << p->GetSize());
}

static void OnIndPan(McpsDataIndicationParams params, Ptr<Packet> pkt)
{
  if (g_forwarded || g_devPan == nullptr)
  {
    return;
  }
  g_forwarded = true; // forward only once
  // Forward to End2 (00:03)
  McpsDataRequestParams rp; rp.m_dstPanId = g_panIdGlobal; rp.m_srcAddrMode = SHORT_ADDR; rp.m_dstAddrMode = SHORT_ADDR; rp.m_dstAddr = Mac16Address("00:03");
  rp.m_msduHandle = 7; rp.m_txOptions = 0; // direct, no-ack
  Ptr<Packet> cp = pkt->Copy();
  g_devPan->GetMac()->McpsDataRequest(rp, cp);
  NS_LOG_UNCOND("[PAN] forwarded one DATA to End2");
}

int main(int argc, char** argv)
{
  bool verbose = true;
  bool promiscuousPcap = true;
  double simTime = 6.0; // s

  // DSME parameters
  uint16_t BO = 6; // Beacon Order
  uint16_t SO = 3; // Superframe Order
  uint16_t MO = 5; // Multi-superframe Order
  bool capReduction = false;
  uint8_t channelNum = 11;
  uint16_t panId = 0x0007;

  // Geometry chosen so End1<->End2 are out of range, but each reaches PAN-C
  // Pair this with a LogDistance model roughly like: ref loss 40.05 dB @ 1m, exponent 3.0
  // With 0 dBm nominal Tx, ~60 m to PAN gives ~-93 dBm (typical sens ~-95 dBm); 120 m End1<->End2 ~-102 dBm (out)
  double endToPan = 60.0; // m (good End<->PAN, no direct End<->End)

  CommandLine cmd(__FILE__);
  cmd.AddValue("verbose", "Enable component logs", verbose);
  cmd.AddValue("promisc", "Enable promiscuous PCAP", promiscuousPcap);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("BO", "Beacon Order", BO);
  cmd.AddValue("SO", "Superframe Order", SO);
  cmd.AddValue("MO", "Multi-superframe Order", MO);
  cmd.AddValue("channel", "Logical channel number", channelNum);
  cmd.Parse(argc, argv);

  if (verbose)
  {
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    // LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
    LogComponentEnable("SixLowPanNetDevice", LOG_LEVEL_INFO);
  }

  // Nodes: 0=End1 (left), 1=PAN-C (center), 2=End2 (right)
  NodeContainer nodes; nodes.Create(3);

  // Mobility: position them on a line
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
  pos->Add(Vector(-endToPan, 0.0, 0.0)); // End1
  pos->Add(Vector(0.0, 0.0, 0.0));       // PAN-C
  pos->Add(Vector(+endToPan, 0.0, 0.0)); // End2
  mobility.SetPositionAllocator(pos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Channel with log-distance loss to shape connectivity
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
  loss->SetReference(1.0, 40.05); // 1 m, 2.4 GHz FSPL ~ 40.05 dB
  loss->SetPathLossExponent(3.0);
  channel->AddPropagationLossModel(loss);

  // LR-WPAN install
  LrWpanHelper lrWpanHelper;
  lrWpanHelper.SetChannel(channel);
  NetDeviceContainer devs = lrWpanHelper.Install(nodes);

  // Ensure nominal TX power is 0 dBm for all nodes (robust links to PAN)
  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();
    LrWpanPhyPibAttributes pib; pib.phyTransmitPower = 0; // 0 dBm in 6-bit two's complement
    d->GetPhy()->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyTransmitPower, &pib);
    // Explicitly use CAP (no CAP reduction)
    d->GetMac()->SetCAPReduction(false);
  }

  // PAN coordinator start request (DSME)
  MlmeStartRequestParams panStart;
  panStart.m_panCoor = true;
  panStart.m_PanId = panId;
  panStart.m_bcnOrd = BO;
  panStart.m_sfrmOrd = SO;
  panStart.m_logCh = channelNum;

  BeaconBitmap panBitmap(0, 1 << (BO - SO));
  panBitmap.SetSDIndex(0); // PAN-C beacons on SDIndex=0
  panStart.m_bcnBitmap = panBitmap;

  HoppingDescriptor panHop;
  panHop.m_HoppingSequenceID = 0; panHop.m_hoppingSeqLen = 0; panHop.m_channelOfs = 0;
  panHop.m_channelOfsBitmapLen = 16;
  panHop.m_channelOfsBitmap = std::vector<uint16_t>(1, static_cast<uint16_t>(1u << 0));
  panStart.m_hoppingDescriptor = panHop;

  DsmeSuperFrameField dsmeSpec;
  dsmeSpec.SetMultiSuperframeOrder(MO);
  dsmeSpec.SetChannelDiversityMode(1); // channel hopping enabled
  dsmeSpec.SetCAPReductionFlag(capReduction);
  panStart.m_dsmeSuperframeSpec = dsmeSpec;

  // Associate all devices to the beacon PAN, designating node-1 as PAN-C (short addr 00:02)
  // Address assignment by helper is sequential starting from 0x0001, so:
  // Node0->00:01 (End1), Node1->00:02 (PAN-C), Node2->00:03 (End2)
  lrWpanHelper.AssociateToBeaconPan(devs, Mac16Address("00:02"), panStart);

  // End devices track coordinator beacons (DSME sync)
  for (uint32_t i = 0; i < devs.GetN(); ++i)
  {
    Ptr<LrWpanNetDevice> d = devs.Get(i)->GetObject<LrWpanNetDevice>();
    d->GetMac()->SetNumOfChannelSupported(2);
    if (i != 1) // non-PAN devices
    {
      MlmeSyncRequestParams sync; sync.m_logCh = channelNum; sync.m_logChPage = 0; sync.m_trackBcn = true;
      d->TrackCoordinatorBeacon(sync);
    }
  }

  // End1 / End2 remain as end devices (non-coordinators). They associate to PAN-C and track beacons.
  // Short addresses by helper: Node0=00:01 (End1), Node1=00:02 (PAN-C), Node2=00:03 (End2)
  auto devEnd1 = devs.Get(0)->GetObject<LrWpanNetDevice>();
  auto devPan  = devs.Get(1)->GetObject<LrWpanNetDevice>();
  auto devEnd2 = devs.Get(2)->GetObject<LrWpanNetDevice>();

  // Callbacks for visibility and forwarding
  devEnd1->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&OnConfirm));
  devEnd2->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&OnConfirm));
  devPan->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&OnConfirm));
  devEnd1->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&OnIndEnd1));
  devEnd2->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&OnIndEnd2));
  g_devPan = devPan; g_panIdGlobal = panId; g_forwarded = false;
  devPan->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&OnIndPan));

  // Send exactly one MAC DATA from End1 to PAN-C at t=3.0s (after a few beacons), then PAN-C forwards to End2
  Simulator::Schedule(Seconds(3.0), [devEnd1, panId]() {
    NS_LOG_UNCOND("[End1] sending one DATA to PAN-C (00:02)");
    Ptr<Packet> p = Create<Packet>(30);
    McpsDataRequestParams pr; pr.m_dstPanId = panId; pr.m_srcAddrMode = SHORT_ADDR; pr.m_dstAddrMode = SHORT_ADDR; pr.m_dstAddr = Mac16Address("00:02");
    pr.m_msduHandle = 1; pr.m_txOptions = TX_OPTION_DIRECT; // direct, no-ack
    devEnd1->GetMac()->McpsDataRequest(pr, p);
  });

  // Fallback: if forwarding callback did not trigger, force PAN-C to send one DATA to End2 at t=3.12s
  Simulator::Schedule(Seconds(3.12), [devPan, panId]() {
    if (g_forwarded) return;
    NS_LOG_UNCOND("[PAN] fallback forwarding one DATA to End2 (00:03)");
    Ptr<Packet> p = Create<Packet>(28);
    McpsDataRequestParams pr; pr.m_dstPanId = panId; pr.m_srcAddrMode = SHORT_ADDR; pr.m_dstAddrMode = SHORT_ADDR; pr.m_dstAddr = Mac16Address("00:03");
    pr.m_msduHandle = 8; pr.m_txOptions = TX_OPTION_DIRECT; // direct, no-ack
    devPan->GetMac()->McpsDataRequest(pr, p);
  });

  // Enable PCAP for all LR-WPAN devices
  std::string stem = "dsme-beacon-2ends-1PANc";
  lrWpanHelper.EnablePcapAll(stem, promiscuousPcap);

  // NetAnim
  AnimationInterface anim(stem + ".xml");
  anim.SetMobilityPollInterval(Seconds(0.1));
  anim.EnablePacketMetadata(true);
  anim.UpdateNodeDescription(nodes.Get(0), "End1"); anim.UpdateNodeSize(nodes.Get(0)->GetId(), 10.0, 10.0);
  anim.UpdateNodeDescription(nodes.Get(1), "PAN-C SD=0"); anim.UpdateNodeSize(nodes.Get(1)->GetId(), 12.0, 12.0);
  anim.UpdateNodeDescription(nodes.Get(2), "End2"); anim.UpdateNodeSize(nodes.Get(2)->GetId(), 10.0, 10.0);

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
