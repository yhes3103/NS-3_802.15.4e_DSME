/*
 * DSME Enhanced Beacon example with PCAP and NetAnim output
 */

#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/spectrum-module.h>
#include <ns3/netanim-module.h>
#include <ns3/lr-wpan-module.h>

using namespace ns3;

static void
TxTrace(Ptr<const Packet> p)
{
  std::cout << Simulator::Now().GetSeconds() << "s TX: size=" << p->GetSize() << "\n";
}

int main(int argc, char** argv)
{
  uint16_t panId = 0x1234;
  uint8_t channel = 11;
  uint8_t bo = 6; // Beacon Order
  uint8_t so = 4; // Superframe Order
  uint8_t mo = 2; // Multisuperframe Order (DSME)
  bool enableAnim = false; // enable NetAnim output
  bool enablePcap = true;  // enable PCAP output
  bool noDestroy = false;  // skip Simulator::Destroy() as a workaround if segfault happens
  bool capReduction = false; // DSME CAP reduction flag
  bool channelDiversity = false; // DSME channel diversity mode (0=adaptation, 1=hopping)

  CommandLine cmd;
  cmd.AddValue("panId", "PAN ID", panId);
  cmd.AddValue("channel", "802.15.4 channel (11..26)", channel);
  cmd.AddValue("bo", "Beacon Order", bo);
  cmd.AddValue("so", "Superframe Order (<= BO)", so);
  cmd.AddValue("mo", "Multisuperframe Order (>= SO)", mo);
  cmd.AddValue("enableAnim", "Enable NetAnim output (XML)", enableAnim);
  cmd.AddValue("enablePcap", "Enable PCAP capture", enablePcap);
  cmd.AddValue("noDestroy", "Skip Simulator::Destroy() (workaround)", noDestroy);
  cmd.AddValue("capReduction", "Enable DSME CAP reduction flag", capReduction);
  cmd.AddValue("channelDiversity", "Enable DSME channel diversity mode", channelDiversity);
  cmd.Parse(argc, argv);

  NodeContainer nodes;
  nodes.Create(2);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Simple channel
  Ptr<SingleModelSpectrumChannel> ch = CreateObject<SingleModelSpectrumChannel>();
  ch->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
  ch->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

  // Install LR-WPAN on two nodes
  LrWpanHelper lrWpan;
  NetDeviceContainer devs = lrWpan.Install(nodes);
  Ptr<LrWpanNetDevice> dev0 = DynamicCast<LrWpanNetDevice>(devs.Get(0));
  Ptr<LrWpanNetDevice> dev1 = DynamicCast<LrWpanNetDevice>(devs.Get(1));
  dev0->SetChannel(ch);
  dev1->SetChannel(ch);

  Ptr<LrWpanMac> mac0 = dev0->GetMac();
  mac0->SetShortAddress(Mac16Address("00:01"));
  mac0->SetPanId(panId);
  Ptr<LrWpanMac> mac1 = dev1->GetMac();
  mac1->SetShortAddress(Mac16Address("00:02"));
  // Accept all beacons during sync
  mac1->SetPanId(0xffff);

  // Enable DSME mode on coordinator
  mac0->SetDsmeModeEnabled();

  // Start as PAN coordinator with beaconing
  MlmeStartRequestParams start;
  start.m_PanId = panId;
  start.m_logCh = channel;
  start.m_logChPage = 0;
  start.m_startTime = 0; // now
  start.m_bcnOrd = bo;
  start.m_sfrmOrd = so;
  start.m_panCoor = true;
  start.m_battLifeExt = false;
  start.m_coorRealgn = false;
  // Configure DSME Superframe parameters in the START.request
  start.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(mo);
  if (capReduction)
  {
    start.m_dsmeSuperframeSpec.SetCAPReductionFlag(true);
  }
  if (channelDiversity)
  {
    start.m_dsmeSuperframeSpec.SetChannelDiversityMode(true);
  }
  // Safe defaults for Channel Hopping descriptor to avoid uninitialized length
  start.m_hoppingDescriptor.m_HoppingSequenceID = 0;    // static sequence
  start.m_hoppingDescriptor.m_channelOfs = 0;           // no offset
  start.m_hoppingDescriptor.m_channelOfsBitmapLen = 16; // 16 channels bitmap length (fits into 1 word)
  start.m_hoppingDescriptor.m_channelOfsBitmap = std::vector<uint16_t>(1, 0x0000);
  mac0->MlmeStartRequest(start);

  // Device sync to beacons and track
  MlmeSyncRequestParams sync;
  sync.m_logCh = channel;
  sync.m_logChPage = 0;
  sync.m_trackBcn = true;
  mac1->MlmeSyncRequest(sync);

  // PCAP output for Wireshark
  if (enablePcap)
  {
    lrWpan.EnablePcapAll("lr-wpan-dsme-eb", true);
  }

  // Trace (no context)
  mac0->TraceConnectWithoutContext("MacTx", MakeCallback(&TxTrace));
  mac1->TraceConnectWithoutContext("MacRx", MakeCallback(&TxTrace));

  Simulator::Stop(Seconds(5.0));
  if (enableAnim)
  {
    AnimationInterface anim("lr-wpan-dsme-eb.xml");
    AnimationInterface::SetConstantPosition(nodes.Get(0), 0.0, 0.0);
    AnimationInterface::SetConstantPosition(nodes.Get(1), 50.0, 0.0);
    anim.UpdateNodeDescription(nodes.Get(0), "PAN-C");
    anim.UpdateNodeDescription(nodes.Get(1), "DEV");
    anim.UpdateNodeColor(0, 0, 128, 255);
    anim.UpdateNodeColor(1, 128, 0, 0);
    anim.EnablePacketMetadata();
    Simulator::Run();
  }
  else
  {
    Simulator::Run();
  }
  if (!noDestroy)
  {
    Simulator::Destroy();
  }
  return 0;
}
