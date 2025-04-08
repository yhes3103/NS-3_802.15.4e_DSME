/*
 * Copyright (c) 2020 Ritsumeikan University, Shiga, Japan
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

 #include "ns3/core-module.h"
 #include "ns3/internet-apps-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/lr-wpan-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/propagation-module.h"
 #include "ns3/sixlowpan-module.h"
 #include "ns3/spectrum-module.h"
 #include "ns3/ipv6-header.h"
 #include <fstream>
 
 using namespace ns3;
 
 #define BO 6
 #define SO 3
 #define MO 5
 
//  #define TREE_DEGREE 3 // The maximum degree of a node in the tree.
//  #define NUM_COORD 4 // The number of coord, PAN-C need to be included.
//  #define NUM_RFD (NUM_COORD - 1) * TREE_DEGREE // The number of RFD.

#define NUM_COORD 2 // The number of coord, PAN-C need to be included.
#define NUM_RFD 2 // The number of RFD.
 
 #define BIT(X) (1 << 2^X)
 
 static double pktSent = 0;
 // static double throughput = 0;
 
 typedef enum
 {
     CHANNEL_ADAPTATION = 0,
     CHANNEL_HOPPING = 1
 } LrWpanDsmeChannelDiversity;
 
 void SendIPv6Packet(Ptr<NetDevice> dev, Mac48Address dstMac)
{
    // Payload: 10 bytes
    Ptr<Packet> payload = Create<Packet>(10);

    // Pseudo IPv6 header
    Ipv6Header ipv6Header;
    ipv6Header.SetSource(Ipv6Address("fe80::"));
    ipv6Header.SetDestination(Ipv6Address("fe80::"));
    ipv6Header.SetPayloadLength(payload->GetSize());
    ipv6Header.SetNextHeader(17);
    ipv6Header.SetHopLimit(1);
    ipv6Header.SetTrafficClass(0);
    ipv6Header.SetFlowLabel(0);

    payload->AddHeader(ipv6Header);

    NS_LOG_UNCOND("Sending packet (include IPv6 Header) of size: " << payload->GetSize() << " bytes");

    dev->Send(payload, dstMac, 0x86DD);
}
 
 static void
 dataSentMacConfirm(McpsDataConfirmParams params) // McpsDataConfirmCallBack
 {
     // In the case of transmissions with the Ack flag activated, the transaction is only
     // successful if the Ack was received.
     if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
     {
         NS_LOG_UNCOND("**********" << Simulator::Now().As(Time::S)
                                    << " | Transmission successfully sent");
         pktSent += 1;
     }
 }
 
 
 int main(int argc, char** argv) {
     bool verbose = true;
 
     CommandLine cmd(__FILE__);
     cmd.AddValue("verbose", "turn on log components", verbose);
     cmd.Parse(argc, argv);
 
     if (verbose) {
         LogComponentEnableAll(LOG_PREFIX_TIME);
         LogComponentEnableAll(LOG_PREFIX_FUNC);
         LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
         //LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
         // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
         // LogComponentEnable("LrWpanHelper", LOG_LEVEL_ALL);
         // LogComponentEnable("LrWpanNetDevice", LOG_LEVEL_ALL);
         //LogComponentEnable("Ping6Application", LOG_LEVEL_INFO);
         LogComponentEnable("SixLowPanNetDevice", LOG_LEVEL_INFO);
     }
     
     NodeContainer nodes;
     nodes.Create(NUM_COORD + NUM_RFD);
 
     MobilityHelper mobility;
     mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
 
     mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                   "X",
                                   StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"),
                                   "Y",
                                   StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
 
     mobility.Install(nodes);
 
     LrWpanHelper lrWpanHelper(true);
     // Add and install the LrWpanNetDevice for each node
     NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);
 
     uint16_t panChannelOfs = 0;
     
     std::vector<uint16_t> channelOffsets;
     // Setting channel offset array
     for(int i = 0; i < 16; i++)
     {
         channelOffsets.push_back(i);
     }
 
     uint16_t numOfChannelsSupported = 6;
 
     // // In this example, Hopping Sequence is {1, 2, 3, 4, 5, 6}
     // std::vector<uint16_t> hoppingSequence;
     // for(int i = 0; i < numOfChannelsSupported; i++)
     // {
     //     hoppingSequence[i] = i + 1;
     // }
 
     // callback hook
     McpsDataConfirmCallback cb1;
     cb1 = MakeCallback(&dataSentMacConfirm);

 
     // Dsme Network Parameters
     uint16_t panId = 5;
     uint16_t bcnOrder = BO;
     uint16_t multisuperfrmOrder = MO;
     uint16_t superfrmOrder = SO;
     uint8_t channelNum = 11;
     bool capReduction = false;
 
     for (unsigned int i = 0; i < lrwpanDevices.GetN(); ++i) 
     {
         Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
         dev->GetMac()->SetMcpsDataConfirmCallback(cb1);
 
         dev->GetMac()->SetNumOfChannelSupported(numOfChannelsSupported);
         
         // Cap Reduction setting
         dev->GetMac()->SetCAPReduction(capReduction);
     }
 
     /**
      * Pan Coord Settings
      */ 
     MlmeStartRequestParams startParams;
     startParams.m_panCoor = true;
     startParams.m_PanId = panId;
     startParams.m_bcnOrd = bcnOrder;
     startParams.m_sfrmOrd = superfrmOrder;
     startParams.m_logCh = channelNum;
 
     BeaconBitmap bitmap(0, 1 << (bcnOrder - superfrmOrder));
     bitmap.SetSDIndex(0);                  // PAN-C beacon use SDIDx = 0 (beacon TX at SDIdx 0)
     startParams.m_bcnBitmap = bitmap;
 
     HoppingDescriptor hoppingDescriptor;
     hoppingDescriptor.m_HoppingSequenceID = 0x00;
     hoppingDescriptor.m_hoppingSeqLen = 0;
     hoppingDescriptor.m_channelOfs = panChannelOfs;
     hoppingDescriptor.m_channelOfsBitmapLen = 16;
     hoppingDescriptor.m_channelOfsBitmap.resize(1, BIT(panChannelOfs));   
 
     startParams.m_hoppingDescriptor = hoppingDescriptor;
 
     DsmeSuperFrameField dsmeSuperframeField;
     dsmeSuperframeField.SetMultiSuperframeOrder(multisuperfrmOrder);
     dsmeSuperframeField.SetChannelDiversityMode(CHANNEL_HOPPING);
     dsmeSuperframeField.SetCAPReductionFlag(capReduction);
 
     startParams.m_dsmeSuperframeSpec = dsmeSuperframeField;
 
     lrWpanHelper.AssociateToBeaconPan(lrwpanDevices
                                         , Mac16Address("00:01")
                                         , startParams);
 
 
     // 2nd level Coordinator setting, let other coordinator associate with pan-C
     for (unsigned int i = 1; i < NUM_COORD; ++i) {
         MlmeSyncRequestParams syncParams;
         syncParams.m_logChPage = 0; 
         syncParams.m_trackBcn = true; 
         lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->TrackCoordinatorBeacon(syncParams);
 
         MlmeStartRequestParams params;
         params.m_panCoor = false;
         params.m_PanId = panId;
         params.m_bcnOrd = bcnOrder;
         params.m_sfrmOrd = superfrmOrder;
 
         BeaconBitmap bitmap(0, 1 << (bcnOrder - superfrmOrder));
         bitmap.SetSDIndex(i);                 
         params.m_bcnBitmap = bitmap;
 
         HoppingDescriptor hoppingDescriptor;
         hoppingDescriptor.m_HoppingSequenceID = 0x00;
         hoppingDescriptor.m_hoppingSeqLen = 0;
         hoppingDescriptor.m_channelOfs = channelOffsets[i];
         hoppingDescriptor.m_channelOfsBitmapLen = 16;
         hoppingDescriptor.m_channelOfsBitmap.resize(1, 1 + (2 << i));   
 
         params.m_hoppingDescriptor = hoppingDescriptor;
 
         // Pan Descriptor
         PanDescriptor panDescriptor;
         panDescriptor.m_coorPanId = panId;
         panDescriptor.m_coorShortAddr = Mac16Address("00:01");
         panDescriptor.m_logCh = channelNum;
 
         SuperframeField superframeField;
         superframeField.SetSuperframeOrder(superfrmOrder);
         superframeField.SetBeaconOrder(bcnOrder);
         panDescriptor.m_superframeSpec = superframeField;
 
         panDescriptor.m_dsmeSuperframeSpec = dsmeSuperframeField;
         panDescriptor.m_bcnBitmap = bitmap;
 
         lrWpanHelper.CoordBoostrap(lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()
                                     , panDescriptor
                                     , i
                                     , params);
     }
     
     SixLowPanHelper sixlowpan;
     NetDeviceContainer sixlowDevices = sixlowpan.Install(lrwpanDevices);

     for(unsigned i = 0; i < sixlowDevices.GetN(); i++)
     {
         Ptr<SixLowPanNetDevice> dev = sixlowDevices.Get(i)->GetObject<SixLowPanNetDevice>();
         dev->SetHC1CompMethod(true);
         dev->SetMeshUnder(true);
         dev->SetDataFlooding(true);
     }
 
     // MAC addresses
     Ptr<NetDevice> dev0 = sixlowDevices.Get(0); // sender
     Ptr<NetDevice> dev1 = sixlowDevices.Get(1); // receiver
 
     Mac48Address dstMac = Mac48Address::ConvertFrom(dev1->GetAddress());

    // 手動觸發封包傳送
     Simulator::Schedule(Seconds(1.0), &SendIPv6Packet, dev0, dstMac);

     Simulator::Stop(Seconds(1.474560001));
     Simulator::Run();
 
     Simulator::Destroy();
 }
 