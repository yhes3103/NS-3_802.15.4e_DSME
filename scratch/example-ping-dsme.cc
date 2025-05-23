/*
 * Copyright (c) 2013 Universita' di Firenze, Italy
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
 * Author: Tommaso Pecorella <tommaso.pecorella@unifi.it>
 */

#include "ns3/core-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/spectrum-module.h"

#include <fstream>

using namespace ns3;

static void
dataSentMacConfirm(McpsDataConfirmParams params)
{
    // In the case of transmissions with the Ack flag activated, the transaction is only
    // successful if the Ack was received.
    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
    {
        NS_LOG_UNCOND("**********" << Simulator::Now().As(Time::S)
                                   << " | Transmission successfully sent");
    }
}

int
main(int argc, char** argv)
{
    bool verbose = true;
    bool disablePcap = false;
    bool disableAsciiTrace = false;
    bool enableLSixlowLogLevelInfo = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "turn on log components", verbose);
    cmd.AddValue("disable-pcap", "disable PCAP generation", disablePcap);
    cmd.AddValue("disable-asciitrace", "disable ascii trace generation", disableAsciiTrace);
    cmd.AddValue("enable-sixlowpan-loginfo",
                 "enable sixlowpan LOG_LEVEL_INFO (used for tests)",
                 enableLSixlowLogLevelInfo);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnableAll(LOG_PREFIX_TIME);
        LogComponentEnableAll(LOG_PREFIX_FUNC);
        // LogComponentEnable("Ping6Application", LOG_LEVEL_INFO);
        LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
        // LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
        // LogComponentEnable("LrWpanNetDevice", LOG_LEVEL_INFO);
        LogComponentEnable("SixLowPanNetDevice", LOG_LEVEL_INFO);
        // LogComponentEnable("Ipv6L3Protocol", LOG_LEVEL_INFO);
    }

    NodeContainer nodes;
    nodes.Create(2);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(0.0),
                                  "MinY",
                                  DoubleValue(0.0),
                                  "DeltaX",
                                  DoubleValue(20),
                                  "DeltaY",
                                  DoubleValue(20),
                                  "GridWidth",
                                  UintegerValue(3),
                                  "LayoutType",
                                  StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    LrWpanHelper lrWpanHelper;
    // Add and install the LrWpanNetDevice for each node
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);

    // Dsme Network Parameters
    uint16_t panId = 7;
    uint16_t bcnOrder = 6;
    uint16_t multisuperfrmOrder = 5;
    uint16_t superfrmOrder = 3;
    uint8_t channelNum = 11;

    bool capReduction = false;

    uint16_t panChannelOfs = 0;
    
    std::vector<uint16_t> channelOffsets;
    channelOffsets.push_back(panChannelOfs);
    channelOffsets.push_back(0);
    channelOffsets.push_back(1);

    uint16_t numOfChannelsSupported = 2;

    // callback hook
    McpsDataConfirmCallback cb1;
    cb1 = MakeCallback(&dataSentMacConfirm);

    for(unsigned int i = 0; i < lrwpanDevices.GetN(); ++i)
    {
        Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
        dev->GetMac()->SetMcpsDataConfirmCallback(cb1);

        dev->GetMac()->SetNumOfChannelSupported(numOfChannelsSupported);
    }

    // Pan Coord mlme-start.request params
    MlmeStartRequestParams startParams;
    startParams.m_panCoor = true;
    startParams.m_PanId = panId;
    startParams.m_bcnOrd = bcnOrder;
    startParams.m_sfrmOrd = superfrmOrder;
    startParams.m_logCh = channelNum;

    BeaconBitmap bitmap(0, 1 << (bcnOrder - superfrmOrder));
    bitmap.SetSDIndex(0);                  // SD = 0 目前占用
    startParams.m_bcnBitmap = bitmap;

    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = panChannelOfs;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, 1);   

    startParams.m_hoppingDescriptor = hoppingDescriptor;

    DsmeSuperFrameField dsmeSuperframeField;
    dsmeSuperframeField.SetMultiSuperframeOrder(multisuperfrmOrder);
    dsmeSuperframeField.SetChannelDiversityMode(1);  // Channel Hopping
    dsmeSuperframeField.SetCAPReductionFlag(capReduction);

    startParams.m_dsmeSuperframeSpec = dsmeSuperframeField;

    lrWpanHelper.AssociateToBeaconPan(lrwpanDevices, Mac16Address("00:01"), startParams);

    unsigned int numOfCoord = 2;

    // 2nd level Coordinator setting
    for(unsigned int i = 1; i < numOfCoord; ++i)
    {
        // lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetAsCoordinator();

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
        bitmap.SetSDIndex(i);                  // SD = 8 目前占用
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

    // GTSs setting
    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetMcpsDataReqGts(false);
    }

    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->ResizeScheduleGTSsEvent(bcnOrder, 
                                                                                              multisuperfrmOrder, 
                                                                                              superfrmOrder);
    }

    // Fake PAN association and short address assignment.
    // This is needed because the lr-wpan module does not provide (yet)
    // a full PAN association procedure.
    lrWpanHelper.AssociateToPan(lrwpanDevices, 1);

    InternetStackHelper internetv6;
    internetv6.Install(nodes);

    SixLowPanHelper sixlowpan;
    NetDeviceContainer devices = sixlowpan.Install(lrwpanDevices);

    Ipv6AddressHelper ipv6;
    ipv6.SetBase(Ipv6Address("2001:2::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer deviceInterfaces;
    deviceInterfaces = ipv6.Assign(devices);

    if (enableLSixlowLogLevelInfo)
    {
        std::cout << "Device 0: pseudo-Mac-48 "
                  << Mac48Address::ConvertFrom(devices.Get(0)->GetAddress()) << ", IPv6 Address "
                  << deviceInterfaces.GetAddress(0, 1) << std::endl;
        std::cout << "Device 1: pseudo-Mac-48 "
                  << Mac48Address::ConvertFrom(devices.Get(1)->GetAddress()) << ", IPv6 Address "
                  << deviceInterfaces.GetAddress(1, 1) << std::endl;
    }

    uint32_t packetSize = 10;
    uint32_t maxPacketCount = 5;
    Time interPacketInterval = Seconds(1.0);
    Ping6Helper ping6;

    ping6.SetLocal(deviceInterfaces.GetAddress(0, 1));
    ping6.SetRemote(deviceInterfaces.GetAddress(1, 1));

    ping6.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
    ping6.SetAttribute("Interval", TimeValue(interPacketInterval));
    ping6.SetAttribute("PacketSize", UintegerValue(packetSize));
    ApplicationContainer apps = ping6.Install(nodes.Get(0));

    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(1.47));

    if (!disableAsciiTrace)
    {
        // AsciiTraceHelper ascii;
        // lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Ping-6LoW-lr-wpan.tr"));
    }
    if (!disablePcap)
    {
        //lrWpanHelper.EnablePcapAll(std::string("Ping-6LoW-lr-wpan"), true);
    }
    if (enableLSixlowLogLevelInfo)
    {
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
        Ipv6RoutingHelper::PrintNeighborCacheAllAt(Seconds(9), routingStream);
    }

    // 這個程式我記得是吃有線的 CSMA，然後我有改掉 bug，不然原本跑起來也很奇怪

    Simulator::Stop(Seconds(1.474560001));
    Simulator::Run();
    Simulator::Destroy();
}
