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

#include <fstream>

using namespace ns3;

static double pktRecv = 0;
static double pktSent = 0;

static void
dataSentMacConfirm(McpsDataConfirmParams params)
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

static void dataIndication(McpsDataIndicationParams params, Ptr<Packet> p) {
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | Received DATA packet of size " << p->GetSize());
    pktRecv += 1;
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
        // LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
        // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
        LogComponentEnable("LrWpanHelper", LOG_LEVEL_ALL);
        LogComponentEnable("Ping6Application", LOG_LEVEL_INFO);
    }

    NodeContainer nodes;
    nodes.Create(17);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                  "X",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"),
                                  "Y",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));

    mobility.Install(nodes);

    LrWpanHelper lrWpanHelper(true);
    // Add and install the LrWpanNetDevice for each node
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);

    // Dsme Network Parameters
    uint16_t panId = 7;
    uint16_t bcnOrder = 14;
    uint16_t multisuperfrmOrder = 13;
    uint16_t superfrmOrder = 10;
    uint8_t channelNum = 11;

    bool capReduction = false;

    uint16_t panChannelOfs = 0;

    std::vector<uint16_t> channelOffsets;
    channelOffsets.push_back(panChannelOfs);

    uint16_t numOfChannelsSupported = 1;

    // callback hook
    McpsDataConfirmCallback cb1;
    cb1 = MakeCallback(&dataSentMacConfirm);

    McpsDataIndicationCallback cb2;
    cb2 = MakeCallback(&dataIndication);

    for (unsigned int i = 0; i < lrwpanDevices.GetN(); ++i) {
        Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
        dev->GetMac()->SetMcpsDataConfirmCallback(cb1);
        dev->GetMac()->SetMcpsDataIndicationCallback(cb2);

        dev->GetMac()->SetNumOfChannelSupported(numOfChannelsSupported);

        dev->GetMac()->SetDsmeMacIsForIntegratingWithHigerLayer(true);
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
    dsmeSuperframeField.SetChannelDiversityMode(1);
    dsmeSuperframeField.SetCAPReductionFlag(capReduction);

    startParams.m_dsmeSuperframeSpec = dsmeSuperframeField;

    lrWpanHelper.AssociateToBeaconPan(lrwpanDevices
                                        , Mac16Address("00:01")
                                        , startParams);

    unsigned int numOfCoord = 3;

    // 2nd level Coordinator setting
    for (unsigned int i = 1; i < numOfCoord; ++i) {
        // lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetAsCoordinator();
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->SetBecomeCoordAfterAssociation(true);

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
        hoppingDescriptor.m_channelOfs = channelOffsets[0];
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
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetMcpsDataReqGts(true);
    }

    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->ResizeScheduleGTSsEvent(bcnOrder, 
                                                                                              multisuperfrmOrder, 
                                                                                              superfrmOrder);
    }

    // superframe ID = 0
    for (int i = 0; i < 6; ++i) {
        int childIdx = 1 + i + 2;

        // Channel Offset setting
        lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[0]);

        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[0]
                                , 0, i);

        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[0]
                                , 0, i);
    }


    for (int i = 0; i < 6; ++i) {
        int childIdx = 1 + i + 2;
        lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), 60,  251.65824, 100000.0, 7.86432);
    }


    // superframe ID = 1
    for (int i = 0; i < 6; ++i) {
        int childIdx = 2 + i + 8;

        // Channel Offset setting
        lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[0]);

        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[0]
                                , 1, i);

        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[0]
                                , 1, i);
    }


    for (int i = 0; i < 6; ++i) {
        int childIdx = 2 + i + 8;
        lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(2)->GetAddress(), 60,  251.65824, 100000.0, 7.86432);
    }

    AsciiTraceHelper ascii;
    lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Ping-6LoW-lr-wpan-beacon.tr"));
    lrWpanHelper.EnablePcapAll(std::string("Ping-6LoW-lr-wpan-beacon"), true);

    Simulator::Stop(Seconds(510));

    Simulator::Run();

    std::cout << "pktSent: " << pktSent << std::endl;
    std::cout << "pktRecv: " << pktRecv << std::endl;
    std::cout << "Delivery ratio: " << pktRecv / pktSent << std::endl;

    Simulator::Destroy();
}
