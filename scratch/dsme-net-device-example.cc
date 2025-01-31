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
    nodes.Create(8);

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

    LrWpanHelper lrWpanHelper(true);
    // Add and install the LrWpanNetDevice for each node
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);

    Ptr<LrWpanNetDevice> dev1 = lrwpanDevices.Get(0)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev2 = lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev3 = lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev4 = lrwpanDevices.Get(3)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev5 = lrwpanDevices.Get(4)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev6 = lrwpanDevices.Get(5)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev7 = lrwpanDevices.Get(6)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev8 = lrwpanDevices.Get(7)->GetObject<LrWpanNetDevice>();

    McpsDataConfirmCallback cb1;
    cb1 = MakeCallback(&dataSentMacConfirm);
    dev1->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev2->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev3->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev4->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev5->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev6->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev7->GetMac()->SetMcpsDataConfirmCallback(cb1);
    dev8->GetMac()->SetMcpsDataConfirmCallback(cb1);

    // Fake PAN association, coordinator assignment, short address assignment and initialization
    // of beacon-enabled mode in 802.15.4-2011.
    // This is needed because the lr-wpan module does not provide (yet)
    // a full PAN association procedure.

    // AssociateToBeaconPan (devices, PAN ID, Coordinator Address, Beacon Order, Superframe Order)

    // Must be careful not setting the beacon order (BO) and the superframe order (SO) too far apart
    // or the ping reply (ICMPV6 echo reply) can time out during the inactive period of the
    // superframe. A full time table of the BO/SO time equivalence can be found at the end of this
    // document. The current configuration is BO = 14, SO = 13 :

    //           Contention Access Period (CAP)                           Inactive
    //              (125.82912 secs)                                     (125.82088)
    //   |---------------------------------------------|-------------------------------------------|
    // Beacon Beacon
    //                            Beacon Interval = 251.65 secs
    //   |-----------------------------------------------------------------------------------------|

    uint16_t panId = 7;
    uint16_t bcnOrder = 14;
    uint16_t multisuperfrmOrder = 13;
    uint16_t superfrmOrder = 12;

    bool capReduction = false;

    uint16_t panChannelOfs = 1;

    // Mlme-Start.Request params
    MlmeStartRequestParams startParams;
    startParams.m_panCoor = true;
    startParams.m_PanId = panId;
    startParams.m_bcnOrd = bcnOrder;
    startParams.m_sfrmOrd = superfrmOrder;

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

    // Channel Offset
    for (unsigned int i = 1 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetChannelOffset(panChannelOfs);
    }
    
    // GTSs setting
    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetMcpsDataReqGts(true);
    }

    for (unsigned int i = 1 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->ResizeScheduleGTSsEvent(bcnOrder, multisuperfrmOrder, superfrmOrder);
    }

    for (unsigned int i = 1 ; i < lrwpanDevices.GetN(); ++i) {
        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(0)->GetObject<LrWpanNetDevice>(), true, 1, 0, i - 1);
        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>(), false, 1, 0, i - 1);
    }

    int pktSize = 60;

    // for (unsigned int i = 1 ; i < lrwpanDevices.GetN(); ++i) {
    //     lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(i), lrwpanDevices.Get(0)->GetAddress(), pktSize, 35.395320068, 10.0, 100);
    // }

    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(1), lrwpanDevices.Get(0)->GetAddress(), pktSize, 35.395320068, 100000.0, 125.8295);
    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(2), lrwpanDevices.Get(0)->GetAddress(), pktSize, 39.327480134, 100000.0, 125.8295);
    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(3), lrwpanDevices.Get(0)->GetAddress(), pktSize, 43.259640068, 100000.0, 125.8295);
    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(4), lrwpanDevices.Get(0)->GetAddress(), pktSize, 47.191800095, 100000.0, 125.8295);
    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(5), lrwpanDevices.Get(0)->GetAddress(), pktSize, 51.123960150, 100000.0, 125.8295);
    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(6), lrwpanDevices.Get(0)->GetAddress(), pktSize, 55.056120134, 100000.0, 125.8295);
    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(7), lrwpanDevices.Get(0)->GetAddress(), pktSize, 58.988280150, 100000.0, 125.8295);

    AsciiTraceHelper ascii;
    lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Ping-6LoW-lr-wpan-beacon.tr"));
    lrWpanHelper.EnablePcapAll(std::string("Ping-6LoW-lr-wpan-beacon"), true);

    Simulator::Stop(Seconds(200));

    Simulator::Run();
    Simulator::Destroy();
}

// BO/SO values to time equivalence
// These times are only valid for a 250kbps O-QPSK modulation,
// times differ with other modulation configurations.

// +------------------------+
// | BO/SO |  Time (secs)   |
// +------------------------+
// |   0   | 0.01536 secs   |
// |   1   | 0.03072 secs   |
// |   2   | 0.06144 secs   |
// |   3   | 0.12288 secs   |
// |   4   | 0.24576 secs   |
// |   5   | 0.49152 secs   |
// |   6   | 0.98304 secs   |
// |   7   | 1.96608 secs   |
// |   8   | 3.93216 secs   |
// |   9   | 7.86432 secs   |
// |   10  | 15.72864 secs  |
// |   11  | 31.45728 secs  |
// |   12  | 62.91456 secs  |
// |   13  | 125.82912 secs |
// |   14  | 251.65 secs    |
// +------------------------+
