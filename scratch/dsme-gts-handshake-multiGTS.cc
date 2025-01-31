/*
 * Copyright (c) 2019 Ritsumeikan University, Shiga, Japan.
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
 * Author:  Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

/*
 *   Coordinator              End Device
 *       N0   <----------------  N1
 *      (dev0)                 (dev1)
 *
 * This example demonstrate the usage of the MAC primitives involved in
 * direct transmissions for the beacon enabled mode of IEEE 802.15.4-2011.
 * A single packet is sent from an end device to the coordinator during the CAP
 * of the first incoming superframe.
 *
 * This example do not demonstrate a full protocol stack usage.
 * For full protocol stack usage refer to 6lowpan examples.
 *
 */

#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>

#include <iostream>

using namespace ns3;

static std::vector<uint8_t> dev0DsmeSAB;
static std::vector<uint8_t> dev1DsmeSAB;

static void
BeaconIndication(MlmeBeaconNotifyIndicationParams params, Ptr<Packet> p)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | Received BEACON packet of size " << p->GetSize());
}

static void
DataIndication(McpsDataIndicationParams params, Ptr<Packet> p)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | Received DATA packet of size " << p->GetSize());
}

static void
TransEndIndication(McpsDataConfirmParams params)
{
    // In the case of transmissions with the Ack flag activated, the transaction is only
    // successful if the Ack was received.
    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Transmission successfully sent");
    } else if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_NO_ACK) {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | No Acknowlegment received.");
    }
}

static void
DataIndicationCoordinator(McpsDataIndicationParams params, Ptr<Packet> p)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s Coordinator Received DATA packet (size " << p->GetSize() << " bytes)");
}

static void
StartConfirm(MlmeStartConfirmParams params)
{
    if (params.m_status == MLMESTART_SUCCESS)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " Beacon status SUCESSFUL ");
    }
}

static void MlmeDsmeGtsIndication(Ptr<LrWpanNetDevice> device, MlmeDsmeGtsIndicationParams params) {
    // 檢查參數
    std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
              << " [" << device->GetMac()->GetShortAddress() << "]"
              << " MLME-DSME-GTS.indication: " << " | "
              << " Manage Type : " << (uint16_t) params.m_manageType << " | "
              << " Direction : " << (uint16_t) params.m_direction << " | "
              << " Priority : " << (uint16_t) params.m_prioritizedChAccess << " | "
              << " Num Of Slots : " << (uint16_t) params.m_numSlot << " | "
              << " Preferred Superframe ID : " << params.m_preferredSuperframeID << " | "
              << " Preferred Slot ID : " << (uint16_t) params.m_preferredSlotID << " | \n" 
              << " SAB Spec: " 
              << "\n";
    
    uint32_t maxSuperframeId = device->GetMac()->GetNumOfSuperframesInABeaconInterval() /
                                device->GetMac()->GetNumOfMultisuperframesInABeaconInterval();
    
    // std::cout<< maxSuperframeId << std::endl;

    static bool first = true;

    if (params.m_manageType == GTS_ALLOCATION) {
        params.m_dsmeSABSpec.Print(std::cout);

        MlmeDsmeGtsResponseParams respParams;
        respParams.m_devAddr = params.m_devAddr;
        respParams.m_manageType = params.m_manageType;
        respParams.m_direction = params.m_direction;
        respParams.m_prioritizedChAccess = params.m_prioritizedChAccess;

        respParams.m_channelOfs = 1;

        if (params.m_preferredSuperframeID >= maxSuperframeId 
            || params.m_preferredSlotID >= 16) {
            respParams.m_status = MLMEDSMEGTS_INVALID_PARAMETER;

        } else {
            if (first) {
                dev0DsmeSAB[params.m_preferredSuperframeID] = 0b00001110;
            } else {
                dev0DsmeSAB[params.m_preferredSuperframeID] = 0b00011111;
            }
            
            respParams.m_dsmeSABSpec.setCAPReduction(device->GetMac()->isCAPReductionOn());
            respParams.m_dsmeSABSpec.setSABSubBlkLen(1);
            respParams.m_dsmeSABSpec.setSABSubBlkIdx(params.m_preferredSuperframeID);
            respParams.m_dsmeSABSpec.setSABSubBlk(dev0DsmeSAB[params.m_preferredSuperframeID]);

            respParams.m_status = MLMEDSMEGTS_SUCCESS;

            first = false;
        }

        Simulator::ScheduleNow(&LrWpanMac::MlmeDsmeGtsResponse,
                            device->GetMac(),
                            respParams);

    } else if (params.m_manageType == GTS_DEALLOCATION) {
        params.m_dsmeSABSpec.Print(std::cout);

        MlmeDsmeGtsResponseParams respParams;
        respParams.m_devAddr = params.m_devAddr;
        respParams.m_manageType = params.m_manageType;
        respParams.m_direction = params.m_direction;
        respParams.m_prioritizedChAccess = params.m_prioritizedChAccess;

        respParams.m_channelOfs = 1;

        respParams.m_dsmeSABSpec = std::move(params.m_dsmeSABSpec);

        respParams.m_status = MLMEDSMEGTS_SUCCESS;

        Simulator::ScheduleNow(&LrWpanMac::MlmeDsmeGtsResponse,
                                device->GetMac(),
                                respParams);
       
    } else if (params.m_manageType == GTS_EXPIRATION) {
        params.m_dsmeSABSpec.Print(std::cout);

        MlmeDsmeGtsRequestParams reqParams;
        reqParams.m_devAddr = params.m_devAddr;
        reqParams.m_manageType = GTS_DEALLOCATION;
        reqParams.m_direction = params.m_direction;
        reqParams.m_prioritizedChAccess = params.m_prioritizedChAccess;
        reqParams.m_dsmeSABSpec = std::move(params.m_dsmeSABSpec);

        Simulator::ScheduleNow(&LrWpanMac::MlmeDsmeGtsRequest,
                                device->GetMac(),
                                reqParams);
    }

}

static void MlmeDsmeGtsConfirm(Ptr<LrWpanNetDevice> device, MlmeDsmeGtsConfirmParams params) {
    if (params.m_status == MLMEDSMEGTS_REQ_SUCCESS) {
        std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
              << " [" << device->GetMac()->GetShortAddress() << " | "
              << device->GetMac()->GetExtendedAddress() << "]"
              << " MLME-DSME-GTS.confirm: MLMEDSMEGTS_REQ_SUCCESS " 
              << " new Dsme SAB is : "
              << "\n";

              params.m_dsmeSABSpec.Print(std::cout);

    } else if (params.m_status == MLMEDSMEGTS_REQ_DENIED) {
        std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
              << " [" << device->GetMac()->GetShortAddress() << " | "
              << device->GetMac()->GetExtendedAddress() << "]"
              << " MLME-DSME-GTS.confirm: Gts request denied " 
              << "\n";

    } else if (params.m_status == MLMEDSMEGTS_REQ_INVALID_PARAMETER) {
        std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
              << " [" << device->GetMac()->GetShortAddress() << " | "
              << device->GetMac()->GetExtendedAddress() << "]"
              << " MLME-DSME-GTS.confirm: Gts request failed due to invalid parameter " 
              << "\n";
    }
}

// static void CommStatusIndication(Ptr<LrWpanNetDevice> device, MlmeCommStatusIndicationParams params) {
//     // Used by coordinator higher layer to inform results of a
//     // association procedure from its mac layer.This is implemented by other protocol stacks
//     // and is only here for demonstration purposes.

//     switch (params.m_status) {
//         case LrWpanMlmeCommStatus::MLMECOMMSTATUS_TRANSACTION_EXPIRED:
//             std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
//                     << " [" << device->GetMac()->GetShortAddress() << " | "
//                     << device->GetMac()->GetExtendedAddress() << "]"
//                     << " MLME-comm-status.indication: Transaction for device " << params.m_dstExtAddr
//                     << " EXPIRED in pending transaction list\n";
//             break;
//         case LrWpanMlmeCommStatus::MLMECOMMSTATUS_NO_ACK:
//             std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
//                     << " [" << device->GetMac()->GetShortAddress() << " | "
//                     << device->GetMac()->GetExtendedAddress() << "]"
//                     << " MLME-comm-status.indication: NO ACK from " << params.m_dstExtAddr
//                     << " device registered in the pending transaction list\n";
//             break;

//         case LrWpanMlmeCommStatus::MLMECOMMSTATUS_CHANNEL_ACCESS_FAILURE:
//             std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
//                     << " [" << device->GetMac()->GetShortAddress() << " | "
//                     << device->GetMac()->GetExtendedAddress() << "]"
//                     << " MLME-comm-status.indication: CHANNEL ACCESS problem in transaction for "
//                     << params.m_dstExtAddr << " registered in the pending transaction list\n";
//             break;

//     default:
//         break;
//     }
// }

int
main(int argc, char* argv[])
{
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);

    LrWpanHelper lrWpanHelper;

    // Create 2 nodes, and a NetDevice for each one
    Ptr<Node> n0 = CreateObject<Node>();
    Ptr<Node> n1 = CreateObject<Node>();

    Ptr<LrWpanNetDevice> dev0 = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev1 = CreateObject<LrWpanNetDevice>();

    lrWpanHelper.EnablePcap(std::string("dsme-gts-handshake-dev0.pcap"), dev0, true, true);
    lrWpanHelper.EnablePcap(std::string("dsme-gts-handshake-dev1.pcap"), dev1, true, true);


    dev0->SetAddress(Mac16Address("00:01"));
    dev1->SetAddress(Mac16Address("00:02"));

    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    dev0->SetChannel(channel);
    dev1->SetChannel(channel);

    n0->AddDevice(dev0);
    n1->AddDevice(dev1);

    ///////////////// Mobility   ///////////////////////
    Ptr<ConstantPositionMobilityModel> sender0Mobility =
        CreateObject<ConstantPositionMobilityModel>();

    sender0Mobility->SetPosition(Vector(0, 0, 0));
    dev0->GetPhy()->SetMobility(sender0Mobility);

    Ptr<ConstantPositionMobilityModel> sender1Mobility =
        CreateObject<ConstantPositionMobilityModel>();

    Ptr<ConstantPositionMobilityModel> sender2Mobility =
        CreateObject<ConstantPositionMobilityModel>();

    sender1Mobility->SetPosition(Vector(0, 10, 0)); // 10 m distance
    dev1->GetPhy()->SetMobility(sender1Mobility);


    /////// MAC layer Callbacks hooks/////////////

    MlmeStartConfirmCallback cb0;
    cb0 = MakeCallback(&StartConfirm);
    dev0->GetMac()->SetMlmeStartConfirmCallback(cb0);

    McpsDataConfirmCallback cb1;
    cb1 = MakeCallback(&TransEndIndication);
    dev1->GetMac()->SetMcpsDataConfirmCallback(cb1);

    MlmeBeaconNotifyIndicationCallback cb3;
    cb3 = MakeCallback(&BeaconIndication);
    dev1->GetMac()->SetMlmeBeaconNotifyIndicationCallback(cb3);

    McpsDataIndicationCallback cb4;
    cb4 = MakeCallback(&DataIndication);
    dev1->GetMac()->SetMcpsDataIndicationCallback(cb4);

    McpsDataIndicationCallback cb5;
    cb5 = MakeCallback(&DataIndicationCoordinator);
    dev0->GetMac()->SetMcpsDataIndicationCallback(cb5);

    dev0->GetMac()->SetMlmeDsmeGtsIndicationCallback(MakeBoundCallback(&MlmeDsmeGtsIndication, dev0));
    dev1->GetMac()->SetMlmeDsmeGtsIndicationCallback(MakeBoundCallback(&MlmeDsmeGtsIndication, dev1));

    dev0->GetMac()->SetMlmeDsmeGtsConfirmCallback(MakeBoundCallback(&MlmeDsmeGtsConfirm, dev0));
    dev1->GetMac()->SetMlmeDsmeGtsConfirmCallback(MakeBoundCallback(&MlmeDsmeGtsConfirm, dev1));

    // Dsme
    dev0->GetMac()->SetDsmeModeEnabled();
    dev1->GetMac()->SetDsmeModeEnabled();

    dev0->GetMac()->SetChannelHoppingEnabled();
    dev1->GetMac()->SetChannelHoppingEnabled();

    dev0->GetMac()->SetNumOfChannelSupported(15);
    dev1->GetMac()->SetNumOfChannelSupported(15);

    dev1->GetMac()->SetChannelOffset(1);

    //////////// Manual device association ////////////////////
    // Note: We manually associate the devices to a PAN coordinator
    //       because currently there is no automatic association behavior (bootstrap);
    //       The PAN COORDINATOR does not need to associate or set its
    //       PAN Id or its own coordinator id, these are set
    //       by the MLME-start.request primitive when used.

    dev1->GetMac()->SetPanId(5);
    dev1->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));
    dev1DsmeSAB.resize(static_cast<uint64_t>(1 << (14 - 6)), 0);

    ///////////////////// Start transmitting beacons from coordinator ////////////////////////
    MlmeStartRequestParams params;
    params.m_panCoor = true;
    params.m_PanId = 5;

    params.m_bcnOrd = 14;
    params.m_sfrmOrd = 6;

    // Beacon Bitmap
    BeaconBitmap bitmap(0, 1 << (14 - 6));
    bitmap.SetSDBitmap(0);                  // SD = 0 目前占用
    params.m_bcnBitmap = bitmap;

    dev0DsmeSAB.resize(static_cast<uint64_t>(1 << (14 - 6)), 0);

    // Dsme 
    params.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(12);
    params.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);
    params.m_dsmeSuperframeSpec.SetCAPReductionFlag(false);

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = 1;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, 2);    // offset = 1 目前占用

    params.m_hoppingDescriptor = hoppingDescriptor;
    
    Simulator::ScheduleWithContext(1,
                                   Seconds(2.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   dev0->GetMac(),
                                   params);

    /////////////////////// Dsme SAB setting /////////////////////////
    dev1->GetMac()->ResizeMacDSMESAB(false, 14, 6);
    dev1->GetMac()->ResizeScheduleGTSsEvent(14, 12, 6);

    ///////////////////// Gsme Gts Handshake ////////////////////////
    MlmeDsmeGtsRequestParams params2;
    params2.m_devAddr = Mac16Address("00:01");
    params2.m_manageType = GTS_ALLOCATION;
    params2.m_direction = 0x00;
    params2.m_prioritizedChAccess = 0x01;
    params2.m_numSlot = 3;
    params2.m_preferredSuperframeID = 2;
    params2.m_preferredSlotID = 1;
    params2.m_dsmeSABSpec.setCAPReduction(false);
    params2.m_dsmeSABSpec.setSABSubBlkLen(1);
    params2.m_dsmeSABSpec.setSABSubBlkIdx(0);
    params2.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[0]);  // DSME-TODO

    Simulator::ScheduleWithContext(1,
                                   Seconds(3.0),
                                   &LrWpanMac::MlmeDsmeGtsRequest,
                                   dev1->GetMac(),
                                   params2);

    ///////////////////// Transmission of data Packets from end device //////////////////////

    // Dsme
    dev1->GetMac()->SetMultisuperframeOrder(12);

    Ptr<Packet> p1 = Create<Packet>(5);
    McpsDataRequestParams params3;
    params3.m_dstPanId = 5;
    params3.m_srcAddrMode = SHORT_ADDR;
    params3.m_dstAddrMode = SHORT_ADDR;
    params3.m_dstAddr = Mac16Address("00:01");
    params3.m_msduHandle = 0;
    params3.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack
    params3.m_txOptions |= TX_OPTION_GTS;

    /////////////////////////////////////////////////////////////////////////////////////
    // Examples of time parameters for transmissions in the first incoming superframe. //
    /////////////////////////////////////////////////////////////////////////////////////

    // 2.981 sec      No time to finish CCA in CAP, the transmission at this time will cause
    //                the packet to be deferred to the next superframe.

    // 2.982 sec      No time to finish random backoff delay in CAP, the  transmission at this
    //                time will cause the packet to be deferred to the next superframe.

    // 2.93 sec       Enough time, the packet can be transmitted within the CAP of the first
    // superframe

    // MCPS-DATA.request Beacon enabled Direct Transmission (dev1)
    // Frame transmission from End Device to Coordinator (Direct transmission)
    Simulator::ScheduleWithContext(1,
                                   Seconds(759.559016000),
                                   &LrWpanMac::McpsDataRequest,
                                   dev1->GetMac(),
                                   params3,
                                   p1);

    std::cout << " Symbol Rate (per sec) " << dev0->GetPhy()->GetDataOrSymbolRate(false) << std ::endl;
    std::cout << "**********************************" << std ::endl;

    dev1DsmeSAB[params2.m_preferredSuperframeID] = 0b00001110;

    ///////////////////// Gsme Gts Handshake ////////////////////////
    MlmeDsmeGtsRequestParams params4;
    params4.m_devAddr = Mac16Address("00:01");
    params4.m_manageType = GTS_ALLOCATION;
    params4.m_direction = 0x01;
    params4.m_prioritizedChAccess = 0x01;
    params4.m_numSlot = 5;
    params4.m_preferredSuperframeID = 0;
    params4.m_preferredSlotID = 0;
    params4.m_dsmeSABSpec.setCAPReduction(false);
    params4.m_dsmeSABSpec.setSABSubBlkLen(3);
    params4.m_dsmeSABSpec.setSABSubBlkIdx(0);
    params4.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[0]);  
    params4.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[1]);  
    params4.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[2]);  

    Simulator::ScheduleWithContext(1,
                                   Seconds(800.0),
                                   &LrWpanMac::MlmeDsmeGtsRequest,
                                   dev1->GetMac(),
                                   params4);

    Simulator::Stop(Seconds(2000));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
