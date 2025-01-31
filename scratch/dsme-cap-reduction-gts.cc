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

#define BO 14
#define SO 6
#define MO 12

#define CHNNEL_ADAPTATION 0
#define CHNNEL_HOPPING 1

#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/multi-model-spectrum-channel.h>

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

    /**
     *  DSME GTS allocation handshaking Flow
     *  Stack : 
     *  -------------------------------------------------------
     *  |[device - HigherLayer]   |  [PAN-C - HigherLayerMAC] | 
     *  |[device - MAC]           |  [PAN-C - MAC]            |
     *  -------------------------------------------------------
     * 
     *  1. [device -  HigherLayer]  ->  [device -  MAC]          :  MLME-DSME-GTS.request    
     *  2. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Request command 
     *? 3. [PAN-C  -  MAC]          ->  [PAN-C  -  HigherLayer]  :  MLME-DSME-GTS.indication 
     *  4. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-DSME-GTS.response
     *  5. [PAN-C  -  MAC]          ->  [device -  MAC]          :  DSME GTS Reponse command
     *  6. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-COMM-STATUS.indication
     *  7. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Notify command
     *  8. [device -  MAC]          ->  [device -  HigherLayer]  :  MLME-DSME-GTS.confirm 
     */ 

    // 檢查參數
    std::cout << Simulator::Now().As(Time::S)
              << " Coordinator " << device->GetNode()->GetId() << "\n"
              << " [MLME-DSME-GTS.indication] "
              << " Device short address : " << " [" << device->GetMac()->GetShortAddress() << "]" << " | "
              << " Manage Type : " << (uint16_t) params.m_manageType << " | "
              << " Direction : " << (uint16_t) params.m_direction << " | "
              << " Priority : " << (uint16_t) params.m_prioritizedChAccess << " | "
              << " Num Of Slots : " << (uint16_t) params.m_numSlot << " | "
              << " Preferred Superframe ID : " << params.m_preferredSuperframeID << " | "
              << " Preferred Slot ID : " << (uint16_t) params.m_preferredSlotID << " | \n" 
              << " SAB Spec :";

    
    params.m_dsmeSABSpec.Print(std::cout);

    uint32_t maxSuperframeId = device->GetMac()->GetNumOfSuperframesInABeaconInterval() /
                                device->GetMac()->GetNumOfMultisuperframesInABeaconInterval();
    
    // std::cout<< maxSuperframeId << std::endl;

    MlmeDsmeGtsResponseParams respParams;
    respParams.m_devAddr = params.m_devAddr;
    respParams.m_manageType = params.m_manageType;
    respParams.m_direction = params.m_direction;
    respParams.m_prioritizedChAccess = params.m_prioritizedChAccess;

    respParams.m_channelOfs = 1;

    if (params.m_manageType == GTS_ALLOCATION) {
        if (params.m_preferredSuperframeID >= maxSuperframeId 
            || params.m_preferredSlotID >= 16) {
            respParams.m_status = MLMEDSMEGTS_INVALID_PARAMETER;

        } else {
            
            /**
             *! Set SAB manually, need to check the sanity.
            */
            dev0DsmeSAB[params.m_preferredSuperframeID] = 0b0000000000000110;
            
            respParams.m_dsmeSABSpec.setCAPReduction(device->GetMac()->isCAPReductionOn());
            respParams.m_dsmeSABSpec.setSABSubBlkLen(1); //! This need to be fix , not a fix length.
            respParams.m_dsmeSABSpec.setSABSubBlkIdx(params.m_preferredSuperframeID);
            respParams.m_dsmeSABSpec.setSABSubBlk(dev0DsmeSAB[params.m_preferredSuperframeID]);

            respParams.m_status = MLMEDSMEGTS_SUCCESS;
        }
    }

    Simulator::ScheduleNow(&LrWpanMac::MlmeDsmeGtsResponse,
                           device->GetMac(),
                           respParams);
}

static void MlmeDsmeGtsConfirm(Ptr<LrWpanNetDevice> device, MlmeDsmeGtsConfirmParams params) {
    
    /**
     *  DSME GTS allocation handshaking Flow
     *  Stack : 
     *  -------------------------------------------------------
     *  |[device - HigherLayer]   |  [PAN-C - HigherLayerMAC] | 
     *  |[device - MAC]           |  [PAN-C - MAC]            |
     *  -------------------------------------------------------
     * 
     *  1. [device -  HigherLayer]  ->  [device -  MAC]          :  MLME-DSME-GTS.request    
     *  2. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Request command 
     *  3. [PAN-C  -  MAC]          ->  [PAN-C  -  HigherLayer]  :  MLME-DSME-GTS.indication 
     *  4. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-DSME-GTS.response
     *  5. [PAN-C  -  MAC]          ->  [device -  MAC]          :  DSME GTS Reponse command
     *  6. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-COMM-STATUS.indication
     *  7. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Notify command
     *? 8. [device -  MAC]          ->  [device -  HigherLayer]  :  MLME-DSME-GTS.confirm 
     */ 

    /*
        Here are the higher layer of the GTS request device, device MAC send GTS.confirm to its higher layer (which is here).
    */

    if (params.m_status == MLMEDSMEGTS_REQ_SUCCESS) {
        std::cout << Simulator::Now().As(Time::S) << " Coordinator " 
              << device->GetNode()->GetId() << "\n"
              << " [MLME-DSME-GTS.confirm]"
              << " Device short address : " << "[" << device->GetMac()->GetShortAddress() << "]" << " | "
              << " Device extended address : "<< "[" << device->GetMac()->GetExtendedAddress() << "]" << " | "
              << " MLME-DSME-GTS.confirm Status : MLMEDSMEGTS_REQ_SUCCESS " << "\n"
              << " new Dsme SAB is : ";

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
    /**
     *  DSME GTS allocation handshaking Flow
     *  Stack : 
     *  -------------------------------------------------------
     *  |[device - HigherLayer]   |  [PAN-C - HigherLayerMAC] | 
     *  |[device - MAC]           |  [PAN-C - MAC]            |
     *  -------------------------------------------------------
     * 
     *  1. [device -  HigherLayer]  ->  [device -  MAC]          :  MLME-DSME-GTS.request    
     *  2. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Request command 
     *  3. [PAN-C  -  MAC]          ->  [PAN-C  -  HigherLayer]  :  MLME-DSME-GTS.indication 
     *  4. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-DSME-GTS.response
     *  5. [PAN-C  -  MAC]          ->  [device -  MAC]          :  DSME GTS Reponse command
     *? 6. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-COMM-STATUS.indication
     *  7. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Notify command
     *  8. [device -  MAC]          ->  [device -  HigherLayer]  :  MLME-DSME-GTS.confirm 
     */

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
    // LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
    // LogComponentEnable("PacketMetadata", LOG_LEVEL_ALL);  // debug
    // LogComponentEnable("Packet", LOG_LEVEL_ALL);         // debug

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();

    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    LrWpanHelper lrWpanHelper;

    Ptr<Node> node_PanC = CreateObject<Node>();
    Ptr<Node> node_Dev1 = CreateObject<Node>();
    Ptr<LrWpanNetDevice> devPanC = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev1 = CreateObject<LrWpanNetDevice>();

    // lrWpanHelper.EnablePcap(std::string("dsme-gts-handshake-dev0.pcap"), devPanC, true, true);
    // lrWpanHelper.EnablePcap(std::string("dsme-gts-handshake-dev1.pcap"), dev1, true, true);

    devPanC->SetAddress(Mac16Address("00:01"));
    dev1->SetAddress(Mac16Address("00:02"));

    devPanC->SetChannel(channel);
    dev1->SetChannel(channel);

    node_PanC->AddDevice(devPanC);
    node_Dev1->AddDevice(dev1);

    ///////////////// Mobility   ///////////////////////
    Ptr<ConstantPositionMobilityModel> sender0Mobility =
        CreateObject<ConstantPositionMobilityModel>();

    sender0Mobility->SetPosition(Vector(0, 0, 0));
    devPanC->GetPhy()->SetMobility(sender0Mobility);

    Ptr<ConstantPositionMobilityModel> sender1Mobility =
        CreateObject<ConstantPositionMobilityModel>();

    Ptr<ConstantPositionMobilityModel> sender2Mobility =
        CreateObject<ConstantPositionMobilityModel>();

    sender1Mobility->SetPosition(Vector(0, 10, 0)); // 10 m distance
    dev1->GetPhy()->SetMobility(sender1Mobility);


    /////// MAC layer Callbacks hooks/////////////

    MlmeStartConfirmCallback cb0;
    cb0 = MakeCallback(&StartConfirm);
    devPanC->GetMac()->SetMlmeStartConfirmCallback(cb0);

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
    devPanC->GetMac()->SetMcpsDataIndicationCallback(cb5);

    devPanC->GetMac()->SetMlmeDsmeGtsIndicationCallback(MakeBoundCallback(&MlmeDsmeGtsIndication, devPanC));

    dev1->GetMac()->SetMlmeDsmeGtsConfirmCallback(MakeBoundCallback(&MlmeDsmeGtsConfirm, dev1));

    // Dsme
    devPanC->GetMac()->SetDsmeModeEnabled();
    dev1->GetMac()->SetDsmeModeEnabled();

    devPanC->GetMac()->SetChannelHoppingEnabled();
    dev1->GetMac()->SetChannelHoppingEnabled();

    devPanC->GetMac()->SetNumOfChannelSupported(15);
    dev1->GetMac()->SetNumOfChannelSupported(15);

    dev1->GetMac()->SetChannelOffset(1);

    ///////////////////// Start transmitting beacons from coordinator ////////////////////////
    MlmeStartRequestParams startReqParams;
    startReqParams.m_panCoor = true;
    startReqParams.m_PanId = 5;

    startReqParams.m_bcnOrd = BO;
    startReqParams.m_sfrmOrd = SO;
    uint8_t multiSuperframeOrder = MO;

    // Beacon Bitmap
    BeaconBitmap bitmap(0, 1 << (startReqParams.m_bcnOrd - startReqParams.m_sfrmOrd));
    bitmap.SetSDBitmap(0);                  // SD = 0 目前占用
    startReqParams.m_bcnBitmap = bitmap;

    dev0DsmeSAB.resize(static_cast<uint64_t>(1 << (startReqParams.m_bcnOrd - startReqParams.m_sfrmOrd)), 0);

    // Dsme 
    startReqParams.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(multiSuperframeOrder);
    startReqParams.m_dsmeSuperframeSpec.SetChannelDiversityMode(CHNNEL_HOPPING); 
    startReqParams.m_dsmeSuperframeSpec.SetCAPReductionFlag(true); //! Set CAP reduction here

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00; // 0x00 : Default hopping sequence, check table 34a in 802.15.4e spec.
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = 1;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, 2);    // offset = 1 目前占用 , 紀錄有哪些offset有被占用(bitmap, 1代表有在用)
                                                          // 2 = 0b01000000...
                                                          //        |  -> bit(1) channel offset : 1
    startReqParams.m_hoppingDescriptor = hoppingDescriptor;
    // MLME-START.request primitive is used by the PAN coordinator to initiate a new PAN or to begin using a new superframe configuration.
    Simulator::ScheduleWithContext(1,
                                   Seconds(2.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   devPanC->GetMac(),
                                   startReqParams);


    //////////// Manual device association ////////////////////
    // Note: We manually associate the devices to a PAN coordinator
    //       because currently there is no automatic association behavior (bootstrap);
    //       The PAN COORDINATOR does not need to associate or set its
    //       PAN Id or its own coordinator id, these are set
    //       by the MLME-start.request primitive when used.

    dev1->GetMac()->SetPanId(5);
    dev1->GetMac()->SetAssociatedCoor(Mac16Address("00:01"));
    dev1DsmeSAB.resize(static_cast<uint64_t>(1 << (BO - SO)), 0);
    /////////////////////// Dsme SAB setting /////////////////////////
    // Set the size of Slot allocation block & resize the length of GTS which need to be scheduled.
    dev1->GetMac()->ResizeMacDSMESAB(false, startReqParams.m_bcnOrd, startReqParams.m_sfrmOrd);
    dev1->GetMac()->ResizeScheduleGTSsEvent(startReqParams.m_bcnOrd, multiSuperframeOrder, startReqParams.m_sfrmOrd);
    dev1->GetMac()->SetCAPReduction(true);

    ///////////////////// Gsme Gts Handshake ////////////////////////

    /**
     *  DSME GTS allocation handshaking Flow
     *  Stack : 
     *  -------------------------------------------------------
     *  |[device - HigherLayer]   |  [PAN-C - HigherLayerMAC] | 
     *  |[device - MAC]           |  [PAN-C - MAC]            |
     *  -------------------------------------------------------
     * 
     *? 1. [device -  HigherLayer]  ->  [device -  MAC]          :  MLME-DSME-GTS.request    
     *  2. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Request command 
     *  3. [PAN-C  -  MAC]          ->  [PAN-C  -  HigherLayer]  :  MLME-DSME-GTS.indication 
     *  4. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-DSME-GTS.response
     *  5. [PAN-C  -  MAC]          ->  [device -  MAC]          :  DSME GTS Reponse command
     *  6. [PAN-C  -  HigherLayer]  ->  [PAN-C  -  MAC]          :  MLME-COMM-STATUS.indication
     *  7. [device -  MAC]          ->  [PAN-C  -  MAC]          :  DSME GTS Notify command
     *  8. [device -  MAC]          ->  [device -  HigherLayer   :  MLME-DSME-GTS.confirm 
     */ 

    // MlmeDsmeGtsRequestParams gtsRequestParams;
    // gtsRequestParams.m_devAddr = Mac16Address("00:01"); // 要request的Addr
    // gtsRequestParams.m_manageType = GTS_ALLOCATION;
    // gtsRequestParams.m_direction = 0x00; // 0x00 : TX
    // gtsRequestParams.m_prioritizedChAccess = 0x01;
    // gtsRequestParams.m_numSlot = 1;
    // gtsRequestParams.m_preferredSuperframeID = 2;
    // gtsRequestParams.m_preferredSlotID = 1;
    // gtsRequestParams.m_dsmeSABSpec.setCAPReduction(false);
    // gtsRequestParams.m_dsmeSABSpec.setSABSubBlkLen(1); // The length of the DSME SAB sub-block in units. One unit = 7 slot (No CAP reduction) or 15 slot (For CAP reduction).
    // gtsRequestParams.m_dsmeSABSpec.setSABSubBlkIdx(0); // Indicate the beginning of the DSME SAB Sub-block in the entire SAB (not in unit).
    // gtsRequestParams.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[0]);  // DSME-TODO

    // Simulator::ScheduleWithContext(1,
    //                                Seconds(3.0),
    //                                &LrWpanMac::MlmeDsmeGtsRequest,
    //                                dev1->GetMac(),
    //                                gtsRequestParams);

    // MlmeDsmeGtsRequestParams gtsRequestParams2;
    // gtsRequestParams2.m_devAddr = Mac16Address("00:01"); // 要request的Addr
    // gtsRequestParams2.m_manageType = GTS_ALLOCATION;
    // gtsRequestParams2.m_direction = 0x00; // 0x00 : TX
    // gtsRequestParams2.m_prioritizedChAccess = 0x01;
    // gtsRequestParams2.m_numSlot = 1;
    // gtsRequestParams2.m_preferredSuperframeID = 2;
    // gtsRequestParams2.m_preferredSlotID = 2;
    // gtsRequestParams2.m_dsmeSABSpec.setCAPReduction(false);
    // gtsRequestParams2.m_dsmeSABSpec.setSABSubBlkLen(1); // The length of the DSME SAB sub-block in units. One unit = 7 slot (No CAP reduction) or 15 slot (For CAP reduction).
    // gtsRequestParams2.m_dsmeSABSpec.setSABSubBlkIdx(0); // Indicate the beginning of the DSME SAB Sub-block in the entire SAB (not in unit).
    // gtsRequestParams2.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[0]);  // DSME-TODO

    // Simulator::ScheduleWithContext(1,
    //                                Seconds(4.0),
    //                                &LrWpanMac::MlmeDsmeGtsRequest,
    //                                dev1->GetMac(),
    //                                gtsRequestParams2);                          

    ///////////////////// Transmission of data Packets from end device //////////////////////

    // Dsme
    // dev1->GetMac()->SetMultisuperframeOrder(multiSuperframeOrder);

    // Ptr<Packet> p1 = Create<Packet>(5);
    // McpsDataRequestParams dataReqParams;
    // dataReqParams.m_dstPanId = 5;
    // dataReqParams.m_srcAddrMode = SHORT_ADDR;
    // dataReqParams.m_dstAddrMode = SHORT_ADDR;
    // dataReqParams.m_dstAddr = Mac16Address("00:01");
    // dataReqParams.m_msduHandle = 0;
    // dataReqParams.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack
    // dataReqParams.m_txOptions |= TX_OPTION_GTS;

    // // MCPS-DATA.request Beacon enabled Direct Transmission (dev1)
    // // Frame transmission from End Device to Coordinator (Direct transmission)
    // Simulator::ScheduleWithContext(1,
    //                                Seconds(759.559016000),
    //                                &LrWpanMac::McpsDataRequest,
    //                                dev1->GetMac(),
    //                                dataReqParams,
    //                                p1);

    std::cout << " Symbol Rate (per sec) " << devPanC->GetPhy()->GetDataOrSymbolRate(false) << std ::endl;
    std::cout << "**********************************" << std ::endl;

    Simulator::Stop(Seconds(1000));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}


