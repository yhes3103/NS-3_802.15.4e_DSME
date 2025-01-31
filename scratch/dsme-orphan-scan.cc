/*
 * Copyright (c) 2022 Tokushima University, Japan.
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
 * Author:  Alberto Gallegos Ramonet <alramonet@is.tokushima-u.ac.jp>
 */

/*
 *      [00:01]                   [00:02]                                   [00:03]
 *  PAN Coordinator 1 (PAN: 5)       End Device                        PAN Coordinator 2 (PAN: 7)
 *       |--------100 m----------------|----------106 m -----------------------|
 *  Channel 12               (Active Scan channels 11-14)                 Channel 14
 *
 *
 * This example demonstrate the usage of the MAC MLME-SCAN.request (ACTIVE scan) primitive as
 * described by IEEE 802.15.4-2011.
 * At the beginning of the simulation, PAN coordinators are set to
 * non-beacon enabled mode and wait for any beacon requests.
 *
 * The end device initiate an Active scan where a beacon request command is transmitted on
 * on each channel. If a beacon coordinator is present and in range in the channel, it responds with
 * a beacon which contains the PAN descriptor with useful information for the association process
 * (channel number, Pan ID, coord address, link quality indicator).
 *
 * LQI range: 0 - 255
 * Where 255 is the MAX possible value used to described how clearly the packet was heard.
 * Typically, a value below 127 is considered a link with poor quality.
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

#include <map>

using namespace ns3;

static std::map<Mac64Address, Mac16Address> devAssociated;

static void ScanConfirm(Ptr<LrWpanNetDevice> device, MlmeScanConfirmParams params) {
    // The algorithm to select which coordinator to associate is not
    // covered by the standard. In this case, we use the coordinator
    // with the highest LQI value obtained from a passive scan and make
    // sure this coordinator allows association.

    std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId()
                          << " [" << device->GetMac()->GetShortAddress() << " | "
                          << device->GetMac()->GetExtendedAddress() << "]"
                          << " Completed Channel Scan. "
                          << "\n";

    if (params.m_status == MLMESCAN_SUCCESS) {
        // Select the coordinator with the highest LQI from the PAN Descriptor List
        int maxLqi = 0;
        int panDescIndex = 0;

        if (params.m_panDescList.size() > 0) {
            for (uint32_t i = 0; i < params.m_panDescList.size(); i++) {
                 std::cout << "Device [" << device->GetMac()->GetShortAddress()
                           << "] found the following PANs:\n";
                           
                    for (long unsigned int i = 0; i < params.m_panDescList.size(); i++) {
                        std::cout << "PAN DESCRIPTOR " << i << ":\n"
                                << "Pan Id: " << params.m_panDescList[i].m_coorPanId
                                << "\nChannel: " << static_cast<uint32_t>(params.m_panDescList[i].m_logCh)
                                << "\nLQI: "
                                << static_cast<uint32_t>(params.m_panDescList[i].m_linkQuality)
                                << "\nCoordinator Short Address: "
                                << params.m_panDescList[i].m_coorShortAddr 
                                << "\n DSME Superframe Spec: "
                                << params.m_panDescList[i].m_dsmeSuperframeSpec
                                << "\n Beacon Bitmap: "
                                << params.m_panDescList[i].m_bcnBitmap
                                << "\n Channel Hopping: "
                                << params.m_panDescList[i].m_channelHoppingSpec
                                << "\n\n";    
                    }

                if (params.m_panDescList[i].m_linkQuality > maxLqi) {
                    maxLqi = params.m_panDescList[i].m_linkQuality;
                    panDescIndex = i;
                }
            }

            // Only request association if the coordinator is permitting association at this moment.
            if (params.m_panDescList[panDescIndex].m_superframeSpec.IsAssocPermit()) {
                std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId()
                          << " [" << device->GetMac()->GetShortAddress() << " | "
                          << device->GetMac()->GetExtendedAddress() << "]"
                          << " MLME-scan.confirm:  Selected PAN ID "
                          << params.m_panDescList[panDescIndex].m_coorPanId << " | LQI "
                          << static_cast<int>(params.m_panDescList[panDescIndex].m_linkQuality)
                          << "\n";

                if (params.m_panDescList[panDescIndex].m_linkQuality >= 127) {
                    MlmeAssociateRequestParams assocParams;
                    assocParams.m_chNum = params.m_panDescList[panDescIndex].m_logCh;
                    assocParams.m_chPage = params.m_panDescList[panDescIndex].m_logChPage;
                    assocParams.m_coordPanId = params.m_panDescList[panDescIndex].m_coorPanId;
                    assocParams.m_coordAddrMode = params.m_panDescList[panDescIndex].m_coorAddrMode;

                    if (params.m_panDescList[panDescIndex].m_coorAddrMode ==
                        LrWpanAddressMode::SHORT_ADDR)
                    {
                        assocParams.m_coordAddrMode = LrWpanAddressMode::SHORT_ADDR;
                        assocParams.m_coordShortAddr =
                            params.m_panDescList[panDescIndex].m_coorShortAddr;
                    }
                    else if (assocParams.m_coordAddrMode == LrWpanAddressMode::EXT_ADDR)
                    {
                        assocParams.m_coordAddrMode = LrWpanAddressMode::EXT_ADDR;
                        assocParams.m_coordExtAddr =
                            params.m_panDescList[panDescIndex].m_coorExtAddr;
                        assocParams.m_coordShortAddr = Mac16Address("ff:fe");
                    }

                    Simulator::ScheduleNow(&LrWpanMac::MlmeAssociateRequest,
                                           device->GetMac(),
                                           assocParams);
                    
                    // pick a vacant beacon slot
                    BeaconBitmap bitmap(0, 1 << (params.m_panDescList[panDescIndex].m_superframeSpec.GetBeaconOrder() 
                                                 - params.m_panDescList[panDescIndex].m_superframeSpec.GetFrameOrder()));
                    
                    for (uint32_t i = 0; i < params.m_panDescList.size(); i++) {
                        if (params.m_panDescList[i].m_coorPanId == params.m_panDescList[panDescIndex].m_coorPanId) {
                            bitmap = bitmap | params.m_panDescList[i].m_bcnBitmap;
                        }
                    }

                    std::cout << "Concluded Beacon Bitmap In Pan: " << params.m_panDescList[panDescIndex].m_coorPanId
                              << bitmap
                              << "\n";
                    
                    // DSME-TODO
                    device->GetMac()->SetTimeSlotToSendBcn(8);
                    device->GetMac()->SetDescIndexOfAssociatedPan(panDescIndex);

                } else {
                    std::cout << Simulator::Now().As(Time::S) << " Node "
                              << device->GetNode()->GetId() << " ["
                              << device->GetMac()->GetShortAddress() << " | "
                              << device->GetMac()->GetExtendedAddress() << "]"
                              << " MLME-scan.confirm: Beacon found but link quality too low for "
                                 "association.\n";
                }
            }

        } else {
            if (params.m_scanType == MLMESCAN_ORPHAN) {
                // std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId()
                //         << " [" << device->GetMac()->GetShortAddress() << " | "
                //         << device->GetMac()->GetExtendedAddress()
                //         << "] MLME-scan.confirm: Find the Coordinator that used to associate.\n";

            } else {
                std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId()
                        << " [" << device->GetMac()->GetShortAddress() << " | "
                        << device->GetMac()->GetExtendedAddress()
                        << "] MLME-scan.confirm: Beacon not found.\n";
            }
        }

    } else {
        std::cout << Simulator::Now().As(Time::S) << " [" << device->GetMac()->GetShortAddress()
                  << " | " << device->GetMac()->GetExtendedAddress()
                  << "]  error occurred, scan failed.\n";
    }
}

static void AssociateIndication(Ptr<LrWpanNetDevice> device, MlmeAssociateIndicationParams params) {
    // This is typically implemented by the  Coordinator next layer (3rd layer or higher).
    // The steps described below are out of the scope of the standard.

    // Here the 3rd layer should check:
    //    a) Whether or not the device was previously associated with this PAN (the coordinator
    //    keeps a list). b) The coordinator have sufficient resources available to allow the
    //    association.
    // If the association fails, status = 1 or 2 and assocShortAddr = FFFF.

    // In this example, the coordinator accepts every association request and have no association
    // limits. Furthermore, previous associated devices are not checked.

    // When short address allocation is on (set initially in the association request), the
    // coordinator is supposed to assign a short address. In here, we just do a dummy address
    // assign. The assigned short address is just a truncated version of the device existing
    // extended address (i.e the default short address).

    MlmeAssociateResponseParams assocRespParams;

    assocRespParams.m_extDevAddr = params.m_extDevAddr;
    assocRespParams.m_status = LrWpanAssociationStatus::ASSOCIATED;

    if (params.capabilityInfo.IsShortAddrAllocOn()) {
        // Truncate the extended address and make an assigned
        // short address based on this. This mechanism is not described by the standard.
        // It is just implemented here as a quick and dirty way to assign short addresses.
        // uint8_t buffer64MacAddr[8];
        // uint8_t buffer16MacAddr[2];

        // params.m_extDevAddr.CopyTo(buffer64MacAddr);
        // buffer16MacAddr[1] = buffer64MacAddr[7];
        // buffer16MacAddr[0] = buffer64MacAddr[6];

        Mac16Address shortAddr = Mac16Address("00:09");
        // shortAddr.CopyFrom(buffer16MacAddr);
        assocRespParams.m_assocShortAddr = shortAddr;

        devAssociated[params.m_extDevAddr] = shortAddr;

    } else {
        // If Short Address allocation flag is false, the device will
        // use its extended address to send data packets and short address will be
        // equal to ff:fe. See 802.15.4-2011 (Section 5.3.2.2)
        assocRespParams.m_assocShortAddr = Mac16Address("ff:fe");
    }

    Simulator::ScheduleNow(&LrWpanMac::MlmeAssociateResponse, device->GetMac(), assocRespParams);
}

static void CommStatusIndication(Ptr<LrWpanNetDevice> device, MlmeCommStatusIndicationParams params) {
    // Used by coordinator higher layer to inform results of a
    // association procedure from its mac layer.This is implemented by other protocol stacks
    // and is only here for demonstration purposes.

    switch (params.m_status) {
        case LrWpanMlmeCommStatus::MLMECOMMSTATUS_TRANSACTION_EXPIRED:
            std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
                    << " [" << device->GetMac()->GetShortAddress() << " | "
                    << device->GetMac()->GetExtendedAddress() << "]"
                    << " MLME-comm-status.indication: Transaction for device " << params.m_dstExtAddr
                    << " EXPIRED in pending transaction list\n";
            break;

        case LrWpanMlmeCommStatus::MLMECOMMSTATUS_NO_ACK:
            std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
                    << " [" << device->GetMac()->GetShortAddress() << " | "
                    << device->GetMac()->GetExtendedAddress() << "]"
                    << " MLME-comm-status.indication: NO ACK from " << params.m_dstExtAddr
                    << " device registered in the pending transaction list\n";
            
            break;

        case LrWpanMlmeCommStatus::MLMECOMMSTATUS_CHANNEL_ACCESS_FAILURE:
            std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
                    << " [" << device->GetMac()->GetShortAddress() << " | "
                    << device->GetMac()->GetExtendedAddress() << "]"
                    << " MLME-comm-status.indication: CHANNEL ACCESS problem in transaction for "
                    << params.m_dstExtAddr << " registered in the pending transaction list\n";
            break;

        default:
            break;
    }
}

static void TransEndIndication(Ptr<LrWpanNetDevice> device, McpsDataConfirmParams params) {
    // In the case of transmissions with the Ack flag activated, the transaction is only
    // successful if the Ack was received.
    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS) {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Transmission successfully sent");

    } else if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_NO_ACK) {
        std::cout << Simulator::Now().As(Time::S) << " Device " << device->GetNode()->GetId()
                    << " [" << device->GetMac()->GetShortAddress() << " | "
                    << device->GetMac()->GetExtendedAddress() << "]"
                    << " MCPS-DATA.confirm: NO ACK from Coordinator" 
                    << " Now assume the device is orphaned\n";
        
        // Then orphan Scan 
        MlmeScanRequestParams scanParams;
        scanParams.m_scanType = MLMESCAN_ORPHAN;
        scanParams.m_scanChannels = 0x7800;
        scanParams.m_chPage = 0;

        // DMSE 
        scanParams.m_linkQualityScan = false;
        scanParams.m_frameCtrlOptions.resize(3);
        scanParams.m_frameCtrlOptions[0] = false;    // PAN_ID_SUPPRESSED
        scanParams.m_frameCtrlOptions[1] = false;    // IES_INCLUDED
        scanParams.m_frameCtrlOptions[2] = false;    // SEQ_#_SUPPRESSED

        Simulator::ScheduleNow(&LrWpanMac::MlmeScanRequest,
                                device->GetMac(),
                                scanParams);
    }
}

static void AssociateConfirm(Ptr<LrWpanNetDevice> device, MlmeAssociateConfirmParams params) {
    // Used by device higher layer to inform the results of a
    // association procedure from its mac layer.This is implemented by other protocol stacks
    // and is only here for demonstration purposes.
    if (params.m_status == LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_SUCCESS) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-associate.confirm: Association with coordinator successful."
                  << " (PAN: " << device->GetMac()->GetPanId()
                  << " | CoordShort: " << device->GetMac()->GetCoordShortAddress()
                  << " | CoordExt: " << device->GetMac()->GetCoordExtAddress() << ")\n";

    } else if (params.m_status == LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_NO_ACK) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-associate.confirm: Association with coordinator FAILED (NO ACK).\n";

    } else {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-associate.confirm: Association with coordinator FAILED.\n";
    }
}

static void MlmeOrphanIndication(Ptr<LrWpanNetDevice> device, MlmeOrphanIndicationPararms params) {
    MlmeOrphanResponsePararms respParams;

    if (devAssociated.count(params.m_orphanAddress)) {
        respParams.m_orphanAddress = params.m_orphanAddress;
        respParams.m_shortAddr = devAssociated[params.m_orphanAddress];
        respParams.m_associateMember = true;

    } else {
        respParams.m_associateMember = false;
    }

    Simulator::ScheduleNow(&LrWpanMac::MlmeOrphanResponse,
                            device->GetMac(),
                            respParams);
}


int main(int argc, char* argv[]) {
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    
    // Create 1 PAN coordinator node, and 1 end device
    Ptr<Node> coord = CreateObject<Node>();
    Ptr<Node> endNode = CreateObject<Node>();

    Ptr<LrWpanNetDevice> coordNetDevice = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> endNodeNetDevice = CreateObject<LrWpanNetDevice>();

    coordNetDevice->SetAddress(Mac16Address("00:01"));
    endNodeNetDevice->SetAddress(Mac16Address("00:02"));

    // Configure Spectrum channel
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    coordNetDevice->SetChannel(channel);
    endNodeNetDevice->SetChannel(channel);

    coord->AddDevice(coordNetDevice);
    endNode->AddDevice(endNodeNetDevice);

    // Mobility
    Ptr<ConstantPositionMobilityModel> coordMobility =
        CreateObject<ConstantPositionMobilityModel>();
    coordMobility->SetPosition(Vector(0, 0, 0));
    coordNetDevice->GetPhy()->SetMobility(coordMobility);

    Ptr<ConstantPositionMobilityModel> endNodeMobility =
        CreateObject<ConstantPositionMobilityModel>();
    endNodeMobility->SetPosition(Vector(100, 0, 0));
    endNodeNetDevice->GetPhy()->SetMobility(endNodeMobility);

    // Coordinator hooks & MAC MLME-scan primitive set
    coordNetDevice->GetMac()->SetMlmeOrphanIndicationCallback(MakeBoundCallback(&MlmeOrphanIndication, coordNetDevice));

    // Devices hooks & MAC MLME-scan primitive set
    endNodeNetDevice->GetMac()->SetDsmeModeEnabled();
    endNodeNetDevice->GetMac()->SetMlmeScanConfirmCallback(MakeBoundCallback(&ScanConfirm, endNodeNetDevice));
    endNodeNetDevice->GetMac()->SetMlmeAssociateConfirmCallback(MakeBoundCallback(&AssociateConfirm, endNodeNetDevice));
    endNodeNetDevice->GetMac()->SetMcpsDataConfirmCallback(MakeBoundCallback(&TransEndIndication, endNodeNetDevice));                                                                                

    // End device N1 broadcast a single BEACON REQUEST for each channel (11, 12, 13, and 14).
    // If a coordinator is present and in range, it will respond with a beacon broadcast.
    // Scan Channels are represented by bits 0-26  (27 LSB)
    //                            ch 14  
    //                              |  
    // 0x7800  = 0000000000000000111100000000000
    MlmeScanRequestParams scanParams;
    scanParams.m_scanType = MLMESCAN_ENHANCED_ACTIVE_SCAN;
    scanParams.m_scanChannels = 0x7800;
    scanParams.m_scanDuration = 14;
    scanParams.m_chPage = 0;

    // DMSE 
    scanParams.m_linkQualityScan = false;
    scanParams.m_frameCtrlOptions.resize(3);
    scanParams.m_frameCtrlOptions[0] = false;    // PAN_ID_SUPPRESSED
    scanParams.m_frameCtrlOptions[1] = false;    // IES_INCLUDED
    scanParams.m_frameCtrlOptions[2] = false;    // SEQ_#_SUPPRESSED

    Simulator::ScheduleWithContext(endNodeNetDevice->GetNode()->GetId(),
                                   Seconds(3.0),
                                   &LrWpanMac::MlmeScanRequest,
                                   endNodeNetDevice->GetMac(),
                                   scanParams);

    // Coordinator hooks
    coordNetDevice->GetMac()->SetDsmeModeEnabled();
    coordNetDevice->GetMac()->SetMlmeAssociateIndicationCallback(MakeBoundCallback(&AssociateIndication, coordNetDevice));
    coordNetDevice->GetMac()->SetMlmeCommStatusIndicationCallback(MakeBoundCallback(&CommStatusIndication, coordNetDevice));

    // PAN coordinator N0 (PAN 5) is set to channel 14 in beacon mode
    // requests.
    MlmeStartRequestParams params;
    params.m_panCoor = true;
    params.m_PanId = 5;
    params.m_bcnOrd = 12;
    params.m_sfrmOrd = 4;
    params.m_logCh = 14;

    // Beacon Bitmap
    BeaconBitmap bitmap(0, 1 << (12 - 4));
    bitmap.SetSDBitmap(0);                  // SD = 0 目前占用
    params.m_bcnBitmap = bitmap;

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = 1;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, 2);    // offset = 1 目前占用

    params.m_hoppingDescriptor = hoppingDescriptor;

    // DSME
    params.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(10);
    params.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);
    params.m_dsmeSuperframeSpec.SetCAPReductionFlag(true);

    Simulator::ScheduleWithContext(coordNetDevice->GetNode()->GetId(),
                                   Seconds(2.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   coordNetDevice->GetMac(),
                                   params);

    // Synchronization
    MlmeSyncRequestParams syncParams;
    syncParams.m_logCh = 14; 
    syncParams.m_logChPage = 0; 
    syncParams.m_trackBcn = true; 

    Simulator::ScheduleWithContext(endNodeNetDevice->GetNode()->GetId(),
                                   Seconds(1009.69699),
                                   &LrWpanMac::MlmeSyncRequest,
                                   endNodeNetDevice->GetMac(),
                                   syncParams);  

    // end device attempt to send data pkt to coordinator after realignment
    Ptr<Packet> p1 = Create<Packet>(5);
    McpsDataRequestParams params4;
    params4.m_dstPanId = 5;
    params4.m_srcAddrMode = SHORT_ADDR;
    params4.m_dstAddrMode = SHORT_ADDR;
    params4.m_dstAddr = Mac16Address("00:01");
    params4.m_msduHandle = 0;
    params4.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack
    params4.m_txOptions |= TX_OPTION_DIRECT;  // Enable direct transmission with Ack

    Simulator::ScheduleWithContext(1,
                                   Seconds(1400.93),
                                   &LrWpanMac::McpsDataRequest,
                                   endNodeNetDevice->GetMac(),
                                   params4,
                                   p1);

    Simulator::Stop(Seconds(1600));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
