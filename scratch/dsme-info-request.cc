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

using namespace ns3;

static std::vector<uint16_t> dev0DsmeSAB;
static std::vector<uint16_t> dev1DsmeSAB;
static std::vector<uint16_t> dev2DsmeSAB;

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
    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS) {
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
                              << " "
                              << bitmap
                              << "\n";
                    
                    // DSME-TODO

                    // if (device->GetMac()->IsCoord()) {
                    //     device->GetMac()->SetTimeSlotToSendBcn(8);
                    //     device->GetMac()->SetDescIndexOfAssociatedPan(panDescIndex);
                    // }

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
            std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId()
                      << " [" << device->GetMac()->GetShortAddress() << " | "
                      << device->GetMac()->GetExtendedAddress()
                      << "] MLME-scan.confirm: Beacon not found.\n";
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
        uint8_t buffer64MacAddr[8];
        uint8_t buffer16MacAddr[2];

        params.m_extDevAddr.CopyTo(buffer64MacAddr);
        buffer16MacAddr[1] = buffer64MacAddr[7];
        buffer16MacAddr[0] = buffer64MacAddr[6];

        Mac16Address shortAddr;
        shortAddr.CopyFrom(buffer16MacAddr);
        assocRespParams.m_assocShortAddr = shortAddr;

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

static void PollConfirm(Ptr<LrWpanNetDevice> device, MlmePollConfirmParams params) {
    if (params.m_status == LrWpanMlmePollConfirmStatus::MLMEPOLL_CHANNEL_ACCESS_FAILURE) {
        std::cout
            << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
            << device->GetMac()->GetShortAddress() << " | "
            << device->GetMac()->GetExtendedAddress() << "]"
            << " MLME-poll.confirm:  CHANNEL ACCESS problem when sending a data request command.\n";

    } else if (params.m_status == LrWpanMlmePollConfirmStatus::MLMEPOLL_NO_ACK) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-poll.confirm: Data Request Command FAILED (NO ACK).\n";

    } else if (params.m_status != LrWpanMlmePollConfirmStatus::MLMEPOLL_SUCCESS) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-poll.confirm: Data Request command FAILED.\n";
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
    
    params.m_dsmeSABSpec.Print(std::cout);

    uint32_t maxSuperframeId = device->GetMac()->GetNumOfSuperframesInABeaconInterval() /
                                device->GetMac()->GetNumOfMultisuperframesInABeaconInterval();
    
    // std::cout<< maxSuperframeId << std::endl;

    MlmeDsmeGtsResponseParams respParams;
    respParams.m_devAddr = params.m_devAddr;
    respParams.m_manageType = params.m_manageType;
    respParams.m_direction = params.m_direction;
    respParams.m_prioritizedChAccess = params.m_prioritizedChAccess;
    respParams.m_channelOfs = device->GetMac()->GetChannelOffset();

    if (params.m_manageType == GTS_ALLOCATION) {
        if (params.m_preferredSuperframeID >= maxSuperframeId 
            || params.m_preferredSlotID >= 16) {
            respParams.m_status = MLMEDSMEGTS_INVALID_PARAMETER;

        } else {
            std::cout << dev0DsmeSAB[params.m_preferredSuperframeID] << std::endl;


            dev1DsmeSAB[params.m_preferredSuperframeID] = 0b0000000000001110;

            respParams.m_dsmeSABSpec.setCAPReduction(device->GetMac()->isCAPReductionOn());
            respParams.m_dsmeSABSpec.setSABSubBlkLen(1);
            respParams.m_dsmeSABSpec.setSABSubBlkIdx(params.m_preferredSuperframeID);
            respParams.m_dsmeSABSpec.setSABSubBlk(dev1DsmeSAB[params.m_preferredSuperframeID]);

            respParams.m_status = MLMEDSMEGTS_SUCCESS;
        }

    } else if (params.m_manageType == GTS_DEALLOCATION) {
        dev1DsmeSAB[params.m_dsmeSABSpec.GetSABSubBlkIdx()] = 0b0000000000001110;

        respParams.m_dsmeSABSpec.setCAPReduction(params.m_dsmeSABSpec.isCAPReduction());
        respParams.m_dsmeSABSpec.setSABSubBlkLen(params.m_dsmeSABSpec.GetSABSubBlkLen());
        respParams.m_dsmeSABSpec.setSABSubBlkIdx(params.m_dsmeSABSpec.GetSABSubBlkIdx());
        respParams.m_dsmeSABSpec.setSABSubBlk(params.m_dsmeSABSpec.GetSABSubBlk()[0]);

        respParams.m_status = MLMEDSMEGTS_SUCCESS;
       
    }

    if (params.m_manageType == GTS_EXPIRATION) {
        MlmeDsmeGtsRequestParams reqParams;
        reqParams.m_devAddr = params.m_devAddr;
        reqParams.m_manageType = GTS_DEALLOCATION;
        reqParams.m_direction = params.m_direction;
        reqParams.m_prioritizedChAccess = params.m_prioritizedChAccess;
        reqParams.m_dsmeSABSpec = std::move(params.m_dsmeSABSpec);

        Simulator::ScheduleNow(&LrWpanMac::MlmeDsmeGtsRequest,
                                device->GetMac(),
                                reqParams);
                                
    } else {
        Simulator::ScheduleNow(&LrWpanMac::MlmeDsmeGtsResponse,
                            device->GetMac(),
                            respParams);
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
        
        dev1DsmeSAB[params.m_dsmeSABSpec.GetSABSubBlkIdx()] = params.m_dsmeSABSpec
                                                                    .GetSABSubBlk()[params.m_dsmeSABSpec.GetSABSubBlkLen() - 1];
        
        // std::cout << std::bitset<16>(dev1DsmeSAB[params.m_dsmeSABSpec.GetSABSubBlkIdx()])
        //           << std::endl;

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

    } else if (params.m_status ==MLMEDSMEGTS_REQ_NO_ACK) {
        std::cout << Simulator::Now().As(Time::S) << " Coordinator " << device->GetNode()->GetId()
              << " [" << device->GetMac()->GetShortAddress() << " | "
              << device->GetMac()->GetExtendedAddress() << "]"
              << " MLME-DSME-GTS.confirm: Gts request failed due to No Ack received " 
              << "\n";
    }
}

static void MlmeDsmeInfoIndication(Ptr<LrWpanNetDevice> device, MlmeDsmeInfoIndicationParams params) {
    std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-DSME-INFO.indication: Dsme Info Request Command Received\n"
                  << " From: " << params.m_devAddr 
                  << " Type: " << (uint32_t) params.m_info << std::endl;
}

static void MlmeDsmeInfoConfirm(Ptr<LrWpanNetDevice> device, MlmeDsmeInfoConfirmParams params) {
    if (params.m_status == LrWpanMlmeDsmeInfoResponseStatus::MLMEDSMEINFO_SUCCESS) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-DSME-INFO.confirm: Dsme Info Request Command Success.\n";
        
        if (params.m_info == InfoType::MLMEDSMEINFO_TIMESTAMP) {
            std::cout << "Timestamp: " << params.m_timestamp
                        << " Superframe ID: " << params.m_superframeID
                        << " Slot ID: " << (uint16_t) params.m_slotID << std::endl;

        } else if (params.m_info == InfoType::MLMEDSMEINFO_DSME_SAB_SPECIFICATION) {
            params.m_dsmeSABSpec.Print(std::cout);

        } else if (params.m_info == InfoType::MLMEDSMEINFO_DSME_PAN_DESCRIPTOR) {
            params.m_panDescriptor.Print(std::cout);
        }

    } else if (params.m_status == LrWpanMlmeDsmeInfoResponseStatus::MLMEDSMEINFO_NO_ACK) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-DSME-INFO.confirm: Dsme Info Request Command FAILED (NO ACK).\n";

    } else if (params.m_status == LrWpanMlmeDsmeInfoResponseStatus::MLMEDSMEINFO_NO_DATA) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-DSME-INFO.confirm: No Dsme Info Reply Command Received.\n";

    } else if (params.m_status == LrWpanMlmeDsmeInfoResponseStatus::MLMEDSMEINFO_INVALID_PARAMETER) {
        std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId() << " ["
                  << device->GetMac()->GetShortAddress() << " | "
                  << device->GetMac()->GetExtendedAddress() << "]"
                  << " MLME-DSME-INFO.confirm: The parameter provided to Dsme Info Request Command is invalid.\n";
    }
}

int main(int argc, char* argv[]) {
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
    
    // Create 1 PAN coordinator node, and 1 normal coordinator
    // 1 end device
    Ptr<Node> coord = CreateObject<Node>();
    Ptr<Node> secondCoord = CreateObject<Node>();
    Ptr<Node> endNode = CreateObject<Node>();

    Ptr<LrWpanNetDevice> coordNetDevice = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> secondCoordNetDevice = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> endNodeNetDevice = CreateObject<LrWpanNetDevice>();

    coordNetDevice->SetAddress(Mac16Address("00:01"));
    secondCoordNetDevice->SetAddress(Mac16Address("00:02"));
    endNodeNetDevice->SetAddress(Mac16Address("00:03"));

    LrWpanHelper lrWpanHelper;

    lrWpanHelper.EnablePcap(std::string("dsme-info-request-coordNetDevice.pcap"), coordNetDevice, true, true);
    lrWpanHelper.EnablePcap(std::string("dsme-info-request-secondCoordNetDevice.pcap"), secondCoordNetDevice, true, true);
    lrWpanHelper.EnablePcap(std::string("dsme-info-request-endNodeNetDevice.pcap"), endNodeNetDevice, true, true);

    // Configure Spectrum channel
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    coordNetDevice->SetChannel(channel);
    secondCoordNetDevice->SetChannel(channel);
    endNodeNetDevice->SetChannel(channel);

    coord->AddDevice(coordNetDevice);
    secondCoord->AddDevice(secondCoordNetDevice);
    endNode->AddDevice(endNodeNetDevice);

    // Mobility
    Ptr<ConstantPositionMobilityModel> coordMobility =
        CreateObject<ConstantPositionMobilityModel>();
    coordMobility->SetPosition(Vector(0, 0, 0));
    coordNetDevice->GetPhy()->SetMobility(coordMobility);

    Ptr<ConstantPositionMobilityModel> secondCoordMobility =
        CreateObject<ConstantPositionMobilityModel>();
    secondCoordMobility->SetPosition(Vector(100, 0, 0));
    secondCoordNetDevice->GetPhy()->SetMobility(secondCoordMobility);

    Ptr<ConstantPositionMobilityModel> endNodeMobility =
        CreateObject<ConstantPositionMobilityModel>();
    endNodeMobility->SetPosition(Vector(80, 50, 0));
    endNodeNetDevice->GetPhy()->SetMobility(endNodeMobility);

    // Pan Coordinator hooks
    dev0DsmeSAB.resize(static_cast<uint64_t>(1 << (14 - 4)), 0);

    coordNetDevice->GetMac()->SetDsmeModeEnabled();
    coordNetDevice->GetMac()->SetMlmeAssociateIndicationCallback(MakeBoundCallback(&AssociateIndication, coordNetDevice));
    coordNetDevice->GetMac()->SetMlmeCommStatusIndicationCallback(MakeBoundCallback(&CommStatusIndication, coordNetDevice));

    coordNetDevice->GetMac()->SetNumOfChannelSupported(15);

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

    // Second coordinator hooks & MAC MLME-scan primitive set
    dev1DsmeSAB.resize(static_cast<uint64_t>(1 << (12 - 4)), 0);
    coordNetDevice->GetMac()->ResizeScheduleGTSsEvent(12, 10, 6);
    secondCoordNetDevice->GetMac()->ResizeScheduleGTSsEvent(12, 10, 6);
    
    secondCoordNetDevice->GetMac()->SetDsmeModeEnabled();
    // secondCoordNetDevice->GetMac()->SetAsCoordinator();
    secondCoordNetDevice->GetMac()->SetBecomeCoordAfterAssociation(true);

    secondCoordNetDevice->GetMac()->SetChannelHoppingEnabled();
    secondCoordNetDevice->GetMac()->SetNumOfChannelSupported(15);

    secondCoordNetDevice->GetMac()->SetMlmeScanConfirmCallback(MakeBoundCallback(&ScanConfirm
                                                                                , secondCoordNetDevice));

    secondCoordNetDevice->GetMac()->SetMlmeAssociateConfirmCallback(MakeBoundCallback(&AssociateConfirm
                                                                                        , secondCoordNetDevice));

    secondCoordNetDevice->GetMac()->SetMlmeAssociateIndicationCallback(MakeBoundCallback(&AssociateIndication
                                                                                        , secondCoordNetDevice));

    secondCoordNetDevice->GetMac()->SetMlmeCommStatusIndicationCallback(MakeBoundCallback(&CommStatusIndication
                                                                                        , secondCoordNetDevice));

    secondCoordNetDevice->GetMac()->SetMlmePollConfirmCallback(MakeBoundCallback(&PollConfirm
                                                                                , secondCoordNetDevice));

    secondCoordNetDevice->GetMac()->SetMlmeDsmeInfoIndicationCallback(MakeBoundCallback(&MlmeDsmeInfoIndication
                                                                                        , secondCoordNetDevice));

    secondCoordNetDevice->GetMac()->SetMlmeDsmeGtsIndicationCallback(MakeBoundCallback(&MlmeDsmeGtsIndication
                                                                                        , secondCoordNetDevice));

    
    secondCoordNetDevice->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&DataIndicationCoordinator));

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

    Simulator::ScheduleWithContext(secondCoordNetDevice->GetNode()->GetId(),
                                   Seconds(3.0),
                                   &LrWpanMac::MlmeScanRequest,
                                   secondCoordNetDevice->GetMac(),
                                   scanParams);

    // Synchronization
    MlmeSyncRequestParams syncParams;
    syncParams.m_logCh = 14; 
    syncParams.m_logChPage = 0; 
    syncParams.m_trackBcn = true; 

    Simulator::ScheduleWithContext(secondCoordNetDevice->GetNode()->GetId(),
                                   Seconds(1010.001),
                                   &LrWpanMac::MlmeSyncRequest,
                                   secondCoordNetDevice->GetMac(),
                                   syncParams);  

    // PAN coordinator N1 (PAN 5) is set to channel 14 in beacon mode
    // requests.
    MlmeStartRequestParams params2;
    params2.m_panCoor = false;
    params2.m_PanId = 5;
    params2.m_bcnOrd = 6;
    params2.m_sfrmOrd = 3;
    params2.m_logCh = 14;

    // Beacon Bitmap
    BeaconBitmap bitmap2(0, 1 << (6 - 3));
    bitmap2.SetSDIndex(8);                  // SD = 8 目前占用
    params2.m_bcnBitmap = bitmap2;

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor2;
    hoppingDescriptor2.m_HoppingSequenceID = 0x00;
    hoppingDescriptor2.m_hoppingSeqLen = 0;
    hoppingDescriptor2.m_channelOfs = 5;
    hoppingDescriptor2.m_channelOfsBitmapLen = 16;
    hoppingDescriptor2.m_channelOfsBitmap.resize(1, 34);    // offset = 1, 5 目前占用

    params2.m_hoppingDescriptor = hoppingDescriptor2;

    // DSME
    params2.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(6);
    params2.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);
    params2.m_dsmeSuperframeSpec.SetCAPReductionFlag(true);

    Simulator::ScheduleWithContext(secondCoordNetDevice->GetNode()->GetId(),
                                   Seconds(1050.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   secondCoordNetDevice->GetMac(),
                                   params2);
    
    // end Node hooks
    endNodeNetDevice->GetMac()->SetDsmeModeEnabled();
    endNodeNetDevice->GetMac()->SetChannelHoppingEnabled();
    endNodeNetDevice->GetMac()->SetNumOfChannelSupported(15);

    endNodeNetDevice->GetMac()->SetMlmeScanConfirmCallback(MakeBoundCallback(&ScanConfirm
                                                                            , endNodeNetDevice));

    endNodeNetDevice->GetMac()->SetMlmeAssociateConfirmCallback(MakeBoundCallback(&AssociateConfirm
                                                                                , endNodeNetDevice));

    endNodeNetDevice->GetMac()->SetMlmeDsmeInfoConfirmCallback(MakeBoundCallback(&MlmeDsmeInfoConfirm
                                                                                , endNodeNetDevice));

    endNodeNetDevice->GetMac()->SetMlmeDsmeGtsConfirmCallback(MakeBoundCallback(&MlmeDsmeGtsConfirm
                                                                                , endNodeNetDevice));

    endNodeNetDevice->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&DataIndication));
    endNodeNetDevice->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&TransEndIndication));

    // Scanning
    MlmeScanRequestParams scanParams2;
    scanParams2.m_scanType = MLMESCAN_ENHANCED_ACTIVE_SCAN;
    scanParams2.m_scanChannels = 0x7800;
    scanParams2.m_scanDuration = 14;
    scanParams2.m_chPage = 0;

    // DMSE 
    scanParams2.m_linkQualityScan = false;
    scanParams2.m_frameCtrlOptions.resize(3);
    scanParams2.m_frameCtrlOptions[0] = false;    // PAN_ID_SUPPRESSED
    scanParams2.m_frameCtrlOptions[1] = false;    // IES_INCLUDED
    scanParams2.m_frameCtrlOptions[2] = false;    // SEQ_#_SUPPRESSED

    Simulator::ScheduleWithContext(endNodeNetDevice->GetNode()->GetId(),
                                   Seconds(1100.0),
                                   &LrWpanMac::MlmeScanRequest,
                                   endNodeNetDevice->GetMac(),
                                   scanParams2);

    // Synchronization
    MlmeSyncRequestParams syncParams2;
    syncParams2.m_logCh = 14; 
    syncParams2.m_logChPage = 0; 
    syncParams2.m_trackBcn = true; 

    Simulator::ScheduleWithContext(endNodeNetDevice->GetNode()->GetId(),
                                   Seconds(2154.002),
                                   &LrWpanMac::MlmeSyncRequest,
                                   endNodeNetDevice->GetMac(),
                                   syncParams2); 

    /////////////////////// Dsme SAB setting /////////////////////////
    endNodeNetDevice->GetMac()->ResizeMacDSMESAB(true, 12, 4);
    dev2DsmeSAB.resize(static_cast<uint64_t>(1 << (12 - 4)), 0);
    endNodeNetDevice->GetMac()->ResizeScheduleGTSsEvent(12, 10, 6);

    // GTS handshake with 2nd coordinator
    MlmeDsmeGtsRequestParams gtsReqParams;
    gtsReqParams.m_devAddr = Mac16Address("00:02");
    gtsReqParams.m_manageType = GTS_ALLOCATION;
    gtsReqParams.m_direction = 0x00;
    gtsReqParams.m_prioritizedChAccess = 0x01;
    gtsReqParams.m_numSlot = 3;
    gtsReqParams.m_preferredSuperframeID = 2;
    gtsReqParams.m_preferredSlotID = 1;
    gtsReqParams.m_dsmeSABSpec.setCAPReduction(true);
    gtsReqParams.m_dsmeSABSpec.setSABSubBlkLen(1);
    gtsReqParams.m_dsmeSABSpec.setSABSubBlkIdx(0);
    gtsReqParams.m_dsmeSABSpec.setSABSubBlk(dev2DsmeSAB[0]);  

    Simulator::ScheduleWithContext(1,
                                   Seconds(2250.0),
                                   &LrWpanMac::MlmeDsmeGtsRequest,
                                   endNodeNetDevice->GetMac(),
                                   gtsReqParams);

    // Dsme Info Request
    MlmeDsmeInfoRequestParams infoReqParams;
    infoReqParams.m_dstAddrMode = SHORT_ADDR;
    infoReqParams.m_dstShortAddr = Mac16Address("00:02");
    infoReqParams.m_info = MLMEDSMEINFO_TIMESTAMP;
    infoReqParams.m_dsmeSABSubBlkLen = 5;
    infoReqParams.m_dsmeSABSubBlkIdx = 0;

    Simulator::ScheduleWithContext(endNodeNetDevice->GetNode()->GetId(),
                                   Seconds(2329.841536315),
                                   &LrWpanMac::MlmeDsmeInfoRequest,
                                   endNodeNetDevice->GetMac(),
                                   infoReqParams);

    Ptr<Packet> p1 = Create<Packet>(5);
    McpsDataRequestParams params3;
    params3.m_dstPanId = 5;
    params3.m_srcAddrMode = SHORT_ADDR;
    params3.m_dstAddrMode = SHORT_ADDR;
    params3.m_dstAddr = Mac16Address("00:02");
    params3.m_msduHandle = 0;
    params3.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack
    params3.m_txOptions |= TX_OPTION_GTS;

    Simulator::ScheduleWithContext(1,
                                   Seconds(2395.368),
                                   &LrWpanMac::McpsDataRequest,
                                   endNodeNetDevice->GetMac(),
                                   params3,
                                   p1);

    ////////////////////////////////////////// Deallocation //////////////////////////////////////////
    dev2DsmeSAB.resize(static_cast<uint64_t>(1 << (12 - 4)), 0);
    dev2DsmeSAB[gtsReqParams.m_preferredSuperframeID] = 0b0000000000001110;

    MlmeDsmeGtsRequestParams params4;
    params4.m_devAddr = Mac16Address("00:02");
    params4.m_manageType = GTS_DEALLOCATION;
    params4.m_direction = 0x00;
    params4.m_prioritizedChAccess = 0x01;
    params4.m_dsmeSABSpec.setCAPReduction(true);
    params4.m_dsmeSABSpec.setSABSubBlkLen(1);
    params4.m_dsmeSABSpec.setSABSubBlkIdx(gtsReqParams.m_preferredSuperframeID);
    params4.m_dsmeSABSpec.setSABSubBlk(dev2DsmeSAB[gtsReqParams.m_preferredSuperframeID]);  

    Simulator::ScheduleWithContext(1,
                                   Seconds(2450.0),
                                   &LrWpanMac::MlmeDsmeGtsRequest,
                                   endNodeNetDevice->GetMac(),
                                   params4);

    // Dsme Info Request
    infoReqParams.m_dstAddrMode = SHORT_ADDR;
    infoReqParams.m_dstShortAddr = Mac16Address("00:02");
    infoReqParams.m_info = MLMEDSMEINFO_DSME_SAB_SPECIFICATION;
    infoReqParams.m_dsmeSABSubBlkLen = 5;
    infoReqParams.m_dsmeSABSubBlkIdx = 0;

    Simulator::ScheduleWithContext(endNodeNetDevice->GetNode()->GetId(),
                                   Seconds(2456.00),
                                   &LrWpanMac::MlmeDsmeInfoRequest,
                                   endNodeNetDevice->GetMac(),
                                   infoReqParams);                               

    Simulator::Stop(Seconds(2600));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
