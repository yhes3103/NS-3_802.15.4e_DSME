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

#define SLOT_VACANT 0

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
                                           
                    
                    Simulator::ScheduleNow(&LrWpanMac::BeaconScheduling,
                                            device->GetMac(),
                                            LrWpanBeaconSchedulingPolicy::LEGACY);
                    
                    

                    
                    // // pick a vacant beacon slot
                    // BeaconBitmap bitmap(0, 1 << (params.m_panDescList[panDescIndex].m_superframeSpec.GetBeaconOrder() 
                    //                              - params.m_panDescList[panDescIndex].m_superframeSpec.GetFrameOrder()));
                    
                    // for (uint32_t i = 0; i < params.m_panDescList.size(); i++) {
                    //     if (params.m_panDescList[i].m_coorPanId == params.m_panDescList[panDescIndex].m_coorPanId) {
                    //         bitmap = bitmap | params.m_panDescList[i].m_bcnBitmap;
                    //     }
                    // }

                    // std::cout << "Beacon bitmap infos in Pan " << params.m_panDescList[panDescIndex].m_coorPanId << " : "
                    //           << bitmap
                    //           << "\n";
                    
                    // //!< Set what timeslot to TX beacon (Beacon scheduling)
                    // // TODO : Need to peek current beacon bitmap in order to choose a vacant time slot for transmitting a beacon.   
                    
                    // //device->GetMac()->SetAsCoordinator(); // TODO : set coord here will assert, need to fix or workaround

                    // uint8_t vacantTimeSlotToSendBcn;
                    // // random every time
                    // srand(time(0)); 
                    // // vacantTimeSlotToSendBcn = rand() % (1 <<  ((uint32_t)params.m_panDescList[panDescIndex].m_superframeSpec.GetBeaconOrder() 
                    // //                                          - (uint32_t)params.m_panDescList[panDescIndex].m_superframeSpec.GetFrameOrder()));

                    // vacantTimeSlotToSendBcn = rand() % (8) +1; 

                    // std::cout << "BO = " << (uint32_t)params.m_panDescList[panDescIndex].m_superframeSpec.GetBeaconOrder() << " ,"
                    //           << "SO = "   << (uint32_t)params.m_panDescList[panDescIndex].m_superframeSpec.GetFrameOrder() << "\n";
                    // std::cout << "Doing beacon scheduling now , choose vacant timeslot [" << (uint32_t)vacantTimeSlotToSendBcn << "]" << "\n";
                    
                    // // Check timeslot is vacant or not
                    // std::vector<uint16_t> currentSDBitmap = bitmap.GetSDBitmap();
                    // if(currentSDBitmap[vacantTimeSlotToSendBcn] == SLOT_VACANT)
                    // {
                    //     device->GetMac()->SetTimeSlotToSendBcn(vacantTimeSlotToSendBcn);
                    // }

                    // device->GetMac()->SetDescIndexOfAssociatedPan(panDescIndex);

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

int main(int argc, char* argv[]) {
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    // LogComponentEnable("DefaultSimulatorImpl", LOG_LEVEL_ALL);
    // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_ALL);

/**                  
 *  This program try to simulate beacon scheduling manually, there are two coordinators in the simulation.
 *  1st coordinator [00:01] already running in PAN.
 *  2nd coordinator [00:02](the other one) will try to join the PAN by SCAN.request, SYNC.request and Start request. 
 * 
 *                          Topology
 * 
 *      [00:01]                                 [00:02]                                
 *  PAN Coordinator 1st (PAN: 5)        PAN Coordinator 2nd (PAN: 7)                       
 *       |----------------- 100 m -----------------|
 *  Channel 14                          (Active Scan channels 11-14)     
 * 
 */
    // Create 1 PAN coordinator node, and 1 end device
    Ptr<Node> coord = CreateObject<Node>();
    Ptr<Node> secondCoord = CreateObject<Node>();

    Ptr<LrWpanNetDevice> coordNetDevice = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> secondCoordNetDevice = CreateObject<LrWpanNetDevice>();

    coordNetDevice->SetAddress(Mac16Address("00:01"));
    secondCoordNetDevice->SetAddress(Mac16Address("00:02"));

    LrWpanHelper lrWpanHelper;
    
    lrWpanHelper.EnablePcap(std::string("dsme-bootstrap-with-beacon-select-coordNetDevice.pcap"), coordNetDevice, true, true);
    lrWpanHelper.EnablePcap(std::string("dsme-bootstrap-with-beacon-select-secondCoordNetDevice.pcap"), secondCoordNetDevice, true, true);

    // Configure Spectrum channel
    // Maybe can try to modify LossModel to the rangeLossmodel to setting the TX range
    
    // [Example]
    // TypeId
    // RangePropagationLossModel::GetTypeId()
    // {
    //     static TypeId tid = TypeId("ns3::RangePropagationLossModel")
    //                             .SetParent<PropagationLossModel>()
    //                             .SetGroupName("Propagation")
    //                             .AddConstructor<RangePropagationLossModel>()
    //                             .AddAttribute("MaxRange",
    //                                           "Maximum Transmission Range (meters)",
    //                                           DoubleValue(250),
    //                                           MakeDoubleAccessor(&RangePropagationLossModel::m_range),
    //                                           MakeDoubleChecker<double>());
    //     return tid;
    // }

    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    coordNetDevice->SetChannel(channel);
    secondCoordNetDevice->SetChannel(channel);

    coord->AddDevice(coordNetDevice);
    secondCoord->AddDevice(secondCoordNetDevice);

    // Mobility
    Ptr<ConstantPositionMobilityModel> coordMobility =
        CreateObject<ConstantPositionMobilityModel>();
    coordMobility->SetPosition(Vector(0, 0, 0));
    coordNetDevice->GetPhy()->SetMobility(coordMobility);

    Ptr<ConstantPositionMobilityModel> secondCoordMobility =
        CreateObject<ConstantPositionMobilityModel>();
    secondCoordMobility->SetPosition(Vector(100, 0, 0));
    secondCoordNetDevice->GetPhy()->SetMobility(secondCoordMobility);

    // Devices hooks & MAC MLME-scan primitive set
    secondCoordNetDevice->GetMac()->SetDsmeModeEnabled();
    // secondCoordNetDevice->GetMac()->SetAsCoordinator();
    secondCoordNetDevice->GetMac()->SetBecomeCoordAfterAssociation(true);
    
    secondCoordNetDevice->GetMac()->SetMlmeScanConfirmCallback(MakeBoundCallback(&ScanConfirm, secondCoordNetDevice));
    secondCoordNetDevice->GetMac()->SetMlmeAssociateConfirmCallback(MakeBoundCallback(&AssociateConfirm, secondCoordNetDevice));
    secondCoordNetDevice->GetMac()->SetMlmePollConfirmCallback(MakeBoundCallback(&PollConfirm, secondCoordNetDevice));

    /** 
     *      [00:01]                                 [00:02]                                
     *  PAN Coordinator 1st (PAN: 5)        PAN Coordinator 2nd (PAN: 7)                       
     *       |----------------- 100 m -----------------|
     *  Channel 14                          (Active Scan channels 11-14)     
     * 
     * Start Setting PAN coordinator 1st (PAN 5)
     * PAN coordinator 1st (PAN 5) is set to channel 14 in beacon mode requests.
     */

    // Coordinator hooks
    coordNetDevice->GetMac()->SetDsmeModeEnabled();
    coordNetDevice->GetMac()->SetMlmeAssociateIndicationCallback(MakeBoundCallback(&AssociateIndication, coordNetDevice));
    coordNetDevice->GetMac()->SetMlmeCommStatusIndicationCallback(MakeBoundCallback(&CommStatusIndication, coordNetDevice));
    
    MlmeStartRequestParams params;
    params.m_panCoor = true;
    params.m_PanId = 5;
    params.m_bcnOrd = 12;
    params.m_sfrmOrd = 4;
    params.m_logCh = 14;

    // Beacon Bitmap
    BeaconBitmap bitmap(0, 1 << (12 - 4));
    bitmap.SetSDBitmap(0);                  // SD = 0 , set beacon send in SDindex = 0
    params.m_bcnBitmap = bitmap;

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = 1;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, 2);    // offset = 1 目前占用

    params.m_hoppingDescriptor = hoppingDescriptor;

    // DSME SuperframeSpec
    params.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(10); // MO
    params.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);  // Channel divercity
    params.m_dsmeSuperframeSpec.SetCAPReductionFlag(true);   // CAP reduction 

    Simulator::ScheduleWithContext(coordNetDevice->GetNode()->GetId(),
                                   Seconds(2.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   coordNetDevice->GetMac(),
                                   params);



    /** 
     *      [00:01]                                 [00:02]                                
     *  PAN Coordinator 1st (PAN: 5)         Coordinator 2nd (PAN: 5)                       
     *       |----------------- 100 m -----------------|
     *  Channel 14                          (Active Scan channels 11-14)     
     * 

     * Coordinator 2nd broadcast a single BEACON REQUEST for each channel (11, 12, 13, and 14).
     * If a coordinator is present and in range, it will respond with a beacon broadcast.
     * Scan Channels are represented by bits 0-26  (27 LSB)
     *                            ch 14  
     *                              |  
     * 0x7800  = 0000000000000000111100000000000
     */

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
                                   Seconds(1050.001),
                                   &LrWpanMac::MlmeSyncRequest,
                                   secondCoordNetDevice->GetMac(),
                                   syncParams);  

    // PAN coordinator N1 (PAN 5) is set to channel 14 in beacon mode
    // requests.
    MlmeStartRequestParams params2;
    params2.m_panCoor = false;
    params2.m_PanId = 5;

    //? Note : Some params no need to set here because the coordinator has associated with the PAN-C, the params will extract at EndStartRequest().

    // params2.m_bcnOrd = 6;
    // params2.m_sfrmOrd = 3;
    params2.m_logCh = 14;

    // // Beacon Bitmap
    // BeaconBitmap bitmap2(0, 1 << (6 - 3));
    // bitmap2.SetSDIndex(8);                  // SD = 8 目前占用
    // params2.m_bcnBitmap = bitmap2;

    // Hopping Descriptor
    // HoppingDescriptor hoppingDescriptor2;
    // hoppingDescriptor2.m_HoppingSequenceID = 0x00;
    // hoppingDescriptor2.m_hoppingSeqLen = 0;
    // hoppingDescriptor2.m_channelOfs = 5;
    // hoppingDescriptor2.m_channelOfsBitmapLen = 16;
    // hoppingDescriptor2.m_channelOfsBitmap.resize(1, 34);    // offset = 1, 5 目前占用

    // params2.m_hoppingDescriptor = hoppingDescriptor2;

    // // DSME
    // params2.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(6);
    // params2.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);
    // params2.m_dsmeSuperframeSpec.SetCAPReductionFlag(false);

    Simulator::ScheduleWithContext(secondCoordNetDevice->GetNode()->GetId(),
                                   Seconds(1100.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   secondCoordNetDevice->GetMac(),
                                   params2);

    Simulator::Stop(Seconds(1300));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
