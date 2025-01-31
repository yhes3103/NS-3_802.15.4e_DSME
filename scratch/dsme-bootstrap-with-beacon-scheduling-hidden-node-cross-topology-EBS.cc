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

#define DEVICE_CNT 7

/**                  
 *  This program try to simulate beacon scheduling manually.
 *  And it will cause Hidden Node Problem !!
 */

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

                    // Sending association request
                    Simulator::ScheduleNow(&LrWpanMac::MlmeAssociateRequest,
                                           device->GetMac(),
                                           assocParams);
                                           
                    device->GetMac()->SetDescIndexOfAssociatedPan(panDescIndex);
                    // Simulator::ScheduleNow(&LrWpanMac::BeaconScheduling,
                    //                        device->GetMac(),
                    //                        params,
                    //                        device->GetMac()->GetDescIndexOfAssociatedPan());          

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

    // Note : 
    // Set AssociationStatus = ASSOCIATED_EBS here to enable Enhanced beacon scheduling.
    assocRespParams.m_status = LrWpanAssociationStatus::ASSOCIATED_EBS;


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

    // After received the association request, the PAN coordinator TX association response back.
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
                  << "\n"
                  << " (PAN: " << device->GetMac()->GetPanId()
                  << " | CoordShort: " << device->GetMac()->GetCoordShortAddress()
                  << " | CoordExt: " << device->GetMac()->GetCoordExtAddress() << ")"
                  << "\n";

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

    /**
     *  Doing Beacon Scheduling after associate successfully.
     *! MlmeStartRequest will be scheduled after beacon scheduling allocation successful.
     */

    Simulator::ScheduleNow(&LrWpanMac::BeaconScheduling,
                        device->GetMac(),
                        LrWpanBeaconSchedulingPolicy::EBS); // Use Enhanced beacon scheduling here

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


void SetNodePosition(std::vector<Ptr<LrWpanNetDevice>> devVector, std::vector<Ptr<ConstantPositionMobilityModel>> cstPosMobilityModelVector)
{
    /**
     * Set Star topology
     * Use a circle with raduis = 100 m
     * Pan-C  : Origin
     * Others : x-y axis (+-) at distance of 100m
     * 
     *                  ^
     *                  |
     *              (3) O 100             
     *                  |
     *          (4)     |     (1)
     * ----------O------O------O----------->
     *         -100     |(0)  100
     *                  |
     *              (2) O -100
     *                  |
     *                  |
     */ 

    // Set the position of each node
    cstPosMobilityModelVector[0]->SetPosition(Vector(0, 0, 0));    // Set Pan-C at origin
    cstPosMobilityModelVector[1]->SetPosition(Vector(50, 0, 0));  
    cstPosMobilityModelVector[2]->SetPosition(Vector(0, -50, 0));
    cstPosMobilityModelVector[3]->SetPosition(Vector(-50, 0, 0));
    cstPosMobilityModelVector[4]->SetPosition(Vector(0, 50, 0));
    cstPosMobilityModelVector[5]->SetPosition(Vector(30, 0, 0));
    cstPosMobilityModelVector[6]->SetPosition(Vector(0, -30, 0));
    cstPosMobilityModelVector[7]->SetPosition(Vector(30, 0, 0));

    for(int i = 0; i < DEVICE_CNT + 1; i++)
    {
        devVector[i]->GetPhy()->SetMobility(cstPosMobilityModelVector[i]);
    }

}

int main(int argc, char* argv[]) {
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    // LogComponentEnable("DefaultSimulatorImpl", LOG_LEVEL_ALL);
    // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_ALL);

    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    /**
     * Adjust *path loss exponent* parameter to change Transmit range.
     * The path loss exponent, whose value is normally in the range of 2 ~ 4.
     *  
     *  [Parameters]
        Tx Power: 0 dBm
        Receiver Sensitivity: -106.58 dBm
        CCA channel busy condition: Rx power > -96.58 dBm
        Log distance reference loss at 1 m distance for channel 11 (2405 MHz): 40.0641 dB
        Log distance free space path loss exponent: 3

        [Calculate the tansmission TX distance]
        40.0641 + 10 * r * log(d) < 106.58  (r is path loss exponent)

     **/ 

    propModel->SetReference(1.0, 40.0641);  // Reference loss at 1m distance for 2405 MHz (channel 11)
    propModel->SetPathLossExponent(3.9);   // Max TX distance : 50.76 m for r = 3.9
    std::cout << "GetPathLossExponent() :  " << propModel->GetPathLossExponent() << "\n";

    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    std::vector<Ptr<LrWpanNetDevice>> deviceVector;
    std::vector<Ptr<Node>> nodesVector;

    // LrWpanNetDevice & Node Initail setting
    char addrStr[] = "00:00";
    for (int deviceIdx = 0; deviceIdx < DEVICE_CNT + 1; deviceIdx++) 
    {   
        Ptr<Node> node = CreateObject<Node>();
        Ptr<LrWpanNetDevice> device = CreateObject<LrWpanNetDevice>();

        // Set short address of each node
        addrStr[4]++;
        if (addrStr[4] > '9' && addrStr[4] < 'a') 
        {
            addrStr[4] = 'a';
        } else if (addrStr[4] > 'f') 
        {
            addrStr[3]++;
            addrStr[4] = '0';
        }
        device->SetAddress(Mac16Address(addrStr));
        device->SetChannel(channel);

        node->AddDevice(device);
        std::cout << "addrStr = " << addrStr << "\n";
        // Device DSME params setting
        device->GetMac()->SetDsmeModeEnabled();
        device->GetMac()->SetBecomeCoordAfterAssociation(true);
        // Devices callback hooks and 
        device->GetMac()->SetMlmeScanConfirmCallback(MakeBoundCallback(&ScanConfirm, device));
        device->GetMac()->SetMlmeAssociateConfirmCallback(MakeBoundCallback(&AssociateConfirm, device));
        device->GetMac()->SetMlmePollConfirmCallback(MakeBoundCallback(&PollConfirm, device));        

        // Add element into array
        nodesVector.push_back(node);
        deviceVector.push_back(device);
    }

    // PAN Coordinator hooks
    // Choose device[0] as Pan coordinator here
    deviceVector[0]->GetMac()->SetDsmeModeEnabled();
    deviceVector[0]->GetMac()->SetMlmeAssociateIndicationCallback(MakeBoundCallback(&AssociateIndication, deviceVector[0]));
    deviceVector[0]->GetMac()->SetMlmeCommStatusIndicationCallback(MakeBoundCallback(&CommStatusIndication, deviceVector[0]));

    // Set the position for each lr-wpan device
    std::vector<Ptr<ConstantPositionMobilityModel>> constPosMobilityModelVector;
    for(int i = 0; i < DEVICE_CNT + 1; i++)
    {
        Ptr<ConstantPositionMobilityModel> constPosMobilityModel = CreateObject<ConstantPositionMobilityModel>();
        constPosMobilityModelVector.push_back(constPosMobilityModel);
    }
    SetNodePosition(deviceVector, constPosMobilityModelVector);

    MlmeStartRequestParams params;
    params.m_panCoor = true;
    params.m_PanId = 5;
    params.m_bcnOrd = 8; // Beacon Order     (BO)
    params.m_sfrmOrd = 4; // Superframe Order (SO)
    params.m_logCh = 14;

    // Beacon Bitmap
    BeaconBitmap bitmap(0, 1 << (params.m_bcnOrd - params.m_sfrmOrd));
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
    params.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(8); // MO
    params.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);  // Channel divercity
    params.m_dsmeSuperframeSpec.SetCAPReductionFlag(false);   // CAP reduction 

    // Run and set PAN-C manually, bypass the association flow.
    Simulator::ScheduleWithContext(deviceVector[0]->GetNode()->GetId(),
                                   Seconds(2.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   deviceVector[0]->GetMac(),
                                   params);


    /** 
     *  PAN coordinator (PAN 5) is set to channel 14 in beacon mode requests.

     *  Other Coordinator broadcast a single BEACON REQUEST for each channel (11, 12, 13, and 14).
     *  If a coordinator is present and in range, it will respond with a beacon broadcast.
     *  Scan Channels are represented by bits 0-26  (27 LSB)
     *                        ch 14  ch 11
     *                            |  |
     *  0x7800  = 0000000000000000111100000000000
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

    Time scanRequestTime = Seconds(3.0);

    for (int deviceIdx = 1; deviceIdx < DEVICE_CNT + 1; deviceIdx++) 
    {
        Simulator::ScheduleWithContext(deviceVector[deviceIdx]->GetNode()->GetId(),
                                    Seconds(scanRequestTime.GetSeconds()),
                                    &LrWpanMac::MlmeScanRequest,
                                    deviceVector[deviceIdx]->GetMac(),
                                    scanParams);
        deviceVector[deviceIdx]->GetMac()->SetMlmeScanReqTime(scanRequestTime);
    }

    // Synchronization
    MlmeSyncRequestParams syncParams;
    syncParams.m_logCh = 14; 
    syncParams.m_logChPage = 0; 
    syncParams.m_trackBcn = true; 

    for (int deviceIdx = 1; deviceIdx < DEVICE_CNT + 1; deviceIdx++) 
    {
        Simulator::ScheduleWithContext(deviceVector[deviceIdx]->GetNode()->GetId(),
                                    Seconds(1050.001),
                                    &LrWpanMac::MlmeSyncRequest,
                                    deviceVector[deviceIdx]->GetMac(),
                                    syncParams);
    }
 
    Simulator::Stop(Seconds(2000));
    Simulator::Run();

    // Calculating Beacon scheduling allocation successful rate.
    uint32_t totalAllocFailCnt = 0;
    uint32_t totalAllocCtrlPktCnt = 0;
    double successRatio = 0;
    double allocPeriod = 0;

    for(int devIdx = 1; devIdx < DEVICE_CNT + 1; devIdx++)
    {
        totalAllocFailCnt += deviceVector[devIdx]->GetMac()->GetBcnSchedulingFailCnt();
        totalAllocCtrlPktCnt += deviceVector[devIdx]->GetMac()->GetBcnSchedulingCtrlPktCnt();

        double tempTime = deviceVector[devIdx]->GetMac()->GetMlmeStartReqTime().GetSeconds()
                          - deviceVector[devIdx]->GetMac()->GetMlmeScanReqTime().GetSeconds();

        if(tempTime > allocPeriod)
        {
            allocPeriod = tempTime;
        }
    }

    successRatio = 1 - ((double)(totalAllocFailCnt) / (double)(totalAllocFailCnt + DEVICE_CNT));

    std::cout << "[ Beacon scheduling performace ]" << "\n"
              << "- Allocation successful ratio : " << successRatio << "\n" 
              << "- Beacon scheduling total control packet count : " << totalAllocCtrlPktCnt << "\n"
              << "- Beacon scheduling period (mlmeScan ~ mlmeStart) : " << allocPeriod << "\n"
              << std::endl;

    Simulator::Destroy();
    return 0;
}
