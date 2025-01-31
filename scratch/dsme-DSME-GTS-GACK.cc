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
#include <queue>

using namespace ns3;

#define BO 6
#define SO 3
#define MO 5

#define TREE_DEGREE 3 // The maximum degree of a node in the tree.
#define NUM_COORD 4 // The number of coord, PAN-C need to be included.
#define NUM_RFD (NUM_COORD - 1) * TREE_DEGREE // The number of RFD.
#define ROUND_ROBIN_FACTOR TREE_DEGREE



#define BIT(X) (1 << 2^X)

static double pktRecv = 0;
static double pktSent = 0;
// static double throughput = 0;

typedef enum
{
    CHANNEL_ADAPTATION = 0,
    CHANNEL_HOPPING = 1
} LrWpanDsmeChannelDiversity;

void GenerateRoundRobinQueue(std::queue<int> *queue, int coorIDx)
{
    for(int deviceIdx = 0; deviceIdx < ROUND_ROBIN_FACTOR; deviceIdx++)
    {
        int childLrWpanDevIdx = deviceIdx + NUM_COORD + (coorIDx * TREE_DEGREE);
        queue->push(childLrWpanDevIdx);
    }
}

void ClearRoundRobinQueue(std::queue<int>* queue) 
{

    while (!queue->empty()) queue->pop();
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

static void dataIndication(McpsDataIndicationParams params, Ptr<Packet> p) {
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | The RX higher layer received DATA packet of size " << p->GetSize() << " bytes");
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
        // LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
        // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
        LogComponentEnable("LrWpanHelper", LOG_LEVEL_ALL);
        LogComponentEnable("Ping6Application", LOG_LEVEL_INFO);
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
    McpsDataIndicationCallback cb2;
    cb2 = MakeCallback(&dataIndication);

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
        dev->GetMac()->SetMcpsDataIndicationCallback(cb2);

        dev->GetMac()->SetNumOfChannelSupported(numOfChannelsSupported);
        
        // Cap Reduction setting
        dev->GetMac()->SetCAPReduction(capReduction);

        // Set the group ack policy to GROUP_ACK_DSME_GTS.
        dev->GetMac()->SetGroupAckPolicy(LrWpanGroupAckPolicy::GROUP_ACK_DSME_GTS);
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
    
    // GTSs setting
    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) 
    {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetMcpsDataReqGts(true);
    }

    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) 
    {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->ResizeScheduleGTSsEvent(bcnOrder, 
                                                                                              multisuperfrmOrder, 
                                                                                              superfrmOrder);
    }

    // Setting packet size
    // int pktSize = 10;

    /**
     * Use argv (user input) here to change packet size.
     * example command : ./ns3 run dsme-cap-reduction-EGACK-tree -- 30 > log.log 2>&1
     * 30 means to 30 bytes packet in the simulation.
    */
    int pktSize = atoi(argv[1]);

    /**
     * This is a Queue for round robin purpose. (Stores the RFD lrWpanDeviceIdx)
     * Beacuse the device need to allocate GTS fairly, so here choose round robin algorithm to implement.
    */
    std::queue<int> childLrWpanDevIdxQueue;

    for(int coorIDx = 0; coorIDx < NUM_COORD - 1; coorIDx++) // minus one beacause of PAN-C 
    {
        int coordLrWpanDevIdx = coorIDx + 1; // plus one because the lrWpanDevIDx of PAN-C is 0

        /**
         *! TX & packet traffic settings
        */
        // Setting parameters of sending the data packets.
        // This will continuously TX packet in the setting interval with non-stop transmission , in order to test max throuhgput.

        double slot_0_StartTime = 1.177216;
        // double slot_7_StartTime = 1.16928;
        /**
         * slotTimeInterval (aka. slot time or aBaseSlotDuration) calculated by 
         * aBaseSuperframeDuration * 2^superframeOrder / aNumSuperframeSlots / symbol Rate
         * which can be written as ---> (960*2^SO/16) / 62500
         * In this case , SO = 3,  slotTimeInterval = (960*2^3/16) / 62500 = 0.00768
        */
        double setTime = slot_0_StartTime;
        double slotTimeInterval = 0.00768; // slot time

        for(int superframeID = 1; superframeID < 4; superframeID++)
        {
            /**
             * According to the topology to push the RFD lrWpanDeviceIdx into queue.
            */
            GenerateRoundRobinQueue(&childLrWpanDevIdxQueue, coorIDx);
            for(int slotIdx = 0; slotIdx < 6; slotIdx++)
            {
                int childLrWpanDevIdx = childLrWpanDevIdxQueue.front(); // Peek the first element from queue.
                
                // Slot 5 is GACK slot for DSME-GTS GACK policy
                if(slotIdx == 5)
                {
                    // Setting coordinator
                    lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(coordLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), false, 1, // Coord for TX
                                channelOffsets[coordLrWpanDevIdx], superframeID, 5);  

                    // Setting RFDs
                    int queueSize = childLrWpanDevIdxQueue.size();
                    for(int i = 0; i < queueSize; i++)
                    {
                        // std::cout << "childLrWpanDevIdxQueue.size() " << childLrWpanDevIdxQueue.size() << std::endl;
                        childLrWpanDevIdx = childLrWpanDevIdxQueue.front();   
                        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), true, 1,  // Devices for RX
                                                channelOffsets[coordLrWpanDevIdx], superframeID, 5); 
                        childLrWpanDevIdxQueue.pop();
                    }
                }
                else
                {
                    // Setting the GTS slot for the corresponding RFD.
                    lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(coordLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), true, 1, // Coord for RX
                                            channelOffsets[coordLrWpanDevIdx], superframeID, slotIdx);               

                    lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), false, 1, // Devices for TX
                                            channelOffsets[coordLrWpanDevIdx], superframeID, slotIdx);    

                    // Call traffic API to keep sending packet to test maximum thruoghput.
                    lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childLrWpanDevIdx), lrwpanDevices.Get(coordLrWpanDevIdx)->GetAddress(), pktSize, setTime, 0.005, 0.0001);  
                    // move the first element to the end of the queue and remove it from start.
                    childLrWpanDevIdxQueue.push(childLrWpanDevIdxQueue.front()); 
                    childLrWpanDevIdxQueue.pop();
                }
                setTime += slotTimeInterval; // add for the next GTS slot.
            }
            setTime += (10 * slotTimeInterval); // Because CAP reduction is disabled, we need to calculate next GTS time. Bypass 10 slot time to get the correct GTS time of next superframe.
        }
        ClearRoundRobinQueue(&childLrWpanDevIdxQueue); // Reset (clear) queue.
    }

    AsciiTraceHelper ascii;
    lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Gack.tr"));
    lrWpanHelper.EnablePcapAll(std::string("Gack"), true);
    Simulator::Stop(Seconds(1.469056274));
    // Simulator::Stop(Seconds(1.346176274));
    // Simulator::Stop(Seconds(1.223296274));
    Simulator::Run();

    std::cout << "pktSent: " << pktSent << std::endl;
    std::cout << "pktRecv: " << pktRecv << std::endl;
    std::cout << "Delivery ratio: " << pktRecv / pktSent << std::endl;

    double totalSendSize = (double)(pktRecv * (double)pktSize * 8);
    double superframeDuration = (double)(960 * 8 / (double)62500);

    std::cout << "Throughput: " << (double)(totalSendSize / (double)(superframeDuration * 4) / (double)1000) << " (kbits/sec)" << std::endl;

    Simulator::Destroy();
}
