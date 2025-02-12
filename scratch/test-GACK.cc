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

#define BO 6
#define SO 3
#define MO 5

#define NUM_COORD 4 // The number of coord, PAN-C need to be included.
#define TREE_DEGREE 3 // The maximum degree of a node in the tree.
#define NUM_RFD (NUM_COORD - 1) * TREE_DEGREE // The number of RFD.
#define ROUND_ROBIN_FACTOR TREE_DEGREE

#define GACK_1_CHANNEL_IDX 3  // GACK1 Channel ID
#define GACK_1_SPF_IDX 1  // GACK1 Superframe ID
#define GACK_1_SLOT_IDX 5  // GACK1 Slot ID

#define GACK_2_CHANNEL_IDX 3  // GACK2 Channel ID
#define GACK_2_SPF_IDX 3  // GACK2 Superframe ID
#define GACK_2_SLOT_IDX 5  // GACK2 Slot ID

// #define BIT(X) (1 << 2^X)
#define BIT(X) (1 << X)

// static double pktRecv = 0;
// static double pktSent = 0;

double pktRecv = 0;
double pktSent = 0;

typedef enum
{
    CHANNEL_ADAPTATION = 0,
    CHANNEL_HOPPING = 1  // 開啟 Channel Hopping
} LrWpanDsmeChannelDiversity;

void GenerateRoundRobinQueue(std::queue<int> *queue, int coorIDx)
{
    for(int deviceIdx = 0; deviceIdx < ROUND_ROBIN_FACTOR; deviceIdx++)
    {
        int childLrWpanDevIdx = deviceIdx + NUM_COORD + (coorIDx * TREE_DEGREE);
        // 一開始 coorIDx = 0
        // 所以 childLrWpanDevIdx = 0 + 4 + 0 = 4
        //                        = 1 + 4 + 0 = 5
        //                        = 2 + 4 + 0 = 6
        // 接著 coorIDx = 1
        // 所以 childLrWpanDevIdx = 0 + 4 + 3 = 7
        //                        = 1 + 4 + 3 = 8
        //                        = 2 + 4 + 3 = 9
        // 接著 coorIDx = 2
        // 所以 childLrWpanDevIdx = 0 + 4 + 6 = 10 0a
        //                        = 1 + 4 + 6 = 11 0b
        //                        = 2 + 4 + 6 = 12 0c
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

    // 如果設備成功傳輸資料，會呼叫 m_mcpsDataConfirmCallback()
    // 而 m_mcpsDataConfirmCallback() 已經和 dataSentMacConfirm() 綁定，因此會進入這個 function
    if(params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
    {
        NS_LOG_UNCOND("**********" << Simulator::Now().As(Time::S)
                                   << " | Transmission successfully sent");
        pktSent += 1;
    }
}

static void dataIndication(McpsDataIndicationParams params, Ptr<Packet> p)
{
    // 如果 Coordinator 收到資料，會進入這個 function
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | Received DATA packet of size " << p -> GetSize());
    pktRecv += 1;
}

int main(int argc, char** argv)
{
    bool verbose = true;

    if(verbose)
    {
        LogComponentEnableAll(LOG_PREFIX_TIME);  // 時間
        LogComponentEnableAll(LOG_PREFIX_FUNC);  // 進入哪個 function
        LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);  // lr-wpan 的 LOG_INFO
        // LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
        // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
        // LogComponentEnable("LrWpanHelper", LOG_LEVEL_ALL);
        // LogComponentEnable("Ping6Application", LOG_LEVEL_INFO);
    }
    
    NodeContainer nodes;  // 創建節點容器類，定義在 node-container.cc 裡面
    nodes.Create(NUM_COORD + NUM_RFD);  // 使用節點容器類的 Create 方法建立 13 個節點 (4 + 9)

    MobilityHelper mobility;  // 創建移動助手類
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");  // 設定所有節點都靜止不動

    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",  // 位置分配器
                                  "X",  // 隨機在 x 軸上灑點，範圍為 0 到 100 公尺
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"),
                                  "Y",  // 隨機在 y 軸上撒點，範圍為 0 到 100 公尺
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));

    mobility.Install(nodes);  // 把移動模型安裝到所有節點上

    LrWpanHelper lrWpanHelper(true);  // true 代表用多傳播路徑模型，可加可不加，因為我們假設所有節點的環境都一樣
    
    // Add and install the LrWpanNetDevice for each node
    // NetDeviceContainer 本質上是一個 vector，裡面存放的是 NetDevice 類型的智慧指標
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);  // 把網卡安裝到所有節點上

    uint16_t panChannelOfs = 0;  // PAC Coordinator 的 Channel Offset
    
    // Setting channel offset array
    // 802.15.4e 2.4Ghz suppose 16 channel
    std::vector<uint16_t> channelOffsets;
    for(int i = 0; i < 16; i++)
    {
        // channelOffsets[16] = [0、1、2、3 ... ]
        // 0 給 PAN Coordinator
        // 1、2、3 給 Coordinator
        // 其他給 device
        channelOffsets.push_back(i);
    }

    uint16_t numOfChannelsSupported = 6;  // 支援的通道數量

    // // In this example, Hopping Sequence is {1, 2, 3, 4, 5, 6}
    // std::vector<uint16_t> hoppingSequence;
    // for(int i = 0; i < numOfChannelsSupported; i++)
    // {
    //     hoppingSequence[i] = i + 1;
    // }

    // callback hook
    McpsDataConfirmCallback cb1;  // SetMcpsDataConfirmCallback 支援的參數類型為 McpsDataConfirmCallback
    cb1 = MakeCallback(&dataSentMacConfirm);  // 用 MakeCallback 將 lr-wpan 的 function 與回呼函式做綁定
    McpsDataIndicationCallback cb2;
    cb2 = MakeCallback(&dataIndication);

    // Dsme Network Parameters
    uint16_t panId = 5;
    uint16_t bcnOrder = BO;
    uint16_t multisuperfrmOrder = MO;
    uint16_t superfrmOrder = SO;
    uint8_t channelNum = 11;  // PAN Coordinator 一開始的 Channel Number
    bool capReduction = false;  // 是否開啟 CAP Reduction

    for(unsigned int i = 0; i < lrwpanDevices.GetN(); i++)
    {
        // GetN() 很多個類都有定義，這邊是定義在 net-device-container.cc 裡面
        // GetN() 代表 DeviceContainer 中的 device 數量

        // Get(i) 很多個類都有定義，這邊是定義在 net-device-container.cc 裡面
        // Get(i) 代表 DeviceContainer 中的第 i 個 device，它是一個 NetDevice 類型的智慧指標
        // GetObject<T>() 定義在 core-module.h 底下的 object.h
        // GetObject<T>() 可以把目標轉換成 T 類型的智慧指標，如果是空指標則返回 nullptr
        Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();

        // GetMac() 定義在 lr-wpan-net-device 裡面
        // GetMac() 會返回 "m_mac" 物件指標，而這個物件指標可以存取 LrWpanMac 中的 public 成員
        // 所以 dev -> GetMac 等價於 m_mac，然後再透過 m_mac 存取 SetMcpsDataConfirmCallback()
        dev->GetMac()->SetMcpsDataConfirmCallback(cb1);
        dev->GetMac()->SetMcpsDataIndicationCallback(cb2);

        // 把支援的通道數量傳到 lr-wpan 裡面
        dev->GetMac()->SetNumOfChannelSupported(numOfChannelsSupported);
        
        // Cap Reduction setting
        dev->GetMac()->SetCAPReduction(capReduction);

        // Set the group ack policy to IEEE 802.15.4e legacy group ack.
        // 傳遞一個列舉變數到 lr-wpan
        dev->GetMac()->SetGroupAckPolicy(LrWpanGroupAckPolicy::GROUP_ACK_LEGACY);
    }

    // Pan Coord mlme-start.request params
    MlmeStartRequestParams startParams;
    startParams.m_panCoor = true;  // 設定是否要變成一個 PAN Coordinator
    startParams.m_PanId = panId;  // 設定 PAN 的識別碼為 5
    startParams.m_bcnOrd = BO;  // 設定 BO = 6
    startParams.m_sfrmOrd = SO;  // 設定 SO = 3
    startParams.m_logCh = channelNum;

    BeaconBitmap bitmap(0, 1 << (BO - SO));  // 直接初始化 BeaconBitmap 的建構函式
    bitmap.SetSDIndex(0);  // PAN-C beacon use SDIDx = 0 (beacon TX at SDIdx 0)
    startParams.m_bcnBitmap = bitmap;

    // DataRequest Data class 存取 HoppingDescriptor struct 裡面的成員
    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = panChannelOfs;  // 0
    hoppingDescriptor.m_channelOfsBitmapLen = 16;

    // resize 是 C++ 的語法，resize(a,b) a 代表將 vector 調整為 a 個元素，多餘的元素則刪掉
    // 如果該 vector 的元素數量小於 a，則會將 vector 擴充到 a，然後剩下的元素用 b 填入
    // m_channelOfsBitmap 一開始有 0 個元素，透過 resize 調整成 1 個元素，並且該元素的值為 1 (0b0000000000000001)
    // 代表第一個 Superframe (SDIDx = 0) 中已經有人傳送 Beacon 了
    hoppingDescriptor.m_channelOfsBitmap.resize(1, BIT(panChannelOfs));

    startParams.m_hoppingDescriptor = hoppingDescriptor;

    DsmeSuperFrameField dsmeSuperframeField;
    dsmeSuperframeField.SetMultiSuperframeOrder(multisuperfrmOrder);
    dsmeSuperframeField.SetChannelDiversityMode(CHANNEL_HOPPING);
    dsmeSuperframeField.SetCAPReductionFlag(capReduction);
    dsmeSuperframeField.SetGACKFlag(true);
    startParams.m_dsmeSuperframeSpec = dsmeSuperframeField;

    /**
     * GroupACK field will be sent at PAN descriptor header IE (aka. a field in Enhanced beacon)
    */
    GroupACK groupAckField;

    //  GACK 1 located at Superframe ID = 1 & slot 5 & channel 3
    groupAckField.SetGACK1ChannelID(GACK_1_CHANNEL_IDX);  // 3
    groupAckField.SetGACK1SuperframeID(GACK_1_SPF_IDX);  // 1
    groupAckField.SetGACK1SlotID(GACK_1_SLOT_IDX);  // 5
    
    //  GACK 2 located at Superframe ID = 3 & slot 5 & channel 3
    groupAckField.SetGACK2ChannelID(GACK_2_CHANNEL_IDX);  // 3 (這裡原本是 GACK_1_CHANNEL_IDX 應該是打錯)
    groupAckField.SetGACK2SuperframeID(GACK_2_SPF_IDX);  // 3
    groupAckField.SetGACK2SlotID(GACK_2_SLOT_IDX);  // 5
    
    // PAN Coordinator 的短位址為 00:01，startParams 就是一些 PAN Coordinator 的參數 (設定給 lr-wpan 助手類)
    // 遍歷所有網路設備，如果是 PAN Coordinator，則進入 MlmeStartRequest() 開始發送 Beacon
    // 如果是其它設備則設定它們要與哪個 PAN Coordinator 做 Beaocn Assoication
    lrWpanHelper.AssociateToBeaconPan(lrwpanDevices
                                        , Mac16Address("00:01")
                                        , startParams);


    // 2nd level Coordinator setting, let other coordinator associate with pan-C
    for(unsigned int i = 1; i < NUM_COORD; ++i)
    {
        MlmeSyncRequestParams syncParams;
        syncParams.m_logChPage = 0;
        syncParams.m_trackBcn = true;

        // Channel 11 監聽 PAN Coordinator 發出的 Beacon
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->TrackCoordinatorBeacon(syncParams);

        MlmeStartRequestParams params;
        params.m_panCoor = false;
        params.m_PanId = panId;
        params.m_bcnOrd = BO;
        params.m_sfrmOrd = SO;

        BeaconBitmap bitmap(0, 1 << (BO - SO));
        bitmap.SetSDIndex(i);                 
        params.m_bcnBitmap = bitmap;

        HoppingDescriptor hoppingDescriptor;
        hoppingDescriptor.m_HoppingSequenceID = 0x00;
        hoppingDescriptor.m_hoppingSeqLen = 0;
        hoppingDescriptor.m_channelOfs = channelOffsets[i];
        hoppingDescriptor.m_channelOfsBitmapLen = 16;
        hoppingDescriptor.m_channelOfsBitmap.resize(1, 1 + (2 << i));   

        params.m_hoppingDescriptor = hoppingDescriptor;

        // Pan Descriptor，設定給 lr-wpan
        PanDescriptor panDescriptor;
        panDescriptor.m_coorPanId = panId;
        panDescriptor.m_coorShortAddr = Mac16Address("00:01");
        panDescriptor.m_logCh = channelNum;
        panDescriptor.m_gACKSpec = groupAckField;

        SuperframeField superframeField;
        superframeField.SetSuperframeOrder(superfrmOrder);
        superframeField.SetBeaconOrder(bcnOrder);
        panDescriptor.m_superframeSpec = superframeField;

        panDescriptor.m_dsmeSuperframeSpec = dsmeSuperframeField;
        panDescriptor.m_bcnBitmap = bitmap;

        // 把自己設定成 Coordinator (可以發出 Beacon)
        lrWpanHelper.CoordBoostrap(lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()
                                    , panDescriptor
                                    , i
                                    , params);
    }
    
    // GTSs setting
    // 每個設備都開啟 DSME-GTS 功能
    for(unsigned int i = 0 ; i < lrwpanDevices.GetN(); i++)
    {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetMcpsDataReqGts(true);
    }

    // 不知道這在幹嘛
    for(unsigned int i = 0 ; i < lrwpanDevices.GetN(); i++)
    {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->ResizeScheduleGTSsEvent(bcnOrder, 
                                                                                              multisuperfrmOrder, 
                                                                                              superfrmOrder);
    }

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
        double slot_0_StartTime = 1.11578;
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
            for(int slotIdx = 0; slotIdx < 15; slotIdx++)
            {
                int childLrWpanDevIdx = childLrWpanDevIdxQueue.front(); // Peek the first element from queue.
                
                if(superframeID == 1 || superframeID == 3)
                {
                    if(slotIdx < 4) // We want to allocate slot 0 ~ slot 3
                    {
                        // Setting the GTS slot for the corresponding RFD.
                        // 在 CFP 中的前 4 個 time slot 分配 DSME-GTS 給 Coorinator 和 device
                        // true 代表是接收端、false 代表是傳送端
                        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(coordLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), true, 1, // Coord for RX
                                                channelOffsets[coordLrWpanDevIdx], superframeID, slotIdx);               

                        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), false, 1, // Devices for TX
                                                channelOffsets[coordLrWpanDevIdx], superframeID, slotIdx); 

                        // Call traffic API , in this case only needs to send one packet.
                        lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childLrWpanDevIdx), lrwpanDevices.Get(coordLrWpanDevIdx)->GetAddress(), pktSize, setTime, 100.0, 10000.0); 
                        
                        // move the first element to the end of the queue and remove it from start
                        // 原本 queue 可能是 [4,5,6]，當 4 傳完資料後會變成 [5,6,4]
                        childLrWpanDevIdxQueue.push(childLrWpanDevIdxQueue.front()); 
                        childLrWpanDevIdxQueue.pop();
                    }
                    else if((superframeID == GACK_1_SPF_IDX) && slotIdx == 14) // Allocate GACK at last slot in the loop
                    {
                        // Setting coordinator
                        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(coordLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), false, 1, // Coord for TX
                                    channelOffsets[coordLrWpanDevIdx], GACK_1_SPF_IDX, GACK_1_SLOT_IDX); 

                        // Setting RFDs
                        int queueSize = childLrWpanDevIdxQueue.size();

                        // Setting the devices GTS
                        // 幫 queue 內的 device 分配一個 Rx DSME-GTS 
                        for(int i = 0; i < queueSize; i++)
                        {
                            childLrWpanDevIdx = childLrWpanDevIdxQueue.front();     
                            lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), true, 1,  // Devices for RX
                                                    channelOffsets[coordLrWpanDevIdx], GACK_1_SPF_IDX, GACK_1_SLOT_IDX); 
                            childLrWpanDevIdxQueue.pop();
                        }
                    }
                    else if ((superframeID == GACK_2_SPF_IDX) && slotIdx == 14)
                    {
                        // Setting coordinator
                        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(coordLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), false, 1, // Coord for TX
                                    channelOffsets[coordLrWpanDevIdx], GACK_2_SPF_IDX, GACK_2_SLOT_IDX); 
                        int queueSize = childLrWpanDevIdxQueue.size();
                        // Setting the devices GTS 
                        for(int i = 0; i < queueSize; i++)
                        {
                            childLrWpanDevIdx = childLrWpanDevIdxQueue.front();     
                            lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childLrWpanDevIdx)->GetObject<LrWpanNetDevice>(), true, 1,  // Devices for RX
                                                    channelOffsets[coordLrWpanDevIdx], GACK_2_SPF_IDX, GACK_2_SLOT_IDX); 
                            childLrWpanDevIdxQueue.pop();
                        }
                    }
                }
                setTime += slotTimeInterval; // add for the next GTS slot.
            }
            setTime += slotTimeInterval; // need to bypass the first slot for beacon before process next superframe.
        }
        ClearRoundRobinQueue(&childLrWpanDevIdxQueue); // Reset (clear) queue.
    }

    // 這應該是產生 wireshark 封包
    // AsciiTraceHelper ascii;
    // lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Gack.tr"));
    // lrWpanHelper.EnablePcapAll(std::string("Gack"), true);

    // Simulator::Stop(Seconds(1.96607));
    Simulator::Stop(Seconds(1.35));
    // Simulator::Stop(Seconds(1.22));
    // Simulator::Stop(Seconds(50));

    Simulator::Run();

    std::cout << "pktSent: " << pktSent << std::endl;
    std::cout << "pktRecv: " << pktRecv << std::endl;
    std::cout << "Delivery ratio: " << pktRecv / pktSent << std::endl;

    double totalSendSize = (double)(pktRecv * (double)pktSize * 8);
    double superframeDuration = (double)(960 * 8 / (double)62500);

    std::cout << "Throughput: " << (double)(totalSendSize / (double)(superframeDuration * 4) / (double)1000) << " (kbits/sec)" << std::endl;

    Simulator::Destroy();
}
