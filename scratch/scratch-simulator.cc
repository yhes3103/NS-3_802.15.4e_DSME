#include "ns3/core-module.h"  // 定義各種核心頭文件
#include "ns3/point-to-point-helper.h" // 定義點對點網路的 Topology
#include "ns3/internet-module.h"  // 定義 傳輸層/網路層 的功能，例如：IP address、Routing protocol、TCP/UDP
#include "ns3/applications-module.h"  // 定義應用層的資料傳輸
using namespace ns3;

int main(int argc, char* argv[])
{
    Time::SetResolution(Time::NS);
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);  // 把應用層的 INFO 訊息印出來
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    
    NodeContainer node;  // 節點容器類變數
    node.Create(2);  // 創建兩個節點

    PointToPointHelper point_class_var;   // 助手類變數
    point_class_var.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    point_class_var.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer device;  // 設備容器類變數
    device = point_class_var.Install(node);  // Helper 可建立通道、安裝網路設備
    // 我們後續可能會對網路設備做操作，例如：分配 IP 位址，因此 Install 會有設備容器類的 return
    // 而這個 device 代表兩個節點的網路設備

    InternetStackHelper stack;  // 包含在 internet-module.h 裡面
    stack.Install(node);  // 初始化網路層、傳輸層參數，例如：選 IPv4/IPv6、選 TCP/UDP ...

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");  // 10.0.0.0 為子網路的位址、10.0.0.255 為廣播位址
    Ipv4InterfaceContainer interface = address.Assign(device);  // IP + 子網路遮罩 = interface

    UdpEchoServerHelper echoServer(9);  // Server 使用 port 9 監聽數據

    ApplicationContainer Server_Apps = echoServer.Install(node.Get(0));  // 把應用程式安裝在 node0 (Server)
    Server_Apps.Start(Seconds(2.0));  // Server 會在 1 秒的時候打開，然後使用 port 9 監聽數據，如果收到數據
                                      // echoServer 這個應用程式會 return 一個一模一樣的封包給 Client
    Server_Apps.Stop(Seconds(10.0));  // 當模擬時間超過 10 秒時，則不再監聽數據

    UdpEchoClientHelper echoClient(interface.GetAddress(0), 9);  // 設定 destination 的 IP、子網路遮罩、port number
    echoClient.SetAttribute("MaxPackets", UintegerValue(1));  // 要送幾個封包
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));  // 每幾秒送一次
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));  // 封包的大小

    ApplicationContainer Client_Apps = echoClient.Install(node.Get(1));  // 把應用程式安裝在 node1 (Client)
    Client_Apps.Start(Seconds(2.0));  // Client 會在 2 秒的時候發一個封包給 Server
    Client_Apps.Stop(Seconds(10.0));

    NS_LOG_UNCOND("Scratch Simulator");

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
