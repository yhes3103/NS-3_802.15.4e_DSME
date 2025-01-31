#include "ns3/hash.h"
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

int AddrConvertToInt(uint8_t* addrBuf)
{ 
    return (int)(addrBuf[0] << 8) | addrBuf[1];
}

int main(int argc, char** argv) {

    // char* buffer = "123";
    // size_t buffer_size = 10;

    // uint32_t buffer_hash = Hash32(buffer, buffer_size);

    // std::string s = "123";
    // uint32_t string_hash = Hash32(s);

    std::vector<Mac16Address> addrArr;

    for(int i = 0;i < 32;i++)
    {
        addrArr.push_back(Mac16Address::Allocate());
        uint8_t buffer16MacAddr[2];
        addrArr[i].CopyTo(buffer16MacAddr);
        std::cout << "addrArr : " << AddrConvertToInt(buffer16MacAddr) << "\n";
        char addrBuf[20];
        sprintf(addrBuf, "%d", AddrConvertToInt(buffer16MacAddr) + 3);
        std::cout << "addrBuf : " << addrBuf << "\n";

        uint32_t buffer_hash = Hash64(addrBuf, 20);        
        std::cout << "buffer_hash : " << buffer_hash % 61 << "\n";
    }





    // std::cout << "string_hash : " << string_hash << "\n";


    Simulator::Stop(Seconds(100));

    Simulator::Run();

    Simulator::Destroy();
}




