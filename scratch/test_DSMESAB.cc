#include "ns3/core-module.h"
#include <iostream>
#include <vector>
#include <bitset>

using namespace ns3;

// howard: 幫助理解 DSMESAB 實作

int main()
{
    bool m_macCAPReductionFlag = true;
    uint8_t m_macBeaconOrder = 6;
    uint8_t m_macSuperframeOrder = 3;

    std::vector<uint16_t> m_macDSMESAB;
    std::vector<uint8_t> m_macDSMESABCapOff;

    Time::SetResolution(Time::NS);

    if(m_macCAPReductionFlag)
    {
        // initial: 0000000000000000 0000000000000000 0000000000000000 ...
        m_macDSMESAB.resize(static_cast<uint16_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder)), 0);
        std::cout << m_macDSMESAB.size() << std::endl;

        // 第 2 個 Superframe 第 5 個 time slot 有人申請 DSME-GTS
        m_macDSMESAB[2] |= (1 << 5);

        // 第 4 個 Superframe 第 1 個 time slot 有人申請 DSME-GTS
        m_macDSMESAB[4] |= (1 << 1);

        // 第 6 個 Superframe 第 15 個 time slot 有人申請 DSME-GTS
        m_macDSMESAB[6] |= (1 << 15);
            
        for(size_t i = 0; i < m_macDSMESAB.size(); ++i)
        {
            std::cout << "Superframe " << i << " " << std::bitset<16>(m_macDSMESAB[i]) << std::endl;
        }
    }
    else
    {
        m_macDSMESABCapOff.resize(static_cast<uint8_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder)), 0);
        std::cout << m_macDSMESABCapOff.size() << std::endl;

        // 第 6 個 Superframe 第 5 個 time slot 有人申請 DSME-GTS
        m_macDSMESABCapOff[6] |= (1 << 5);

        for(size_t i = 0; i < m_macDSMESABCapOff.size(); ++i)
        {
            std::cout << "Superframe " << i << " " << std::bitset<8>(m_macDSMESABCapOff[i]) << std::endl;
        }
    }

    return 0;
}
