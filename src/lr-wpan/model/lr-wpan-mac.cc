#include "lr-wpan-mac.h"
#include "lr-wpan-csmaca.h"
#include "lr-wpan-mac-header.h"
#include "lr-wpan-mac-pl-headers.h"
#include "lr-wpan-mac-trailer.h"
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <random>

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT std::clog << "[address " << m_shortAddress << "] ";

#define MCPS_DATA_SENDING_LOG 0

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LrWpanMac");
NS_OBJECT_ENSURE_REGISTERED(LrWpanMac);

TypeId
LrWpanMac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LrWpanMac")
            .SetParent<Object>()
            .SetGroupName("LrWpan")
            .AddConstructor<LrWpanMac>()
            .AddAttribute("PanId",
                          "16-bit identifier of the associated PAN",
                          UintegerValue(),
                          MakeUintegerAccessor(&LrWpanMac::m_macPanId),
                          MakeUintegerChecker<uint16_t>())
            .AddTraceSource("MacTxEnqueue",
                            "Trace source indicating a packet has been "
                            "enqueued in the transaction queue",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macTxEnqueueTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacTxDequeue",
                            "Trace source indicating a packet has was "
                            "dequeued from the transaction queue",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macTxDequeueTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacIndTxEnqueue",
                            "Trace source indicating a packet has been "
                            "enqueued in the indirect transaction queue",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macIndTxEnqueueTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacIndTxDequeue",
                            "Trace source indicating a packet has was "
                            "dequeued from the indirect transaction queue",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macIndTxDequeueTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacTx",
                            "Trace source indicating a packet has "
                            "arrived for transmission by this device",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macTxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacTxOk",
                            "Trace source indicating a packet has been "
                            "successfully sent",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macTxOkTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacTxDrop",
                            "Trace source indicating a packet has been "
                            "dropped during transmission",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macTxDropTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacIndTxDrop",
                            "Trace source indicating a packet has been "
                            "dropped from the indirect transaction queue"
                            "(The pending transaction list)",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macIndTxDropTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacPromiscRx",
                            "A packet has been received by this device, "
                            "has been passed up from the physical layer "
                            "and is being forwarded up the local protocol stack.  "
                            "This is a promiscuous trace,",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macPromiscRxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacRx",
                            "A packet has been received by this device, "
                            "has been passed up from the physical layer "
                            "and is being forwarded up the local protocol stack.  "
                            "This is a non-promiscuous trace,",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macRxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacRxDrop",
                            "Trace source indicating a packet was received, "
                            "but dropped before being forwarded up the stack",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macRxDropTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("Sniffer",
                            "Trace source simulating a non-promiscuous "
                            "packet sniffer attached to the device",
                            MakeTraceSourceAccessor(&LrWpanMac::m_snifferTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("PromiscSniffer",
                            "Trace source simulating a promiscuous "
                            "packet sniffer attached to the device",
                            MakeTraceSourceAccessor(&LrWpanMac::m_promiscSnifferTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("MacStateValue",
                            "The state of LrWpan Mac",
                            MakeTraceSourceAccessor(&LrWpanMac::m_lrWpanMacState),
                            "ns3::TracedValueCallback::LrWpanMacState")
            .AddTraceSource("MacIncSuperframeStatus",
                            "The period status of the incoming superframe",
                            MakeTraceSourceAccessor(&LrWpanMac::m_incSuperframeStatus),
                            "ns3::TracedValueCallback::SuperframeState")
            .AddTraceSource("MacOutSuperframeStatus",
                            "The period status of the outgoing superframe",
                            MakeTraceSourceAccessor(&LrWpanMac::m_outSuperframeStatus),
                            "ns3::TracedValueCallback::SuperframeState")
            .AddTraceSource("MacState",
                            "The state of LrWpan Mac",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macStateLogger),
                            "ns3::LrWpanMac::StateTracedCallback")
            .AddTraceSource("MacSentPkt",
                            "Trace source reporting some information about "
                            "the sent packet",
                            MakeTraceSourceAccessor(&LrWpanMac::m_sentPktTrace),
                            "ns3::LrWpanMac::SentTracedCallback")
            .AddTraceSource("IfsEnd",
                            "Trace source reporting the end of an "
                            "Interframe space (IFS)",
                            MakeTraceSourceAccessor(&LrWpanMac::m_macIfsEndTrace),
                            "ns3::Packet::TracedCallback");
    return tid;
}

LrWpanMac::LrWpanMac() {
    // First set the state to a known value, call ChangeMacState to fire trace source.
    m_lrWpanMacState = MAC_IDLE;

    ChangeMacState(MAC_IDLE);

    m_incSuperframeStatus = INACTIVE;
    m_outSuperframeStatus = INACTIVE;

    m_macRxOnWhenIdle = true;
    m_macPanId = 0xffff;
    m_macCoordShortAddress = Mac16Address("ff:ff");
    m_deviceCapability = DeviceType::FFD;
    m_associationStatus = ASSOCIATED;

    m_macPromiscuousMode = false;

    m_forDsmeNetDeviceIntegrateWithHigerLayer = false;
    m_acceptAllHilowPkt = false;
    m_gtsContinuePktSendingFromCap = false;
    m_enhancedGTSForwarding = false;
    m_record = nullptr;
    m_record2 = nullptr;

    m_macMaxFrameRetries = 6;
    
    m_retransmission = 0;
    m_numCsmacaRetry = 0;
    m_txPkt = nullptr;
    m_rxPkt = nullptr;
    m_ifs = 0;

    m_macLIFSPeriod = 40;  // 40
    m_macSIFSPeriod = 12;  // 12

    m_panCoor = false;
    m_coord = false;
    m_macBeaconOrder = 15;
    m_macSuperframeOrder = 15;
    m_macTransactionPersistenceTime = 500; // 0x01F5
    m_macAssociationPermit = true;  // PAN-C 允許其他人跟他做 associate
    m_macAutoRequest = true;

    m_incomingBeaconOrder = 15;
    m_incomingSuperframeOrder = 15;
    m_beaconTrackingOn = false;
    m_numLostBeacons = 0;

    m_pendPrimitive = MLME_NONE;
    m_channelScanIndex = 0;
    m_maxEnergyLevel = 0;

    m_originalChannelInCAP = 11;

    // m_macResponseWaitTime = aBaseSuperframeDuration * 32;
    m_macResponseWaitTime = aBaseSuperframeDuration * 64;
    m_assocRespCmdWaitTime = 960;

    m_maxTxQueueSize = m_txQueue.max_size();
    m_maxIndTxQueueSize = m_indTxQueue.max_size();

    Ptr<UniformRandomVariable> uniformVar = CreateObject<UniformRandomVariable>();
    uniformVar->SetAttribute("Min", DoubleValue(0.0));
    uniformVar->SetAttribute("Max", DoubleValue(255.0));
    m_macDsn = SequenceNumber8(uniformVar->GetValue());
    m_macBsn = SequenceNumber8(uniformVar->GetValue());
    m_shortAddress = Mac16Address("00:00");

    m_macEnhAckWaitDuration = 0x360;   // 864 μs
    m_macImplicitBroadcast = false;

    m_macDSMEcapable = true;
    m_macHoppingCapable = true;
    m_macDSMEenabled = false;
    m_macHoppingEnabled = false;

    m_macGACKFlag = false;
    m_macCAPReductionFlag = false;

    m_macChannelDiversityMode = 0x01;

    m_macMultisuperframeOrder = 15;

    m_macSDindex = -1;
    m_macChannelOfs = 0;
    m_macDeferredBcnUsed = false;

    m_macSyncParentShortAddr = Mac16Address("ff:ff");

    m_macBcnSlotLen = 60;

    m_macDSMEGTSExpirationTime = 7;

    m_macPANCoordinatorBSN = 0;

    m_simpleAddress = Mac8Address(0xff);

    // Table 52n
    m_macUseEnhancedBeacon = true;
    m_macEbsn = SequenceNumber8(uniformVar->GetValue());
    m_macEBAutoSA = EBAutoSA_FULL;

    m_becomeCoord = false;
    m_sendBcn = false;
    realignmentRecevied = false;
    m_macBcnSchedulingAllocStatus = ALLOC_TO_BE_DONE;

    m_incSuperframe = false;
    m_isBcnAllocCollision = false;
    m_needBcnSchedulingAgain = false;
    m_bcnScehdulingFailCnt = 0;
    m_allocationSequence = 0;
    m_bcnSchedulingCtrlPktCount = 0;
    m_groupAckPolicy = GROUP_ACK_DISABLED;
    m_legacyGackBitmap = 0;
    m_legacyGackDevList = 0;
    m_legacyGackIdx = 0;
    m_legacyGackDirections = 0;
    m_legacyGackIndexCounter = 0;
}

LrWpanMac::~LrWpanMac() {

}

void
LrWpanMac::DoInitialize()
{
    if (m_macRxOnWhenIdle)
    {
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
    }
    else
    {
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TRX_OFF);
    }

    Object::DoInitialize();
}

void
LrWpanMac::DoDispose()
{
    if (m_csmaCa)
    {
        m_csmaCa->Dispose();
        m_csmaCa = nullptr;
    }
    m_txPkt = nullptr;

    for (uint32_t i = 0; i < m_txQueue.size(); i++)
    {
        m_txQueue[i]->txQPkt = nullptr;
        m_txQueue[i]->txQMsduHandle = 0;
    }
    m_txQueue.clear();

    for (uint32_t i = 0; i < m_indTxQueue.size(); i++)
    {
        m_indTxQueue[i]->txQPkt = nullptr;
        m_indTxQueue[i]->seqNum = 0;
        m_indTxQueue[i]->dstExtAddress = nullptr;
        m_indTxQueue[i]->dstShortAddress = nullptr;
    }
    m_indTxQueue.clear();

    m_phy = nullptr;
    m_mcpsDataConfirmCallback = MakeNullCallback<void, McpsDataConfirmParams>();
    m_mcpsDataIndicationCallback = MakeNullCallback<void, McpsDataIndicationParams, Ptr<Packet>>();
    m_mlmeStartConfirmCallback = MakeNullCallback<void, MlmeStartConfirmParams>();
    m_mlmeBeaconNotifyIndicationCallback =
        MakeNullCallback<void, MlmeBeaconNotifyIndicationParams, Ptr<Packet>>();
    m_mlmeSyncLossIndicationCallback = MakeNullCallback<void, MlmeSyncLossIndicationParams>();
    m_mlmePollConfirmCallback = MakeNullCallback<void, MlmePollConfirmParams>();
    m_mlmeScanConfirmCallback = MakeNullCallback<void, MlmeScanConfirmParams>();
    m_mlmeAssociateConfirmCallback = MakeNullCallback<void, MlmeAssociateConfirmParams>();
    m_mlmeAssociateIndicationCallback = MakeNullCallback<void, MlmeAssociateIndicationParams>();
    m_mlmeCommStatusIndicationCallback = MakeNullCallback<void, MlmeCommStatusIndicationParams>();

    m_mlmeDisassociateConfirmCallback = MakeNullCallback<void, MlmeDisassociateConfirmParams>();
    m_mlmeDisassociateIndicationCallback = MakeNullCallback<void, MlmeDisassociateIndicationParams>();

    m_mlmeDsmeGtsIndicationCallback = MakeNullCallback<void, MlmeDsmeGtsIndicationParams>();
    m_mlmeDsmeInfoConfirmCallback = MakeNullCallback<void, MlmeDsmeInfoConfirmParams>();
    m_mlmeDsmeInfoIndicationCallback = MakeNullCallback<void, MlmeDsmeInfoIndicationParams>();
    m_mlmeDsmeLinkStatusReportIndicationCallback = MakeNullCallback<void, MlmeDsmeLinkStatusReportIndicationCallback>();
    m_mlmeDsmeLinkStatusRptConfirmCallback = MakeNullCallback<void, MlmeDsmeLinkStatusRptConfirmParams>();

    m_mlmeStartRequestCallback = MakeNullCallback<void, MlmeStartRequestParams>();

    m_beaconEvent.Cancel();

    Object::DoDispose();
}

bool
LrWpanMac::GetRxOnWhenIdle()
{
    return m_macRxOnWhenIdle;
}

void
LrWpanMac::SetRxOnWhenIdle(bool rxOnWhenIdle)
{
    NS_LOG_FUNCTION(this << rxOnWhenIdle);
    m_macRxOnWhenIdle = rxOnWhenIdle;

    if (m_lrWpanMacState == MAC_IDLE)
    {
        if (m_macRxOnWhenIdle)
        {
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
        }
        else
        {
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TRX_OFF);
        }
    }
}

void LrWpanMac::SetSimpleAddress(Mac8Address address) {
    NS_LOG_FUNCTION(this << address);
    m_simpleAddress = address;
}

void
LrWpanMac::SetShortAddress(Mac16Address address)
{
    NS_LOG_FUNCTION(this << address);
    m_shortAddress = address;
}

Mac8Address LrWpanMac::GetSimpleAddress() const {
    NS_LOG_FUNCTION(this);
    return m_simpleAddress;
}

Mac16Address
LrWpanMac::GetShortAddress() const
{
    NS_LOG_FUNCTION(this);
    return m_shortAddress;
}

Mac64Address
LrWpanMac::GetExtendedAddress() const
{
    return m_selfExt;
}

void
LrWpanMac::McpsDataRequest(McpsDataRequestParams params, Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << p);

#if MCPS_DATA_SENDING_LOG
    NS_LOG_DEBUG("Prepare a data packet with size:" << p->GetSize() << " bytes");
#endif
    m_mcpsDataRequestParams = params;

    McpsDataConfirmParams confirmParams;
    confirmParams.m_msduHandle = params.m_msduHandle;

    // howard: 正常這裡要檢查 Packet 有沒有符合 Spec 的大小

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_DATA, m_macDsn.GetValue());
    m_macDsn++;

    macHdr.SetSrcAddrMode(params.m_srcAddrMode);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    macHdr.SetDstAddrMode(params.m_dstAddrMode);
    macHdr.SetDstAddrFields(params.m_dstPanId, params.m_dstAddr);

    // IEEE 802.15.4-2006 (7.5.6.1)
    // Src & Dst PANs are identical, PAN compression is ON
    // only the dst PAN is serialized making the MAC header 2 bytes smaller
    if ((params.m_dstAddrMode != NO_PANID_ADDR && params.m_srcAddrMode != NO_PANID_ADDR) &&
        (macHdr.GetDstPanId() == macHdr.GetSrcPanId()))
    {
        macHdr.SetPanIdComp();
    }

    macHdr.SetSecDisable();
    // extract the first 3 bits in TxOptions
    int b0 = params.m_txOptions & TX_OPTION_ACK;
    int b1 = params.m_txOptions & TX_OPTION_GTS;
    int b2 = params.m_txOptions & TX_OPTION_INDIRECT;
    int b3 = params.m_txOptions & TX_OPTION_DIRECT;

    // NS_LOG_INFO("params.m_txOptions = " << std::to_string(params.m_txOptions));

    if (b0 == TX_OPTION_ACK)
    {
        // Set AckReq bit only if the destination is not the broadcast address.
        if (macHdr.GetDstAddrMode() == SHORT_ADDR)
        {
            // short address and ACK requested.
            Mac16Address shortAddr = macHdr.GetShortDstAddr();
            if (shortAddr.IsBroadcast() || shortAddr.IsMulticast())
            {
                NS_LOG_LOGIC("LrWpanMac::McpsDataRequest: requested an ACK on broadcast or "
                             "multicast destination ("
                             << shortAddr << ") - forcefully removing it.");
                macHdr.SetNoAckReq();
                params.m_txOptions &= ~uint8_t(TX_OPTION_ACK);
            }
            else
            {
                // howard: 改成這樣，取消 ACK
                if(m_NoACK)
                {
                    macHdr.SetNoAckReq();
                }
                else
                {
                    // GACK 會用到
                    macHdr.SetAckReq();
                }
                // macHdr.SetAckReq();
            }
        }
        else
        {
            // other address (not short) and ACK requested
            macHdr.SetAckReq();
        }
    }
    else
    {
        macHdr.SetNoAckReq();
    }

    if (b1 == TX_OPTION_GTS) {
        NS_LOG_INFO("進來 CFP 傳資料");
#if MCPS_DATA_SENDING_LOG
        NS_LOG_DEBUG("Sending a data packet during a GTS period.");
#endif

        // DSME-TODO
        // NS_ASSERT(m_lrWpanMacState == MAC_GTS);

        p->AddHeader(macHdr);

        LrWpanMacTrailer macTrailer;
        // Calculate FCS if the global attribute ChecksumEnable is set.
        if (Node::ChecksumEnabled()) {
            macTrailer.EnableFcs(true);
            macTrailer.SetFcs(p);
        }

        p->AddTrailer(macTrailer);
        NS_LOG_INFO("MAC Header: " << macHdr.GetSerializedSize() << " bytes");
        NS_LOG_INFO("MAC Footer: " << macTrailer.GetSerializedSize() << " bytes");

        NS_LOG_INFO(m_incGtsEvent.IsRunning());
        NS_LOG_INFO(m_gtsEvent.IsRunning());

        if((m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) && m_lrWpanMacState == MAC_GTS)
        {
            // NS_LOG_INFO("進來 CFP 傳資料");
            m_txPkt = p;
            ChangeMacState(MAC_GTS_SENDING);
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
        }
        // howard: 不知道在幹嘛
        // else
        // {
        //     m_txPktGts = p;
        // }
    }
    else if (b2 == TX_OPTION_INDIRECT)
    {
        // howard: 移除此功能
    }
    else if (b3 == TX_OPTION_DIRECT)
    {
        NS_LOG_INFO("進來 CAP 傳資料");
        // Direct Tx
        // From this point the packet will be pushed to a Tx queue and immediately
        // use a slotted (beacon-enabled) or unslotted (nonbeacon-enabled) version of CSMA/CA
        // before sending the packet, depending on whether it has previously
        // received a valid beacon or not.

        p->AddHeader(macHdr);
        NS_LOG_INFO("MAC Header: " << macHdr.GetSerializedSize() << " bytes");

        LrWpanMacTrailer macTrailer;
        // Calculate FCS if the global attribute ChecksumEnable is set.
        if (Node::ChecksumEnabled())
        {
            macTrailer.EnableFcs(true);
            macTrailer.SetFcs(p);
        }
        p->AddTrailer(macTrailer);
        NS_LOG_INFO("MAC Footer: " << macTrailer.GetSerializedSize() << " bytes");

        Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
        txQElement->txQMsduHandle = params.m_msduHandle;
        txQElement->txQPkt = p;
        EnqueueTxQElement(txQElement);
        CheckQueue();
    }
}

void LrWpanMac::MlmeStartRequest(MlmeStartRequestParams params) {
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_deviceCapability == DeviceType::FFD);

    // Mark primitive as pending and save the start params while the new page and channel is set.
    m_startParams = params;
    m_pendPrimitive = MLME_START_REQ;
    LrWpanPhyPibAttributes pibAttr;
    pibAttr.phyCurrentPage = m_startParams.m_logChPage;
    m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentPage, &pibAttr);
}

void LrWpanMac::MlmeSyncRequest(MlmeSyncRequestParams params) {
    NS_LOG_FUNCTION(this);
    //NS_ASSERT(params.m_logCh <= 26 && m_macPanId != 0xffff);

    NS_LOG_DEBUG("SYNC START"); // debug

    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second
    // change phy current logical channel
    LrWpanPhyPibAttributes pibAttr;
    pibAttr.phyCurrentChannel = params.m_logCh;
    m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);

    // Enable Phy receiver
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);

    uint64_t searchSymbols;
    Time searchBeaconTime;

    if (m_trackingEvent.IsRunning()) {
        m_trackingEvent.Cancel();
    }

    if (params.m_trackBcn) {
        m_numLostBeacons = 0;
        // search for a beacon for a time = incomingSuperframe symbols + 960 symbols
        searchSymbols = (((uint64_t)1 << m_incomingBeaconOrder) + 1) * aBaseSuperframeDuration;
        searchBeaconTime = Seconds((double)searchSymbols / symbolRate);
        m_beaconTrackingOn = true;
        m_trackingEvent =
            Simulator::Schedule(searchBeaconTime, &LrWpanMac::BeaconSearchTimeout, this);
        
    } else {
        m_beaconTrackingOn = false;
    }
}

// See IEEE 802.15.4e-2012 Section 5.2.2.1 
void LrWpanMac::SendOneEnhancedBeacon() {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(m_lrWpanMacState == MAC_IDLE);
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:ff"));

    if(m_macPanId == 0xffff) // TODO : This is a workaround !! Root cause do not found yet ..
    {                        // After a device leave the PAN (disassociation), the PAN id is set to 0xffff.
        return;
    }

    m_startOfBcnSlot = Simulator::Now();

    LrWpanMacHeader macHdr;                         // Enhanced Beacon MDR
    BeaconPayloadHeader macPayload;                 // Enhanced Beacon payload
    Ptr<Packet> beaconPacket = Create<Packet>();
    LrWpanMacTrailer macTrailer;

    // Set MHD Frame control field
    if (m_panCoor) {
        macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_BEACON);
        macHdr.SetSeqNum(m_macPANCoordinatorBSN.GetValue());
        m_macPANCoordinatorBSN++;

    } else {
        macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_BEACON);
        macHdr.SetSeqNum(m_macEbsn.GetValue());
        m_macEbsn++;
    }

    macHdr.SetSecDisable();
    macHdr.SetNoAckReq();

    // DSME: indicate this is a enhanced beacon
    macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4);

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff")); // broadcast packet

    // see IEEE 802.15.4-2011 Section 5.1.2.4
    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    // If a broadcast data or command frame is pending
    //, the Frame Pending field shall be set to one
    // See IEEE 802.15.4e-2012 5.2.2.1.1
    if (m_indTxQueue.size()) {
        macHdr.SetFrmPend();

        // DSME-TODO
        // Set the Pending Address Fields
    }

    // Superframe spec, GTS, Pending address list are optional if IE is used.
    // macPayload.SetGtsFields(GetGtsFields());
    
    beaconPacket->AddHeader(macPayload);

    // DSME PAN Descriptor IE should be sent in periodic enhanced beacon frame (DSME beacon mode)
    if (m_macDSMEenabled && m_csmaCa->IsSlottedCsmaCa()) {  
        macHdr.SetIEListPresent();

        PayloadIETermination termination;
        beaconPacket->AddHeader(termination);       

        HeaderIETermination termination2;
        beaconPacket->AddHeader(termination2);

        TimeSync timeSync;
        timeSync.SetBeaconTimeStamp(m_startOfBcnSlot.ToInteger(Time::NS));

        // timeSync.SetBeaconOffsetTimeStamp();
        m_dsmePanDescriptorIE.SetTimeSync(timeSync);

        // NS_LOG_DEBUG("TEST : " << m_macSDBitmap);
        m_dsmePanDescriptorIE.SetBeaconBitmap(m_macSDBitmap);

        ChannelHopping channelHoppingField;
        channelHoppingField.SetHoppingSequenceID(m_macHoppingSeqID);
        channelHoppingField.SetPANCoordinatorBSN(m_macPANCoordinatorBSN.GetValue() - 1);
        channelHoppingField.SetChannelOffset(m_macChannelOfs);
        channelHoppingField.SetChannelOffsetBitmapLength(m_macChannelOfsBitmapLen);
        channelHoppingField.SetChannelOffsetBitmap(m_macChannelOfsBitmap);
        m_dsmePanDescriptorIE.SetChannelHopping(channelHoppingField);
        if(m_groupAckPolicy == GROUP_ACK_LEGACY)
        {
            NS_LOG_DEBUG(m_legacyGroupAck);
        }
        m_dsmePanDescriptorIE.SetGroupACK(m_legacyGroupAck);

        PendingAddrFields pndAddrFields = GetPendingAddrFields();
        m_dsmePanDescriptorIE.SetPendingAddrFields(pndAddrFields);
        
        // DSME-TODO
        m_dsmePanDescriptorIE.SetHeaderIEDescriptor(m_dsmePanDescriptorIE.GetSerializedSize() - 2
                                                    , HEADERIE_DSME_PAN_DESCRIPTOR); // debug     

        beaconPacket->AddHeader(m_dsmePanDescriptorIE);
    }

    beaconPacket->AddHeader(macHdr); 

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(beaconPacket);
    }

    beaconPacket->AddTrailer(macTrailer);

    // Set the Beacon packet to be transmitted
    // howard: 使用 PdDataRequest() 傳到實體層
    m_txPkt = beaconPacket;

    if (m_csmaCa->IsSlottedCsmaCa()) {
        m_outSuperframeStatus = BEACON;
        if (isCAPReductionOn()
            && m_macSDindex % (m_multiSuperframeDuration / m_superframeDuration) != 0) {
            NS_LOG_DEBUG("Outgoing superframe Active Portion (Beacon + CFP + CFP): "
                        << m_superframeDuration << " symbols"
                        <<" Next Beacon will at : " << (Simulator::Now() + Seconds((double) m_beaconInterval / 62500)).As(Time::S));

        } else {
            NS_LOG_DEBUG("Outgoing superframe Active Portion (Beacon + CAP + CFP): "
                    << m_superframeDuration << " symbols"
                    <<" Next Beacon will at : " << (Simulator::Now() + Seconds((double) m_beaconInterval / 62500)).As(Time::S));
        }
 
    } else {
        NS_LOG_DEBUG("Outgoing Enhanced Beacon Frame response to Enhanced Beacon Request" );
    }

    m_BeaconStartTxTime = Simulator::Now();

    ChangeMacState(MAC_SENDING);
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

void
LrWpanMac::EndStartRequest()
{
    NS_LOG_FUNCTION(this);
    // The primitive is no longer pending (Channel & Page have been set)
    m_pendPrimitive = MLME_NONE;

    if(m_startParams.m_panCoor)
    {
        m_panCoor = true;
        m_macPanId = m_startParams.m_PanId;
    }
    else
    {
        m_coord = true;
        m_macPanId = m_startParams.m_PanId;
    }

    NS_ASSERT(m_startParams.m_PanId != 0xffff);

    if(m_panCoor)
    {
        m_macBeaconOrder = m_startParams.m_bcnOrd;
    }
    else 
    {
        // howard:
        // Extract BO infos from associated PAN-C (從 PAN-C 提取 BO)
        // 這個是直接在 Upper Layer 設定給 MAC Header field 裡面的，這邊只是從那邊做調用
        m_macBeaconOrder = m_panDescriptorList[m_descIdxOfAssociatedPan].m_superframeSpec.GetBeaconOrder();
    }
    
    if (m_macBeaconOrder == 15) {
        // Non-beacon enabled PAN
        // Cancel any ongoing events and CSMA-CA process
        m_macSuperframeOrder = 15;
        m_fnlCapSlot = 15;
        m_beaconInterval = 0;

        m_csmaCa->Cancel();
        m_capEvent.Cancel();
        m_cfpEvent.Cancel();
        m_incCapEvent.Cancel();
        m_incCfpEvent.Cancel();
        m_trackingEvent.Cancel();
        m_scanEvent.Cancel();
        m_scanEnergyEvent.Cancel();

        m_csmaCa->SetUnSlottedCsmaCa();

        if (!m_mlmeStartConfirmCallback.IsNull()) {
            MlmeStartConfirmParams confirmParams;
            confirmParams.m_status = MLMESTART_SUCCESS;
            m_mlmeStartConfirmCallback(confirmParams);
        }

        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
    }
    else
    {
        if(m_panCoor) 
        {
            m_macSuperframeOrder = m_startParams.m_sfrmOrd;
            
            // howard: 是否開啟省電模式，這邊預設不開啟
            m_csmaCa->SetBatteryLifeExtension(m_startParams.m_battLifeExt);
        }
        else
        {
            // Because the device has associated already, here just to extract the superframe infos (BO, SO, etc.)
            m_macSuperframeOrder = m_panDescriptorList[m_descIdxOfAssociatedPan].m_superframeSpec.GetFrameOrder();
            m_csmaCa->SetBatteryLifeExtension(m_panDescriptorList[m_descIdxOfAssociatedPan].m_superframeSpec.IsBattLifeExt());
        }
        // 使用 slotted-CSMA/CA
        m_csmaCa->SetSlottedCsmaCa();

        // DSME-TODO
        // TODO: Calculate the real Final CAP slot (requires GTS implementation)
        //  FinalCapSlot = Superframe duration slots - CFP slots.
        //  In the current implementation the value of the final cap slot is equal to
        //  the total number of possible slots in the superframe (15).
        // m_fnlCapSlot = 15;

        // Setting final cap timeslot
        m_fnlCapSlot = 8;

        // BI = 2^BO * aBaseSuperframeDuration
        // if BO = 6, BI = 2^6 * 960 = 61400 symbol
        m_beaconInterval = (static_cast<uint32_t>(1 << m_macBeaconOrder)) * aBaseSuperframeDuration;

        // SD = 2^SO * aBaseSuperframeDuration
        // if SO = 3, SD = 2^3 * 960 = 7680 symbol
        m_superframeDuration = (static_cast<uint32_t>(1 << m_macSuperframeOrder)) * aBaseSuperframeDuration;
            
        // DSME
        /* howard: 這裡 symbolRate 會拿到 62500，因為 2.4 Ghz O-QPSK 每秒可傳 62500 個 symbol */
        uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

        // 每個 beacon 的間隔時間
        Time bcnTime = Seconds((double)m_beaconInterval / symbolRate);

        // 每個 Superframe 的間隔時間
        Time superfmTime = Seconds((double)m_superframeDuration / symbolRate);

        NS_LOG_DEBUG("**********************************************************************************************");
        NS_LOG_DEBUG(" m_coord = " << m_coord);
        NS_LOG_DEBUG(" Beacon Interval: " << m_beaconInterval << " symbols, " << bcnTime << " seconds");
        NS_LOG_DEBUG(" Superframe duration: " << m_superframeDuration << " symbols, " << superfmTime << " seconds");
        
        if(m_macDSMEenabled)
        {
            // Dsme superframe specification 
            //!< set parameters  from the * Next higher layer * to the * MAC layer *
            if(m_panCoor)
            {
                m_macMultisuperframeOrder = m_startParams.m_dsmeSuperframeSpec.GetMultiSuperframeOrder();
                m_macChannelDiversityMode = m_startParams.m_dsmeSuperframeSpec.GetChannelDiversityMode();
                m_macGACKFlag = m_startParams.m_dsmeSuperframeSpec.GetGACKFlag();
                m_macCAPReductionFlag = m_startParams.m_dsmeSuperframeSpec.GetCAPReductionFlag();
                m_macDeferredBcnUsed = m_startParams.m_dsmeSuperframeSpec.GetDeferredBeaconFalg();

                NS_LOG_DEBUG(" Dsme Superframe Spec: " << m_startParams.m_dsmeSuperframeSpec);

                // BeaconBitmap 
                m_macSDBitmap = m_startParams.m_bcnBitmap;

                m_macSDindex = m_macSDBitmap.GetSDIndex();
                
                // Hopping Descriptor
                m_macHoppingSeqID = m_startParams.m_hoppingDescriptor.m_HoppingSequenceID;

                if(m_macHoppingSeqID)  // 0x00
                {
                    m_hoppingSeqLen = m_startParams.m_hoppingDescriptor.m_hoppingSeqLen;
                    m_macHoppingSeqList = m_startParams.m_hoppingDescriptor.m_hoppingSeq;
                }
                else
                {
                    // PAN-C 會進來這邊
                    m_hoppingSeqLen = 0;
                }
                m_macChannelOfs = m_startParams.m_hoppingDescriptor.m_channelOfs;
                m_macChannelOfsBitmapLen = m_startParams.m_hoppingDescriptor.m_channelOfsBitmapLen;
                m_macChannelOfsBitmap = m_startParams.m_hoppingDescriptor.m_channelOfsBitmap;
            }
            else
            {
                DsmeSuperFrameField dsmeSuperframeField = m_panDescriptorList[m_descIdxOfAssociatedPan].m_dsmeSuperframeSpec;

                m_macMultisuperframeOrder = dsmeSuperframeField.GetMultiSuperframeOrder();
                m_macChannelDiversityMode = dsmeSuperframeField.GetChannelDiversityMode();
                m_macGACKFlag = dsmeSuperframeField.GetGACKFlag();
                m_macCAPReductionFlag = dsmeSuperframeField.GetCAPReductionFlag();
                m_macDeferredBcnUsed = dsmeSuperframeField.GetDeferredBeaconFalg();

                NS_LOG_DEBUG(" Dsme Superframe Spec: " << dsmeSuperframeField);
                // Update Beacon bitmap
                m_macSDBitmap = m_panDescriptorList[m_descIdxOfAssociatedPan].m_bcnBitmap;
                
                m_macSDBitmap.SetSDBitmap(m_choosedSDIndexToSendBcn);
                m_macSDBitmap.SetSDIndex(m_choosedSDIndexToSendBcn);
                m_macSDindex = m_choosedSDIndexToSendBcn;
                
                m_macHoppingSeqID = m_startParams.m_hoppingDescriptor.m_HoppingSequenceID;

                if (m_macHoppingSeqID) {
                    m_hoppingSeqLen = m_startParams.m_hoppingDescriptor.m_hoppingSeqLen;
                    m_macHoppingSeqList = m_startParams.m_hoppingDescriptor.m_hoppingSeq;
                } else {
                    m_hoppingSeqLen = 0;
                }

                m_macChannelOfs = m_startParams.m_hoppingDescriptor.m_channelOfs;
                m_macChannelOfsBitmapLen = m_startParams.m_hoppingDescriptor.m_channelOfsBitmapLen;
                m_macChannelOfsBitmap = m_startParams.m_hoppingDescriptor.m_channelOfsBitmap;
            }

            // howard: 增加 DSME PAN descriptro IE 進 Header IEs
            NS_LOG_DEBUG(" Extract from asscoiated, BO = " << (uint32_t)m_macBeaconOrder << ", SO = " << (uint32_t)m_macSuperframeOrder << "\n");
            
            // 定義在 lr-wapn-mac-pl-header，傳 m_fnlCapSlot
            m_dsmePanDescriptorIE.SetSuperframeField(m_macBeaconOrder,
                                                    m_macSuperframeOrder,
                                                    m_fnlCapSlot,
                                                    m_csmaCa->GetBatteryLifeExtension(),
                                                    m_panCoor,
                                                    m_macAssociationPermit);
            
            PendingAddrFields pndAddrFields = GetPendingAddrFields();
            m_dsmePanDescriptorIE.SetPendingAddrFields(pndAddrFields);

            m_dsmePanDescriptorIE.SetDsmeSuperFrameField(m_macMultisuperframeOrder,
                                                        m_macChannelDiversityMode,
                                                        m_macGACKFlag,
                                                        m_macCAPReductionFlag,
                                                        m_macDeferredBcnUsed);

            // DSME-TODO
            TimeSync timeSync;
            timeSync.SetBeaconTimeStamp(m_startOfBcnSlot.ToInteger(Time::MS));
            // timeSync.SetBeaconOffsetTimeStamp();
            m_dsmePanDescriptorIE.SetTimeSync(timeSync);

            m_dsmePanDescriptorIE.SetBeaconBitmap(m_macSDBitmap);

            // DSME-TODO
            ChannelHopping channelHoppingField;
            channelHoppingField.SetHoppingSequenceID(m_macHoppingSeqID);
            channelHoppingField.SetPANCoordinatorBSN(m_macPANCoordinatorBSN.GetValue());
            channelHoppingField.SetChannelOffset(m_macChannelOfs);
            channelHoppingField.SetChannelOffsetBitmapLength(m_macChannelOfsBitmapLen);
            channelHoppingField.SetChannelOffsetBitmap(m_macChannelOfsBitmap);
            m_dsmePanDescriptorIE.SetChannelHopping(channelHoppingField); 

            // DSME-TODO
            GroupACK groupAckField;
            m_dsmePanDescriptorIE.SetGroupACK(groupAckField);

            // multi-superframe duration
            m_multiSuperframeDuration = (static_cast<uint32_t>(1 << m_macMultisuperframeOrder)) * aBaseSuperframeDuration;
            m_numOfMultisuperframes = static_cast<uint32_t>(1 << (m_macBeaconOrder - m_macMultisuperframeOrder));
            m_numOfSuperframes = static_cast<uint64_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder));

            Time multisuperfmTime = Seconds((double)m_multiSuperframeDuration / symbolRate);

            NS_LOG_DEBUG(" Multisuperframe duration: " << m_multiSuperframeDuration << " symbols, " << multisuperfmTime << " seconds");
            NS_LOG_DEBUG(" Num of Multisuperframe in a beacon interval " << m_numOfMultisuperframes);
            NS_LOG_DEBUG(" Num of Superframe in a beacon interval " << m_numOfSuperframes);
            
            NS_LOG_DEBUG(" SD Bitmap infos : " << m_macSDBitmap);
            NS_LOG_DEBUG(" Channel Hopping infos : " << channelHoppingField);

            m_scheduleGTSsEvent.resize(m_numOfSuperframes / m_numOfMultisuperframes);
        }

        if(m_macCAPReductionFlag)
        {
            // howard: 原本是這樣
            // m_macDSMESABCapOff.resize(static_cast<uint64_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder)), 0);
            m_macDSMESAB.resize(static_cast<uint16_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder)), 0);
        }
        else
        {
            // howard: 原本是這樣
            // m_macDSMESABCapOff.resize(static_cast<uint64_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder)), 0);
            m_macDSMESABCapOff.resize(static_cast<uint8_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder)), 0);
        }

        if(m_macDSMEenabled)
        {
            NS_LOG_DEBUG("isPanCoordinator ? value of [m_panCoor] = " << m_panCoor);
            if(m_panCoor)
            {
                m_multisuperframeStartEvent = Simulator::ScheduleNow(&LrWpanMac::StartMultisuperframe, this, OUTGOING);

                // Send a First EB here, the subsequent (隨後的) EBs will be scheduled at the following flow.
                // Flow : PdDataConfirm -> StartCAP() -> StartCFP() -> ** StartRemainingPeriod() **
                m_beaconEvent = Simulator::ScheduleNow(&LrWpanMac::SendOneEnhancedBeacon, this);
                PurgeDsmeACT();
            }
            else
            {
                m_sendBcn = true;
            }

            if(!m_forDsmeNetDeviceIntegrateWithHigerLayer)
            {
                NS_LOG_INFO("進來ScheduleGts");
                ScheduleGts(false);
            }
        }
        NS_LOG_DEBUG("**********************************************************************************************");       
    }
}

void LrWpanMac::StartCAP(SuperframeType superframeType)
{
    uint32_t each_Timeslot_Duration;
    uint64_t cap_Duration;
    Time cap_time;

    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if(superframeType == OUTGOING)
    {
        m_incSuperframe = false;
        m_outSuperframeStatus = CAP;
        each_Timeslot_Duration = m_superframeDuration / 16; // Beacon + 8 CAP + 7 CFP timeslot = 16 timeslot ， slot的時長 
        cap_Duration = each_Timeslot_Duration * (m_fnlCapSlot + 1); // Beacon + CAP slot, unit : symbols ， CAP的時長 (symbol)
        cap_time = Seconds((double)cap_Duration / symbolRate); // CAP的時長 (sec)

        // CAP 開始的時間要減去傳送 beacon 的時間
        
        // NS_LOG_INFO("CAP time = " << cap_time.As(Time::S));
        // NS_LOG_INFO("m_macBeaconTxTime = " << m_macBeaconTxTime.As(Time::S));
        // NS_LOG_INFO("m_BeaconStartTxTime = " << m_BeaconStartTxTime.As(Time::S));
        cap_time = cap_time - (m_macBeaconTxTime - m_BeaconStartTxTime); // Minus beacon TX transmission time (TX end - TX begin)
        // NS_LOG_INFO("減去 beacon 後的 CAP time = " << cap_time.As(Time::S));

        NS_LOG_DEBUG("Outgoing superframe CAP duration " << (cap_time.GetSeconds() * symbolRate) << " symbols (" << cap_time.As(Time::S) << ")");
        NS_LOG_DEBUG("Each time slot duration " << each_Timeslot_Duration << " symbols");
        m_capEvent = Simulator::Schedule(cap_time, &LrWpanMac::StartCFP, this, SuperframeType::OUTGOING);
    }
    else
    {
        m_incSuperframe = true;
        m_incSuperframeStatus = CAP;
        each_Timeslot_Duration = m_incomingSuperframeDuration / 16;
        cap_Duration = each_Timeslot_Duration * (m_incomingFnlCapSlot + 1);
        cap_time = Seconds((double)(cap_Duration - m_rxBeaconSymbols) / symbolRate);

        NS_LOG_DEBUG("Incoming superframe CAP duration " << (cap_time.GetSeconds() * symbolRate) << " symbols (" << cap_time.As(Time::S) << ")");
        NS_LOG_DEBUG("Each time slot duration " << each_Timeslot_Duration << " symbols");
        m_incCapEvent = Simulator::Schedule(cap_time, &LrWpanMac::StartCFP, this, SuperframeType::INCOMING);
    }
    CheckQueue();
}

void
LrWpanMac::StartCFP(SuperframeType superframeType)
{
    uint32_t activeSlot;
    uint64_t cfpDuration;
    Time endCfpTime;
    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if (superframeType == INCOMING) 
    {
        activeSlot = m_incomingSuperframeDuration / 16;
        cfpDuration = activeSlot * (15 - m_incomingFnlCapSlot);
        endCfpTime = Seconds((double)cfpDuration / symbolRate);

        NS_LOG_DEBUG("CFP start at : " << Simulator::Now().As(Time::S));
        // NS_LOG_DEBUG("Each slot interval =  "<< (double)m_superframeDuration / 16 / 62500);

        if (cfpDuration > 0) 
        {
            m_incSuperframeStatus = CFP;
        }

        if (m_macDSMEenabled) 
        {
            if (m_incomingFirstCFP) 
            {
                
                // just replace CAP portion with CFP
                // cfpDuration = activeSlot * (m_fnlCapSlot + 1);
                cfpDuration = activeSlot * (m_incomingFnlCapSlot + 1);

                // Old Sync method
                // endCfpTime = Seconds((double)cfpDuration / symbolRate);
                // endCfpTime -= (Simulator::Now() - m_macBeaconRxTime);
                endCfpTime = Seconds((double)(cfpDuration - m_rxBeaconSymbols) / symbolRate);

                NS_LOG_DEBUG("Incoming superframe first CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");

                m_incomingFirstCFP = false;
                m_incCfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartCFP,
                                        this,
                                        SuperframeType::INCOMING);
            } 
            else 
            {      
                NS_LOG_DEBUG("Incoming superframe CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");

                m_incCfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartRemainingPeriod,
                                        this,
                                        SuperframeType::INCOMING);            
            } 
        }
    } 
    else // superframeType == OUTGOING
    {
        activeSlot = m_superframeDuration / 16;         // 一個slot的長度
        cfpDuration = activeSlot * (15 - m_fnlCapSlot); // CFP的時長
        endCfpTime = Seconds((double)cfpDuration / symbolRate);

        NS_LOG_DEBUG("CFP start at = " << Simulator::Now().As(Time::S));
        // NS_LOG_DEBUG("Each slot interval =  "<< (double)m_superframeDuration / 16 / 62500);
        
        if (cfpDuration > 0)
        {
            m_outSuperframeStatus = CFP;
        }

        // NS_LOG_DEBUG("m_firstCFP : " << m_firstCFP);
        if (m_macDSMEenabled) 
        {
            if (m_firstCFP) 
            {
                // just replace CAP portion with CFP
                cfpDuration = activeSlot * (m_fnlCapSlot + 1);
                endCfpTime = Seconds((double)cfpDuration / symbolRate);
                // Old Sync method
                // endCfpTime -= (Simulator::Now() - m_macBeaconTxTime);

                // New Sync method
                endCfpTime -= (m_macBeaconTxTime - m_BeaconStartTxTime);

                NS_LOG_DEBUG("(First CFP) Outgoing superframe first CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");
                
                m_firstCFP = false;
                m_cfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartCFP,
                                        this,
                                        SuperframeType::OUTGOING);
            } 
            else 
            {    
                NS_LOG_DEBUG("Outgoing superframe CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")"); 
                m_cfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartRemainingPeriod,
                                        this,
                                        SuperframeType::OUTGOING);
            }
        }
    }
}

void LrWpanMac::ScheduleGts(bool indication)
{
    NS_LOG_DEBUG("Gts Scheduling");

    if(m_coord && indication)
    {
        return;
    }

    if(m_panCoor)
    {
        return;
    }

    if(m_macDsmeACT.size())
    {
        uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // 62500 symbols/sec by default
        
        for(auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it) 
        {
            for(unsigned int i = 0; i < it->second.size(); ++i) 
            {
                if(!it->second[i].m_allocated)
                {
                    uint32_t each_Timeslot_Duration;
                    uint64_t first_Timeslot;
                    uint64_t superframe_Duration;
                    uint64_t cap_Duration;
                    uint64_t cfp_Duration;
                    Time each_Timeslot_time;
                    Time first_Timeslot_time;
                    Time superframe_time;
                    Time cap_time;
                    Time cfp_time;
                    Time startGts_time;

                    if(m_coord) 
                    {
                        each_Timeslot_Duration = m_superframeDuration / 16;
                        each_Timeslot_time = Seconds((double) each_Timeslot_Duration / symbolRate);

                        superframe_Duration = (it->second[i].m_superframeID) * m_superframeDuration;
                        superframe_time = Seconds((double)superframe_Duration / symbolRate);

                        if(isCAPReductionOn()) // TODO : 第二個multisuperframe開始的CAP不會有CAP，他會變成全部都CFP，要改為判斷這裡是不是superframeID = 0
                        {
                            first_Timeslot = each_Timeslot_Duration * 1;  // first timeslot is used for beacon tx
                            first_Timeslot_time = Seconds((double) first_Timeslot / symbolRate);

                            cfp_Duration = each_Timeslot_Duration * it->second[i].m_slotID;
                            cfp_time = Seconds((double)cfp_Duration / symbolRate);
                            startGts_time = superframe_time + first_Timeslot_time + cfp_time;
                        }
                        else
                        {
                            //! calculate the general time between beacon TX time (slot0) ~ CFP start time , which equals to CAP end time.
                            cap_Duration = each_Timeslot_Duration * (m_fnlCapSlot + 1); // calculate CAP duration period, timeslot 0(Beacon) ~ timeslot 8, so we need to plus one (CAP長度)
                            cap_time = Seconds((double)cap_Duration / symbolRate); // calculate when the CAP end

                            cfp_Duration = each_Timeslot_Duration * it->second[i].m_slotID;
                            cfp_time = Seconds((double)cfp_Duration / symbolRate);
                            
                            startGts_time = superframe_time + cap_time + cfp_time;
                            NS_LOG_DEBUG("time slot: " << each_Timeslot_Duration << " symbol" << " (" << each_Timeslot_time.As(Time::S) << ")");
                            NS_LOG_DEBUG("Superframe: " << superframe_Duration << " symbol" << " (" << superframe_time.As(Time::S) << ")");
                            NS_LOG_DEBUG("CAP period: " << cap_Duration << " symbol" << " (" << cap_time.As(Time::S) << ")");
                            NS_LOG_DEBUG("CFP period: " << cfp_Duration << " symbol" << " (" << cfp_time.As(Time::S) << ")");
                            NS_LOG_DEBUG("Start GTS at " << startGts_time.As(Time::S));
                        }
                    }
                    else // RFD , not coord
                    {
                        each_Timeslot_Duration = m_incomingSuperframeDuration / 16;
                        superframe_Duration = (it->second[i].m_superframeID) * m_incomingSuperframeDuration;
                        superframe_time = Seconds((double)superframe_Duration / symbolRate);
                        if(isCAPReductionOn())
                        {
                            first_Timeslot = each_Timeslot_Duration * 1;  // first timeslot is used for beacon tx
                            first_Timeslot_time = Seconds((double) first_Timeslot / symbolRate);
                            // NS_LOG_INFO("m_macBeaconRxTime = " << m_macBeaconRxTime);
                            // NS_LOG_INFO("first_Timeslot_time = " << first_Timeslot_time);
                            first_Timeslot_time -= (Simulator::Now() - m_macBeaconRxTime);
                            cfp_Duration = each_Timeslot_Duration * it->second[i].m_slotID;
                            cfp_time = Seconds((double)cfp_Duration / symbolRate);
                            startGts_time = superframe_time + first_Timeslot_time + cfp_time;
                        }
                        else
                        {
                            cap_Duration = each_Timeslot_Duration * (m_incomingFnlCapSlot + 1);
                            cap_time = Seconds((double)cap_Duration / symbolRate);
                            cap_time -= (Simulator::Now() - m_macBeaconRxTime);
                            cfp_Duration = each_Timeslot_Duration * it->second[i].m_slotID;
                            cfp_time = Seconds((double)cfp_Duration / symbolRate);
                            startGts_time = superframe_time + cap_time + cfp_time;
                        }
                    }

                    if(it->second[i].m_direction) // GTS for RX
                    {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGts_time     // Schedule GTS start event
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::INCOMING
                                                        , it->second[i].m_superframeID
                                                        , i);                
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);
                    } 
                    else  // GTS for TX
                    {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGts_time     // Schedule GTS start event
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::OUTGOING
                                                        , it->second[i].m_superframeID
                                                        , i);                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);
                    }
                }
            }
        }
    }
}

void LrWpanMac::PurgeDsmeACT()
{
    NS_LOG_DEBUG("Dsme ACT Purging");

    // deallocate or expire
    if(m_macDsmeACT.size())
    {
        for(auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it)
        {
            for(unsigned int i = 0; i < it->second.size(); ++i)
            {
                if(it->second[i].m_deallocated)  // Check ACT element has been deallocated or not.
                { 
                    it->second.erase(it->second.begin() + i);
                }
            }
        }
    }
}

void LrWpanMac::StartGTS(SuperframeType superframeType, uint16_t superframeID, int idx) {
    uint32_t activeSlot;

    if (m_macDsmeACT[superframeID][idx].m_deallocated) {
        return;
    }

    if (m_macDsmeACT[superframeID][idx].m_expired) {
        return;
    }

    m_curGTSSuperframeID = superframeID;
    m_curGTSIdx = idx;
    m_currentGTSIdx = (int)m_macDsmeACT[superframeID][idx].m_slotID;

    NS_LOG_DEBUG("Current superframeID : " << m_curGTSSuperframeID << " GTSIDx : " << (int)m_macDsmeACT[superframeID][idx].m_slotID);

    if (m_macDsmeACT[superframeID][idx].m_direction) 
    {   // TODO : 這裡怪怪的，應該要是判斷經過m_macDSMEGTSExpirationTime次數的MSF沒收到封包，才要執行expiration，而不是算次數
        // m_macDsmeACT[superframeID][idx].m_cnt++;
    }

    if (m_macDsmeACT[superframeID][idx].m_allocated) 
    {
        Time startGtsTime;        
        uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);  // symbols per second

        if (m_coord) 
        {
            startGtsTime = Seconds((double) m_multiSuperframeDuration / symbolRate);
        } 
        else 
        {
            startGtsTime = Seconds((double) m_incomingMultisuperframeDuration / symbolRate);
        }

        /*
         * Schedule StartGTS  
        */
        if (m_macDsmeACT[superframeID][idx].m_direction) // RX
        {
            m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::INCOMING
                                                        , m_macDsmeACT[superframeID][idx].m_superframeID
                                                        , idx);
            
            NS_LOG_DEBUG("Schedule an Rx GTS that will launch at:" 
                        << "(" << startGtsTime.As(Time::S) << ")");

        } 
        else // TX
        {
            m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::OUTGOING
                                                        , m_macDsmeACT[superframeID][idx].m_superframeID
                                                        , idx);
                        
            NS_LOG_DEBUG("Schedule an Tx GTS that will launch at:" 
                        << "(" << startGtsTime.As(Time::S) << ")");
        }
    }

    if (m_coord) 
    {
        activeSlot = m_superframeDuration / 16;
    } 
    else 
    {
        activeSlot = m_incomingSuperframeDuration / 16;
    }

    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);
    uint64_t gtsDuration = activeSlot * m_macDsmeACT[superframeID][idx].m_numSlot;

    // debug
    Time endGtsTime; 
    if (m_macDsmeACT[superframeID][idx].m_slotID == 6 || m_macDsmeACT[superframeID][idx].m_slotID == 14) 
    {
        // ? 這裡因為時間sync的關係，不讓ENDGTS先完成會使macState出錯，是一個workaround
        endGtsTime = Seconds((double)gtsDuration / symbolRate) - NanoSeconds(10); 
        // endGtsTime = Seconds((double)gtsDuration / symbolRate);
    }
    else 
    {
        endGtsTime = Seconds((double)gtsDuration / symbolRate) - NanoSeconds(1);
    }

    if (superframeType == OUTGOING) 
    {
        NS_LOG_DEBUG("Outgoing Gts Tx duration " << gtsDuration << " symbols ("
                                                 << endGtsTime.As(Time::S) << ")");
    
        // Scheduling EndGTS time
        m_gtsEvent = Simulator::Schedule(endGtsTime
                                        , &LrWpanMac::EndGTS
                                        , this
                                        , SuperframeType::OUTGOING);

        NS_LOG_DEBUG("Channel Offset: " << m_macDsmeACT[superframeID][idx].m_channelID); // debug
        
        // Channel Hopping setting
        uint16_t ch;
        uint8_t cfpSlotNum;

        if (m_forDsmeNetDeviceIntegrateWithHigerLayer)   // For dsme-net-device setting
        {
            ch = (m_macDsmeACT[superframeID][idx].m_channelID + m_macDsmeACT[superframeID][idx].m_slotID) 
                            % m_numOfChannels;
        }  
        else 
        {
            if(isCAPReductionOn() && m_curGTSSuperframeID != 0)
            {
                cfpSlotNum = 15; // parameters " l " in the formula.
                NS_LOG_DEBUG("m_macPANCoordinatorBSN = " << (uint16_t)m_macPANCoordinatorBSN.GetValue());
                // Calculate the channel ID for channel hopping by spec.
                ch = (m_curGTSSuperframeID * cfpSlotNum + m_macDsmeACT[superframeID][idx].m_slotID + m_macDsmeACT[superframeID][idx].m_channelID + (uint16_t)m_macPANCoordinatorBSN.GetValue()) % m_numOfChannels;
            }
            else
            {
                cfpSlotNum = 7; // parameters " l " in the formula.
                NS_LOG_DEBUG("m_macPANCoordinatorBSN = " << (uint16_t)m_macPANCoordinatorBSN.GetValue());
                ch = (m_curGTSSuperframeID * cfpSlotNum + m_macDsmeACT[superframeID][idx].m_slotID + m_macDsmeACT[superframeID][idx].m_channelID + (uint16_t)m_macPANCoordinatorBSN.GetValue()) % m_numOfChannels;
            }
        }

        ch += 11; // Because the channel in 802.15.4 2.4GHz band channel use 11~26 , we need to add offset 11 here.
        NS_LOG_DEBUG("Hop to Channel Num: " << ch); // debug

        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = ch;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);                                
        
    } 
    else 
    {
        NS_LOG_DEBUG("Incoming Gts Rx duration " << gtsDuration << " symbols ("
                                                 << endGtsTime.As(Time::S) << ")");
        
        // turn on RX
        
        m_incGtsEvent = Simulator::Schedule(endGtsTime
                                        , &LrWpanMac::EndGTS
                                        , this
                                        , SuperframeType::INCOMING);

        NS_LOG_DEBUG("Channel Offset: " << m_macDsmeACT[superframeID][idx].m_channelID); // debug
        
        // channel hopping part
        uint16_t ch;
        uint8_t cfpSlotNum;
        // For dsme-net-device setting
        if (m_forDsmeNetDeviceIntegrateWithHigerLayer) {
            ch = (m_macDsmeACT[superframeID][idx].m_channelID + m_macDsmeACT[superframeID][idx].m_slotID) 
                            % m_numOfChannels;
        } else 
        {
            if(isCAPReductionOn() && m_curGTSSuperframeID != 0)
            {
                cfpSlotNum = 15; // parameters " l " in the formula.
                NS_LOG_DEBUG("m_macPANCoordinatorBSN = " << (uint16_t)m_macPANCoordinatorBSN.GetValue());
                ch = (m_curGTSSuperframeID * cfpSlotNum + m_macDsmeACT[superframeID][idx].m_slotID + m_macDsmeACT[superframeID][idx].m_channelID + (uint16_t)m_macPANCoordinatorBSN.GetValue()) % m_numOfChannels;
            }
            else
            {
                cfpSlotNum = 7; // parameters " l " in the formula.
                NS_LOG_DEBUG("m_macPANCoordinatorBSN = " << (uint16_t)m_macPANCoordinatorBSN.GetValue());
                ch = (m_curGTSSuperframeID * cfpSlotNum + m_macDsmeACT[superframeID][idx].m_slotID + m_macDsmeACT[superframeID][idx].m_channelID + (uint16_t)m_macPANCoordinatorBSN.GetValue()) % m_numOfChannels;
            }
        }

        ch += 11; // Because the channel in 802.15.4 2.4GHz band channel use 11~26 , we need to add offset 11 here.
        NS_LOG_DEBUG("Hop to Channel Num: " << ch); // debug

        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = ch;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);                        
    }

    SetLrWpanMacStateToGTS(superframeID, idx);

    // howard: 新增 GTS Forwarding
    if(!m_txQueue.empty() && m_gtsContinuePktSendingFromCap)
    {
        Ptr<TxQueueElement> txQElement = m_txQueue.front();
        m_txPkt = txQElement->txQPkt;
    }

    if(m_txPkt && m_gtsContinuePktSendingFromCap)
    {
        ChangeMacState(MAC_GTS_SENDING);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
    }

    // DSME-TODO
    if (m_txPktGts) {
        m_txPkt = m_txPktGts;
        ChangeMacState(MAC_GTS_SENDING);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
    }

    if(m_groupAckPolicy)
    {
        switch (m_groupAckPolicy)
        {
            case GROUP_ACK_LEGACY:

                if((int)m_macDsmeACT[superframeID][idx].m_slotID == m_legacyGroupAck.GetGACK1SlotID() ||
                (int)m_macDsmeACT[superframeID][idx].m_slotID == m_legacyGroupAck.GetGACK2SlotID())
                {
                    if(m_macDsmeACT[superframeID][idx].m_direction == 0 && m_coord) // TX
                    {
                        // Delay 3ms for the GTS TX-RX time diff
                        // This will avoid for sending GACK packet before the receiver turn on RX.
                        Simulator::Schedule(Time("3ms"), 
                                            &LrWpanMac::SendLegacyGroupAck,
                                            this);
                        m_legacyGackIndexCounter = 0;
                    }
                }

                break;
            case GROUP_ACK_ENHANCED:

                if((int)m_macDsmeACT[superframeID][idx].m_slotID == ENHANCED_GROUP_ACK_FIRST_SLOT ||
                (int)m_macDsmeACT[superframeID][idx].m_slotID == ENHANCED_GROUP_ACK_SECOND_SLOT)
                {
                    if(m_macDsmeACT[superframeID][idx].m_direction == 0) // TX
                    {
                        // Delay 3ms for the GTS TX-RX time diff
                        // This will avoid for sending GACK packet before the receiver turn on RX.
                        Simulator::Schedule(Time("3ms"), 
                                            &LrWpanMac::SendEnhancedGroupAck,
                                            this);
                    }
                }

                break;   
            default:
                break;
        }
    }

}

void LrWpanMac::EndGTS(SuperframeType superframeType) {
    if (superframeType == OUTGOING) {
            NS_LOG_DEBUG("Outgoing Tx GTS End. ");
    } else {
            NS_LOG_DEBUG("Incoming RX GTS End. ");
    }

    LrWpanPhyPibAttributes pibAttr;
    pibAttr.phyCurrentChannel = m_originalChannelInCAP;
    m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);

    if (m_lrWpanMacState == MAC_ACK_PENDING) {
        m_ackWaitTimeout.Cancel();
    }

    m_setMacState.Cancel();
    m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);

    m_txPkt = nullptr;
    m_txPktGts = nullptr;

    m_phy->CancelPdDataRequest();

    // m_gtsRetrieve = false;
}

void LrWpanMac::StartSuperframe() 
{
    // howard: 不知道為何要打這個
    // m_startFirstSuperframeEvent.Cancel();

    // 除了第一個 Superframe，其他 Superframe 都會進來這裡把 Superframe + 1
    if(m_curSuperframeIDx < (m_numOfSuperframes / m_numOfMultisuperframes) - 1 && !m_isFirstSuperframe)
    {
        m_curSuperframeIDx++;
    }
    
    NS_LOG_DEBUG("************************************ SuperframeIDx : " << GetSuperframeIDx() << " start ************************************");

    uint64_t symbolRate;
    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    Time nextSuperframestartTime = Seconds((double) m_superframeDuration / symbolRate);
    Simulator::Schedule(nextSuperframestartTime, &LrWpanMac::StartSuperframe, this);
    NS_LOG_INFO("下個 Superframe 會發生在: " << (Simulator::Now() + nextSuperframestartTime).As(Time::S));
    m_isFirstSuperframe = false;
}

void LrWpanMac::StartMultisuperframe(SuperframeType superframeType)
{
    static int Superframe_count = 0;
    static int Multisuperframe_count = 0;

    m_multisuperframeSeq++;

    Time endMultisuperframeTime;
    uint64_t symbolRate;    
    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    // Reset the current superframeIDx to zero.
    m_isFirstSuperframe = true;
    SetSuperframeIDx(0);
    if(superframeType == OUTGOING) 
    {
        // 計算 Multi-Superframe 會持續多久時間
        endMultisuperframeTime = Seconds((double) m_multiSuperframeDuration / symbolRate);

        if(Multisuperframe_count == 0)
        {
            NS_LOG_INFO("初始化 Multi-Superframe");
            Multisuperframe_count++;
        }

        // 當前模擬時間 + Multi-Superframe 會持續多久時間
        NS_LOG_DEBUG("下個 Multi-Superframe 會發生在: " << (Simulator::Now() + endMultisuperframeTime).As(Time::S));
                    
        // Schedule next multisuperframe start timing, and keep calculating next time , run forever
        m_multisuperframeEndEvent = Simulator::Schedule(endMultisuperframeTime, 
                                                        &LrWpanMac::StartMultisuperframe,
                                                        this,
                                                        SuperframeType::OUTGOING);
    }

    if(Simulator::Now() <= Seconds((double) m_superframeDuration / symbolRate))
    {

        Time nextSuperframestartTime = Seconds((double) m_superframeDuration / symbolRate);
        m_startFirstSuperframeEvent = Simulator::Schedule(nextSuperframestartTime, &LrWpanMac::StartSuperframe, this);

        if(Superframe_count == 0)
        {
            NS_LOG_INFO("初始化 Superframe");
            NS_LOG_INFO("第一個 Superframe 會發生在: " << nextSuperframestartTime.As(Time::S));
            Superframe_count++;
        }
    }
    
    // howard: 不知道為何不會進來這裡
    else // INCOMING superframe 
    {
        endMultisuperframeTime = Seconds((double) m_incomingMultisuperframeDuration / symbolRate);

        // substract the Beacon Rx Time slots
        endMultisuperframeTime -= (Simulator::Now() - m_macBeaconRxTime);
        NS_LOG_DEBUG("Start Incoming multisuperframe multisuperframe Active Portion: " << m_incomingMultisuperframeDuration << " symbols"
                    << " (" << endMultisuperframeTime.As(Time::S) << ")");
    }
}

void LrWpanMac::StartRemainingPeriod(SuperframeType superframeType)
{
    uint64_t remainingDurationUntilNextBcn;   // end
    Time endRemainingTime;
    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if(superframeType == INCOMING)
    {
        remainingDurationUntilNextBcn = m_incomingBeaconInterval - m_incomingSuperframeDuration;
        // NS_LOG_INFO("m_incomingBeaconInterval = " << m_incomingBeaconInterval);
        // NS_LOG_INFO("m_incomingSuperframeDuration = " << m_incomingSuperframeDuration);

        endRemainingTime = Seconds((double)remainingDurationUntilNextBcn / symbolRate);

        if(remainingDurationUntilNextBcn > 0)
        {
            m_incSuperframeStatus = REMAINING;
        }

        m_beaconEvent = Simulator::Schedule(endRemainingTime, &LrWpanMac::AwaitBeacon, this);
    } 
    else 
    {
        // DSME-TODO
        //! Calculate the next enhanced beacon TX time and schedule the SendOneEnhancedBeacon()
        //! 計算方式為 : BI - (現在的時間 - 上一次TX EB的時間)
        endRemainingTime = Seconds((double)(m_beaconInterval) / symbolRate);
        endRemainingTime -= (Simulator::Now() - m_startOfBcnSlot);
        remainingDurationUntilNextBcn = endRemainingTime.ToInteger(Time::S) * symbolRate;

        std::cout << Simulator::Now().As(Time::MS) << std::endl;
        // NS_LOG_DEBUG(" m_startOfBcnSlot = "<< m_startOfBcnSlot.As(Time::S));
        // NS_LOG_DEBUG(" endRemainingTime = "<< endRemainingTime.As(Time::S));

        if(remainingDurationUntilNextBcn > 0)
        {
            m_outSuperframeStatus = REMAINING;
        }
                                              
        m_beaconEvent = Simulator::Schedule(endRemainingTime, &LrWpanMac::SendOneEnhancedBeacon, this);
        Simulator::Schedule(endRemainingTime, &LrWpanMac::PurgeDsmeACT, this);
    }
}

void
LrWpanMac::AwaitBeacon()
{
    m_incSuperframeStatus = BEACON;

    // TODO: If the device waits more than the expected time to receive the beacon (wait = 46
    // symbols for default beacon size)
    //       it should continue with the start of the incoming CAP even if it did not receive the
    //       beacon. At the moment, the start of the incoming CAP is only triggered if the beacon is
    //       received. See MLME-SyncLoss for details.
}

void
LrWpanMac::BeaconSearchTimeout() {
    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if (m_numLostBeacons > aMaxLostBeacons) {
        // NS_LOG_DEBUG("SYNC FAILED"); // debug

        if (!m_mlmeSyncLossIndicationCallback.IsNull()) {
            MlmeSyncLossIndicationParams syncLossParams;
            // syncLossParams.m_logCh =
            syncLossParams.m_lossReason = MLMESYNCLOSS_BEACON_LOST;
            syncLossParams.m_panId = m_macPanId;
            m_mlmeSyncLossIndicationCallback(syncLossParams);

            m_beaconTrackingOn = false;
            m_numLostBeacons = 0;
        }

    } else {
        m_numLostBeacons++;

        // Search for one more beacon
        uint64_t searchSymbols;
        Time searchBeaconTime;
        searchSymbols = (((uint64_t)1 << m_incomingBeaconOrder) + 1) * aBaseSuperframeDuration;
        searchBeaconTime = Seconds((double)searchSymbols / symbolRate);
        m_trackingEvent =
            Simulator::Schedule(searchBeaconTime, &LrWpanMac::BeaconSearchTimeout, this);
    }
}

void LrWpanMac::CheckQueue() {
    NS_LOG_FUNCTION(this);

    // Pull a packet from the queue and start sending if we are not already sending.
    if(m_lrWpanMacState == MAC_IDLE && !m_txQueue.empty() && !m_setMacState.IsRunning()) {
        // TODO: this should check if the node is a coordinator and using the outcoming superframe
        // not just the PAN coordinator
        if(m_csmaCa->IsUnSlottedCsmaCa() || (m_outSuperframeStatus == CAP && m_coord) || m_incSuperframeStatus == CAP)
        {
            // check MAC is not in a IFS
            if (!m_ifsEvent.IsRunning()) {
                Ptr<TxQueueElement> txQElement = m_txQueue.front();
                m_txPkt = txQElement->txQPkt;

                m_setMacState =
                    Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_CSMA);
            }
        }
    }
}

GtsFields
LrWpanMac::GetGtsFields()
{
    GtsFields gtsFields;

    // TODO: Logic to populate the GTS Fields from local information here

    return gtsFields;
}

PendingAddrFields
LrWpanMac::GetPendingAddrFields()
{
    PendingAddrFields pndAddrFields;
    // DSME-TODO
    // TODO: Logic to populate the Pending Address Fields from local information here
    uint8_t numOfShortAddr = 0;
    uint8_t numOfExtAddr = 0;    

    LrWpanMacHeader peekedMacHdr;

    for (auto iter = m_indTxQueue.begin(); iter != m_indTxQueue.end(); iter++) {
        (*iter)->txQPkt->PeekHeader(peekedMacHdr);

        if (peekedMacHdr.GetDstAddrMode() == SHORT_ADDR) {
            numOfShortAddr++;
            pndAddrFields.AddAddress(peekedMacHdr.GetShortDstAddr());
        }
    }

    pndAddrFields.SetNumOfShortAdrr(numOfShortAddr);
    pndAddrFields.SetNumOfExtAdrr(numOfExtAddr);
    
    return pndAddrFields;
}

uint32_t LrWpanMac::GetNumOfMultisuperframesInABeaconInterval() const {
    return m_numOfMultisuperframes;
}

uint64_t LrWpanMac::GetNumOfSuperframesInABeaconInterval() const {
    return m_numOfSuperframes;
}

void LrWpanMac::ResizeMacDSMESAB(bool capReduction, uint8_t bcnOrder, uint8_t sfrmOrd) {
    if (capReduction) {
        m_macDSMESAB.resize(static_cast<uint64_t>(1 << (bcnOrder - sfrmOrd)), 0);
    } else {
        m_macDSMESABCapOff.resize(static_cast<uint64_t>(1 << (bcnOrder - sfrmOrd)), 0);
    }
}

void LrWpanMac::ResizeScheduleGTSsEvent(uint8_t bcnOrder, 
                                        uint8_t multisfrmOrd, 
                                        uint8_t sfrmOrd) {

    m_numOfMultisuperframes = static_cast<uint32_t>(1 << (bcnOrder - multisfrmOrd));
    m_numOfSuperframes = static_cast<uint64_t>(1 << (bcnOrder - sfrmOrd));
    
    // Guess : beacause the multisuperframe will repeat the duty cycle,
    //         here divide the numOfMultisuperframes in order to schedule only one time.
    m_scheduleGTSsEvent.resize(m_numOfSuperframes / m_numOfMultisuperframes);
    // m_scheduleGTSsEvent.resize(m_numOfSuperframes / m_numOfMultisuperframes);
}

uint8_t LrWpanMac::GenerateSABSubBlock(uint8_t slotID, uint8_t numSlot) {
    uint8_t subBlk = 0b00000000;

    subBlk |= (0b00000001 << slotID);

    for (int i = 1; i < numSlot; ++i) {
        subBlk |= (0b00000001 << (slotID + i));
    }

    return subBlk;
}

uint16_t LrWpanMac::GenerateSABSubBlockCapOn(uint8_t slotID, uint8_t numSlot) {
    uint16_t subBlk = 0b0000000000000000;

    subBlk |= (0b0000000000000001 << slotID);

    for (int i = 1; i < numSlot; ++i) {
        subBlk |= (0b0000000000000001 << (slotID + i));
    }

    return subBlk;
}

void LrWpanMac::AddDsmeACTEntity(uint16_t superframeID, macDSMEACTEntity entity) {
    if (superframeID >= (m_numOfSuperframes / m_numOfMultisuperframes)) 
    {
        // Sanity check
        NS_FATAL_ERROR(this << " the superframe ID: " << superframeID 
                          << " is larger than the number of superframe in: " << (m_numOfSuperframes / m_numOfMultisuperframes));
    }

    if (m_macCAPReductionFlag) 
    {
        if (entity.m_slotID >= 15)  // Sanity check
        {
            NS_FATAL_ERROR(this << " the slot ID: " << entity.m_slotID << " is larger than the number of CFP timeslots: 15");
        }

    }
    else 
    {
        if (entity.m_slotID >= 7)  // Sanity check
        {
            NS_FATAL_ERROR(this << " the slot ID: " << entity.m_slotID << " is larger than the number of CFP timeslots: 7");
        }
    }

    m_macDsmeACT[superframeID].push_back(entity);
}

void LrWpanMac::SetChannelOffset(uint16_t offset) {
    m_macChannelOfs = offset;
}

void LrWpanMac::AddPanDescriptor(PanDescriptor descriptor) {
    m_panDescriptorList.push_back(descriptor);
}

void LrWpanMac::SetNumOfChannelSupported(uint16_t num) {
    m_numOfChannels = num;
}

bool LrWpanMac::IsIncomingSuperframe() {
    return m_incSuperframe;
}

void LrWpanMac::SetRecord(std::map<Address, std::pair<Address, std::vector<int64_t>>> &record) {
    m_record = &record;
}

void LrWpanMac::SetRecord(std::map<std::pair<Address, Address>, std::vector<std::pair<int64_t, int64_t>>> &record) {
    m_record2 = &record;
}

void LrWpanMac::ReceiveRecordKeyAndValueIdx(std::pair<Address, Address> recordkey, unsigned int recordValueIdx) {
    m_recordkey = recordkey;
    // m_recordValueIdx = recordValueIdx;

    m_recordKeys.push_back(std::move(m_recordkey));
}

void LrWpanMac::SetBecomeCoordAfterAssociation(bool on) {
    m_becomeCoord = on;
}

void
LrWpanMac::SetCsmaCa(Ptr<LrWpanCsmaCa> csmaCa)
{
    m_csmaCa = csmaCa;
}

void
LrWpanMac::SetPhy(Ptr<LrWpanPhy> phy)
{
    m_phy = phy;
}

Ptr<LrWpanPhy>
LrWpanMac::GetPhy()
{
    return m_phy;
}

void
LrWpanMac::SetMcpsDataIndicationCallback(McpsDataIndicationCallback c)
{
    m_mcpsDataIndicationCallback = c;
}

void
LrWpanMac::SetMlmeAssociateIndicationCallback(MlmeAssociateIndicationCallback c)
{
    m_mlmeAssociateIndicationCallback = c;
}

void LrWpanMac::SetMlmeDisassociateIndicationCallback(MlmeDisassociateIndicationCallback c) {
    m_mlmeDisassociateIndicationCallback = c;
}

void
LrWpanMac::SetMlmeCommStatusIndicationCallback(MlmeCommStatusIndicationCallback c)
{
    m_mlmeCommStatusIndicationCallback = c;
}

void
LrWpanMac::SetMlmeStartRequestCallback(MlmeStartRequestCallback c)
{
    m_mlmeStartRequestCallback = c;
}

void
LrWpanMac::SetMcpsDataConfirmCallback(McpsDataConfirmCallback c)
{
    m_mcpsDataConfirmCallback = c;
}

void
LrWpanMac::SetMlmeStartConfirmCallback(MlmeStartConfirmCallback c)
{
    m_mlmeStartConfirmCallback = c;
}

void
LrWpanMac::SetMlmeScanConfirmCallback(MlmeScanConfirmCallback c)
{
    m_mlmeScanConfirmCallback = c;
}

void
LrWpanMac::SetMlmeAssociateConfirmCallback(MlmeAssociateConfirmCallback c)
{
    m_mlmeAssociateConfirmCallback = c;
}

void LrWpanMac::SetMlmeDisassociateConfirmCallback(MlmeDisassociateConfirmCallback c) {
    m_mlmeDisassociateConfirmCallback = c;
}

void
LrWpanMac::SetMlmeBeaconNotifyIndicationCallback(MlmeBeaconNotifyIndicationCallback c)
{
    m_mlmeBeaconNotifyIndicationCallback = c;
}

void
LrWpanMac::SetMlmeSyncLossIndicationCallback(MlmeSyncLossIndicationCallback c)
{
    m_mlmeSyncLossIndicationCallback = c;
}

void
LrWpanMac::SetMlmePollConfirmCallback(MlmePollConfirmCallback c)
{
    m_mlmePollConfirmCallback = c;
}

void LrWpanMac::SetMlmeDsmeGtsIndicationCallback(MlmeDsmeGtsIndicationCallback c) {
    m_mlmeDsmeGtsIndicationCallback = c;
}

void LrWpanMac::SetMlmeOrphanIndicationCallback(MlmeOrphanIndicationCallback c) {
    m_mlmeOrphanIndicationCallback = c;
}

void LrWpanMac::SetMlmeDsmeInfoIndicationCallback(MlmeDsmeInfoIndicationCallback c) {
    m_mlmeDsmeInfoIndicationCallback = c;
}

void LrWpanMac::SetMlmeDsmeInfoConfirmCallback(MlmeDsmeInfoConfirmCallback c) {
    m_mlmeDsmeInfoConfirmCallback = c;
}

void LrWpanMac::SetMlmeDsmeGtsConfirmCallback(MlmeDsmeGtsConfirmCallback c) {
    m_mlmeDsmeGtsConfirmCallback = c;
}

void LrWpanMac::PdDataIndication(uint32_t psduLength, Ptr<Packet> p, uint8_t lqi) {

    // howard: 把封包傳到 MAC Layer
    // This indication occur when phy layer received a packet and transfer the packet to MAC the layer.

    NS_ASSERT(m_lrWpanMacState == MAC_IDLE || m_lrWpanMacState == MAC_ACK_PENDING ||
              m_lrWpanMacState == MAC_CSMA || m_lrWpanMacState == MAC_GTS);
    NS_LOG_FUNCTION(this << psduLength << p << (uint16_t)lqi);

    bool acceptFrame;

    Ptr<Packet> originalPkt = p->Copy(); // because we will strip headers
    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    // Feed packet capture and sniffer traces on RX as well
    m_promiscSnifferTrace(originalPkt);
    m_snifferTrace(originalPkt);

    LrWpanMacTrailer receivedMacTrailer;
    p->RemoveTrailer(receivedMacTrailer);

    if (Node::ChecksumEnabled()) {
        receivedMacTrailer.EnableFcs(true);
    }

    // level 1 filtering
    if (!receivedMacTrailer.CheckFcs(p)) 
    {
        m_macRxDropTrace(originalPkt);
    } 
    else 
    {
        LrWpanMacHeader receivedMacHdr;
        p->RemoveHeader(receivedMacHdr);

        McpsDataIndicationParams params;
        params.m_dsn = receivedMacHdr.GetSeqNum();
        params.m_mpduLinkQuality = lqi;

        params.m_srcPanId = receivedMacHdr.GetSrcPanId();
        params.m_srcAddrMode = receivedMacHdr.GetSrcAddrMode();
        params.m_srcAddr = receivedMacHdr.GetShortSrcAddr();

        params.m_dstPanId = receivedMacHdr.GetDstPanId();
        params.m_dstAddrMode = receivedMacHdr.GetDstAddrMode();
        params.m_dstAddr = receivedMacHdr.GetShortDstAddr();

        // NS_LOG_DEBUG("receivedMacHdr.GetSrcPanId()" << receivedMacHdr.GetSrcPanId());
        // NS_LOG_DEBUG("receivedMacHdr.GetDstPanId()" << receivedMacHdr.GetDstPanId());

        // NS_LOG_DEBUG("receivedMacHdr.GetShortDstAddr()" << receivedMacHdr.GetShortDstAddr());
        // NS_LOG_DEBUG("receivedMacHdr.GetShortSrcAddr()" << receivedMacHdr.GetShortSrcAddr());

        // For Hilow
        if (m_acceptAllHilowPkt && receivedMacHdr.GetType() == LrWpanMacHeader::LRWPAN_MAC_DATA) {
            if (!m_mcpsDataIndicationCallback.IsNull()) {
                m_mcpsDataIndicationCallback(params, p);

            } else {
                NS_LOG_ERROR(this << " Data Indication Callback not initialized");
            }
        }
        
        // level 3 frame filtering
        acceptFrame = (receivedMacHdr.GetType() != LrWpanMacHeader::LRWPAN_MAC_RESERVED);

        if (acceptFrame) {
            // DSME-TODO
            // acceptFrame = (receivedMacHdr.GetFrameVer() <= 1);
            acceptFrame = (receivedMacHdr.GetFrameVer() <= 2);
        }

        if (acceptFrame && (receivedMacHdr.GetDstAddrMode() > 1)) {
            // DSME-TODO
            // Accept frame if:

            // 1) Have the same macPanId
            // 2) Or is Message to all PANs
            // 3) Or Is a beacon and the macPanId is not present (bootstrap)
            acceptFrame = ((receivedMacHdr.GetDstPanId() == m_macPanId ||
                            receivedMacHdr.GetDstPanId() == 0xffff) ||
                           (m_macPanId == 0xffff && receivedMacHdr.IsBeacon())) ||
                          (m_macPanId == 0xffff && receivedMacHdr.IsCommand());
        }

        if (acceptFrame && (receivedMacHdr.GetShortDstAddr() == Mac16Address("FF:FF")))
        {
            // A broadcast message (e.g. beacons) should not be received by the device who
            // issues it.
            acceptFrame = (receivedMacHdr.GetShortSrcAddr() != GetShortAddress());
            // TODO: shouldn't this be filtered by the PHY?
        }

        if (acceptFrame && (receivedMacHdr.GetDstAddrMode() == SHORT_ADDR))
        {
            if (receivedMacHdr.GetShortDstAddr() == m_shortAddress)
            {
                // unicast, for me
                acceptFrame = true;
            }
            else if (receivedMacHdr.GetShortDstAddr().IsBroadcast() ||
                     receivedMacHdr.GetShortDstAddr().IsMulticast())
            {
                // broadcast or multicast
                if (receivedMacHdr.IsAckReq())
                {
                    // discard broadcast/multicast with the ACK bit set
                    // DSME-TODO: 會讓 Dsme Gts Reponse 沒辦法被接收, 所以先註解掉
                    // acceptFrame = false;
                }
                else
                {
                    acceptFrame = true;
                }
            }
            else
            {
                acceptFrame = false;
            }
        }

        if (acceptFrame) {
            m_macRxTrace(originalPkt);
            // \todo: What should we do if we receive a frame while waiting for an ACK?
            //        Especially if this frame has the ACK request bit set, should we reply with
            //        an ACK, possibly missing the pending ACK?

            // If the received frame is a frame with the ACK request bit set, we immediately
            // send back an ACK. If we are currently waiting for a pending ACK, we assume the
            // ACK was lost and trigger a retransmission after sending the ACK.
            if ((receivedMacHdr.IsData() || receivedMacHdr.IsCommand()) &&
                receivedMacHdr.IsAckReq() &&
                (receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4_2003 
                 || receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4_2006) 
                ) 
            {
                // Cancel any pending MAC state change, ACKs have higher priority.
                m_setMacState.Cancel();
                ChangeMacState(MAC_IDLE);

                // save received packet to process the appropriate indication/response after
                // sending ACK (PD-DATA.confirm)
                m_rxPkt = originalPkt->Copy();

                // LOG Commands with ACK required.
                CommandPayloadHeader receivedMacPayload;
                p->PeekHeader(receivedMacPayload);

                //! Send Ack here when received a data pkt 
                // TODO : Need to check GACK is enabled or not

                if(m_groupAckPolicy == GROUP_ACK_ENHANCED)
                {
                    NS_LOG_DEBUG("Enhanced Group Ack enabled, aggregate acks into Group Acks");
                    // start generate the hash table key and write into bitmap.
                    NS_LOG_DEBUG("Generate the hash table key , addr = " << receivedMacHdr.GetShortSrcAddr() << " seq = " 
                                    << (uint32_t)receivedMacHdr.GetSeqNum() << " Push back into packet buffer");
                    uint64_t bitLocation = GenerateHashTableKey(receivedMacHdr.GetShortSrcAddr(),(uint32_t)receivedMacHdr.GetSeqNum());

                    // Set group ack bitmap according to the key location
                    m_enhancedGACKBitmap |= ((uint64_t)1 << bitLocation);
                    NS_LOG_DEBUG("bitLocation = " << bitLocation);
                    PrintGroupAckBitmap();
        
                    if (m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) 
                    { 
                        ChangeMacState(MAC_GTS);
                    }

                }
                else if(m_groupAckPolicy == GROUP_ACK_LEGACY)
                {
                    NS_LOG_DEBUG("Legacy Group Ack enabled, aggregate acks into Group Acks");
                    // 0. Parse slot location and Set the location GACK bitmap to 1
                    m_legacyGackBitmap |= (1 << m_curGTSIdx);
                    std::bitset<16> printBitmap = m_legacyGackBitmap;
                    NS_LOG_DEBUG("m_legacyGackBitmap update as = " << printBitmap);
                    // 1. Parse TX addr to bitmap location - GACK Device List
                    uint8_t buffer16MacAddr[2];
                    receivedMacHdr.GetShortSrcAddr().CopyTo(buffer16MacAddr);
                    m_legacyGackDevList |= (1 << ((int)(buffer16MacAddr[0] << 8) | buffer16MacAddr[1]));
                    // 2. Record Gack Index
                    m_legacyGackIdx |= ((m_curGTSIdx & (0x0F)) << (m_legacyGackIndexCounter * 4));
                    m_legacyGackIndexCounter++;
                    // 3. Parse slot location and Set GTS directions to 1 
                    m_legacyGackDirections = 0; // TX for uplink always is 0 , no need to update bitmap.
                    if (m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) 
                    { 
                        ChangeMacState(MAC_GTS);
                    }
                }
                else // Normal flow
                {
                    // m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SendAck, this, receivedMacHdr.GetSeqNum());
                }                                               
            }

            if (receivedMacHdr.GetSrcAddrMode() == SHORT_ADDR) 
            {
                NS_LOG_DEBUG("Packet from " << params.m_srcAddr);
            } 

            if (receivedMacHdr.GetDstAddrMode() == SHORT_ADDR) 
            {
                NS_LOG_DEBUG("Packet to " << params.m_dstAddr);
            }

            if(receivedMacHdr.IsBeacon())
            {
                // DSME-TODO
                // The received beacon size in symbols
                // Beacon = 5 bytes Sync Header (SHR) +  1 byte PHY header (PHR) + PSDU (default
                // 17 bytes)
                m_rxBeaconSymbols = m_phy->GetPhySHRDuration() +
                                    1 * m_phy->GetPhySymbolsPerOctet() +
                                    (originalPkt->GetSize() * m_phy->GetPhySymbolsPerOctet());

                // The start of Rx beacon time and start of the Incoming superframe Active
                // Period
              
                // howard: 不知道這裡為何用 MillSeconds
                m_macBeaconRxTime = Simulator::Now() - MilliSeconds(double(m_rxBeaconSymbols) / symbolRate);

                if(m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4)
                {
                    NS_LOG_DEBUG("Enhanced Beacon Received; forwarding up (m_macBeaconRxTime: " << m_macBeaconRxTime.As(Time::S) << ")");
                    /**
                     * !TODO : Here is a workaround, assume the pan-C addr is 00:01
                     * Purpose - Cap reduction feature : Let the non coord device can sync cap and cfp period correctly.
                    */
                    Mac16Address panCoordAddr("00:01");
                    if(m_coord == 0 && receivedMacHdr.GetShortSrcAddr() == panCoordAddr)
                    {
                        m_isFirstSuperframe = true;
                    }
                }
                else
                {
                    NS_LOG_DEBUG("Beacon Received; forwarding up (m_macBeaconRxTime: " << m_macBeaconRxTime.As(Time::S) << ")");
                }

                // Fill the PAN descriptor
                PanDescriptor panDescriptor;

                DsmePANDescriptorIE receivedDsmePANDescriptorIEHeaderIE;

                GroupACK receivedGroupAckField;

                // Extract the Header and Payload IE List here
                if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4
                    && receivedMacHdr.IsIEListPresent()) {
                    // DSME-TODO
                    // 要怎麼知道第一個 HeaderIE 一定是 Dsme Pan descriptor?
                    p->RemoveHeader(receivedDsmePANDescriptorIEHeaderIE);

                    // NS_LOG_DEBUG(this << " Receive Dsme Pan Descriptor IE with attribute: "); // debug
                    // receivedDsmePANDescriptorIEHeaderIE.Print(std::cout);

                    panDescriptor.m_dsmeSuperframeSpec = receivedDsmePANDescriptorIEHeaderIE.GetDsmeSuperFrameField();
                    panDescriptor.m_timeSyncSpec = receivedDsmePANDescriptorIEHeaderIE.GetTimeSync();
                    //!< Received beacon bitmap from one beacon
                    panDescriptor.m_bcnBitmap = receivedDsmePANDescriptorIEHeaderIE.GetBeaconBitmap();
                    panDescriptor.m_channelHoppingSpec = receivedDsmePANDescriptorIEHeaderIE.GetChannelHopping();

                    panDescriptor.m_gACKSpec = receivedDsmePANDescriptorIEHeaderIE.GetGroupACK();
                    receivedGroupAckField = panDescriptor.m_gACKSpec;

                    if(m_groupAckPolicy == GROUP_ACK_LEGACY)
                    {
                        // Group ack field in PAN descriptor IE
                        NS_LOG_DEBUG("Received Pan descriptor IE - Group Ack " << receivedGroupAckField);
                    }

                    if(receivedMacHdr.GetShortSrcAddr() == Mac16Address("00:01") && m_shortAddress != Mac16Address("00:01"))
                    {
                      m_macPANCoordinatorBSN = receivedDsmePANDescriptorIEHeaderIE.GetChannelHopping().GetPANCoordinatorBSN();

                    }
                    // NS_LOG_DEBUG("SD Bitmap In IE of the Enhanced Beacon" << receivedDsmePANDescriptorIEHeaderIE.GetBeaconBitmap()); // debug
                    // NS_LOG_DEBUG("Channel Hopping In IE of the Enhanced Beacon" << receivedDsmePANDescriptorIEHeaderIE.GetChannelHopping()); // debug
     
                    // Dsme superframe specification 
                    m_incomingMultisuperframeOrder = panDescriptor.m_dsmeSuperframeSpec.GetMultiSuperframeOrder();
                    m_incomingChannelDiversityMode = panDescriptor.m_dsmeSuperframeSpec.GetChannelDiversityMode();
                    m_incomingGACKFlag = panDescriptor.m_dsmeSuperframeSpec.GetGACKFlag();
                    m_incomingCAPReductionFlag = panDescriptor.m_dsmeSuperframeSpec.GetCAPReductionFlag();
                    m_incomingDeferredBcnUsed = panDescriptor.m_dsmeSuperframeSpec.GetDeferredBeaconFalg();

                    // BeaconBitmap
                    // DSME-TODO
                    m_incomingSDBitmap = panDescriptor.m_bcnBitmap;

                    /**
                     * After receiving a Enhance Beacon, coordinator update their beacon bitmap subsequently.
                     * The update method is 'OR' bitwise operation.
                     * It use 'OR' operation between [incoming beacon bitmap] and [original beacon bitmap].
                     * 
                     *              incoming beacon bitmap | original beacon bitmap
                     */

                    std::vector<uint16_t> incomingSDBitmap = m_incomingSDBitmap.GetSDBitmap(); // Get the incoming beacon bitmap.
                    std::vector<uint16_t> newSDBitmap = m_macSDBitmap.GetSDBitmap(); // Create a new beacon bitmap, use the original first. 

                    // Make sure two vectors (bitmap) have same size.
                    // NS_LOG_DEBUG("newSDBitmap.size() = " << newSDBitmap.size() << "  incomingSDBitmap.size() = " << incomingSDBitmap.size());
                    if(newSDBitmap.size() == 0)
                    {
                        newSDBitmap = incomingSDBitmap;
                    }
                    else if(newSDBitmap.size() == incomingSDBitmap.size())
                    {
                        std::transform(
                            newSDBitmap.begin(), 
                            newSDBitmap.end(),
                            incomingSDBitmap.begin(),
                            newSDBitmap.begin(),
                            [](uint16_t a, uint16_t b) { return a | b; } // Do the OR operation.
                        );
                    }
                    else 
                    {
                        NS_LOG_DEBUG("Error: Vectors must have the same size for bitwise operation.");
                    }

                    m_macSDBitmap.SetBitmapLength(m_incomingSDBitmap.GetSDBitmapLength());
                    m_macSDBitmap.SetSDIndex(m_incomingSDBitmap.GetSDIndex());                        
                    m_macSDBitmap.SetSDBitmap(newSDBitmap); // Set the bitmap, which is the OR operation of the two bitmap.
                    // NS_LOG_DEBUG("m_macSDBitmap.bitmapLength = " << m_macSDBitmap.GetSDBitmapLength());
                    // NS_LOG_DEBUG("Rcv bcn from panDescriptorIE, m_incomingSDBitmap = " << m_incomingSDBitmap);
                    // NS_LOG_DEBUG("After Vector bitwise operation, m_macSDBitmap = " << m_macSDBitmap);
                    
                    // DSME-TODO
                    // 這個應該是要從 m_incomingSDBitmap 取出來
                    m_incSDindex = panDescriptor.m_bcnBitmap.GetSDIndex();

                    // incoming multi-superframe duration
                    NS_LOG_INFO("計算 m_incomingMultisuperframeDuration = " << m_incomingMultisuperframeDuration);
                    m_incomingMultisuperframeDuration = (static_cast<uint32_t>(1 << m_incomingMultisuperframeOrder)) * aBaseSuperframeDuration;
    

                    HeaderIETermination termination;
                    p->RemoveHeader(termination);

                    PayloadIETermination termination2;
                    p->RemoveHeader(termination2);
                    
                    // DSME-TODO
                    // Extract the Payload IE list here if any
                }

                BeaconPayloadHeader receivedMacPayload;
                p->RemoveHeader(receivedMacPayload);

                if (receivedMacHdr.GetSrcAddrMode() == SHORT_ADDR) {
                    panDescriptor.m_coorAddrMode = SHORT_ADDR;
                    panDescriptor.m_coorShortAddr = receivedMacHdr.GetShortSrcAddr();
                }

                panDescriptor.m_coorPanId = receivedMacHdr.GetSrcPanId();
                panDescriptor.m_gtsPermit = receivedMacPayload.GetGtsFields().GetGtsPermit();
                panDescriptor.m_linkQuality = lqi;
                panDescriptor.m_logChPage = m_phy->GetCurrentPage();
                panDescriptor.m_logCh = m_phy->GetCurrentChannelNum();

                // DSME
                if(m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4 && receivedMacHdr.IsIEListPresent())
                {
                    panDescriptor.m_superframeSpec = receivedDsmePANDescriptorIEHeaderIE.GetSuperframeField();
                }
                else
                {
                    panDescriptor.m_superframeSpec = receivedMacPayload.GetSuperframeSpecField();
                }

                panDescriptor.m_timeStamp = m_macBeaconRxTime;

                // Process beacon when device belongs to a PAN (associated device)
                if (!m_scanEvent.IsRunning() && m_macPanId == receivedMacHdr.GetDstPanId()) {
                    // We need to make sure to cancel any possible ongoing unslotted CSMA/CA
                    // operations when receiving a beacon (e.g. Those taking place at the
                    // beginning of an Association).
                    m_csmaCa->Cancel();

                    SuperframeField incomingSuperframe;

                    // DSME
                    if(m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4 && receivedMacHdr.IsIEListPresent())
                    {
                        incomingSuperframe = receivedDsmePANDescriptorIEHeaderIE.GetSuperframeField();
                    }
                    else
                    {
                        incomingSuperframe = receivedMacPayload.GetSuperframeSpecField();
                    }

                    m_incomingBeaconOrder = incomingSuperframe.GetBeaconOrder();
                    m_incomingSuperframeOrder = incomingSuperframe.GetFrameOrder();
                    m_incomingFnlCapSlot = incomingSuperframe.GetFinalCapSlot();

                    m_incomingBeaconInterval =(static_cast<uint32_t>(1 << m_incomingBeaconOrder)) * aBaseSuperframeDuration;
                    m_incomingSuperframeDuration = aBaseSuperframeDuration * (static_cast<uint32_t>(1 << m_incomingSuperframeOrder));

                    if (incomingSuperframe.IsBattLifeExt())
                    {
                        m_csmaCa->SetBatteryLifeExtension(true);
                    }
                    else
                    {
                        m_csmaCa->SetBatteryLifeExtension(false);
                    }

                    if (m_incomingBeaconOrder < 15 && !m_csmaCa->IsSlottedCsmaCa()) {
                        m_csmaCa->SetSlottedCsmaCa();
                    }

                    // TODO: get Incoming frame GTS Fields here

                    if (m_macDSMEenabled  //? Seems no use at CAP reduction
                        && (m_incSDindex % (m_incomingMultisuperframeDuration / m_incomingSuperframeDuration))) {
                        m_incMultisuperframeStartEvent = Simulator::ScheduleNow(&LrWpanMac::StartMultisuperframe, 
                                                                        this, 
                                                                        INCOMING);
                    }
                    
                    // DSME-TODO
                    // Time Synchronization
                    if (m_macPanId == panDescriptor.m_coorPanId
                        && m_macCoordShortAddress == panDescriptor.m_coorShortAddr) {
                        m_startOfBcnSlotOfSyncParent = NanoSeconds(panDescriptor.m_timeSyncSpec.GetBeaconTimeStamp());
                    }
                    

                    // For dsme-net-device-throughput-15-channels... testing usage, So comment it
                    //! Schedule the beacon transmission timing for coordinators (for those not PAN-C) after received a EB
                    //! Note : Only for the first time, the subsequence EB will be scheduled by StartRemainingPeriod()
                    if (!m_forDsmeNetDeviceIntegrateWithHigerLayer) {
                        if (m_macDSMEenabled && m_coord && !m_panCoor && m_sendBcn) {
                            // NS_LOG_DEBUG("Simulator::Now() = " << Simulator::Now().As(Time::S));
                            // NS_LOG_DEBUG("m_startOfBcnSlotOfSyncParent = " << m_startOfBcnSlotOfSyncParent.As(Time::S));
                            Time scheduleBcnTime = Seconds(((double)m_incomingSuperframeDuration * m_choosedSDIndexToSendBcn) 
                                                        / symbolRate) // Calculate the total superframe time in BI
                                                        - (Simulator::Now() - m_startOfBcnSlotOfSyncParent); // Minus the times when the Parent coordinator send it beacon.
                            NS_LOG_DEBUG("Simulator::Now() - m_startOfBcnSlotOfSyncParent = " << (Simulator::Now() - m_startOfBcnSlotOfSyncParent).As(Time::S)); // debug
                            // NS_LOG_DEBUG("m_incomingSuperframeDuration = " << m_incomingSuperframeDuration);
                            // NS_LOG_DEBUG("m_startOfBcnSlotOfSyncParent = " << m_startOfBcnSlotOfSyncParent.As(Time::S));
                            // NS_LOG_DEBUG("m_choosedSDIndexToSendBcn = " << m_choosedSDIndexToSendBcn);
                            // NS_LOG_DEBUG("scheduleBcnTime = " << scheduleBcnTime.As(Time::S));
                            m_setMacState = Simulator::Schedule(scheduleBcnTime
                                                                , &LrWpanMac::SetLrWpanMacState
                                                                , this
                                                                , MAC_IDLE);

                            m_beaconEvent = Simulator::Schedule(scheduleBcnTime
                                                                , &LrWpanMac::SendOneEnhancedBeacon
                                                                , this);
                            
                            m_sendBcn = false;
                        }
                    }

                    PurgeDsmeACT();

                    // DSME-TODO
                    // For dsme-net-device setting use only 
                    // ScheduleGtsSyncToCoord(m_incSDindex);

                    if(!m_forDsmeNetDeviceIntegrateWithHigerLayer)
                    {
                        if(receivedMacHdr.GetShortSrcAddr() == GetCoordShortAddress())
                        {
                            ScheduleGts(true);
                        }
                    }

                    // CAP reduction 
                    // DSME-TODO
                    // 應該從 dsme pan descriptor 拿出資訊來檢查 CAP reduction

                    if (m_macDSMEenabled && m_incomingCAPReductionFlag) 
                    {
                        Time timeSinceBeaconTx = Simulator::Now() - m_macBeaconTxTime;
                        // NS_LOG_DEBUG("timeSinceBeaconTx.GetSeconds() : " << timeSinceBeaconTx.GetSeconds());
                        // NS_LOG_DEBUG("(double)(m_incomingSuperframeDuration / (double)symbolRate) : " << (double)(m_incomingSuperframeDuration / (double)symbolRate));
                        if((m_coord == 0 && m_isFirstSuperframe == false)) // Check for non-coord device (RFD), start a cap or cfp
                        {
                            NS_LOG_DEBUG("Incoming superframe Active Portion (Beacon + CFP + CFP): "
                                        << m_incomingSuperframeDuration << " symbols");
                            m_incomingFirstCFP = true;
                            m_incCfpEvent = Simulator::ScheduleNow(&LrWpanMac::StartCFP,
                                                                this,
                                                                SuperframeType::INCOMING);
                        }
                        else if(m_coord == 0 && m_isFirstSuperframe == true) // Check for non-coord device (RFD), start a cap or cfp
                        {
                            m_incCapEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                        this,
                                                        SuperframeType::INCOMING);  
                                                        
                            m_setMacState =
                                Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);
                            m_isFirstSuperframe = false;
                        }
                        else if ((m_incSDindex % (m_incomingMultisuperframeDuration / m_incomingSuperframeDuration)) // non first superframe period for cap reduction (coord device)
                         || (m_macSDindex == 0 && (timeSinceBeaconTx.GetSeconds() >= (double)(m_incomingSuperframeDuration / (double)symbolRate))))
                        {
                            NS_LOG_DEBUG("Incoming superframe Active Portion (Beacon + CFP + CFP): "
                                        << m_incomingSuperframeDuration << " symbols");
                            m_incomingFirstCFP = true;
                            m_incCfpEvent = Simulator::ScheduleNow(&LrWpanMac::StartCFP,
                                                                this,
                                                                SuperframeType::INCOMING);
                        }
                        else // First superframe period for cap reduction (coord device)
                        {
                            m_incCapEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                        this,
                                                        SuperframeType::INCOMING);  
                                                        
                            m_setMacState =
                                Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE); 
                        }

                    } 
                    else 
                    {
                        m_incCapEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                        this,
                                                        SuperframeType::INCOMING);  
                        m_setMacState =
                            Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);                         
                    }                

                } else if (!m_scanEvent.IsRunning() && m_macPanId == 0xFFFF) {
                    NS_LOG_DEBUG(this << " Device not associated, cannot process beacon");
                    return;
                }

                if (m_macAutoRequest) {
                    if (p->GetSize() > 0) {      // the beacon contains any beacon payload
                        // 檢查回呼函式有沒有被綁定，有就回傳 false，沒有就回傳 true
                        if(!m_mlmeBeaconNotifyIndicationCallback.IsNull())
                        {
                            // DSME-TODO
                            // The beacon contains payload, send the beacon notification.
                            MlmeBeaconNotifyIndicationParams beaconParams;

                            if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4) {
                                beaconParams.m_ebsn = receivedMacHdr.GetSeqNum();
                                beaconParams.m_beaconType = 0x01;

                            } else {
                                beaconParams.m_bsn = receivedMacHdr.GetSeqNum();
                                beaconParams.m_beaconType = 0x00;
                            }
                            
                            beaconParams.m_panDescriptor = panDescriptor;
                            beaconParams.m_sduLength = p->GetSize();
                            beaconParams.m_sdu = p;                     // might include payload IE
                            m_mlmeBeaconNotifyIndicationCallback(beaconParams, originalPkt);
                        }
                    }

                    if (m_trackingEvent.IsRunning()) {  // currently synchronizing with a coordinator
                        // check if MLME-SYNC.request was previously issued and running
                        // Sync. is necessary to handle pending messages (indirect
                        // transmissions)
                        m_trackingEvent.Cancel();
                        m_numLostBeacons = 0;

                        if (m_beaconTrackingOn) {
                            // if tracking option is on keep tracking the next beacon
                            uint64_t searchSymbols;
                            Time searchBeaconTime;

                            searchSymbols =
                                ((static_cast<uint64_t>(1 << m_incomingBeaconOrder)) +
                                 1) * aBaseSuperframeDuration;
                            searchBeaconTime =
                                Seconds(static_cast<double>(searchSymbols / symbolRate));
                            m_trackingEvent =
                                Simulator::Schedule(searchBeaconTime,
                                                    &LrWpanMac::BeaconSearchTimeout,
                                                    this);
                        }

                        PendingAddrFields pndAddrFields;

                        // DSME-TODO
                        if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4
                        && receivedMacHdr.IsIEListPresent()) {
                            pndAddrFields = receivedDsmePANDescriptorIEHeaderIE.GetPendingAddrFields();
                        } else {
                            pndAddrFields = receivedMacPayload.GetPndAddrFields();
                        }
                    } 
                }
            } 
            else if (receivedMacHdr.IsData() && !m_mcpsDataIndicationCallback.IsNull()) 
            {
                // If it is a data frame, push it up the stack.
                // Fow hilow
                // if (!m_acceptAllHilowPkt) {
                //     NS_LOG_DEBUG("Data Packet is for me; forwarding up");
                //     m_mcpsDataIndicationCallback(params, p);
                // }

                NS_LOG_DEBUG("Data Packet is for me; forwarding up");
                m_mcpsDataIndicationCallback(params, p);

                if (m_incGtsEvent.IsRunning()) {
                    m_macDsmeACT[m_curGTSSuperframeID][m_curGTSIdx].m_cnt = 0;
                }
            } 
            
            if(receivedMacHdr.IsAcknowledgment() && receivedMacHdr.IsIEListPresent() && m_groupAckPolicy == GROUP_ACK_ENHANCED) // E-GACK
            {
                LrWpanMacHeader peekedMacHdr;
                EnhancedGroupAckDescriptorIE receivedEnhancedGackIE;
                p->RemoveHeader(receivedEnhancedGackIE);
                NS_LOG_DEBUG("Received Enhanced Group Ack Bitmap = " << receivedEnhancedGackIE);

                if(receivedEnhancedGackIE.GetGroupAckBitmap() == 0)
                {
                    NS_LOG_DEBUG("There is no packet need to Group Ack");
                    return;
                }

                NS_LOG_DEBUG("Starting Check received Group Ack bitmap");

                for (size_t i = 0; i < m_groupAckPktBuffer.size(); i++) 
                {
                    // After received E-GACK bitmap, start vertify the ack seq with the same hash function to generate the key.
                    uint64_t bitLocation = GenerateHashTableKey(m_shortAddress, m_groupAckPktBuffer[i]);
                    m_enhancedGACKBitmap |= ((uint64_t)1 << bitLocation);
                    NS_LOG_DEBUG("m_groupAckPktBuffer element = " << m_groupAckPktBuffer[i] << " , bitLocation = " << bitLocation);

                    // Check the corresponding location.
                    if(receivedEnhancedGackIE.GetGroupAckBitmap() & ((uint64_t)1 << bitLocation))
                    {
                        NS_LOG_DEBUG("Seq : " << m_groupAckPktBuffer[i] << " Ack successfully");
                        uint64_t newBitmap = receivedEnhancedGackIE.GetGroupAckBitmap();
                        newBitmap &= ~((uint64_t)1 << bitLocation);
                        receivedEnhancedGackIE.SetGroupAckBitmap(newBitmap);
                        // NS_LOG_DEBUG("After processing, Bitmap = " << receivedEnhancedGackIE);

                        if (!m_mcpsDataConfirmCallback.IsNull()) 
                        {
                            if (!m_forDsmeNetDeviceIntegrateWithHigerLayer) 
                            {
                                // Received Ack packet, send McpsDataConfirmParams to the higher layer to trigger McpsDataConfirm.
                                McpsDataConfirmParams confirmParams;
                                confirmParams.m_msduHandle = m_mcpsDataRequestParams.m_msduHandle;
                                confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                                m_mcpsDataConfirmCallback(confirmParams);
                            }                                
                        }
                    }
                    else // If the expect location in the bitmap is not 1 , the packet is transmit fail.
                    {
                        NS_LOG_DEBUG("Packet with seq num " << m_groupAckPktBuffer[i] << " transmit fail, ready to retransmit");
                    }
                }                    

                ResetEnhancedGroupAckBitmap();   
                ResetEnhancedGroupAckBuffer();   

                Time ifsWaitTime = Seconds((double)m_macLIFSPeriod / symbolRate);                            
                if (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()) 
                {
                    m_txPkt = nullptr;
                    m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_GTS);
                    m_ifsEvent = Simulator::Schedule(ifsWaitTime,
                                                    &LrWpanMac::IfsWaitTimeout,
                                                    this,
                                                    ifsWaitTime);
                }     
            }

            if(receivedMacHdr.IsAcknowledgment() && receivedMacHdr.IsIEListPresent() && m_groupAckPolicy == GROUP_ACK_LEGACY) // Legacy GACK
            {
                LrWpanMacHeader peekedMacHdr;
                LegacyGroupAckIE receivedLegacyGackIE;
                p->RemoveHeader(receivedLegacyGackIE);
                std::bitset<16> gackBitmap = receivedLegacyGackIE.GetGackBitmapField();
                NS_LOG_DEBUG("Received Legacy Group Ack Bitmap = " << gackBitmap);

                if(receivedLegacyGackIE.GetGackBitmapField() == 0)
                {
                    NS_LOG_DEBUG("There is no packet need to Group Ack");
                    return;
                }

                NS_LOG_DEBUG("Starting Check received Group Ack bitmap");
                for (size_t i = 0; i < m_legacyGackGTSIdxBuffer.size(); i++) 
                {
                    if(receivedLegacyGackIE.GetGackBitmapField() & ((uint16_t)1 << m_legacyGackGTSIdxBuffer[i]))
                    {
                        NS_LOG_DEBUG("Packet at slot " << m_legacyGackGTSIdxBuffer[i] << " , Ack successfully");
                        if (!m_mcpsDataConfirmCallback.IsNull()) 
                        {
                            if (!m_forDsmeNetDeviceIntegrateWithHigerLayer) 
                            {
                                // Received Ack packet, send McpsDataConfirmParams to the higher layer to trigger McpsDataConfirm.
                                McpsDataConfirmParams confirmParams;
                                confirmParams.m_msduHandle = m_mcpsDataRequestParams.m_msduHandle;
                                confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                                m_mcpsDataConfirmCallback(confirmParams);
                            }                                
                        }
                    }
                    else
                    {
                        NS_LOG_DEBUG("Packet at slot : " << m_legacyGackGTSIdxBuffer[i] << " transmit fail, ready to retransmit");
                    }
                }

                ResetLegacyGroupAckBuffer();

                Time ifsWaitTime = Seconds((double)m_macLIFSPeriod / symbolRate);                            
                if (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()) 
                {
                    m_txPkt = nullptr;
                    m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_GTS);
                    m_ifsEvent = Simulator::Schedule(ifsWaitTime,
                                                    &LrWpanMac::IfsWaitTimeout,
                                                    this,
                                                    ifsWaitTime);
                }     
            }
        }
        else
        {
            m_macRxDropTrace(originalPkt);
        }
    }
}

// void
// LrWpanMac::SendAck(uint8_t seqno)
// {
//     NS_LOG_FUNCTION(this << static_cast<uint32_t>(seqno));
//     NS_LOG_DEBUG("Send Ack");
//     NS_ASSERT(m_lrWpanMacState == MAC_IDLE);

//     // Generate a corresponding ACK Frame.
//     LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT, seqno);
//     LrWpanMacTrailer macTrailer;
//     Ptr<Packet> ackPacket = Create<Packet>(0);
//     ackPacket->AddHeader(macHdr);

//     // Calculate FCS if the global attribute ChecksumEnable is set.
//     if (Node::ChecksumEnabled()) {
//         macTrailer.EnableFcs(true);
//         macTrailer.SetFcs(ackPacket);
//     }

//     ackPacket->AddTrailer(macTrailer);

//     // Enqueue the ACK packet for further processing
//     // when the transmitter is activated.
//     m_txPkt = ackPacket;

//     // Switch transceiver to TX mode. Proceed sending the Ack on confirm.

//     if (m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) { 
//         ChangeMacState(MAC_GTS_SENDING);
//     } else {
//         ChangeMacState(MAC_SENDING);
//     }
    
//     m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
// }

void
LrWpanMac::EnqueueTxQElement(Ptr<TxQueueElement> txQElement)
{
    if (m_txQueue.size() < m_maxTxQueueSize)
    {
        m_txQueue.emplace_back(txQElement);
        m_macTxEnqueueTrace(txQElement->txQPkt);
    }
    else
    {
        if (!m_mcpsDataConfirmCallback.IsNull())
        {
            McpsDataConfirmParams confirmParams;
            confirmParams.m_msduHandle = txQElement->txQMsduHandle;
            confirmParams.m_status = IEEE_802_15_4_TRANSACTION_OVERFLOW;
            m_mcpsDataConfirmCallback(confirmParams);
        }
        NS_LOG_DEBUG("TX Queue with size " << m_txQueue.size() << " is full, dropping packet");
        m_macTxDropTrace(txQElement->txQPkt);
    }
}

void
LrWpanMac::RemoveFirstTxQElement()
{   
    Ptr<TxQueueElement> txQElement = m_txQueue.front();
    Ptr<const Packet> p = txQElement->txQPkt;
    m_numCsmacaRetry += m_csmaCa->GetNB() + 1;

    Ptr<Packet> pkt = p->Copy();
    LrWpanMacHeader hdr;
    pkt->RemoveHeader(hdr);
    if (!hdr.GetShortDstAddr().IsBroadcast() && !hdr.GetShortDstAddr().IsMulticast())
    {
        m_sentPktTrace(p, m_retransmission + 1, m_numCsmacaRetry);
    }

    txQElement->txQPkt = nullptr;
    txQElement = nullptr;
    m_txQueue.pop_front();
    m_txPkt = nullptr;
    m_retransmission = 0;
    m_numCsmacaRetry = 0;
    m_macTxDequeueTrace(p);
}

void LrWpanMac::DsmeGtsAckWaitTimeout() {
    NS_LOG_FUNCTION(this);

    if (!m_mlmeDsmeGtsConfirmCallback.IsNull()) {
        MlmeDsmeGtsConfirmParams confirmParams;
        confirmParams.m_status = MLMEDSMEGTS_REQ_NO_ACK;

        m_mlmeDsmeGtsConfirmCallback(confirmParams);
    }
}

void LrWpanMac::DsmeInfoAckWaitTimeout() {
    NS_LOG_FUNCTION(this);

    if (!m_mlmeDsmeInfoConfirmCallback.IsNull()) {
        MlmeDsmeInfoConfirmParams confirmParams;
        confirmParams.m_status = MLMEDSMEINFO_NO_ACK;

        m_mlmeDsmeInfoConfirmCallback(confirmParams);
    }
}

void
LrWpanMac::IfsWaitTimeout(Time ifsTime)
{
    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);
    Time lifsTime = Seconds((double)m_macLIFSPeriod / symbolRate);
    Time sifsTime = Seconds((double)m_macSIFSPeriod / symbolRate);

    if (ifsTime == lifsTime)
    {
        NS_LOG_DEBUG("LIFS of " << m_macLIFSPeriod << " symbols (" << ifsTime.As(Time::S)
                                << ") completed ");
        if(m_incGtsEvent.IsRunning() || (m_gtsEvent.IsRunning() && m_lrWpanMacState == MAC_ACK_PENDING)) 
        {
            // Here add change state in order to let mac state keep at MAC_GTS. (原本會卡住，一個GTS就只能傳一個封包)
            // >> More packet can be sent in the GTS
            ChangeMacState(MAC_GTS);
        }

        if((m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) && (m_lrWpanMacState == MAC_GTS))
        {
            // howard: 新增 Enhanced GTS forwarding
            NS_LOG_DEBUG("Current TX queue size: " << m_txQueue.size());
            if(m_enhancedGTSForwarding && !m_txQueue.empty())
            {
                Ptr<TxQueueElement> txElem = m_txQueue.front();
                m_txPkt = txElem->txQPkt;
                // 52 (上層封包) + 5 bytes (Mesh Header) + 2 bytes (HC1) + 9 bytes (MAC Header) + 2 bytes (Footer)
                if(m_txPkt->GetSize() <= 70)
                {
                    ChangeMacState(MAC_GTS_SENDING);
                    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
                }
            }
        }

    }
    else if (ifsTime == sifsTime)
    {
        NS_LOG_DEBUG("SIFS of " << m_macSIFSPeriod << " symbols (" << ifsTime.As(Time::S)
                                << ") completed ");
        if(m_incGtsEvent.IsRunning() || (m_gtsEvent.IsRunning() && m_lrWpanMacState == MAC_ACK_PENDING) )
        {
            // Here add change state in order to let mac state keep at MAC_GTS. (原本會卡住，一個GTS就只能傳一個封包)
            // >> More packet can be sent in the GTS
            ChangeMacState(MAC_GTS);
        }
    }
    else
    {
        NS_LOG_DEBUG("Unknown IFS size (" << ifsTime.As(Time::S) << ") completed ");
    }

    m_macIfsEndTrace(ifsTime);
    CheckQueue();
}

void
LrWpanMac::RemovePendTxQElement(Ptr<Packet> p)
{
    LrWpanMacHeader peekedMacHdr;
    p->PeekHeader(peekedMacHdr);

    for (auto it = m_indTxQueue.begin(); it != m_indTxQueue.end(); it++)
    {
        if (peekedMacHdr.GetDstAddrMode() == SHORT_ADDR)
        {
            if (((*it)->dstShortAddress == peekedMacHdr.GetShortDstAddr()) &&
                ((*it)->seqNum == peekedMacHdr.GetSeqNum()))
            {
                m_macIndTxDequeueTrace(p);
                m_indTxQueue.erase(it);
                break;
            }
        }
    }
    p = nullptr;
}

void
LrWpanMac::PdDataConfirm(LrWpanPhyEnumeration status)
{
    NS_ASSERT(m_lrWpanMacState == MAC_SENDING || m_lrWpanMacState == MAC_GTS_SENDING);
    NS_LOG_FUNCTION(this << status << m_txQueue.size());

    // NS_LOG_DEBUG("進來 PdDataConfirm");

    LrWpanMacHeader macHdr;
    Time ifsWaitTime;
    double symbolRate;

    symbolRate = m_phy->GetDataOrSymbolRate(false); // symbols per second

    m_txPkt->PeekHeader(macHdr);
    if (status == IEEE_802_15_4_PHY_SUCCESS) {
        if (!macHdr.IsAcknowledgment()) 
        {
            if (macHdr.IsBeacon()) 
            {
                // Start CAP only if we are in beacon mode (i.e. if slotted csma-ca is running)
                if (m_csmaCa->IsSlottedCsmaCa()) 
                {
                    // The Tx Beacon in symbols
                    // Beacon = 5 bytes Sync Header (SHR) +  1 byte PHY header (PHR) + PSDU (default
                    // 17 bytes)
                    uint64_t beaconSymbols = m_phy->GetPhySHRDuration() +
                                             1 * m_phy->GetPhySymbolsPerOctet() +
                                             (m_txPkt->GetSize() * m_phy->GetPhySymbolsPerOctet());

                    // The beacon Tx time and start of the Outgoing superframe Active Period
                    
                    // howard:
                    // 不太清楚為什麼要這樣做，beacon time 已經很小了，取完毫秒會變成 0
                    m_macBeaconTxTime = Simulator::Now() - MilliSeconds(static_cast<double>(beaconSymbols) / symbolRate);

                    PurgeDsmeACT();

                    // DSME-TODO
                    // For Dsme-net-device setting use only 
                    // ScheduleGtsSyncToCoord(m_choosedSDIndexToSendBcn);

                    if (!m_forDsmeNetDeviceIntegrateWithHigerLayer) {
                        ScheduleGts(false);
                    }

                    if (m_macDSMEenabled && m_macCAPReductionFlag) {
                        if (m_macSDindex % (m_multiSuperframeDuration / m_superframeDuration) == 0) // Check current superframe is the first superframe or not.
                        {                    
                            // uint64_t symbolRate;
                            // symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false)    ; 
                            // uint64_t nextmultisuperframeDuraion = (m_numOfSuperframes / m_numOfMultisuperframes) * m_superframeDuration;
                            // Time endmultisuperframeTime = Seconds((double)nextmultisuperframeDuraion / symbolRate);

                            // NS_LOG_DEBUG("m_macSDindex % (m_multiSuperframeDuration / m_superframeDuration) == 0");
                            m_capEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                                this,
                                                                SuperframeType::OUTGOING); 
                            // Simulator::Schedule(endmultisuperframeTime, &LrWpanMac::StartCAP, this, SuperframeType::OUTGOING);                                  

                        } 
                        else // The coordinator which is not PAN-C. Also, in CAP reduction, other coordinators will schedule CFP here.
                        {
                            m_firstCFP = true;
                            m_cfpEvent = Simulator::ScheduleNow(&LrWpanMac::StartCFP,
                                                                this,
                                                                SuperframeType::OUTGOING);      
                        }

                    } 
                    else 
                    {
                        m_capEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                            this,
                                                            SuperframeType::OUTGOING);                         
                    }

                    NS_LOG_DEBUG("Beacon Sent (m_macBeaconTxTime: " << m_macBeaconTxTime.As(Time::S)
                                                                    << ")");

                    if (!m_mlmeStartConfirmCallback.IsNull())
                    {
                        MlmeStartConfirmParams mlmeConfirmParams;
                        mlmeConfirmParams.m_status = MLMESTART_SUCCESS;
                        m_mlmeStartConfirmCallback(mlmeConfirmParams);
                    }
                }
                ifsWaitTime = Seconds(static_cast<double>(GetIfsSize()) / symbolRate);
                m_txPkt = nullptr;

            } 
            else if (macHdr.IsAckReq())  // We have sent a regular data packet, check if we have to it for an ACK.
            {                            
                // we sent a regular data frame or command frame (e.g. AssocReq command) that
                // require ACK wait for the ack or the next retransmission timeout start retransmission timer.

                Time waitTime = Seconds(static_cast<double>(GetMacAckWaitDuration()) / symbolRate);

                // DSME
                if (macHdr.IsData()) 
                {
                    NS_LOG_DEBUG("Send the data packet successfully !");
                    // std::cout << Simulator::Now().GetNanoSeconds() << ", " << GetShortAddress() << " successfully sent the data packet" << std::endl;

                    /**
                     * Recording the packet sequence number before received a enhanced group ack bitmap.
                    */
                    if(m_groupAckPolicy == GROUP_ACK_LEGACY && (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()))
                    {
                        m_legacyGackGTSIdxBuffer.push_back((uint32_t)m_currentGTSIdx);
                    }  

                    /**
                     * Recording the packet sequence number before received a enhanced group ack bitmap.
                    */
                    if(m_groupAckPolicy == GROUP_ACK_ENHANCED && (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()))
                    {
                        // NS_LOG_DEBUG("Push back into pktbuf , seq = " << (uint32_t)macHdr.GetSeqNum());
                        m_groupAckPktBuffer.push_back((uint32_t)macHdr.GetSeqNum());
                    }

                    // (*m_record)[GetShortAddress()] = {macHdr.GetShortDstAddr(), {Simulator::Now().GetNanoSeconds()}};
                    if (m_record != nullptr) {
                        (*m_record)[GetShortAddress()] = {macHdr.GetShortDstAddr(), {}};
                        (*m_record)[GetShortAddress()].second.push_back(Simulator::Now().GetNanoSeconds());
                    }

                    if (m_record2 != nullptr && m_recordKeys.size() > 0) {
                        if ((*m_record2).count(m_recordKeys.front()) == 0) {
                            (*m_record2)[m_recordKeys.front()] = {};
                            (*m_record2)[m_recordKeys.front()] = {std::make_pair(0, Simulator::Now().GetNanoSeconds())};
                        } else {
                            (*m_record2)[m_recordKeys.front()].push_back(std::make_pair(0, Simulator::Now().GetNanoSeconds()));
                        }

                        m_recordKeys.pop_front();
                    }
                }
                
                m_setMacState.Cancel();

                // TODO : If enabled GACK, no need to ACK_PENDING, maybe wait a ifs time is enough.
                if(m_groupAckPolicy == GROUP_ACK_LEGACY || m_groupAckPolicy == GROUP_ACK_ENHANCED)
                {
                    m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_ACK_PENDING);
                    ifsWaitTime = Seconds(static_cast<double>(m_macSIFSPeriod) / symbolRate);
                    m_ifsEvent =  Simulator::Schedule(ifsWaitTime, &LrWpanMac::IfsWaitTimeout, this, ifsWaitTime);
                }
                return;
            } 
            else 
            {
                m_macTxOkTrace(m_txPkt);
                // remove the copy of the packet that was just sent
                if (!m_mcpsDataConfirmCallback.IsNull())
                {
                    McpsDataConfirmParams confirmParams;
                    // NS_ASSERT_MSG(m_txQueue.size() > 0, "TxQsize = 0");
                    Ptr<TxQueueElement> txQElement = m_txQueue.front();
                    confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                    confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                    NS_LOG_INFO("callback 上層傳送成功");
                    m_mcpsDataConfirmCallback(confirmParams);
                }
                ifsWaitTime = Seconds(static_cast<double>(GetIfsSize()) / symbolRate);

                RemoveFirstTxQElement();
            }
        }
        else if(macHdr.IsAcknowledgment() && macHdr.IsIEListPresent() && m_groupAckPolicy == GROUP_ACK_LEGACY)
        {
            NS_LOG_DEBUG("Successfully sent a Legacy Group Ack packet.");
            ResetLegacyGroupAckBitmap();
            m_txPkt = nullptr;
        }
        else if(macHdr.IsAcknowledgment() && macHdr.IsIEListPresent() && m_groupAckPolicy == GROUP_ACK_ENHANCED)
        {
            NS_LOG_DEBUG("Successfully sent a Enhanced Group Ack packet.");
            ResetEnhancedGroupAckBitmap();
            m_txPkt = nullptr;
        }
    }

    if(!ifsWaitTime.IsZero())
    {
        m_ifsEvent = Simulator::Schedule(ifsWaitTime, &LrWpanMac::IfsWaitTimeout, this, ifsWaitTime);
    }

    if(m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning())
    {
        // NS_LOG_INFO("進來了1");
        m_setMacState.Cancel();
        m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacStateToGTS
                                                , this
                                                , m_curGTSSuperframeID
                                                , m_curGTSIdx);
    }
    else
    {
        m_setMacState.Cancel();
        m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);
    }
}

void
LrWpanMac::PlmeEdConfirm(LrWpanPhyEnumeration status, uint8_t energyLevel)
{
    NS_LOG_FUNCTION(this << status << energyLevel);

    if (energyLevel > m_maxEnergyLevel)
    {
        m_maxEnergyLevel = energyLevel;
    }

    if (Simulator::GetDelayLeft(m_scanEnergyEvent) >
        Seconds(8.0 / m_phy->GetDataOrSymbolRate(false)))
    {
        m_phy->PlmeEdRequest();
    }
}

void
LrWpanMac::PlmeGetAttributeConfirm(LrWpanPhyEnumeration status,
                                   LrWpanPibAttributeIdentifier id,
                                   LrWpanPhyPibAttributes* attribute)
{
    NS_LOG_FUNCTION(this << status << id << attribute);
}

void
LrWpanMac::PlmeSetTRXStateConfirm(LrWpanPhyEnumeration status)
{
    NS_LOG_FUNCTION(this << status);
    // NS_LOG_INFO("進來了");
    if (m_lrWpanMacState == MAC_SENDING &&
        (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
        NS_ASSERT(m_txPkt);

        // Start sending if we are in state SENDING and the PHY transmitter was enabled.
        m_promiscSnifferTrace(m_txPkt);
        m_snifferTrace(m_txPkt);
        m_macTxTrace(m_txPkt);

        // howard: CAP 正式傳資料到 PHY
        NS_LOG_INFO("Sending packet to the PHY layer: " << m_txPkt->GetSize() << " bytes");
        m_phy->PdDataRequest(m_txPkt->GetSize(), m_txPkt);
        
    } else if (m_lrWpanMacState == MAC_GTS_SENDING &&
                (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS)) {
        NS_ASSERT(m_txPkt);          
        // Start sending if we are in state SENDING and the PHY transmitter was enabled.

        // DSME-TODO
        LrWpanMacTrailer macTrailer;
        m_txPkt->RemoveTrailer(macTrailer);

        LrWpanMacHeader macHdr;
        m_txPkt->RemoveHeader(macHdr);

        if (macHdr.IsCommand()) {
            CommandPayloadHeader cmdPayload;
            m_txPkt->RemoveHeader(cmdPayload);

            if (cmdPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_INFO_REPLY) {
                if (cmdPayload.GetDsmeInfoType() == MLMEDSMEINFO_TIMESTAMP) {
                    // uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);
                    cmdPayload.SetDsmeInfoTimestamp(Simulator::Now().ToInteger(Time::NS));
                }
            }

            m_txPkt->AddHeader(cmdPayload);
        }

        m_txPkt->AddHeader(macHdr);

        if (Node::ChecksumEnabled()) {
            macTrailer.EnableFcs(true);
            macTrailer.SetFcs(m_txPkt);
        }

        m_txPkt->AddTrailer(macTrailer);
        
        m_promiscSnifferTrace(m_txPkt);
        m_snifferTrace(m_txPkt);
        m_macTxTrace(m_txPkt);
        // howard: CFP 正式傳資料到 PHY
        m_phy->PdDataRequest(m_txPkt->GetSize(), m_txPkt);  
    }
    
    else if (m_lrWpanMacState == MAC_CSMA &&
             (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
        // Start the CSMA algorithm as soon as the receiver is enabled.
        m_csmaCa->Start();
    }
    else if (m_lrWpanMacState == MAC_IDLE)
    {
        NS_ASSERT(status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS ||
                  status == IEEE_802_15_4_PHY_TRX_OFF);

        if (status == IEEE_802_15_4_PHY_RX_ON && m_scanEnergyEvent.IsRunning())
        {
            // Kick start Energy Detection Scan
            m_phy->PlmeEdRequest();
        }
        else if (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS)
        {
            // Check if there is not messages to transmit when going idle
            CheckQueue();
        }
    }
    else if (m_lrWpanMacState == MAC_ACK_PENDING)
    {
        NS_ASSERT(status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS);
    }
    else if (m_lrWpanMacState == MAC_GTS) 
    {
        NS_ASSERT(status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS 
                  || status == IEEE_802_15_4_PHY_TX_ON);
    }
    else
    {
        // TODO: What to do when we receive an error?
        // If we want to transmit a packet, but switching the transceiver on results
        // in an error, we have to recover somehow (and start sending again).
        if (m_lrWpanMacState == MAC_GTS_SENDING && (!m_gtsEvent.IsRunning() && !m_incGtsEvent.IsRunning())) {
            return;
        } 

        NS_FATAL_ERROR("Error changing transceiver state");
    }
}

void
LrWpanMac::PlmeSetAttributeConfirm(LrWpanPhyEnumeration status, LrWpanPibAttributeIdentifier id)
{
    NS_LOG_FUNCTION(this << status << id);
    
    if (id == LrWpanPibAttributeIdentifier::phyCurrentPage &&
             m_pendPrimitive == MLME_START_REQ)
    {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS)
        {
            LrWpanPhyPibAttributes pibAttr;
            pibAttr.phyCurrentChannel = m_startParams.m_logCh;
            m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel,
                                           &pibAttr);
        }
        else
        {
            if (!m_mlmeStartConfirmCallback.IsNull())
            {
                MlmeStartConfirmParams confirmParams;
                confirmParams.m_status = MLMESTART_INVALID_PARAMETER;
                m_mlmeStartConfirmCallback(confirmParams);
            }
            NS_LOG_ERROR("Invalid page parameter in MLME-start");
        }
    }
    else if (id == LrWpanPibAttributeIdentifier::phyCurrentChannel &&
             m_pendPrimitive == MLME_START_REQ)
    {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS)
        {
            m_originalChannelInCAP = m_startParams.m_logCh;
            EndStartRequest();
        }
    }
}

void LrWpanMac::SetLrWpanMacStateToGTS(uint16_t superframeID, int idx) {
    ChangeMacState(MAC_GTS);

    if (m_macDsmeACT[superframeID][idx].m_direction) 
    {
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
    } 
    else 
    {
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
    }
}

void
LrWpanMac::SetLrWpanMacState(LrWpanMacState macState)
{
    NS_LOG_FUNCTION(this << "mac state = " << macState);
    if (macState == MAC_IDLE) {
        ChangeMacState(MAC_IDLE);

        if (m_macRxOnWhenIdle)
        {
            // howard: CAP 進來這裡
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
        }
        else
        {
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TRX_OFF);
        }

    } else if (macState == MAC_ACK_PENDING) {
        ChangeMacState(MAC_ACK_PENDING);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);

    } else if (macState == MAC_CSMA) {
        NS_ASSERT(m_lrWpanMacState == MAC_IDLE || m_lrWpanMacState == MAC_ACK_PENDING);
        NS_LOG_INFO("Use carrier sensing and switch receiver state to RX_ON");
        ChangeMacState(MAC_CSMA);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);

    } else if (m_lrWpanMacState == MAC_CSMA && macState == CHANNEL_IDLE) {
        // Channel is idle, set transmitter to TX_ON
        NS_LOG_INFO("Channel is idle, switch transmitter to TX_ON");
        ChangeMacState(MAC_SENDING);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);

    } else if (m_lrWpanMacState == MAC_CSMA && macState == MAC_CSMA_DEFERRED) {
        ChangeMacState(MAC_IDLE);
        m_txPkt = nullptr;
        // The MAC is running on beacon mode and the current packet could not be sent in the
        // current CAP. The packet will be send on the next CAP after receiving the beacon.
        // The PHY state does not change from its current form. The PHY change (RX_ON) will be
        // triggered by the scheduled beacon event.

        NS_LOG_INFO("****** PACKET DEFERRED to the next superframe *****");
    }
}

LrWpanAssociationStatus
LrWpanMac::GetAssociationStatus() const
{
    return m_associationStatus;
}

void
LrWpanMac::SetAssociationStatus(LrWpanAssociationStatus status)
{
    m_associationStatus = status;
}

void LrWpanMac::SetAssociatePermit() {
    m_macAssociationPermit = true;
}

void LrWpanMac::SetAssociateNotPermit() {
    m_macAssociationPermit = false;
}

void
LrWpanMac::SetTxQMaxSize(uint32_t queueSize)
{
    m_maxTxQueueSize = queueSize;
}

void
LrWpanMac::SetIndTxQMaxSize(uint32_t queueSize)
{
    m_maxIndTxQueueSize = queueSize;
}

uint16_t
LrWpanMac::GetPanId() const
{
    return m_macPanId;
}

Mac16Address
LrWpanMac::GetCoordShortAddress() const
{
    return m_macCoordShortAddress;
}

uint16_t LrWpanMac::GetChannelOffset() const {
    return m_macChannelOfs;
}

void
LrWpanMac::SetPanId(uint16_t panId)
{
    m_macPanId = panId;
}

void
LrWpanMac::ChangeMacState(LrWpanMacState newState)
{
    // NS_LOG_DEBUG(this << " change lrwpan mac state from " << m_lrWpanMacState << " to "
    //                   << newState);
    m_macStateLogger(m_lrWpanMacState, newState);
    m_lrWpanMacState = newState;
}

void LrWpanMac::SetDsmeModeEnabled() {
    NS_ASSERT(m_macDSMEcapable);
    m_macDSMEenabled = true;
}

void LrWpanMac::SetDsmeModeDisabled() {
    m_macDSMEenabled = false;
}

void LrWpanMac::SetHoppingSeqLen(uint16_t len) {
    m_hoppingSeqLen = len;
}

void LrWpanMac::SetHoppingSeq(HoppingSequence seq) {
    m_macHoppingSeqList = seq;
}

void LrWpanMac::SetChannelHoppingEnabled() {
    NS_ASSERT(m_macHoppingCapable);
    m_macHoppingEnabled = true;
}

void LrWpanMac::SetChannelHoppingNotEnabled() {
    NS_ASSERT(m_macHoppingCapable);
    m_macHoppingEnabled = false;
}

void LrWpanMac::SetTimeSlotToSendBcn(uint16_t idx) {
    m_choosedSDIndexToSendBcn = idx;
}

void LrWpanMac::SetDescIndexOfAssociatedPan(int idx) {
    m_descIdxOfAssociatedPan = idx;
}

uint16_t LrWpanMac::GetTimeSlotToSendBcn() const {
    return m_choosedSDIndexToSendBcn;
}

uint64_t
LrWpanMac::GetMacAckWaitDuration() const
{
    return m_csmaCa->GetUnitBackoffPeriod() + m_phy->aTurnaroundTime + m_phy->GetPhySHRDuration() +
           ceil(6 * m_phy->GetPhySymbolsPerOctet());
}

uint8_t
LrWpanMac::GetMacMaxFrameRetries() const
{
    return m_macMaxFrameRetries;
}

void
LrWpanMac::PrintTransmitQueueSize()
{
    NS_LOG_DEBUG("Transmit Queue Size: " << m_txQueue.size());
}

void
LrWpanMac::SetMacMaxFrameRetries(uint8_t retries)
{
    m_macMaxFrameRetries = retries;
}

void LrWpanMac::SetAsCoordinator() {
    m_coord = true;
}

void LrWpanMac::SetNotCoordinator() {
    m_coord = false;
}

bool LrWpanMac::IsCoord() const {
    return m_coord;
}

bool LrWpanMac::isCAPReductionOn() {
    return m_macCAPReductionFlag;
}

void LrWpanMac::SetCAPReduction(bool on) {
    m_macCAPReductionFlag = on;
}

void LrWpanMac::SetDsmeMacIsForIntegratingWithHigerLayer(bool on) {
    m_forDsmeNetDeviceIntegrateWithHigerLayer = on;
}

void LrWpanMac::SetAcceptAllHilowPkt(bool on) {
    m_acceptAllHilowPkt = on;
}

void LrWpanMac::SetGtsContinuePktSendingFromCap(bool on) {
    m_gtsContinuePktSendingFromCap = on;
}

void LrWpanMac::SetEnhancedGTSForwarding(bool on) {
    m_enhancedGTSForwarding = on;
}

uint32_t
LrWpanMac::GetIfsSize()
{
    NS_ASSERT(m_txPkt);

    if (m_txPkt->GetSize() <= aMaxSIFSFrameSize)
    {
        return m_macSIFSPeriod;
    }
    else
    {
        return m_macLIFSPeriod;
    }
}

void
LrWpanMac::SetAssociatedCoor(Mac16Address mac)
{
    m_macCoordShortAddress = mac;
}

uint64_t
LrWpanMac::GetTxPacketSymbols()
{
    NS_ASSERT(m_txPkt);
    // Sync Header (SHR) +  8 bits PHY header (PHR) + PSDU
    return (m_phy->GetPhySHRDuration() + 1 * m_phy->GetPhySymbolsPerOctet() +
            (m_txPkt->GetSize() * m_phy->GetPhySymbolsPerOctet()));
}

bool
LrWpanMac::isTxAckReq()
{
    NS_ASSERT(m_txPkt);
    LrWpanMacHeader macHdr;
    m_txPkt->PeekHeader(macHdr);

    return macHdr.IsAckReq();
}

uint16_t
LrWpanMac::GetSuperframeIDx()
{
    return m_curSuperframeIDx;
}

void
LrWpanMac::SetSuperframeIDx(uint16_t curSuperframeIDx)
{
    m_curSuperframeIDx = curSuperframeIDx;
}

void LrWpanMac::SetGroupAckPolicy(LrWpanGroupAckPolicy policy) 
{
    m_groupAckPolicy = policy;
}

uint64_t LrWpanMac::GenerateHashTableKey(Mac16Address devAddr, uint32_t packetSeq)
{

    uint8_t buffer16MacAddr[2];
    devAddr.CopyTo(buffer16MacAddr);

    int bufSize = 20;
    char addrBuf[bufSize] = {0};
    // Addr ConvertTo Int
    sprintf(addrBuf, "%d", (int)((buffer16MacAddr[0] << 8) | buffer16MacAddr[1]) + packetSeq); // We use the dev addr + packet sequence to generate the unique key.
    
    // Generate the value of hash table key, use hash function.
    // Then mod 61 (beacause the group ack hash table bitmap is 64 bit, we find a closest prime numbers here , which is 61).
    uint32_t primeNumClosestToBitmapSize = 61;
    uint64_t hashedValue = Hash64(addrBuf, bufSize);

    // Do the first hash function by mod
    uint64_t hashTableKey = hashedValue % primeNumClosestToBitmapSize;     

    // Check is collision with exist keys
    hashTableKey = CheckCollision(hashTableKey, hashedValue);

    return hashTableKey;
}

bool LrWpanMac::IsHashTableKeyCollision(uint32_t inputHashTableKey)
{
    // Check the spcific bit of the bitmap
    uint64_t mask = (uint64_t)1 << inputHashTableKey;
    // NS_LOG_DEBUG("m_enhancedGACKBitmap & mask = " << (m_enhancedGACKBitmap & mask));
    return (m_enhancedGACKBitmap & mask); // TODO Need to check Sanity    
}

uint32_t LrWpanMac::CheckCollision(uint32_t key, uint64_t hashedVal)
{
    // First Check the key is collision (duplicate with previous keys) or not.
    if(IsHashTableKeyCollision(key))
    {
        // Do the ** double hashing ** or ** Quadratic probing if there is a collision.
        // key = DoDoubleHash(key, hashedVal);
        key = DoQuadraticProb(key, 1);
    }

    return key;
}

uint32_t LrWpanMac::DoQuadraticProb(uint32_t key, uint32_t count)
{
    // Do the ** Quadratic probing ** if there is a collision.
    // initial count = 1;
    // TODO : Need to check if larger than bitmap size, otherwise, it will overflow.
    if (!IsHashTableKeyCollision(key)) // 沒重複了，可以Return
    {
        return key;
    }

    NS_LOG_DEBUG("Found key " << key << " collision");
    uint32_t newKey = key + (count * count);  // ! May excceed to limit of table size, need check
    return DoQuadraticProb(newKey, count + 1);
}

void LrWpanMac::PrintGroupAckBitmap()
{
    std::bitset<64> bitmap(m_enhancedGACKBitmap); 
    std::string bitmapStr = bitmap.to_string();
    NS_LOG_DEBUG("Group Ack Bitmap : " << bitmapStr);
}

void LrWpanMac::ResetEnhancedGroupAckBitmap()
{
    m_enhancedGACKBitmap = 0;
}

void LrWpanMac::ResetLegacyGroupAckBitmap()
{
    m_legacyGackBitmap = 0;
}

void LrWpanMac::SendEnhancedGroupAck()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("m_lrWpanMacState - " << m_lrWpanMacState);
    NS_ASSERT(m_lrWpanMacState == MAC_GTS); // If error check here
    NS_ASSERT(m_groupAckPolicy == GROUP_ACK_ENHANCED);

    if(m_macPanId == 0xffff) // TODO : This is a workaround !! Root cause do not found yet ..
    {                        // After a device leave the PAN (disassociation), the PAN id is set to 0xffff.
        return;
    }

    Ptr<Packet> enhancedGroupAckPacket = Create<Packet>();

    LrWpanMacHeader macHdr;                         // Enhanced Group Ack MDR
                                                    // Enhanced Group Ack has no payload

    /**
     *  Set MHD Frame control field
     */

    macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT); // Frame Type
    macHdr.SetSecDisable(); // Security Enabled
    // Frame pending
    if (m_indTxQueue.size()) 
    {
        macHdr.SetFrmPend();  // DSME-TODO : Set the Pending Address Fields
    }
    macHdr.SetNoAckReq(); // AR
    macHdr.SetNoPanIdComp(); // PAN ID compression
    macHdr.SetNoSeqNumSup() ;
    macHdr.SetIEListPresent(); // If there is a IE , set to one
    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4);
    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);

    /**
     *  Set Sequence Number field
     */
    macHdr.SetSeqNum(m_macDsn.GetValue());
    m_macDsn++;

    /**
     *  Set Destination addr
     */
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff")); // broadcast packet

    /**
     *  Set Source addr
     */
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    /**
     * Set Enhanced Group Payload IE 
    */
    PayloadIETermination plIETermination;
    enhancedGroupAckPacket->AddHeader(plIETermination);     

    /**
     * Set Enhanced Group Header IE 
    */

    HeaderIETermination hdrIETermination;
    enhancedGroupAckPacket->AddHeader(hdrIETermination);

    EnhancedGroupAckDescriptorIE enhancedGackDescriptorIE;

    enhancedGackDescriptorIE.SetIELength(8); // bitmap size : 8 bytes = 64 bits 
    enhancedGackDescriptorIE.SetHeaderIEDescriptor();

    // For testing purpose
    // uint64_t testBitmap = 4;
    // enhancedGackDescriptorIE.SetGroupAckBitmap(testBitmap); 

    enhancedGackDescriptorIE.SetGroupAckBitmap(m_enhancedGACKBitmap);  
    
    // Add Header IE
    enhancedGroupAckPacket->AddHeader(enhancedGackDescriptorIE);

    // Add Header
    enhancedGroupAckPacket->AddHeader(macHdr); 

    LrWpanMacTrailer macTrailer;
    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(enhancedGroupAckPacket);
    }

    enhancedGroupAckPacket->AddTrailer(macTrailer);

    // Set the Beacon packet to be transmitted
    m_txPkt = enhancedGroupAckPacket;

    NS_LOG_DEBUG("Send a Enhanced Group Ack Packet");

    ChangeMacState(MAC_GTS_SENDING);
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

void LrWpanMac::ResetEnhancedGroupAckBuffer()
{
    m_groupAckPktBuffer.clear();
}

void LrWpanMac::SendLegacyGroupAck()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("m_lrWpanMacState - " << m_lrWpanMacState);
    NS_ASSERT(m_lrWpanMacState == MAC_GTS); // If error check here
    NS_ASSERT(m_groupAckPolicy == GROUP_ACK_LEGACY);

    if(m_macPanId == 0xffff) // TODO : This is a workaround !! Root cause do not found yet ..
    {                        // After a device leave the PAN (disassociation), the PAN id is set to 0xffff.
        return;
    }

    Ptr<Packet> legacyGroupAckPacket = Create<Packet>();

    LrWpanMacHeader macHdr;                         // Legacy Group Ack MDR
                                                    // Legacy Group Ack has no payload

    /**
     *  Set MHD Frame control field
     */

    macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT); // Frame Type
    macHdr.SetSecDisable(); // Security Enabled
    // Frame pending
    if (m_indTxQueue.size()) 
    {
        macHdr.SetFrmPend();  // DSME-TODO : Set the Pending Address Fields
    }
    macHdr.SetNoAckReq(); // AR
    macHdr.SetNoPanIdComp(); // PAN ID compression
    macHdr.SetNoSeqNumSup() ;
    macHdr.SetIEListPresent(); // If there is a IE , set to one
    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4);
    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);

    /**
     *  Set Sequence Number field
     */
    macHdr.SetSeqNum(m_macDsn.GetValue());
    m_macDsn++;

    /**
     *  Set Destination addr
     */
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff")); // broadcast packet

    /**
     *  Set Source addr
     */
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    /**
     * Set legacy Group Payload IE 
    */
    PayloadIETermination plIETermination;
    legacyGroupAckPacket->AddHeader(plIETermination);     

    /**
     * Set legacy Group Header IE 
    */
    HeaderIETermination hdrIETermination;
    legacyGroupAckPacket->AddHeader(hdrIETermination);

    // TODO 
    LegacyGroupAckIE legacyGackIE;

    legacyGackIE.SetIELength(7); // bitmap size : 7bytes

    legacyGackIE.SetGackBitmapField(m_legacyGackBitmap);
    legacyGackIE.SetGackDevListField(m_legacyGackDevList);
    legacyGackIE.SetGackIdxBitmapField(m_legacyGackIdx);
    legacyGackIE.SetGtsDirectionBitmapField(m_legacyGackDirections);
    
    // Add Header IE
    legacyGroupAckPacket->AddHeader(legacyGackIE);

    // Add Header
    legacyGroupAckPacket->AddHeader(macHdr); 

    LrWpanMacTrailer macTrailer;
    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(legacyGroupAckPacket);
    }

    legacyGroupAckPacket->AddTrailer(macTrailer);

    // Set the Beacon packet to be transmitted
    m_txPkt = legacyGroupAckPacket;

    NS_LOG_DEBUG("Send a Legacy Group Ack Packet");

    ChangeMacState(MAC_GTS_SENDING);
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

void LrWpanMac::ResetLegacyGroupAckBuffer()
{
    m_legacyGackGTSIdxBuffer.clear();
}

void LrWpanMac::SetDsmeGtsPayloadLength(uint8_t len)
{
    m_dsmeGtsGackPayloadLength = len;
}

void LrWpanMac::ResetDsmeGtsGackPayloads()
{
    m_dsmeGtsGackPayloads.clear();
}

void LrWpanMac::ResetDsmeGtsGackPayloadLength()
{
    m_dsmeGtsGackPayloadLength = 0;
}

void LrWpanMac::ResetDsmeGtsGroupAckBuffer()
{
    m_dsmeGtsGroupAckPktBuffer.clear();
}

void LrWpanMac::Set6lowpanDataNoACK(bool NoACK)
{
    m_NoACK = NoACK;
}

} // namespace ns3
