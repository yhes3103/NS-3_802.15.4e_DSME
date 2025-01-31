/*
 * Copyright (c) 2011 The Boeing Company
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
 * Authors:
 *  Gary Pei <guangyu.pei@boeing.com>
 *  kwong yin <kwong-sang.yin@boeing.com>
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 *  Erwan Livolant <erwan.livolant@inria.fr>
 *  Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */
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





#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT std::clog << "[address " << m_shortAddress << "] ";

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
    m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
    m_deviceCapability = DeviceType::FFD;
    m_associationStatus = ASSOCIATED;
    m_selfExt = Mac64Address::Allocate();

    m_macPromiscuousMode = false;

    m_acceptAllHilowPkt = false;
    m_gtsContinuePktSendingFromCap = false;
    m_record = nullptr;

    m_macMaxFrameRetries = 3;
    
    m_retransmission = 0;
    m_numCsmacaRetry = 0;
    m_txPkt = nullptr;
    m_rxPkt = nullptr;
    m_ifs = 0;

    m_macLIFSPeriod = 40;
    m_macSIFSPeriod = 12;

    m_panCoor = false;
    m_coord = false;
    m_macBeaconOrder = 15;
    m_macSuperframeOrder = 15;
    m_macTransactionPersistenceTime = 500; // 0x01F5
    m_macAssociationPermit = true;
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

    // DSME-TODO
    // m_hoppingSeqID
    // m_macChannelPage;
    // m_numOfChannels;
    // m_macPhyConfiguration;
    // m_macExtBitmap;
    // m_hoppingSeqLen;
    // m_macHoppingSeqList;
    // m_macCurrentHop;
    // m_hopDwellTime;
    // m_macChannelIndex;
    // m_macLinkStatusStatisticPeriod;

    m_macGACKFlag = false;
    m_macCAPReductionFlag = false;

    m_macChannelDiversityMode = 0x01;

    m_macMultisuperframeOrder = 15;

    // m_macDSMESAB;
    // m_macDsmeACT;
    m_macSDindex = 0;
    // m_macBcnBitmap;
    m_macChannelOfs = 0;
    m_macDeferredBcnUsed = false;
    // m_macSyncParentExtAddr;
    m_macSyncParentShortAddr = Mac16Address("ff:ff");

    m_macBcnSlotLen = 60;

    m_macDSMEGTSExpirationTime = 7;

    // m_macChannelOfsBitmapLen;

    // m_macChannelOfsBitmap;

    m_macPANCoordinatorBSN = SequenceNumber8(uniformVar->GetValue());;

    // m_macNeighborInformationTable;

    m_simpleAddress = Mac8Address(0xff);

    // Table 52n
    m_macUseEnhancedBeacon = true;
    m_macEbsn = SequenceNumber8(uniformVar->GetValue());
    m_macEBAutoSA = EBAutoSA_FULL;

    m_sendBcn = false;
    realignmentRecevied = false;

    m_incSuperframe = false;
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

void
LrWpanMac::SetExtendedAddress(Mac64Address address)
{
    NS_LOG_FUNCTION(this << address);
    m_selfExt = address;
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
    NS_LOG_FUNCTION(this);
    return m_selfExt;
}

void
LrWpanMac::McpsDataRequest(McpsDataRequestParams params, Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << p);

    NS_LOG_DEBUG("Prepare a data packet with size:" << p->GetSize() << " bytes");

    m_mcpsDataRequestParams = params;

    McpsDataConfirmParams confirmParams;
    confirmParams.m_msduHandle = params.m_msduHandle;

    // TODO: We need a drop trace for the case that the packet is too large or the request
    // parameters are maleformed.
    //       The current tx drop trace is not suitable, because packets dropped using this trace
    //       carry the mac header and footer, while packets being dropped here do not have them.

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_DATA, m_macDsn.GetValue());
    m_macDsn++;

    if (p->GetSize() > LrWpanPhy::aMaxPhyPacketSize - aMinMPDUOverhead)
    {
        // Note, this is just testing maximum theoretical frame size per the spec
        // The frame could still be too large once headers are put on
        // in which case the phy will reject it instead
        NS_LOG_ERROR(this << " packet too big: " << p->GetSize());
        confirmParams.m_status = IEEE_802_15_4_FRAME_TOO_LONG;
        if (!m_mcpsDataConfirmCallback.IsNull())
        {
            m_mcpsDataConfirmCallback(confirmParams);
        }
        return;
    }

    if ((params.m_srcAddrMode == NO_PANID_ADDR) && (params.m_dstAddrMode == NO_PANID_ADDR))
    {
        NS_LOG_ERROR(this << " Can not send packet with no Address field");
        confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
        if (!m_mcpsDataConfirmCallback.IsNull())
        {
            m_mcpsDataConfirmCallback(confirmParams);
        }
        return;
    }
    switch (params.m_srcAddrMode)
    {
    case NO_PANID_ADDR:
        macHdr.SetSrcAddrMode(params.m_srcAddrMode);
        macHdr.SetNoPanIdComp();
        break;
    // DSME-TODO
    // case ADDR_MODE_RESERVED:
    //     NS_ABORT_MSG("Can not set source address type to ADDR_MODE_RESERVED. Aborting.");
    //     break;
    case SIMPLE_ADDR:
        macHdr.SetSrcAddrMode(params.m_srcAddrMode);
        macHdr.SetSrcAddrFields(GetPanId(), GetSimpleAddress());
        break;

    case SHORT_ADDR:
        macHdr.SetSrcAddrMode(params.m_srcAddrMode);
        macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());
        break;
    case EXT_ADDR:
        macHdr.SetSrcAddrMode(params.m_srcAddrMode);
        macHdr.SetSrcAddrFields(GetPanId(), GetExtendedAddress());
        break;
    default:
        NS_LOG_ERROR(this << " Can not send packet with incorrect Source Address mode = "
                          << params.m_srcAddrMode);
        confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
        if (!m_mcpsDataConfirmCallback.IsNull())
        {
            m_mcpsDataConfirmCallback(confirmParams);
        }
        return;
    }

    switch (params.m_dstAddrMode)
    {
    case NO_PANID_ADDR:
        macHdr.SetDstAddrMode(params.m_dstAddrMode);
        macHdr.SetNoPanIdComp();
        break;
    // DSME-TODO
    // case ADDR_MODE_RESERVED:
    //     NS_ABORT_MSG("Can not set destination address type to ADDR_MODE_RESERVED. Aborting.");
    //     break;

    // case SIMPLE_ADDR:
    //     macHdr.SetDstAddrMode(params.m_dstAddrMode);
    //     macHdr.SetDstAddrFields(params.m_dstPanId, params.m_dst); 
    //     break;    

    case SHORT_ADDR:
        macHdr.SetDstAddrMode(params.m_dstAddrMode);
        macHdr.SetDstAddrFields(params.m_dstPanId, params.m_dstAddr);
        break;
    case EXT_ADDR:
        macHdr.SetDstAddrMode(params.m_dstAddrMode);
        macHdr.SetDstAddrFields(params.m_dstPanId, params.m_dstExtAddr);
        break;
    default:
        NS_LOG_ERROR(this << " Can not send packet with incorrect Destination Address mode = "
                          << params.m_dstAddrMode);
        confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
        if (!m_mcpsDataConfirmCallback.IsNull())
        {
            m_mcpsDataConfirmCallback(confirmParams);
        }
        return;
    }

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
                macHdr.SetAckReq();
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

        NS_LOG_DEBUG("Sending a data packet during a GTS period.");

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

        if (m_lrWpanMacState == MAC_GTS) {
            m_txPkt = p;

            ChangeMacState(MAC_GTS_SENDING);
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);

        } else {
            m_txPktGts = p;
        }
    }
    else if (b2 == TX_OPTION_INDIRECT)
    {
        // Indirect Tx
        // A COORDINATOR will save the packet in the pending queue and await for data
        // requests from its associated devices. The devices are aware of pending data,
        // from the pending bit information extracted from the received beacon.
        // A DEVICE must be tracking beacons (MLME-SYNC.request is running) before attempting
        // request data from the coordinator.

        // TODO: Check if the current device is coordinator (not just pan coordinator)
        //  Indirect Transmission can only be done by PAN coordinator or coordinators.
        // NS_ASSERT(m_panCoor);
        NS_ASSERT(m_coord);
        p->AddHeader(macHdr);

        LrWpanMacTrailer macTrailer;
        // Calculate FCS if the global attribute ChecksumEnable is set.
        if (Node::ChecksumEnabled()) {
            macTrailer.EnableFcs(true);
            macTrailer.SetFcs(p);
        }

        p->AddTrailer(macTrailer);

        // NS_LOG_ERROR(this << " Indirect transmissions not currently supported");
        // Note: The current Pending transaction list should work for indirect transmissions.
        // However, this is not tested yet. For now, we block the use of indirect transmissions.
        // TODO: Save packet in the Pending Transaction list.
        EnqueueInd (p);
    }
    else if (b3 == TX_OPTION_DIRECT)
    {
        // Direct Tx
        // From this point the packet will be pushed to a Tx queue and immediately
        // use a slotted (beacon-enabled) or unslotted (nonbeacon-enabled) version of CSMA/CA
        // before sending the packet, depending on whether it has previously
        // received a valid beacon or not.

        p->AddHeader(macHdr);

        LrWpanMacTrailer macTrailer;
        // Calculate FCS if the global attribute ChecksumEnable is set.
        if (Node::ChecksumEnabled())
        {
            macTrailer.EnableFcs(true);
            macTrailer.SetFcs(p);
        }
        p->AddTrailer(macTrailer);

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
    NS_ASSERT(m_coord || params.m_panCoor);

    MlmeStartConfirmParams confirmParams;

    if (GetShortAddress() == Mac16Address("ff:ff")) {
        NS_LOG_ERROR(this << " Invalid MAC short address");
        confirmParams.m_status = MLMESTART_NO_SHORT_ADDRESS;

        if (!m_mlmeStartConfirmCallback.IsNull()) {
            m_mlmeStartConfirmCallback(confirmParams);
        }

        return;
    }

    if (params.m_panCoor && 
        ((params.m_bcnOrd > 15) || (params.m_sfrmOrd > params.m_bcnOrd))) {
        confirmParams.m_status = MLMESTART_INVALID_PARAMETER;

        if (!m_mlmeStartConfirmCallback.IsNull()) {
            m_mlmeStartConfirmCallback(confirmParams);
        }

        NS_LOG_ERROR(this << "Incorrect superframe order or beacon order.");
        return;

    } 

    if (params.m_bcnBitmap.GetSDIndex() > 0 && !m_beaconTrackingOn) {
        confirmParams.m_status = MLMESTART_TRACKING_OFF;

        if (!m_mlmeStartConfirmCallback.IsNull()) {
            m_mlmeStartConfirmCallback(confirmParams);
        }

        NS_LOG_ERROR(this << "MAC sublayer is not currently tracking the beacon of its coordinator");

        return;
    }

    // macMultiSuperframeOrder setting...
    if (params.m_panCoor && m_macDSMEenabled && (params.m_bcnOrd != 15)) {
        // 0 ≤ SO ≤ MO ≤ BO ≤ 14
        if (params.m_bcnOrd < params.m_dsmeSuperframeSpec.GetMultiSuperframeOrder()) {
            confirmParams.m_status = MLMESTART_INVALID_PARAMETER;

            if (!m_mlmeStartConfirmCallback.IsNull()) {
                m_mlmeStartConfirmCallback(confirmParams);
            }

            NS_LOG_ERROR(this << "Incorrect pararmeter: Multisuperframe order cannot be larger than beacon order.");

            return;
        }

        if (params.m_dsmeSuperframeSpec.GetMultiSuperframeOrder() < params.m_sfrmOrd) {
            if (!m_mlmeStartConfirmCallback.IsNull()) {
                m_mlmeStartConfirmCallback(confirmParams);
            }

            NS_LOG_ERROR(this << " Incorrect pararmeter: Superframe order cannot be larger than multisuperframe order");

            return;
        }
    }

    // Mark primitive as pending and save the start params while the new page and channel is set.
    m_startParams = params;

    if (!params.m_coorRealgn) {
        m_pendPrimitive = MLME_START_REQ;

        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentPage = m_startParams.m_logChPage;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentPage, &pibAttr);

    } else {
        EndStartRequest();
    }
}

void
LrWpanMac::MlmeScanRequest(MlmeScanRequestParams params)
{
    NS_LOG_FUNCTION(this);

    if (params.m_scanType == MLMESCAN_ENHANCED_ACTIVE_SCAN) {
        NS_ASSERT(m_macDSMEenabled);
    }

    MlmeScanConfirmParams confirmParams;
    confirmParams.m_scanType = params.m_scanType;
    confirmParams.m_chPage = params.m_chPage;

    if (m_scanEvent.IsRunning() || m_scanEnergyEvent.IsRunning()) {
        if (!m_mlmeScanConfirmCallback.IsNull()) {
            confirmParams.m_status = MLMESCAN_SCAN_IN_PROGRESS;
            m_mlmeScanConfirmCallback(confirmParams);
        }

        NS_LOG_ERROR(this << " A channel scan is already in progress");
        return;
    }

    if (params.m_scanDuration > 14 || params.m_scanType > MLMESCAN_ENHANCED_ACTIVE_SCAN) {
        if (!m_mlmeScanConfirmCallback.IsNull()) {
            confirmParams.m_status = MLMESCAN_INVALID_PARAMETER;
            m_mlmeScanConfirmCallback(confirmParams);
        }

        NS_LOG_ERROR(this << "Invalid scan duration or unsupported scan type");
        return;
    }

    // Temporary store macPanId and set macPanId to 0xFFFF to accept all beacons.
    m_macPanIdScan = m_macPanId;
    m_macPanId = 0xFFFF;

    m_panDescriptorList.clear();
    m_energyDetectList.clear();

    // DSME-TODO: stop beacon transmission

    // Cancel any ongoing CSMA/CA operations and set to unslotted mode for scan
    m_csmaCa->Cancel();
    m_capEvent.Cancel();
    m_cfpEvent.Cancel();
    m_incCapEvent.Cancel();
    m_incCfpEvent.Cancel();
    m_trackingEvent.Cancel();
    m_csmaCa->SetUnSlottedCsmaCa();

    m_channelScanIndex = 0;

    NS_LOG_DEBUG("Start Channel Scan.");

    // Mark primitive as pending and save the scan params while the new page and/or channel is set.
    m_scanParams = params;
    m_pendPrimitive = MLME_SCAN_REQ;

    LrWpanPhyPibAttributes pibAttr;
    pibAttr.phyCurrentPage = params.m_chPage;
    m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentPage, &pibAttr);
}

void
LrWpanMac::MlmeAssociateRequest(MlmeAssociateRequestParams params)
{
    NS_LOG_FUNCTION(this);

    // Association is typically preceded by beacon reception and a  MLME-SCAN.request, therefore,
    // the values of the Associate.request params usually come from the information
    // obtained from those operations.
    m_pendPrimitive = MLME_ASSOC_REQ;
    m_associateParams = params;
    bool invalidRequest = false;

    if (params.m_coordPanId == 0xffff)
    {
        invalidRequest = true;
    }

    if (!invalidRequest && params.m_coordAddrMode == SHORT_ADDR)
    {
        if (params.m_coordShortAddr == Mac16Address("ff:ff") ||
            params.m_coordShortAddr == Mac16Address("ff:fe"))
        {
            invalidRequest = true;
        }
    }
    else if (!invalidRequest && params.m_coordAddrMode == EXT_ADDR)
    {
        if (params.m_coordExtAddr == Mac64Address("ff:ff:ff:ff:ff:ff:ff:ff") ||
            params.m_coordExtAddr == Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed"))
        {
            invalidRequest = true;
        }
    }

    if (invalidRequest)
    {
        m_pendPrimitive = MLME_NONE;
        m_associateParams = MlmeAssociateRequestParams();
        NS_LOG_ERROR(this << " Invalid PAN id in Association request");
        if (!m_mlmeAssociateConfirmCallback.IsNull())
        {
            MlmeAssociateConfirmParams confirmParams;
            confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
            confirmParams.m_status = MLMEASSOC_INVALID_PARAMETER;
            m_mlmeAssociateConfirmCallback(confirmParams);
        }
    }
    else
    {
        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentPage = params.m_chPage;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentPage, &pibAttr);
    }
}

void LrWpanMac::EndAssociateRequest() {
    // the primitive is no longer pending (channel & page are set)
    m_pendPrimitive = MLME_NONE;

    // As described in IEEE 802.15.4-2011 (Section 5.1.3.1)
    m_macPanId = m_associateParams.m_coordPanId;
    if (m_associateParams.m_coordAddrMode == SHORT_ADDR) {
        m_macCoordShortAddress = m_associateParams.m_coordShortAddr;

    } else {
        m_macCoordExtendedAddress = m_associateParams.m_coordExtAddr;
        m_macCoordShortAddress = Mac16Address("ff:fe");
    }

    // DSME
    if (m_macDSMEenabled) {
        SendDsmeAssocRequestCommand();
    } else {
        SendAssocRequestCommand();
    }
}

void LrWpanMac::MlmeAssociateResponse(MlmeAssociateResponseParams params) {
    // Associate Short Address (m_assocShortAddr)
    // FF:FF = Association Request failed
    // FF:FE = The association request is accepted, but the device should use its extended address
    // Other = The assigned short address by the coordinator

    NS_LOG_FUNCTION(this);

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    // Mac header Assoc. Response Comm. See 802.15.4-2011 (Section 5.3.2.1)
    macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetPanIdComp();
    macHdr.SetDstAddrFields(m_macPanId, params.m_extDevAddr);
    macHdr.SetSrcAddrFields(0xffff, GetExtendedAddress());

    CommandPayloadHeader macPayload;

    // DSME
    // Dsme association response command
    if (m_macDSMEenabled) {
        macPayload.SetCommandFrameType(CommandPayloadHeader::DSME_ASSOCIATION_RESP);
    } else {
        macPayload.SetCommandFrameType(CommandPayloadHeader::ASSOCIATION_RESP);
    }    

    macPayload.SetShortAddr(params.m_assocShortAddr);

    switch (params.m_status) {
        case LrWpanAssociationStatus::ASSOCIATED:
            macPayload.SetAssociationStatus(CommandPayloadHeader::SUCCESSFUL);
            break;

        case LrWpanAssociationStatus::PAN_AT_CAPACITY:
            macPayload.SetAssociationStatus(CommandPayloadHeader::FULL_CAPACITY);
            break;

        case LrWpanAssociationStatus::PAN_ACCESS_DENIED:
            macPayload.SetAssociationStatus(CommandPayloadHeader::ACCESS_DENIED);
            break;

        case LrWpanAssociationStatus::ASSOCIATED_WITHOUT_ADDRESS:
            NS_LOG_ERROR("Error, Associated without address");
            break;
        
        case LrWpanAssociationStatus::ASSOCIATION_STATUS_FIELD_RESERVED:
            NS_LOG_ERROR("Error, Associated without address");
            break;

        case LrWpanAssociationStatus::HOPPING_SEQ_OFS_DUPLICATION:
            macPayload.SetAssociationStatus(CommandPayloadHeader::HOPPING_SEQ_OFS_DUPLICATION);
            macPayload.SetShortAddr(Mac16Address("FF:FF"));
            break;

        case LrWpanAssociationStatus::FASTA_SUCCESSFUL:
            macPayload.SetAssociationStatus(CommandPayloadHeader::FASTA_SUCCESSFUL);
            break;

        case LrWpanAssociationStatus::DISASSOCIATED:
            NS_LOG_ERROR("Error, device not associated");
            break;
    }

    macHdr.SetSecDisable();
    macHdr.SetNoFrmPend();
    macHdr.SetAckReq();

    // Dsme association response command
    if (m_macDSMEenabled && m_macHoppingEnabled) {
        // beacon mode
        if (m_macHoppingSeqID == 1 && m_csmaCa->IsSlottedCsmaCa() && m_macChannelDiversityMode == 1) {
            macPayload.SetHoppingSeqLen(params.m_hoppingSeqLen);
            macPayload.SetHoppingSequnce(params.m_hoppingSeq);

        } else {
            macPayload.SetHoppingSeqLen(0);
        }
    }

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    if (params.m_status == LrWpanAssociationStatus::FASTA_SUCCESSFUL) {  // FastA
        Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
        txQElement->txQPkt = commandPacket;
        EnqueueTxQElement(txQElement);
        CheckQueue();
        
    } else {
        // Save packet in the Pending Transaction list.
        EnqueueInd(commandPacket);
    }
}

void LrWpanMac::MlmeSyncRequest(MlmeSyncRequestParams params) {
    NS_LOG_FUNCTION(this);
    NS_ASSERT(params.m_logCh <= 26 && m_macPanId != 0xffff);

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

void LrWpanMac::MlmePollRequest(MlmePollRequestParams params) {
    NS_LOG_FUNCTION(this);

    NS_LOG_DEBUG("MLME-POLL.request service called.");

    // TODO: complete poll request (part of indirect transmissions)
    // NS_FATAL_ERROR(this << " Poll request currently not supported");
    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macBsn.GetValue());
    m_macBsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    if (GetShortAddress() == Mac16Address("ff:fe")) {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetSrcAddrFields(0xffff, m_selfExt);
    } else {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetSrcAddrFields(0xffff, GetShortAddress());
    }

    if (params.m_coorAddrMode == SHORT_ADDR) {
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetDstAddrFields(params.m_coorPanId, params.m_coorShortAddr);
    } else if (params.m_coorAddrMode == EXT_ADDR) {
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetDstAddrFields(params.m_coorPanId, params.m_coorExtAddr);
    }

    macHdr.SetNoFrmPend();
    macHdr.SetSecDisable();
    macHdr.SetPanIdComp();
    macHdr.SetAckReq();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DATA_REQ);
    
    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    // Set the Command packet to be transmitted
    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::MlmeDsmeGtsRequest(MlmeDsmeGtsRequestParams params) {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(m_macDSMEenabled);
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:ff"));
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:fe"));

    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG(" Prepare GTS Request Command and Add it to Queue"); // debug

    m_dsmeGtsReqParams = params;

    // Send GTS Request Command
    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();            
    macHdr.SetAckReq();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), params.m_devAddr);

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_GTS_REQ);

    DSMEGTSManagementField managementField;

    // DSME-TODO
    switch (params.m_manageType) {
        case GTS_DEALLOCATION:
            managementField.SetManagementType(0b000);
            break;
        
        case GTS_ALLOCATION:
            managementField.SetManagementType(0b001);
            managementField.SetRX(params.m_direction);
            managementField.SetPrioritizedChannelAccess(params.m_prioritizedChAccess);

            macPayload.SetDsmeGtsNumOfSlot(params.m_numSlot);
            macPayload.SetDsmeGtsPreferredSuperframeId(params.m_preferredSuperframeID);
            macPayload.SetDsmeGtsPreferredSlotId(params.m_preferredSlotID);
            break;

        case GTS_DUPLICATED_ALLOCATION_NOTIF:
            managementField.SetManagementType(0b010);

            break;

        case GTS_REDUCE:
            managementField.SetManagementType(0b011);

            break;

        case GTS_RESTART:
            managementField.SetManagementType(0b100);

            break;

        case GTS_EXPIRATION:
            managementField.SetManagementType(0b101);

            break;

        default:
            break;
    }

    macPayload.SetDsmeGtsManagementField(managementField);

    // Set a bitmap of a sub-block of macDSMESAB
    DSMESABSpecificationField sABSpec;

    // DSME-TODO: 這 CAPReduction Flag怪怪的, 要從哪裡判斷得知?
    sABSpec.setCAPReduction(params.m_dsmeSABSpec.isCAPReduction());
    sABSpec.setSABSubBlkLen(params.m_dsmeSABSpec.GetSABSubBlkLen());
    sABSpec.setSABSubBlkIdx(params.m_dsmeSABSpec.GetSABSubBlkIdx());

    if (sABSpec.isCAPReduction()) {
        sABSpec.setSABSubBlk(params.m_dsmeSABSpec.GetSABSubBlk());
    } else {
        sABSpec.setSABSubBlk(params.m_dsmeSABSpec.GetSABSubBlkCapOff());
    }

    macPayload.SetDsmeGtsSABSpec(sABSpec);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::MlmeDsmeGtsResponse(MlmeDsmeGtsResponseParams params) {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(m_macDSMEenabled);
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:ff"));
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:fe"));

    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG(" Prepare GTS Reply Command and Add it to Queue"); // debug

    m_dsmeGtsRespParams = params;

    // Send GTS Reply Command
    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();            
    macHdr.SetAckReq();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff"));

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_GTS_REPLY);

    DSMEGTSManagementField managementField;

    // DSME-TODO
    switch (params.m_manageType) {
        case GTS_DEALLOCATION:
            managementField.SetManagementType(0b000);
            managementField.SetRX(params.m_direction);
            managementField.SetPrioritizedChannelAccess(params.m_prioritizedChAccess);
            break;
        
        case GTS_ALLOCATION:
            managementField.SetManagementType(0b001);
            managementField.SetRX(params.m_direction);
            managementField.SetPrioritizedChannelAccess(params.m_prioritizedChAccess);
            break;

        case GTS_DUPLICATED_ALLOCATION_NOTIF:

            break;

        case GTS_REDUCE:

            break;

        case GTS_RESTART:

            break;

        default:
            break;
    }

    switch (params.m_status) {
        case MLMEDSMEGTS_REQ_SUCCESS:
            managementField.SetStatus(0b000);
            break;
        
        case MLMEDSMEGTS_REQ_INVALID_PARAMETER:
            managementField.SetStatus(0b010);
            break;

        default:
            managementField.SetStatus(0b001);
            break;
    }

    macPayload.SetDsmeGtsManagementField(managementField);
    macPayload.SetDsmeGtsDestAddress(params.m_devAddr);
    
    if (m_macHoppingEnabled && m_macChannelDiversityMode == 1) {
        macPayload.SetChannelOfs(params.m_channelOfs);
    }
    
    macPayload.SetDsmeGtsSABSpec(params.m_dsmeSABSpec);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::MlmeOrphanResponse(MlmeOrphanResponsePararms params) {
    NS_LOG_FUNCTION(this);

    if (params.m_associateMember) {
        SendCoordinatorRealignmentCmd(true, true, params.m_orphanAddress, params.m_shortAddr);
    } else {
        // ignore this primitive
    }
}

void LrWpanMac::MlmeDsmeInfoRequest(MlmeDsmeInfoRequestParams params) {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(m_macDSMEenabled);

    NS_LOG_DEBUG(" Prepare Dsme Information Request Command and Add it to Queue"); // debug

    MlmeDsmeInfoConfirmParams confirmParams;

    if (params.m_dstAddrMode != SHORT_ADDR 
        && params.m_dstAddrMode != EXT_ADDR) {
        confirmParams.m_status = MLMEDSMEINFO_INVALID_PARAMETER;

        if (!m_mlmeDsmeInfoConfirmCallback.IsNull()) {
            m_mlmeDsmeInfoConfirmCallback(confirmParams);
        }
    }

    if (params.m_info < 0x00 || params.m_info > 0x03) {
        confirmParams.m_status = MLMEDSMEINFO_INVALID_PARAMETER;

        if (!m_mlmeDsmeInfoConfirmCallback.IsNull()) {
            m_mlmeDsmeInfoConfirmCallback(confirmParams);
        }
    }

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    if (params.m_dstAddrMode == SHORT_ADDR) {
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetDstAddrFields(GetPanId(), params.m_dstShortAddr);
    } else if (params.m_dstAddrMode == EXT_ADDR) {
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetDstAddrFields(GetPanId(), params.m_dstExtAddr);
    }

    macHdr.SetAckReq();
    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_INFO_REQ);

    macPayload.SetDsmeInfoType(params.m_info);

    if (params.m_info == MLMEDSMEINFO_DSME_SAB_SPECIFICATION) {
        macPayload.SetDsmeInfoSABSubBlkLen(params.m_dsmeSABSubBlkLen);
        macPayload.SetDsmeInfoSABSubBlkIdx(params.m_dsmeSABSubBlkIdx);
    }

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendCoordinatorRealignmentCmd(bool orphanOrNot, bool channelPagePresent
                                             , Mac64Address dst, Mac16Address storedShortAddr) {
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG(" Prepare Coordinator Realignment Command and Add it to Queue"); // debug

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetExtendedAddress());

    if (orphanOrNot) {
        macHdr.SetAckReq();
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetDstAddrFields(0xffff, dst);
    } else {
        macHdr.SetNoAckReq();
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetDstAddrFields(0xffff, Mac16Address("ff:ff"));
    }

    if (channelPagePresent) {
        macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4_2006);
    } else {
        macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4);
    }

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::COOR_REALIGN);

    macPayload.SetPanId(m_startParams.m_PanId);
    macPayload.SetCoordinatorShortAddress(GetShortAddress());
    macPayload.SetChannelNum(m_startParams.m_logCh);

    if (orphanOrNot) {
        macPayload.SetShortAddr(storedShortAddr);
    } else {
        macPayload.SetShortAddr(Mac16Address("ff:fe"));
    }

    macPayload.SetChannelPage(m_startParams.m_logChPage);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendDsmeGtsReplyWithInvalidParam(Mac16Address dst
                                                , DSMESABSpecificationField sAB) {
    NS_ASSERT(m_macDSMEenabled);
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:ff"));
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:fe"));

    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG(" Prepare GTS Reply Command and Add it to Queue"); // debug

    // Send GTS Reply Command
    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();            
    macHdr.SetAckReq();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), dst);

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_GTS_REPLY);

    DSMEGTSManagementField managementField;
    managementField.SetManagementType(0b000);
    managementField.SetStatus(0b010);

    macPayload.SetDsmeGtsManagementField(managementField);
    macPayload.SetDsmeGtsDestAddress(dst);
    macPayload.SetDsmeGtsSABSpec(sAB);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void
LrWpanMac::SendOneBeacon()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_lrWpanMacState == MAC_IDLE);

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_BEACON, m_macBsn.GetValue());
    m_macBsn++;
    BeaconPayloadHeader macPayload;
    Ptr<Packet> beaconPacket = Create<Packet>();
    LrWpanMacTrailer macTrailer;

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff"));

    // see IEEE 802.15.4-2011 Section 5.1.2.4
    if (GetShortAddress() == Mac16Address("ff:fe"))
    {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetSrcAddrFields(GetPanId(), GetExtendedAddress());
    }
    else
    {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());
    }

    macHdr.SetSecDisable();
    macHdr.SetNoAckReq();

    macPayload.SetSuperframeSpecField(GetSuperframeField());
    macPayload.SetGtsFields(GetGtsFields());
    macPayload.SetPndAddrFields(GetPendingAddrFields());

    beaconPacket->AddHeader(macPayload);
    beaconPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled())
    {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(beaconPacket);
    }

    beaconPacket->AddTrailer(macTrailer);

    // Set the Beacon packet to be transmitted
    m_txPkt = beaconPacket;

    if (m_csmaCa->IsSlottedCsmaCa()) {
        m_outSuperframeStatus = BEACON;

        NS_LOG_DEBUG("Outgoing superframe Active Portion (Beacon + CAP + CFP): "
                    << m_superframeDuration << " symbols");
 
    } else {
        NS_LOG_DEBUG("Outgoing Beacon Frame response to Beacon Request" );
    }

    ChangeMacState(MAC_SENDING);
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

// void LrWpanMac::MlmeBeaconRequest(MlmeBeaconRequestParams params) {

// }

// See IEEE 802.15.4e-2012 Section 5.2.2.1 
void LrWpanMac::SendOneEnhancedBeacon() {
    NS_LOG_FUNCTION(this);
    
    // for dsme-sixlowpan-7-branches-3-levels-hilow-tree-topology.cc
    // if (m_incGtsEvent.IsRunning()) {
    //     m_incGtsEvent.Cancel();
    //     EndGTS(SuperframeType::INCOMING);
    // }

    // if (m_gtsEvent.IsRunning()) {
    //     m_gtsEvent.Cancel();
    //     EndGTS(SuperframeType::OUTGOING);
    // }

    NS_ASSERT(m_lrWpanMacState == MAC_IDLE);
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:ff"));

    m_startOfBcnSlot = Simulator::Now();

    LrWpanMacHeader macHdr;

    if (m_panCoor) {
        macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_BEACON);
        macHdr.SetSeqNum(m_macPANCoordinatorBSN.GetValue());
        m_macPANCoordinatorBSN++;

    } else {
        macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_BEACON);
        macHdr.SetSeqNum(m_macEbsn.GetValue());
        m_macEbsn++;
    }

    BeaconPayloadHeader macPayload;

    Ptr<Packet> beaconPacket = Create<Packet>();
    LrWpanMacTrailer macTrailer;

    // DSME: indicate this is a enhanced beacon
    macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4);

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff"));

    // see IEEE 802.15.4-2011 Section 5.1.2.4
    if (GetShortAddress() == Mac16Address("ff:fe")) {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetSrcAddrFields(GetPanId(), GetExtendedAddress());
    } else {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());
    }

    macHdr.SetSecDisable();
    macHdr.SetNoAckReq();

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

        m_dsmePanDescriptorIE.SetBeaconBitmap(m_macSDBitmap);

        ChannelHopping channelHoppingField;
        channelHoppingField.SetHoppingSequenceID(m_macHoppingSeqID);
        channelHoppingField.SetPANCoordinatorBSN(m_macPANCoordinatorBSN.GetValue() - 1);
        channelHoppingField.SetChannelOffset(m_macChannelOfs);
        channelHoppingField.SetChannelOffsetBitmapLength(m_macChannelOfsBitmapLen);
        channelHoppingField.SetChannelOffsetBitmap(m_macChannelOfsBitmap);
        m_dsmePanDescriptorIE.SetChannelHopping(channelHoppingField); 

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
    m_txPkt = beaconPacket;

    if (m_csmaCa->IsSlottedCsmaCa()) {
        m_outSuperframeStatus = BEACON;

        if (isCAPReductionOn()
            && m_macSDindex % (m_multiSuperframeDuration / m_superframeDuration) != 0) {
            NS_LOG_DEBUG("Outgoing superframe Active Portion (Beacon + CFP + CFP): "
                        << m_superframeDuration << " symbols");

        } else {
            NS_LOG_DEBUG("Outgoing superframe Active Portion (Beacon + CAP + CFP): "
                    << m_superframeDuration << " symbols");
        }
 
    } else {
        NS_LOG_DEBUG("Outgoing Enhanced Beacon Frame response to Enhanced Beacon Request" );
    }

    m_BeaconStartTxTime = Simulator::Now();

    ChangeMacState(MAC_SENDING);
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

void
LrWpanMac::SendBeaconRequestCommand()
{
    NS_LOG_FUNCTION(this);

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    // Beacon Request Command Mac header values See IEEE 802.15.4-2011 (Section 5.3.7)
    macHdr.SetNoPanIdComp();
    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrMode(LrWpanMacHeader::NOADDR);

    macHdr.SetDstAddrFields(0xFFFF,
                            Mac16Address("FF:FF")); // Not associated PAN, broadcast dst address

    macHdr.SetSecDisable();
    macHdr.SetNoAckReq();

    CommandPayloadHeader macPayload;
    macPayload.SetCommandFrameType(CommandPayloadHeader::BEACON_REQ);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled())
    {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendEnhancedBeaconRequestCommand() {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(m_macDSMEenabled);

    LrWpanMacHeader macHdr;
    macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_COMMAND);

    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    CommandPayloadHeader macPayload;
    macPayload.SetCommandFrameType(CommandPayloadHeader::BEACON_REQ);

    commandPacket->AddHeader(macPayload);

    // Enhanced Beacon Request Mac header values See IEEE 802.15.4e-2012 (Section 5.3.7.2)
    macHdr.SetSecDisable();
    macHdr.SetNoFrmPend();
    macHdr.SetNoAckReq();

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);

    if (GetPanId() != 0xffff) {   // associated
        macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);

        if (m_scanParams.m_frameCtrlOptions[0]) {   // PAN_ID_SUPPRESSED
            macHdr.SetPanIdComp();
            macHdr.SetSrcAddrFields(0xffff, GetShortAddress());
            macHdr.SetDstAddrFields(GetPanId(), Mac16Address("FF:FF"));
        } else {
            macHdr.SetNoPanIdComp();
            macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());
            macHdr.SetDstAddrFields(GetPanId(), Mac16Address("FF:FF"));
        }

    } else {                   
        // Not associated PAN, broadcast dst address
        macHdr.SetSrcAddrMode(LrWpanMacHeader::NOADDR);
        macHdr.SetDstAddrFields(0xffff, Mac16Address("FF:FF"));
    }

    if (m_scanParams.m_frameCtrlOptions[1]) {   // IES_INCLUDED
        macHdr.SetIEListPresent();

        // DSME-TODO 
        // filter IE here and any other IE
        // IE request 
        PayloadIETermination termination;
        commandPacket->AddHeader(termination);

        EBFilterIE filterIE;
        filterIE.SetPermitJoiningOn();
        filterIE.SetNotIncludeLinkQualityFilter();
        filterIE.SetNotIncludePercentFilter();
        filterIE.SetNumOfEntriesInPIBIdList(0);

        filterIE.SetOutterIEDescriptorLen(filterIE.GetSerializedSize() - 2);
        filterIE.SetSubIEDescriptorLen(filterIE.GetSerializedSize() - 4);

        commandPacket->AddHeader(filterIE);

        HeaderIETermination termination2;
        commandPacket->AddHeader(termination2);

    } else {
        macHdr.SetNoIEListPresent();
    }

    if (m_scanParams.m_frameCtrlOptions[2]) {   // SEQ_NUM_SUPPRESSED
        macHdr.SetSeqNumSup();
    } else {
        macHdr.SetNoSeqNumSup();
        macHdr.SetSeqNum(m_macDsn.GetValue());
        m_macDsn++;
    }

    macHdr.SetFrameVer(LrWpanMacHeader::IEEE_802_15_4);

    commandPacket->AddHeader(macHdr);    

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendDsmeInfoResponseCommand(Ptr<Packet> rxDsmeInfoReqPkt
                                            , uint16_t& superframeID
                                            , uint8_t& slotID) {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(m_macDSMEenabled);

    NS_LOG_DEBUG("Prepare DSME INFO Reply Command and Add it to Queue");  // debug

    LrWpanMacHeader receivedMacHdr;
    rxDsmeInfoReqPkt->RemoveHeader(receivedMacHdr);
    
    CommandPayloadHeader receivedMacPayload;
    rxDsmeInfoReqPkt->RemoveHeader(receivedMacPayload);

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();
    macHdr.SetAckReq();

    if (receivedMacHdr.GetDstAddrMode() == SHORT_ADDR) {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());
    } else if (receivedMacHdr.GetDstAddrMode() == EXT_ADDR) {
        macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetSrcAddrFields(GetPanId(), GetExtendedAddress());
    }

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), receivedMacHdr.GetShortSrcAddr());

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_INFO_REPLY);

    macPayload.SetDsmeInfoType(receivedMacPayload.GetDsmeInfoType());

    if (receivedMacPayload.GetDsmeInfoType() == MLMEDSMEINFO_TIMESTAMP) {
        // DSME-TODO
        macPayload.SetDsmeInfoTimestamp(Simulator::Now().ToInteger(Time::NS));
        macPayload.SetDsmeInfoSuperframeID(superframeID);
        macPayload.SetDsmeInfoSlotID(slotID);

    } else if (receivedMacPayload.GetDsmeInfoType() == MLMEDSMEINFO_DSME_SAB_SPECIFICATION) {
        // DSME-TODO
        DSMESABSpecificationField sAB;
        sAB.setCAPReduction(true);
        sAB.setSABSubBlkLen(receivedMacPayload.GetDsmeInfoSABSubBlkLen());
        sAB.setSABSubBlkIdx(receivedMacPayload.GetDsmeInfoSABSubBlkIdx());

        for (int i = 0; i < receivedMacPayload.GetDsmeInfoSABSubBlkLen(); ++i) {
            sAB.setSABSubBlk(m_macDSMESAB[receivedMacPayload.GetDsmeInfoSABSubBlkIdx() + i]);  
        }

        macPayload.SetDsmeGtsSABSpec(sAB);

    } else if (receivedMacPayload.GetDsmeInfoType() == MLMEDSMEINFO_DSME_PAN_DESCRIPTOR) {
        macPayload.SetDsmeInfoPANDescriptor(m_dsmePanDescriptorIE);
    }

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    if (receivedMacPayload.GetDsmeInfoType() == MLMEDSMEINFO_TIMESTAMP) {
        m_txPktGts = commandPacket;
    } else {
        Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
        txQElement->txQPkt = commandPacket;
        EnqueueTxQElement(txQElement);
    }
}

void LrWpanMac::SendFastAssocRequestCommand() {
    NS_LOG_FUNCTION(this);

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;

    // DSME-TODO
}

void
LrWpanMac::SendAssocRequestCommand()
{
    NS_LOG_FUNCTION(this);

    // NS_LOG_DEBUG("Send Association Request Command");  // debug

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    // Assoc. Req. Comm. Mac header values See IEEE 802.15.4-2011 (Section 5.3.1.1)
    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetSrcAddrFields(0xffff, GetExtendedAddress());

    if (m_associateParams.m_coordAddrMode == SHORT_ADDR)
    {
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetDstAddrFields(m_associateParams.m_coordPanId, m_associateParams.m_coordShortAddr);
    }
    else
    {
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetDstAddrFields(m_associateParams.m_coordPanId, m_associateParams.m_coordExtAddr);
    }

    macHdr.SetSecDisable();
    macHdr.SetAckReq();

    CommandPayloadHeader macPayload(CommandPayloadHeader::ASSOCIATION_REQ);
    macPayload.SetCapabilityField(m_associateParams.m_capabilityInfo);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled())
    {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendDsmeAssocRequestCommand() {
    NS_LOG_FUNCTION(this);

    // NS_LOG_DEBUG("Send Dsme Association Request Command");  // debug

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();            
    macHdr.SetAckReq();
    
    // Dsme Assoc. Req. Comm. Mac header values See IEEE 802.15.4e-2012 (Section 5.3.11.2)
    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetSrcAddrFields(0xffff, GetExtendedAddress());  // broadcast

    if (m_associateParams.m_coordAddrMode == SHORT_ADDR) {
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetDstAddrFields(m_associateParams.m_coordPanId, m_associateParams.m_coordShortAddr);
    } else {
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetDstAddrFields(m_associateParams.m_coordPanId, m_associateParams.m_coordExtAddr);
    }

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_ASSOCIATION_REQ);
    macPayload.SetCapabilityField(m_associateParams.m_capabilityInfo);

    macPayload.SetHoppingSeqID(m_associateParams.m_hoppingSeqID);
    macPayload.SetChannelOfs(m_associateParams.m_channelOfs);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendDataRequestCommand() {
    // See IEEE 802.15.4-2011 (Section 5.3.5)
    // This command can be sent for 3 different situations:
    // a) In response to a beacon indicating that there is data for the device.
    // b) Triggered by MLME-POLL.request.
    // c) To follow an ACK of an Association Request command and continue the associate process.

    // TODO: Implementation of a) and b) will be done when Indirect transmissions are fully
    // supported.
    //       for now, only case c) is considered.

    NS_LOG_FUNCTION(this);

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    // Mac Header values (Section 5.3.5)
    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetSrcAddrFields(0xffff, m_selfExt);

    if (m_macCoordShortAddress == Mac16Address("ff:fe"))
    {
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        macHdr.SetDstAddrFields(m_macPanId, m_macCoordExtendedAddress);
    }
    else
    {
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        macHdr.SetDstAddrFields(m_macPanId, m_macCoordShortAddress);
    }

    macHdr.SetSecDisable();
    macHdr.SetAckReq();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DATA_REQ);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled())
    {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    // Set the Command packet to be transmitted
    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void
LrWpanMac::SendAssocResponseCommand(Ptr<Packet> rxDataReqPkt)
{
    LrWpanMacHeader receivedMacHdr;
    rxDataReqPkt->RemoveHeader(receivedMacHdr);
    CommandPayloadHeader receivedMacPayload;
    rxDataReqPkt->RemoveHeader(receivedMacPayload);

    NS_ASSERT(receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DATA_REQ);

    Ptr<IndTxQueueElement> indTxQElement = Create<IndTxQueueElement>();
    bool elementFound;
    elementFound = DequeueInd(receivedMacHdr.GetExtSrcAddr(), indTxQElement);

    if (elementFound) {
        Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
        txQElement->txQPkt = indTxQElement->txQPkt;
        m_txQueue.emplace_back(txQElement);
    } else {
        NS_LOG_DEBUG("Requested element not found in pending list");
    }
}

void LrWpanMac::SendDsmeAssocResponseCommand(Ptr<Packet> rxDataReqPkt) {
    LrWpanMacHeader receivedMacHdr;
    rxDataReqPkt->RemoveHeader(receivedMacHdr);
    CommandPayloadHeader receivedMacPayload;
    rxDataReqPkt->RemoveHeader(receivedMacPayload);

    NS_ASSERT(receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DATA_REQ);

    Ptr<IndTxQueueElement> indTxQElement = Create<IndTxQueueElement>();
    bool elementFound;
    elementFound = DequeueInd(receivedMacHdr.GetExtSrcAddr(), indTxQElement);

    if (elementFound) {
        Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
        txQElement->txQPkt = indTxQElement->txQPkt;
        m_txQueue.emplace_back(txQElement);
    } else {
        NS_LOG_DEBUG("Requested element not found in pending list");
    }
}

void LrWpanMac::SendDsmeBeaconAllocNotifyCommand() {
    NS_LOG_FUNCTION(this);

    NS_LOG_DEBUG("Send Dsme Beacon Allocation Notification Command");  // debug

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();            
    macHdr.SetNoAckReq();
    macHdr.SetPanIdComp();
    macHdr.SetSecDisable();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);

    macHdr.SetSrcAddrFields(0xffff, GetShortAddress());
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff"));

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_BEACON_ALLOC_NOTIF);

    // DSME-TODO
    macPayload.SetAllocationBcnSDIndex(m_choosedSDIndexToSendBcn);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);
    
    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendDsmeGtsNotifyCommand(Mac16Address srcAddr, CommandPayloadHeader rxDsmeGtsReplyPayload) {
    NS_LOG_FUNCTION(this);

    NS_ASSERT(GetShortAddress() != Mac16Address("ff:ff"));
    NS_ASSERT(GetShortAddress() != Mac16Address("ff:fe"));

    // uint8_t replyStatus = rxDsmeGtsReplyPayload.GetDsmeGtsManagementField().GetStatus();

    NS_LOG_DEBUG("Prepare DSME GTS Notify Command and Add it to Queue");

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetNoFrmPend();            
    macHdr.SetAckReq();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetSrcAddrFields(GetPanId(), GetShortAddress());

    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
    macHdr.SetDstAddrFields(GetPanId(), Mac16Address("ff:ff"));

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload(CommandPayloadHeader::DSME_GTS_NOTIFY);

    DSMEGTSManagementField managementField;

    switch (m_dsmeGtsReqParams.m_manageType) {
        case GTS_DEALLOCATION:
            managementField.SetManagementType(0b000);
            break;
        
        case GTS_ALLOCATION:
            managementField.SetManagementType(0b001);
            managementField.SetRX(m_dsmeGtsReqParams.m_direction);
            managementField.SetPrioritizedChannelAccess(m_dsmeGtsReqParams.m_prioritizedChAccess);
            // managementField.SetStatus(replyStatus);
            managementField.SetStatus(0b000);
            
            break;

        case GTS_DUPLICATED_ALLOCATION_NOTIF:

            break;

        case GTS_REDUCE:

            break;

        case GTS_RESTART:

            break;

        default:
            break;
    }

    uint16_t channelOfs = rxDsmeGtsReplyPayload.GetChannelOfs();
    DSMESABSpecificationField sABSpec = rxDsmeGtsReplyPayload.GetDsmeGtsSABSpec();

    macPayload.SetDsmeGtsManagementField(managementField);

    macPayload.SetChannelOfs(channelOfs);
    macPayload.SetDsmeGtsDestAddress(srcAddr);
    macPayload.SetDsmeGtsSABSpec(sABSpec);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::SendOrphanNotificationCmd() {
    NS_LOG_FUNCTION(this);

    // NS_ASSERT(m_macCoordExtendedAddress != Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed")
    //          || m_macCoordShortAddress != Mac16Address("FF:FF"));

    NS_LOG_DEBUG("Prepare Orphan Notification Command and Add it to Queue");

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);

    macHdr.SetSrcAddrFields(0xFFFF, GetExtendedAddress());
    macHdr.SetDstAddrFields(0xFFFF, Mac16Address("FF:FF"));

    macHdr.SetNoFrmPend();
    macHdr.SetNoAckReq();
    macHdr.SetPanIdComp();

    macHdr.SetSecDisable();

    CommandPayloadHeader macPayload;
    macPayload.SetCommandFrameType(CommandPayloadHeader::ORPHAN_NOTIF);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;
    EnqueueTxQElement(txQElement);
    CheckQueue();
}

void LrWpanMac::MlmeDisassociateRequest(MlmeDisassociateRequestParams params) {
    // See IEEE 802.15.4-2011 , Section 6.2.3.1

    NS_LOG_FUNCTION(this);

    NS_LOG_DEBUG("Send Disassociation Request Command");  // debug

    m_pendPrimitive = MLME_DISASSOCIATE_REQ;
    m_disassociateParams = params;
    bool invalidRequest = false;

    if (params.m_devPanId == 0xffff) {
        invalidRequest = true;
    }

    if (params.m_devPanId != m_macPanId) {
        invalidRequest = true;
    }

    if (!invalidRequest && params.m_devAddrMode == SHORT_ADDR) {
        if (params.m_shortDevAddr == Mac16Address("ff:ff") ||
            params.m_shortDevAddr == Mac16Address("ff:fe")) {
            invalidRequest = true;
        }

    } else if (!invalidRequest && params.m_devAddrMode == EXT_ADDR) {
        if (params.m_extDevAddr == Mac64Address("ff:ff:ff:ff:ff:ff:ff:ff") ||
            params.m_extDevAddr == Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed")) {
            invalidRequest = true;
        }
    }

    if (invalidRequest) {
        m_pendPrimitive = MLME_NONE;
        m_disassociateParams = MlmeDisassociateRequestParams();
        NS_LOG_ERROR(this << " Invalid PAN id in Disassociation request");

        if (!m_mlmeDisassociateConfirmCallback.IsNull()) {
            MlmeDisassociateConfirmParams confirmParams;
            confirmParams.m_shortDevAddr = Mac16Address("FF:FF");
            confirmParams.m_status = DISASSOCIATE_INVALID_PARAMETER;
            m_mlmeDisassociateConfirmCallback(confirmParams);
        }
        
    } else {
        EndDisassociateRequest();
    }
}

void LrWpanMac::EndDisassociateRequest() {
    m_pendPrimitive = MLME_NONE;
    SendDisassocNotificationCommand();
}

void LrWpanMac::SendDisassocNotificationCommand() {
    // See IEEE 802.15.4-2011 Section 5.3.3 and Section 6.2.3.1

    NS_LOG_FUNCTION(this);

    // NS_LOG_DEBUG("Send Disassociation Notidication Command");  // debug

    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_COMMAND, m_macDsn.GetValue());
    m_macDsn++;
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> commandPacket = Create<Packet>();

    macHdr.SetSecDisable();
    macHdr.SetNoFrmPend();            
    macHdr.SetAckReq();
    macHdr.SetPanIdComp();
    
    // Disassoc. Req. Comm. Mac header values See IEEE 802.15.4-2011 (Section 5.3.3.1)
    macHdr.SetSrcAddrMode(LrWpanMacHeader::EXTADDR);
    macHdr.SetSrcAddrFields(0xffff, GetExtendedAddress());

    if (m_disassociateParams.m_devAddrMode == SHORT_ADDR) {
        macHdr.SetDstAddrMode(LrWpanMacHeader::SHORTADDR);
        // macHdr.SetDstAddrFields(m_disassociateParams.m_devPanId, m_disassociateParams.m_shortDevAddr);
        macHdr.SetDstAddrFields(GetPanId(), m_disassociateParams.m_shortDevAddr);

    } else {
        macHdr.SetDstAddrMode(LrWpanMacHeader::EXTADDR);
        // macHdr.SetDstAddrFields(m_disassociateParams.m_devPanId, m_disassociateParams.m_extDevAddr);
        macHdr.SetDstAddrFields(GetPanId(), m_disassociateParams.m_extDevAddr);
    }
    
    CommandPayloadHeader macPayload(CommandPayloadHeader::DISASSOCIATION_NOTIF);

    macPayload.SetDisassociationReason(m_disassociateParams.m_disassociateReason);

    commandPacket->AddHeader(macPayload);
    commandPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(commandPacket);
    }

    commandPacket->AddTrailer(macTrailer);

    Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
    txQElement->txQPkt = commandPacket;

    // the command is sent by a device wish to leave a PAN, m_txIndirect is ignored
    if ((m_disassociateParams.m_devAddrMode == SHORT_ADDR
        && m_disassociateParams.m_shortDevAddr == GetCoordShortAddress())
        || (m_disassociateParams.m_devAddrMode == EXT_ADDR
            && m_disassociateParams.m_extDevAddr == GetCoordExtAddress())) {

        EnqueueTxQElement(txQElement);
        CheckQueue();

        return;
    }

    // the command is sent by a coordinator that wish a device to leave the PAN
    // , m_txIndirect is considered
    if (!m_disassociateParams.m_txIndirect) {
        EnqueueTxQElement(txQElement);
        CheckQueue();

    } else {             
        EnqueueInd(commandPacket);
    }
}

void LrWpanMac::SendDisassocNotificationCommandIndirect(Ptr<Packet> rxDataReqPkt) {
    LrWpanMacHeader receivedMacHdr;
    rxDataReqPkt->RemoveHeader(receivedMacHdr);
    CommandPayloadHeader receivedMacPayload;
    rxDataReqPkt->RemoveHeader(receivedMacPayload);

    // NS_LOG_DEBUG("SendDisassocNotificationCommandIndirect()"); // debug

    NS_ASSERT(receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DATA_REQ);

    Ptr<IndTxQueueElement> indTxQElement = Create<IndTxQueueElement>();
    bool elementFound;
    elementFound = DequeueInd(receivedMacHdr.GetExtSrcAddr(), indTxQElement);

    if (elementFound) {
        Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
        txQElement->txQPkt = indTxQElement->txQPkt;
        m_txQueue.emplace_back(txQElement);
    } else {
        NS_LOG_DEBUG("Requested element not found in pending list");
    }
}

void LrWpanMac::RemoveReferencesToPAN() {
    m_macPanId = 0xffff;
    m_shortAddress = Mac16Address("00:00");
    m_macCoordShortAddress = Mac16Address("FF:FF");
    m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");

    m_incCapEvent.Cancel();
    m_incCfpEvent.Cancel();
    m_beaconEvent.Cancel();

    NS_LOG_DEBUG("Successfully disassociated from PAN"); // debug
}

void LrWpanMac::CheckDsmeGtsSABFromReplyCmd(DSMESABSpecificationField sAB) {
    NS_LOG_FUNCTION(this);

    m_gtsSuperframeIDs.push_back(sAB.GetSABSubBlkIdx());

    int start = -1;
    int len = 0;
    
    if (sAB.isCAPReduction()) {
        std::vector<uint16_t> subBlk = sAB.GetSABSubBlk();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 15; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            m_gtsStartAndLens.push_back({start, len});
            start = -1;
            len = 0;
        }

    } else {
        std::vector<uint8_t> subBlk = sAB.GetSABSubBlkCapOff();
        
        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 7; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            m_gtsStartAndLens.push_back({start, len});
            start = -1;
            len = 0;
        }
    }  
}

bool LrWpanMac::CheckDsmeGtsSABAndDsmeACTConflict(DSMESABSpecificationField sAB) {
    int start = -1;
    int len = 0;

    if (sAB.isCAPReduction()) {
        std::vector<uint16_t> subBlk = sAB.GetSABSubBlk();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 15; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            for (unsigned int k = 0; k < m_macDsmeACT[sAB.GetSABSubBlkIdx()].size(); ++k) {
                if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID <= start) {
                    if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID + m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_numSlot
                        >= start) {
                        return true;
                    }

                } else {
                    if (start + len >= m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID) {
                        return true;
                    }
                }
            }

            start = -1;
            len = 0;
        }

    } else {
        std::vector<uint8_t> subBlk = sAB.GetSABSubBlkCapOff();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 7; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            for (unsigned int k = 0; k < m_macDsmeACT[sAB.GetSABSubBlkIdx()].size(); ++k) {
                if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID <= start) {
                    if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID + m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_numSlot
                        >= start) {
                        return true;
                    }

                } else {
                    if (start + len >= m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID) {
                        return true;
                    }
                }
            }

            start = -1;
            len = 0;
        }
    }

    return false;
}

bool LrWpanMac::CheckDsmeGtsSABFromReqCmdWithDsmeACT(DSMESABSpecificationField sAB) {
    int start = -1;
    int len = 0;

    if (sAB.isCAPReduction()) {
        std::vector<uint16_t> subBlk = sAB.GetSABSubBlk();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 15; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            for (unsigned int k = 0; k < m_macDsmeACT[sAB.GetSABSubBlkIdx()].size(); ++k) {
                if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID == start
                    && m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_numSlot == len) {
                    return true;
                }
            }

            start = -1;
            len = 0;
        }

    } else {
        std::vector<uint8_t> subBlk = sAB.GetSABSubBlkCapOff();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 7; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            for (unsigned int k = 0; k < m_macDsmeACT[sAB.GetSABSubBlkIdx()].size(); ++k) {
                if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID == start
                    && m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_numSlot == len) {
                    return true;
                }
            }

            start = -1;
            len = 0;
        }
    }

    return false;
}

void LrWpanMac::UpdateDsmeACTAndDeallocate(DSMESABSpecificationField sAB) {
    NS_LOG_DEBUG("Update Dsme ACT and deallocate GTS");

    int start = -1;
    int len = 0;

    if (sAB.isCAPReduction()) {
        std::vector<uint16_t> subBlk = sAB.GetSABSubBlk();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 15; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            for (unsigned int k = 0; k < m_macDsmeACT[sAB.GetSABSubBlkIdx()].size(); ++k) {
                if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID == start
                    && m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_numSlot == len) {
                    m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_deallocated = true;
                }
            }

            for (unsigned int k = 0; k < m_gtsStartAndLens.size(); ++k) {
                if (std::get<0>(m_gtsStartAndLens[k]) == start
                    && std::get<1>(m_gtsStartAndLens[k]) == len
                    && m_gtsSuperframeIDs[k] == sAB.GetSABSubBlkIdx()) {
                    m_gtsStartAndLens.erase(m_gtsStartAndLens.begin() + k);
                    m_gtsSuperframeIDs.erase(m_gtsSuperframeIDs.begin() + k);
                    m_gtsDirections.erase(m_gtsDirections.begin() + k);
                }
            }

            start = -1;
            len = 0;
        }

    } else {
        std::vector<uint8_t> subBlk = sAB.GetSABSubBlkCapOff();

        for (unsigned int i = 0; i < subBlk.size(); ++i) {
            for (int j = 0; j < 7; ++j) {
                len += subBlk[i] & 1;

                if ((len) && (start == -1)) {
                    start = j;
                }

                subBlk[i] >>= 1;
            }

            for (unsigned int k = 0; k < m_macDsmeACT[sAB.GetSABSubBlkIdx()].size(); ++k) {
                if (m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_slotID == start
                    && m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_numSlot == len) {
                    m_macDsmeACT[sAB.GetSABSubBlkIdx()][k].m_deallocated = true;
                }
            }

            for (unsigned int k = 0; k < m_gtsStartAndLens.size(); ++k) {
                if (std::get<0>(m_gtsStartAndLens[k]) == start
                    && std::get<1>(m_gtsStartAndLens[k]) == len
                    && m_gtsSuperframeIDs[k] == sAB.GetSABSubBlkIdx()) {
                    m_gtsStartAndLens.erase(m_gtsStartAndLens.begin() + k);
                    m_gtsSuperframeIDs.erase(m_gtsSuperframeIDs.begin() + k);
                    m_gtsDirections.erase(m_gtsDirections.begin() + k);
                }
            }

            start = -1;
            len = 0;
        }
    }
}

bool LrWpanMac::SearchDsmeACTForAllocatedGTS(Mac16Address addr, uint16_t& superframeID
                                            , uint8_t& slotID) {
    if (m_macDsmeACT.size()) {
        for (auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it) {
            for (unsigned int i = 0; i < it->second.size(); ++i) {
                if (!it->second[i].m_deallocated) {
                    if (it->second[i].m_direction) {
                        if (it->second[i].m_srcAddr == addr) {
                            superframeID = it->second[i].m_superframeID;
                            slotID = it->second[i].m_slotID;
                            return true;
                        }

                    } else {
                        if (it->second[i].m_dstAddr == addr) {
                            superframeID = it->second[i].m_superframeID;
                            slotID = it->second[i].m_slotID;
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}

void
LrWpanMac::LostAssocRespCommand()
{
    // Association response command was not received, return to default values.
    m_macPanId = 0xffff;
    m_macCoordShortAddress = Mac16Address("FF:FF");
    m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");

    // NS_LOG_DEBUG("LostAssocRespCommand()"); // debug

    if (!m_mlmeAssociateConfirmCallback.IsNull())
    {
        MlmeAssociateConfirmParams confirmParams;
        confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
        confirmParams.m_status = MLMEASSOC_NO_DATA;
        m_mlmeAssociateConfirmCallback(confirmParams);
    }
}

void
LrWpanMac::EndStartRequest()
{
    NS_LOG_FUNCTION(this);
    // The primitive is no longer pending (Channel & Page have been set)
    m_pendPrimitive = MLME_NONE;

    if (m_startParams.m_coorRealgn) // Coordinator Realignment
    {
        // TODO: Send realignment request command frame in CSMA/CA
        // NS_LOG_ERROR(this << " Coordinator realignment request not supported");

        // DSME-TODO
        // MlmeSyncLossIndicationCallback;
        /**
         * Send coordinator command using csma-ca
         */
        NS_LOG_DEBUG("MLME-START.request with Coordinator Realignment");

        SendCoordinatorRealignmentCmd(false, true, Mac64Address("ff:ff:ff:ff:ff:ff:ff:ff")
                                      , Mac16Address("ff:ff"));

        return;

    } else {
        if (m_startParams.m_panCoor) {
            m_coord = m_panCoor = true;
            m_macPanId = m_startParams.m_PanId;
        }

        NS_ASSERT(m_startParams.m_PanId != 0xffff);

        if (m_panCoor) {
            m_macBeaconOrder = m_startParams.m_bcnOrd;
        } else {
            m_macBeaconOrder = m_panDescriptorList[m_descIdxOfAssociatedPan].m_superframeSpec
                                                                            .GetBeaconOrder();
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

        } else {
            if (m_panCoor) {
                m_macSuperframeOrder = m_startParams.m_sfrmOrd;
                m_csmaCa->SetBatteryLifeExtension(m_startParams.m_battLifeExt);

            } else {
                m_macSuperframeOrder = 
                        m_panDescriptorList[m_descIdxOfAssociatedPan].m_superframeSpec.GetFrameOrder();

                m_csmaCa->SetBatteryLifeExtension(m_panDescriptorList[m_descIdxOfAssociatedPan]
                                                 .m_superframeSpec.IsBattLifeExt());
            }
            
            m_csmaCa->SetSlottedCsmaCa();

            // DSME-TODO
            // TODO: Calculate the real Final CAP slot (requires GTS implementation)
            //  FinalCapSlot = Superframe duration slots - CFP slots.
            //  In the current implementation the value of the final cap slot is equal to
            //  the total number of possible slots in the superframe (15).
            // m_fnlCapSlot = 15;
            m_fnlCapSlot = 8;

            m_beaconInterval =
                (static_cast<uint32_t>(1 << m_macBeaconOrder)) * aBaseSuperframeDuration;
            m_superframeDuration =
                (static_cast<uint32_t>(1 << m_macSuperframeOrder)) * aBaseSuperframeDuration;
                
            // DSME
            uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second
            Time bcnTime = Seconds((double)m_beaconInterval / symbolRate);
            Time superfmTime = Seconds((double)m_superframeDuration / symbolRate);

            NS_LOG_DEBUG("**********************************************************************************************");
            NS_LOG_DEBUG("                  Beacon Interval: " 
                        << m_beaconInterval << " symbols, " 
                        << bcnTime << " seconds");

            NS_LOG_DEBUG("                  Superframe duration: " 
                        << m_superframeDuration << " symbols, " 
                        << superfmTime << " seconds");
            
            if (m_macDSMEenabled) {
                // Dsme superframe specification 
                if (m_panCoor) {
                    m_macMultisuperframeOrder = m_startParams.m_dsmeSuperframeSpec.GetMultiSuperframeOrder();
                    m_macChannelDiversityMode = m_startParams.m_dsmeSuperframeSpec.GetChannelDiversityMode();
                    m_macGACKFlag = m_startParams.m_dsmeSuperframeSpec.GetGACKFlag();
                    m_macCAPReductionFlag = m_startParams.m_dsmeSuperframeSpec.GetCAPReductionFlag();
                    m_macDeferredBcnUsed = m_startParams.m_dsmeSuperframeSpec.GetDeferredBeaconFalg();

                    NS_LOG_DEBUG("                  Dsme Superframe Spec: " 
                                << m_startParams.m_dsmeSuperframeSpec);

                    // BeaconBitmap
                    m_macSDBitmap = m_startParams.m_bcnBitmap;

                    m_macSDindex = m_macSDBitmap.GetSDIndex();
                    
                    // Hopping Descriptor
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

                } else {
                    DsmeSuperFrameField dsmeSuperframeField = 
                        m_panDescriptorList[m_descIdxOfAssociatedPan].m_dsmeSuperframeSpec;

                    m_macMultisuperframeOrder = dsmeSuperframeField.GetMultiSuperframeOrder();
                    m_macChannelDiversityMode = dsmeSuperframeField.GetChannelDiversityMode();
                    m_macGACKFlag = dsmeSuperframeField.GetGACKFlag();
                    m_macCAPReductionFlag = dsmeSuperframeField.GetCAPReductionFlag();
                    m_macDeferredBcnUsed = dsmeSuperframeField.GetDeferredBeaconFalg();

                    NS_LOG_DEBUG("                  Dsme Superframe Spec: " 
                                << dsmeSuperframeField);

                    m_macSDBitmap = 
                        m_panDescriptorList[m_descIdxOfAssociatedPan].m_bcnBitmap;
                    
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

                // 增加 DSME PAN descriptro IE 進 Header IEs
                m_dsmePanDescriptorIE = DsmePANDescriptorIE();

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
                // GroupACK groupAckField;
                // m_dsmePanDescriptorIE.SetGroupACK(groupAckField);

                // multi-superframe duration
                m_multiSuperframeDuration = 
                    (static_cast<uint32_t>(1 << m_macMultisuperframeOrder)) * aBaseSuperframeDuration;;
                m_numOfMultisuperframes = static_cast<uint32_t>(1 << (m_macBeaconOrder - m_macMultisuperframeOrder));
                m_numOfSuperframes = static_cast<uint64_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder));

                Time multisuperfmTime = Seconds((double)m_multiSuperframeDuration / symbolRate);

                NS_LOG_DEBUG("                  Multisuperframe duration: " 
                            << m_multiSuperframeDuration << " symbols, " 
                            << multisuperfmTime << " seconds");
                            
                NS_LOG_DEBUG("                  Num of Multisuperframe in a beacon interval " 
                            << m_numOfMultisuperframes);

                NS_LOG_DEBUG("                  Num of Superframe in a beacon interval " 
                            << m_numOfSuperframes);
                
                NS_LOG_DEBUG("                  SD Bitmap " << m_macSDBitmap);
                NS_LOG_DEBUG("                  Channel Hopping " << channelHoppingField);

                m_scheduleGTSsEvent.resize(m_numOfSuperframes / m_numOfMultisuperframes);
            }  

            NS_LOG_DEBUG("**********************************************************************************************");       
            NS_LOG_DEBUG("");

            if (m_macCAPReductionFlag) {
                m_macDSMESAB.resize(static_cast<uint64_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder))
                                    , 0);
            } else {
                m_macDSMESABCapOff.resize(static_cast<uint64_t>(1 << (m_macBeaconOrder - m_macSuperframeOrder))
                                            , 0);
            }

            if (m_coord) {
                if (m_macDSMEenabled) {
                    if (m_panCoor) {
                        m_multisuperframeStartEvent = Simulator::ScheduleNow(&LrWpanMac::StartMultisuperframe, 
                                                                            this, 
                                                                            OUTGOING);

                        m_beaconEvent = Simulator::ScheduleNow(&LrWpanMac::SendOneEnhancedBeacon, this);
                        
                        PurgeDsmeACT();
                        ScheduleGts(false);

                    } else {
                        m_sendBcn = true;
                    }

                } else {
                    m_beaconEvent = Simulator::ScheduleNow(&LrWpanMac::SendOneBeacon, this);
                }

            } else {
                // TODO: change the beacon sending according to the startTime parameter (if not PAN
                // coordinator)
                // parameter startTime is ignored in a DSME-enabled PAN.
                // static EventId Schedule (Time const &delay, void (*f)(Us...), Ts&&... args);

                if (m_startParams.m_startTime == 0) {
                    m_beaconEvent = Simulator::ScheduleNow(&LrWpanMac::SendOneBeacon, this);                 

                } else {
                    if (m_beaconTrackingOn) {
                        m_beaconEvent = Simulator::Schedule(Time(10), &LrWpanMac::SendOneBeacon, this);
                    }
                }
            }
        }
    }
}

void LrWpanMac::EndChannelScan() {
    NS_LOG_FUNCTION(this);

    m_channelScanIndex++;

    bool channelFound = false;

    for (int i = m_channelScanIndex; i <= 26; i++)
    {
        if ((m_scanParams.m_scanChannels & (1 << m_channelScanIndex)) != 0)
        {
            channelFound = true;
            break;
        }
        m_channelScanIndex++;
    }

    if (channelFound) {
        // Switch to the next channel in the list and restart scan
        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = m_channelScanIndex;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);

    } else {
        // All scans on the channel list completed
        // Return to the MAC values previous to start of the first scan.
        m_macPanId = m_macPanIdScan;
        m_macPanIdScan = 0;

        // TODO: restart beacon transmissions that were active before the beginning of the scan
        // (i.e when a coordinator perform a scan and it was already transmitting beacons)

        // All channels scanned, report success
        MlmeScanConfirmParams confirmParams;
        confirmParams.m_status = MLMESCAN_SUCCESS;
        confirmParams.m_chPage = m_scanParams.m_chPage;
        confirmParams.m_scanType = m_scanParams.m_scanType;
        confirmParams.m_energyDetList = {};
        confirmParams.m_panDescList = m_panDescriptorList;

        if (!m_mlmeScanConfirmCallback.IsNull()) {
            m_mlmeScanConfirmCallback(confirmParams);
        }

        m_pendPrimitive = MLME_NONE;
        m_channelScanIndex = 0;
        m_scanParams = {};
    }
}

void LrWpanMac::EndOrphanScan() {
    NS_LOG_FUNCTION(this);

    m_channelScanIndex++;

    bool channelFound = false;

    for (int i = m_channelScanIndex; i <= 26; i++) {
        if ((m_scanParams.m_scanChannels & (1 << m_channelScanIndex)) != 0) {
            channelFound = true;
            break;
        }

        m_channelScanIndex++;
    }

    if (channelFound && !realignmentRecevied) {
        // switch to the next channel in the list and restart scan
        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = m_channelScanIndex;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);

    } else {
        // All channels scanned, report success
        MlmeScanConfirmParams confirmParams;
        confirmParams.m_status = MLMESCAN_SUCCESS;
        confirmParams.m_chPage = m_scanParams.m_chPage;
        confirmParams.m_scanType = m_scanParams.m_scanType;
        confirmParams.m_energyDetList = {};
        confirmParams.m_panDescList = m_panDescriptorList;

        if (!m_mlmeScanConfirmCallback.IsNull()) {
            m_mlmeScanConfirmCallback(confirmParams);
        }

        m_pendPrimitive = MLME_NONE;
        m_channelScanIndex = 0;
        m_scanParams = {};

        realignmentRecevied = false;
    }
}

void
LrWpanMac::EndChannelEnergyScan()
{
    NS_LOG_FUNCTION(this);
    // Add the results of channel energy scan to the detectList
    m_energyDetectList.push_back(m_maxEnergyLevel);
    m_maxEnergyLevel = 0;

    m_channelScanIndex++;

    bool channelFound = false;
    for (int i = m_channelScanIndex; i <= 26; i++)
    {
        if ((m_scanParams.m_scanChannels & (1 << m_channelScanIndex)) != 0)
        {
            channelFound = true;
            break;
        }
        m_channelScanIndex++;
    }

    if (channelFound)
    {
        // switch to the next channel in the list and restart scan
        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = m_channelScanIndex;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);
    }
    else
    {
        // Scan on all channels on the list completed
        // Return to the MAC values previous to start of the first scan.
        m_macPanId = m_macPanIdScan;
        m_macPanIdScan = 0;

        // TODO: restart beacon transmissions that were active before the beginning of the scan
        // (i.e when a coordinator perform a scan and it was already transmitting beacons)

        // All channels scanned, report success
        if (!m_mlmeScanConfirmCallback.IsNull())
        {
            MlmeScanConfirmParams confirmParams;
            confirmParams.m_status = MLMESCAN_SUCCESS;
            confirmParams.m_chPage = m_phy->GetCurrentPage();
            confirmParams.m_scanType = m_scanParams.m_scanType;
            confirmParams.m_energyDetList = m_energyDetectList;
            m_mlmeScanConfirmCallback(confirmParams);
        }
        m_pendPrimitive = MLME_NONE;
        m_channelScanIndex = 0;
        m_scanParams = {};
    }
}

void LrWpanMac::EndEnhancedBeaconScan() {
    // DSME-TODO
}

void LrWpanMac::StartCAP(SuperframeType superframeType) {
    uint32_t activeSlot;
    uint64_t capDuration;
    Time endCapTime;
    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if (superframeType == OUTGOING) {
        m_incSuperframe = false;

        m_outSuperframeStatus = CAP;
        activeSlot = m_superframeDuration / 16;
        capDuration = activeSlot * (m_fnlCapSlot + 1);
        endCapTime = Seconds((double)capDuration / symbolRate);
        // Obtain the end of the CAP by adjust the time it took to send the beacon
        // Old Sync method
        // endCapTime -= (Simulator::Now() - m_macBeaconTxTime);

        // New Sync method
        endCapTime -= (m_macBeaconTxTime - m_BeaconStartTxTime);

        NS_LOG_DEBUG("Outgoing superframe CAP duration " << (endCapTime.GetSeconds() * symbolRate)
                                                         << " symbols (" << endCapTime.As(Time::S)
                                                         << ")");
        NS_LOG_DEBUG("Active Slots duration " << activeSlot << " symbols");
    
        m_capEvent =
            Simulator::Schedule(endCapTime, &LrWpanMac::StartCFP, this, SuperframeType::OUTGOING);

    } else {
        m_incSuperframe = true;

        m_incSuperframeStatus = CAP;
        activeSlot = m_incomingSuperframeDuration / 16;
        capDuration = activeSlot * (m_incomingFnlCapSlot + 1);
        endCapTime = Seconds((double)capDuration / symbolRate);
        // Obtain the end of the CAP by adjust the time it took to receive the beacon
        endCapTime -= (Simulator::Now() - m_macBeaconRxTime);

        NS_LOG_DEBUG("Incoming superframe CAP duration " << (endCapTime.GetSeconds() * symbolRate)
                                                         << " symbols (" << endCapTime.As(Time::S)
                                                         << ")");
        NS_LOG_DEBUG("Active Slots duration " << activeSlot << " symbols");
        
        m_incCapEvent =
            Simulator::Schedule(endCapTime, &LrWpanMac::StartCFP, this, SuperframeType::INCOMING);

        // 我覺得作者這裡寫錯了
        // m_capEvent =
        //     Simulator::Schedule(endCapTime, &LrWpanMac::StartCFP, this, SuperframeType::INCOMING);
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

    if (superframeType == INCOMING) {
        activeSlot = m_incomingSuperframeDuration / 16;
        cfpDuration = activeSlot * (15 - m_incomingFnlCapSlot);
        endCfpTime = Seconds((double)cfpDuration / symbolRate);

        if (cfpDuration > 0) {
            m_incSuperframeStatus = CFP;
        }

        if (m_macDSMEenabled) {
            if (m_incomingFirstCFP) {
                
                // just replace CAP portion with CFP
                // cfpDuration = activeSlot * (m_fnlCapSlot + 1);
                cfpDuration = activeSlot * (m_incomingFnlCapSlot + 1);
                endCfpTime = Seconds((double)cfpDuration / symbolRate);
                endCfpTime -= (Simulator::Now() - m_macBeaconRxTime);

                NS_LOG_DEBUG("Incoming superframe first CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");

                m_incomingFirstCFP = false;
                m_incCfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartCFP,
                                        this,
                                        SuperframeType::INCOMING);
                
            } else {      
                NS_LOG_DEBUG("Incoming superframe CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");

                m_incCfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartRemainingPeriod,
                                        this,
                                        SuperframeType::INCOMING);            
            } 

            ScheduleGtsSyncToCoordDuringCfp(m_incSDindex);

        } else {
            NS_LOG_DEBUG("Incoming superframe CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");

            m_incCfpEvent = Simulator::Schedule(endCfpTime,
                                    &LrWpanMac::StartInactivePeriod,
                                    this,
                                    SuperframeType::INCOMING);
        }        

    } else {
        activeSlot = m_superframeDuration / 16;
        cfpDuration = activeSlot * (15 - m_fnlCapSlot);
        endCfpTime = Seconds((double)cfpDuration / symbolRate);

        if (cfpDuration > 0)
        {
            m_outSuperframeStatus = CFP;
        }

        // m_cfpEvent = Simulator::Schedule(endCfpTime,
        //                                  &LrWpanMac::StartInactivePeriod,
        //                                  this,
        //                                  SuperframeType::OUTGOING);

        if (m_macDSMEenabled) {
            if (m_firstCFP) {
                // just replace CAP portion with CFP
                cfpDuration = activeSlot * (m_fnlCapSlot + 1);
                endCfpTime = Seconds((double)cfpDuration / symbolRate);
                // Old Sync method
                // endCfpTime -= (Simulator::Now() - m_macBeaconTxTime);

                // New Sync method
                endCfpTime -= (m_macBeaconTxTime - m_BeaconStartTxTime);

                NS_LOG_DEBUG("Outgoing superframe first CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");
                
                m_firstCFP = false;
                m_cfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartCFP,
                                        this,
                                        SuperframeType::OUTGOING);

            } else {    
                NS_LOG_DEBUG("Outgoing superframe CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")"); 

                m_cfpEvent = Simulator::Schedule(endCfpTime,
                                        &LrWpanMac::StartRemainingPeriod,
                                        this,
                                        SuperframeType::OUTGOING);
            }

            ScheduleGtsSyncToCoordDuringCfp(m_choosedSDIndexToSendBcn);

        } else {
            NS_LOG_DEBUG("Incoming superframe CFP duration " << cfpDuration << " symbols ("
                                                         << endCfpTime.As(Time::S) << ")");
                                                         
            m_cfpEvent = Simulator::Schedule(endCfpTime,
                                    &LrWpanMac::StartInactivePeriod,
                                    this,
                                    SuperframeType::OUTGOING);
        }
    }
}

void LrWpanMac::ScheduleGtsSyncToCoord(uint16_t curSDIndex) {
    NS_LOG_DEBUG("Gts Scheduling");

    if (m_macDsmeACT.size()) {
        uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);

        for (auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it) {
            for (unsigned int i = 0; i < it->second.size(); ++i) {
                if (!it->second[i].m_allocated) {
                    uint32_t activeSlot;
                    uint64_t superframeDurations;
                    uint64_t capDuration;
                    uint64_t firstTimeslot;
                    uint64_t endCapUntilTheGtsDuration;
                    uint64_t endCfpUntilTheGtsDuration;

                    Time endCapTime;
                    Time endFirstTimeslotTime;
                    Time startGtsTime;
                    Time gtsDuration;
                    Time superframeTime;

                    bool twoCfp = false;

                    // DSME-TODO
                    // For Dsme net device setting
                    if (it->second[i].m_superframeID != curSDIndex) {
                        return;
                    }

                    if (m_coord && m_choosedSDIndexToSendBcn == curSDIndex) {
                        activeSlot = m_superframeDuration / 16;

                        // DSME-TODO
                        // For Dsme net device setting
                        superframeDurations = (it->second[i].m_superframeID - curSDIndex) 
                                                * m_superframeDuration;

                        if (isCAPReductionOn()
                            && curSDIndex % (m_multiSuperframeDuration / m_superframeDuration) != 0) {
                            firstTimeslot = activeSlot * 1;  // first timeslot is used for beacon tx
                            endFirstTimeslotTime = Seconds((double) firstTimeslot / symbolRate);
                            endCfpUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                            gtsDuration = Seconds((double)endCfpUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = endFirstTimeslotTime + gtsDuration + superframeTime;

                            twoCfp = true;

                        } else {
                            capDuration = activeSlot * (m_fnlCapSlot + 1);
                            endCapTime = Seconds((double) capDuration / symbolRate);
                            endCapUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                            gtsDuration = Seconds((double)endCapUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = endCapTime + gtsDuration + superframeTime;
                        }

                        // endCapTime -= (Simulator::Now() - m_macBeaconTxTime);
                        // superframeDurations = (it->second[i].m_superframeID) * m_superframeDuration;

                    } else {
                        activeSlot = m_incomingSuperframeDuration / 16;

                        // DSME-TODO
                        // For Dsme net device setting
                        superframeDurations = (it->second[i].m_superframeID - curSDIndex) 
                                                * m_incomingSuperframeDuration;

                        if (isCAPReductionOn()
                            && curSDIndex % (m_incomingMultisuperframeDuration / m_incomingSuperframeDuration) != 0) {
                            firstTimeslot = activeSlot * 1;  // first timeslot is used for beacon tx
                            endFirstTimeslotTime = Seconds((double) firstTimeslot / symbolRate);
                            endFirstTimeslotTime -= (Simulator::Now() - m_macBeaconRxTime);
                            endCfpUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                            gtsDuration = Seconds((double)endCfpUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = endFirstTimeslotTime + gtsDuration + superframeTime;

                            twoCfp = true;

                        } else {
                            capDuration = activeSlot * (m_incomingFnlCapSlot + 1);
                            endCapTime = Seconds((double)capDuration / symbolRate);
                            endCapTime -= (Simulator::Now() - m_macBeaconRxTime);
                            endCapUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;
                            
                            gtsDuration = Seconds((double)endCapUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = endCapTime + gtsDuration + superframeTime;
                        }

                        // superframeDurations = (it->second[i].m_superframeID) * m_incomingSuperframeDuration;
                    }
                    
                    // gtsDuration = Seconds((double)endCapUntilTheGtsDuration / symbolRate);
                    // superframeTime = Seconds((double)superframeDurations / symbolRate);
                    // startGtsTime = endCapTime + gtsDuration + superframeTime;

                    if (it->second[i].m_direction) {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::INCOMING
                                                        , it->second[i].m_superframeID
                                                        , i);
                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);
                        
                        if (twoCfp) {
                            NS_LOG_DEBUG("Schedule an Rx GTS that will launch at:" 
                                        << " endFirstTimeslotTime (" << endFirstTimeslotTime.As(Time::S) << ")"
                                        << " + "
                                        << " endCfpUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                        << " + "
                                        << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                        << " = "
                                        << "(" << startGtsTime.As(Time::S) << ")");

                        } else {
                            NS_LOG_DEBUG("Schedule an Rx GTS that will launch at:" 
                                        << " endCapTime (" << endCapTime.As(Time::S) << ")"
                                        << " + "
                                        << " endCapUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                        << " + "
                                        << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                        << " = "
                                        << "(" << startGtsTime.As(Time::S) << ")");
                        }

                    } else {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::OUTGOING
                                                        , it->second[i].m_superframeID
                                                        , i);
                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);

                        if (twoCfp) {
                            NS_LOG_DEBUG("Schedule an Tx GTS that will launch at:" 
                                        << " endFirstTimeslotTime (" << endFirstTimeslotTime.As(Time::S) << ")"
                                        << " + "
                                        << " endCfpUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                        << " + "
                                        << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                        << " = "
                                        << "(" << startGtsTime.As(Time::S) << ")");

                        } else {
                            NS_LOG_DEBUG("Schedule an Tx GTS that will launch at:" 
                                        << " endCapTime (" << endCapTime.As(Time::S) << ")"
                                        << " + "
                                        << " endCapUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                        << " + "
                                        << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                        << " = "
                                        << "(" << startGtsTime.As(Time::S) << ")");
                        }
                    }
                }
            }
        }
    }
}

void LrWpanMac::ScheduleGtsSyncToCoordDuringCfp(uint16_t curSDIndex) {
    NS_LOG_DEBUG("Gts Scheduling");

    if (m_macDsmeACT.size()) {
        uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);

        for (auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it) {
            for (unsigned int i = 0; i < it->second.size(); ++i) {
                if (!it->second[i].m_allocated) {
                    uint32_t activeSlot;
                    uint64_t superframeDurations;
                    uint64_t endCfpUntilTheGtsDuration;

                    Time startGtsTime;
                    Time gtsDuration;
                    Time superframeTime;

                    // DSME-TODO
                    // For Dsme net device setting
                    if (it->second[i].m_superframeID != curSDIndex) {
                        return;
                    }

                    if (m_coord && m_choosedSDIndexToSendBcn == curSDIndex) {
                        activeSlot = m_superframeDuration / 16;

                        // DSME-TODO
                        // For Dsme net device setting
                        superframeDurations = (it->second[i].m_superframeID - curSDIndex) 
                                                * m_superframeDuration;

                        if (isCAPReductionOn()
                            && curSDIndex % (m_multiSuperframeDuration / m_superframeDuration) != 0) {
                            endCfpUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                            gtsDuration = Seconds((double)endCfpUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = gtsDuration + superframeTime;

                        } else {
                            endCfpUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                            gtsDuration = Seconds((double)endCfpUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = gtsDuration + superframeTime;
                        }

                    } else {
                        activeSlot = m_incomingSuperframeDuration / 16;

                        // DSME-TODO
                        // For Dsme net device setting
                        superframeDurations = (it->second[i].m_superframeID - curSDIndex) 
                                                * m_incomingSuperframeDuration;

                        if (isCAPReductionOn()
                            && curSDIndex % (m_incomingMultisuperframeDuration / m_incomingSuperframeDuration) != 0) {
                            endCfpUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                            gtsDuration = Seconds((double)endCfpUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = gtsDuration + superframeTime;

                        } else {
                            endCfpUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;
                            
                            gtsDuration = Seconds((double)endCfpUntilTheGtsDuration / symbolRate);
                            superframeTime = Seconds((double)superframeDurations / symbolRate);
                            startGtsTime = gtsDuration + superframeTime;
                        }
                    }
                    
                    if (it->second[i].m_direction) {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::INCOMING
                                                        , it->second[i].m_superframeID
                                                        , i);
                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);
                        
                        NS_LOG_DEBUG("Schedule an Rx GTS that will launch at:" 
                                    << " endCfpUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                    << " + "
                                    << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                    << " = "
                                    << "(" << startGtsTime.As(Time::S) << ")");

                    } else {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::OUTGOING
                                                        , it->second[i].m_superframeID
                                                        , i);
                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);

                        NS_LOG_DEBUG("Schedule an Tx GTS that will launch at:" 
                                    << " endCfpUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                    << " + "
                                    << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                    << " = "
                                    << "(" << startGtsTime.As(Time::S) << ")");

                    }
                }
            }
        }
    }
}

void LrWpanMac::ScheduleGts(bool indication) {
    NS_LOG_DEBUG("Gts Scheduling");

    if (m_coord && indication) {
        return;
    }

    if (m_macDsmeACT.size()) {
        uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);

        for (auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it) {
            for (unsigned int i = 0; i < it->second.size(); ++i) {
                if (!it->second[i].m_allocated) {
                    uint32_t activeSlot;
                    uint64_t superframeDurations;
                    uint64_t capDuration;
                    uint64_t endCapUntilTheGtsDuration;

                    Time endCapTime;
                    Time startGtsTime;

                    if (m_coord) {
                        activeSlot = m_superframeDuration / 16;
                        capDuration = activeSlot * (m_fnlCapSlot + 1);
                        endCapTime = Seconds((double)capDuration / symbolRate);
                        // endCapTime -= (Simulator::Now() - m_macBeaconTxTime);
                        // superframeDurations = (it->second[i].m_superframeID) * m_superframeDuration;
                        superframeDurations = (it->second[i].m_superframeID - m_choosedSDIndexToSendBcn) 
                                                * m_superframeDuration;
                        endCapUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;

                    } else {
                        activeSlot = m_incomingSuperframeDuration / 16;
                        capDuration = activeSlot * (m_incomingFnlCapSlot + 1);
                        endCapTime = Seconds((double)capDuration / symbolRate);
                        endCapTime -= (Simulator::Now() - m_macBeaconRxTime);
                        superframeDurations = (it->second[i].m_superframeID) * m_incomingSuperframeDuration;
                        endCapUntilTheGtsDuration = activeSlot * it->second[i].m_slotID;
                    }
                    
                    Time gtsDuration = Seconds((double)endCapUntilTheGtsDuration / symbolRate);
                    Time superframeTime = Seconds((double)superframeDurations / symbolRate);
                    startGtsTime = endCapTime + gtsDuration + superframeTime;

                    if (it->second[i].m_direction) {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::INCOMING
                                                        , it->second[i].m_superframeID
                                                        , i);
                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);
                        
                        NS_LOG_DEBUG("Schedule an Rx GTS that will launch at:" 
                                    << " endCapTime (" << endCapTime.As(Time::S) << ")"
                                    << " + "
                                    << " endCapUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                    << " + "
                                    << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                    << " = "
                                    << "(" << startGtsTime.As(Time::S) << ")");

                    } else {
                        m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::OUTGOING
                                                        , it->second[i].m_superframeID
                                                        , i);
                        
                        // DSME-TODO
                        it->second[i].m_allocated = true;
                        m_scheduleGTSsEvent[it->second[i].m_superframeID].push_back(m_gtsSchedulingEvent);

                        NS_LOG_DEBUG("Schedule an Tx GTS that will launch at:" 
                                    << " endCapTime (" << endCapTime.As(Time::S) << ")"
                                    << " + "
                                    << " endCapUntilTheGtsDuration (" << gtsDuration.As(Time::S) << ")"
                                    << " + "
                                    << " superframeTime (" << superframeTime.As(Time::S) << ")"
                                    << " = "
                                    << "(" << startGtsTime.As(Time::S) << ")");
                    }
                }
            }
        }
    }
}

void LrWpanMac::PurgeDsmeACT() {
    NS_LOG_DEBUG("Dsme ACT Purging");

    // deallocate or expire
    if (m_macDsmeACT.size()) {
        for (auto it = m_macDsmeACT.begin(); it != m_macDsmeACT.end(); ++it) {
            for (unsigned int i = 0; i < it->second.size(); ++i) {
                if (it->second[i].m_deallocated) {
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

    if (m_macDsmeACT[superframeID][idx].m_direction) {  
        m_macDsmeACT[superframeID][idx].m_cnt++;
    }

    // GTS expiration
    if (m_macDsmeACT[superframeID][idx].m_cnt > m_macDSMEGTSExpirationTime) {
        m_macDsmeACT[superframeID][idx].m_expired = true;

        if (!m_mlmeDsmeGtsIndicationCallback.IsNull()) {
            MlmeDsmeGtsIndicationParams gtsIndicationParams;

            gtsIndicationParams.m_devAddr = m_macDsmeACT[superframeID][idx].m_srcAddr;
            gtsIndicationParams.m_manageType = ManagementType::GTS_EXPIRATION;
            gtsIndicationParams.m_direction = !m_macDsmeACT[superframeID][idx].m_direction;
            gtsIndicationParams.m_prioritizedChAccess = m_macDsmeACT[superframeID][idx].m_prioritizedChAccess;

            gtsIndicationParams.m_dsmeSABSpec.setCAPReduction(isCAPReductionOn());
            gtsIndicationParams.m_dsmeSABSpec.setSABSubBlkLen(1);
            gtsIndicationParams.m_dsmeSABSpec.setSABSubBlkIdx(m_macDsmeACT[superframeID][idx].m_superframeID);

            if (isCAPReductionOn()) {
                uint16_t subBlk = GenerateSABSubBlockCapOn(m_macDsmeACT[superframeID][idx].m_slotID
                                                         , m_macDsmeACT[superframeID][idx].m_numSlot);

                gtsIndicationParams.m_dsmeSABSpec.setSABSubBlk(subBlk);

            } else {
                uint8_t subBlk = GenerateSABSubBlock(m_macDsmeACT[superframeID][idx].m_slotID
                                                    , m_macDsmeACT[superframeID][idx].m_numSlot);

                gtsIndicationParams.m_dsmeSABSpec.setSABSubBlk(subBlk);
            }

            m_mlmeDsmeGtsIndicationCallback(gtsIndicationParams);
        }

        return;
    }

    if (m_macDsmeACT[superframeID][idx].m_allocated) {
        Time startGtsTime;

        uint64_t symbolRate;
        symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

        if (m_coord) {
            startGtsTime = Seconds((double) m_multiSuperframeDuration / symbolRate);
        } else {
            startGtsTime = Seconds((double) m_incomingMultisuperframeDuration / symbolRate);
        }

        if (m_macDsmeACT[superframeID][idx].m_direction) {
            m_gtsSchedulingEvent = Simulator::Schedule(startGtsTime
                                                        , &LrWpanMac::StartGTS
                                                        , this
                                                        , SuperframeType::INCOMING
                                                        , m_macDsmeACT[superframeID][idx].m_superframeID
                                                        , idx);
            
            NS_LOG_DEBUG("Schedule an Rx GTS that will launch at:" 
                        << "(" << startGtsTime.As(Time::S) << ")");

        } else {
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

    if (m_coord) {
        activeSlot = m_superframeDuration / 16;
    } else {
        activeSlot = m_incomingSuperframeDuration / 16;
    }

    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false);
    uint64_t gtsDuration = activeSlot * m_macDsmeACT[superframeID][idx].m_numSlot;

    // debug
    Time endGtsTime = Seconds((double)gtsDuration / symbolRate) - NanoSeconds(1);

    if (superframeType == OUTGOING) {
        NS_LOG_DEBUG("Outgoing Gts Tx duration " << gtsDuration << " symbols ("
                                                 << endGtsTime.As(Time::S) << ")");
    
        // turn on TX

        m_gtsEvent = Simulator::Schedule(endGtsTime
                                        , &LrWpanMac::EndGTS
                                        , this
                                        , SuperframeType::OUTGOING);

        NS_LOG_DEBUG("Channel Offset: " << m_macDsmeACT[superframeID][idx].m_channelID); // debug
        
        // channel hopping part
        // For Dsme net device setting
        uint16_t ch = (m_macDsmeACT[superframeID][idx].m_channelID + m_macDsmeACT[superframeID][idx].m_slotID) 
                        % m_numOfChannels;
        // uint16_t ch = (m_macChannelOfs + m_macDsmeACT[superframeID][idx].m_slotID) % m_numOfChannelSupported;
        // uint16_t ch = (m_macChannelOfs + m_macDsmeACT[superframeID][idx].m_slotID) % m_numOfChannels;

        ch += 11;

        NS_LOG_DEBUG("Hop to Channel Num: " << ch); // debug

        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = ch;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);                                
        
    } else {
        NS_LOG_DEBUG("Incoming Gts Rx duration " << gtsDuration << " symbols ("
                                                 << endGtsTime.As(Time::S) << ")");
        
        // turn on RX
        
        m_incGtsEvent = Simulator::Schedule(endGtsTime
                                        , &LrWpanMac::EndGTS
                                        , this
                                        , SuperframeType::INCOMING);

        NS_LOG_DEBUG("Channel Offset: " << m_macDsmeACT[superframeID][idx].m_channelID); // debug
        
        // channel hopping part
        // DSME-TODO
        // For Dsme net device setting
        uint16_t ch = (m_macDsmeACT[superframeID][idx].m_channelID + m_macDsmeACT[superframeID][idx].m_slotID) 
                        % m_numOfChannels;
        // uint16_t ch = (m_macChannelOfs + m_macDsmeACT[superframeID][idx].m_slotID) % m_numOfChannelSupported;
        // uint16_t ch = (m_macChannelOfs + m_macDsmeACT[superframeID][idx].m_slotID) % m_numOfChannels;


        ch += 11;

        NS_LOG_DEBUG("Hop to Channel Num: " << ch); // debug

        LrWpanPhyPibAttributes pibAttr;
        pibAttr.phyCurrentChannel = ch;
        m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel, &pibAttr);                        
    }

    SetLrWpanMacStateToGTS(superframeID, idx);

    if (m_gtsContinuePktSendingFromCap && !m_txQueue.empty()) {
        Ptr<TxQueueElement> txQElement = m_txQueue.front();
        m_txPkt = txQElement->txQPkt;
    }

    if (m_txPkt && m_gtsContinuePktSendingFromCap) {
        LrWpanMacHeader peekedMacHdr;
        m_txPkt->PeekHeader(peekedMacHdr);

        if (peekedMacHdr.IsData()) {
            peekedMacHdr.Print(std::cout);

            if (superframeType == OUTGOING) {
                if (m_macDsmeACT[superframeID][idx].m_dstAddr == peekedMacHdr.GetShortSrcAddr()) {
                    ChangeMacState(MAC_GTS_SENDING);
                    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
                }

            } else {
                if (m_macDsmeACT[superframeID][idx].m_srcAddr == peekedMacHdr.GetShortSrcAddr()) {
                    ChangeMacState(MAC_GTS_SENDING);
                    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
                }
            }
        }
    }

    // DSME-TODO
    if (m_txPktGts) {
        m_txPkt = m_txPktGts;
        ChangeMacState(MAC_GTS_SENDING);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
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

    // m_phy->CanclePdDataRequest();

    // m_gtsRetrieve = false;
}

void LrWpanMac::StartMultisuperframe(SuperframeType superframeType) {
    NS_LOG_FUNCTION(this);

    Time endMultisuperframeTime;
    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if (superframeType == OUTGOING) {
        endMultisuperframeTime = Seconds((double) m_multiSuperframeDuration / symbolRate);

        NS_LOG_DEBUG("Outgoing multisuperframe Active Portion (Beacon + CAP + CFP) + (Beacon + CAP + CFP)...: "
                    << m_multiSuperframeDuration << " symbols"
                    << "(" << endMultisuperframeTime.As(Time::S) << ")");
        
        m_multisuperframeEndEvent = Simulator::Schedule(endMultisuperframeTime, 
                                                        &LrWpanMac::StartMultisuperframe,
                                                        this,
                                                        SuperframeType::OUTGOING);
    } else {
        endMultisuperframeTime = Seconds((double) m_incomingMultisuperframeDuration / symbolRate);

        // substract the Beacon Rx Time slots
        endMultisuperframeTime -= (Simulator::Now() - m_macBeaconRxTime);

        NS_LOG_DEBUG("Incoming multisuperframe Active Portion (Beacon + CAP + CFP) + (Beacon + CAP + CFP)...: "
                    << m_incomingMultisuperframeDuration << " symbols"
                    << "(" << endMultisuperframeTime.As(Time::S) << ")");
    }
}

void
LrWpanMac::StartInactivePeriod(SuperframeType superframeType)
{
    uint64_t inactiveDuration;
    Time endInactiveTime;
    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if (superframeType == INCOMING)
    {
        inactiveDuration = m_incomingBeaconInterval - m_incomingSuperframeDuration;
        endInactiveTime = Seconds((double)inactiveDuration / symbolRate);

        if (inactiveDuration > 0)
        {
            m_incSuperframeStatus = INACTIVE;
        }

        NS_LOG_DEBUG("Incoming superframe Inactive Portion duration "
                     << inactiveDuration << " symbols (" << endInactiveTime.As(Time::S) << ")");
        m_beaconEvent = Simulator::Schedule(endInactiveTime, &LrWpanMac::AwaitBeacon, this);
    }
    else
    {
        inactiveDuration = m_beaconInterval - m_superframeDuration;
        endInactiveTime = Seconds((double)inactiveDuration / symbolRate);

        if (inactiveDuration > 0)
        {
            m_outSuperframeStatus = INACTIVE;
        }

        NS_LOG_DEBUG("Outgoing superframe Inactive Portion duration "
                     << inactiveDuration << " symbols (" << endInactiveTime.As(Time::S) << ")");
        m_beaconEvent = Simulator::Schedule(endInactiveTime, &LrWpanMac::SendOneBeacon, this);
    }
}

void LrWpanMac::StartRemainingPeriod(SuperframeType superframeType) {
    uint64_t remainingDurationUntilNextBcn;   // end
    Time endRemainingTime;
    uint64_t symbolRate;

    symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second

    if (superframeType == INCOMING) {
        remainingDurationUntilNextBcn = m_incomingBeaconInterval - m_incomingSuperframeDuration;
        endRemainingTime = Seconds((double)remainingDurationUntilNextBcn / symbolRate);

        if (remainingDurationUntilNextBcn > 0) {
            m_incSuperframeStatus = REMAINING;
        }

        NS_LOG_DEBUG("Incoming superframe Remaining Portion duration "
                     << remainingDurationUntilNextBcn << " symbols (" << endRemainingTime.As(Time::S) << ")");

        m_beaconEvent = Simulator::Schedule(endRemainingTime, &LrWpanMac::AwaitBeacon, this);

    } else {
        // remainingDurationUntilNextBcn = m_beaconInterval - m_superframeDuration;
        // endRemainingTime = Seconds((double)remainingDurationUntilNextBcn / symbolRate);
        
        // DSME-TODO
        endRemainingTime = Seconds((double)(m_beaconInterval) / symbolRate);
        endRemainingTime -= (Simulator::Now() - m_startOfBcnSlot);
        remainingDurationUntilNextBcn = endRemainingTime.ToInteger(Time::S) * symbolRate;

        // std::cout << Simulator::Now().As(Time::MS) << std::endl;
        // std::cout << m_startOfBcnSlot.As(Time::MS) << std::endl;
        // std::cout << endRemainingTime2.As(Time::MS) << std::endl;

        if (remainingDurationUntilNextBcn > 0) {
            m_outSuperframeStatus = REMAINING;
        }

        NS_LOG_DEBUG("Outgoing multisuperframe Remaining Portion duration "
                     << remainingDurationUntilNextBcn << " symbols (" << endRemainingTime.As(Time::S) << ")");
                                              
        m_beaconEvent = Simulator::Schedule(endRemainingTime, &LrWpanMac::SendOneEnhancedBeacon, this);

        Simulator::Schedule(endRemainingTime, &LrWpanMac::PurgeDsmeACT, this);

        // DSME-TODO
        // For Dsme net device setting use only 
        // Simulator::Schedule(endRemainingTime, &LrWpanMac::ScheduleGtsSyncToCoord, this, m_choosedSDIndexToSendBcn);
        
        // Simulator::Schedule(endRemainingTime, &LrWpanMac::ScheduleGts, this, false);
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
    if (m_lrWpanMacState == MAC_IDLE && !m_txQueue.empty() && !m_setMacState.IsRunning()) {
        // TODO: this should check if the node is a coordinator and using the outcoming superframe
        // not just the PAN coordinator
        if (m_csmaCa->IsUnSlottedCsmaCa() || (m_outSuperframeStatus == CAP && m_coord) ||
            m_incSuperframeStatus == CAP) {

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

SuperframeField
LrWpanMac::GetSuperframeField()
{
    SuperframeField sfrmSpec;

    sfrmSpec.SetBeaconOrder(m_macBeaconOrder);
    sfrmSpec.SetSuperframeOrder(m_macSuperframeOrder);
    sfrmSpec.SetFinalCapSlot(m_fnlCapSlot);

    if (m_csmaCa->GetBatteryLifeExtension())
    {
        sfrmSpec.SetBattLifeExt(true);
    }

    if (m_panCoor)
    {
        sfrmSpec.SetPanCoor(true);
    }

    // used to associate devices via Beacons
    if (m_macAssociationPermit)
    {
        sfrmSpec.SetAssocPermit(true);
    }

    return sfrmSpec;
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

        } else if (peekedMacHdr.GetDstAddrMode() == EXT_ADDR) {
            numOfExtAddr++;
            pndAddrFields.AddAddress(peekedMacHdr.GetExtDstAddr());
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

    m_scheduleGTSsEvent.resize(m_numOfSuperframes / m_numOfMultisuperframes);
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
    if (superframeID >= (m_numOfSuperframes / m_numOfMultisuperframes)) {
        NS_FATAL_ERROR(this << " the superframe ID: " << superframeID 
                          << " is larger than the number of superframe in: " << (m_numOfSuperframes / m_numOfMultisuperframes));
    }

    if (m_macCAPReductionFlag) {
        if (entity.m_slotID >= 15) {
            NS_FATAL_ERROR(this << " the slot ID: " << entity.m_slotID
                        << " is larger than the number of CFP timeslots: 15");
        }

    } else {
        if (entity.m_slotID >= 7) {
            NS_FATAL_ERROR(this << " the slot ID: " << entity.m_slotID
                        << " is larger than the number of CFP timeslots: 7");
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
    NS_ASSERT(m_lrWpanMacState == MAC_IDLE || m_lrWpanMacState == MAC_ACK_PENDING ||
              m_lrWpanMacState == MAC_CSMA || m_lrWpanMacState == MAC_GTS);
    NS_LOG_FUNCTION(this << psduLength << p << (uint16_t)lqi);

    bool acceptFrame;

    // std::cout << "PdDataIndication" << std::endl;

    // NS_LOG_DEBUG("PdDataIndication");

    // from sec 7.5.6.2 Reception and rejection, Std 802.15.4-2006
    // level 1 filtering, test FCS field and reject if frame fails
    // level 2 filtering if promiscuous mode pass frame to higher layer otherwise perform level 3
    // filtering level 3 filtering accept frame if Frame type and version is not reserved, and if
    // there is a dstPanId then dstPanId=m_macPanId or broadcastPanId, and if there is a
    // shortDstAddr then shortDstAddr =shortMacAddr or broadcastAddr, and if beacon frame then
    // srcPanId = m_macPanId if only srcAddr field in Data or Command frame,accept frame if
    // srcPanId=m_macPanId

    Ptr<Packet> originalPkt = p->Copy(); // because we will strip headers
    uint64_t symbolRate = (uint64_t)m_phy->GetDataOrSymbolRate(false); // symbols per second
    m_promiscSnifferTrace(originalPkt);

    m_macPromiscRxTrace(originalPkt);
    // XXX no rejection tracing (to macRxDropTrace) being performed below

    LrWpanMacTrailer receivedMacTrailer;
    p->RemoveTrailer(receivedMacTrailer);

    if (Node::ChecksumEnabled()) {
        receivedMacTrailer.EnableFcs(true);
    }

    // level 1 filtering
    if (!receivedMacTrailer.CheckFcs(p)) {
        m_macRxDropTrace(originalPkt);

    } else {
        LrWpanMacHeader receivedMacHdr;
        p->RemoveHeader(receivedMacHdr);

        McpsDataIndicationParams params;
        params.m_dsn = receivedMacHdr.GetSeqNum();
        params.m_mpduLinkQuality = lqi;
        params.m_srcPanId = receivedMacHdr.GetSrcPanId();
        params.m_srcAddrMode = receivedMacHdr.GetSrcAddrMode();

        switch (params.m_srcAddrMode) {
            case SHORT_ADDR:
                params.m_srcAddr = receivedMacHdr.GetShortSrcAddr();
                break;
            case EXT_ADDR:
                params.m_srcExtAddr = receivedMacHdr.GetExtSrcAddr();
                break;
            default:
                break;
        }

        params.m_dstPanId = receivedMacHdr.GetDstPanId();
        params.m_dstAddrMode = receivedMacHdr.GetDstAddrMode();

        switch (params.m_dstAddrMode) {
            case SHORT_ADDR:
                params.m_dstAddr = receivedMacHdr.GetShortDstAddr();
                break;
            case EXT_ADDR:
                params.m_dstExtAddr = receivedMacHdr.GetExtDstAddr();
                break;
            default:
                break;
        }

        if (m_macPromiscuousMode) {
            // level 2 filtering
            if (receivedMacHdr.GetDstAddrMode() == SHORT_ADDR)
            {
                NS_LOG_DEBUG("Packet from " << params.m_srcAddr);
                NS_LOG_DEBUG("Packet to " << params.m_dstAddr);
            }
            else if (receivedMacHdr.GetDstAddrMode() == EXT_ADDR)
            {
                NS_LOG_DEBUG("Packet from " << params.m_srcExtAddr);
                NS_LOG_DEBUG("Packet to " << params.m_dstExtAddr);
            }

            // TODO: Fix here, this should trigger different Indication Callbacks
            // depending the type of frame received (data,command, beacon)
            if (!m_mcpsDataIndicationCallback.IsNull())
            {
                NS_LOG_DEBUG("promiscuous mode, forwarding up");
                m_mcpsDataIndicationCallback(params, p);
            }
            else
            {
                NS_LOG_ERROR(this << " Data Indication Callback not initialized");
            }

        } else {

            // For Hilow
            if (m_acceptAllHilowPkt && receivedMacHdr.GetType() == LrWpanMacHeader::LRWPAN_MAC_DATA) {
                if (!m_mcpsDataIndicationCallback.IsNull()) {
                    NS_LOG_DEBUG("promiscuous mode, forwarding up");
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

            if (acceptFrame && (receivedMacHdr.GetDstAddrMode() == EXT_ADDR)) {
                acceptFrame = (receivedMacHdr.GetExtDstAddr() == m_selfExt);
            }

            // When PASSIVE or ACTIVE scan is running, reject any frames other than BEACON frames
            if (acceptFrame && (!receivedMacHdr.IsBeacon() && m_scanEvent.IsRunning())) {
                acceptFrame = false;
            }


            // non DSME mode device cannot receive enhanced beacon
            if (acceptFrame && !m_macDSMEenabled && receivedMacHdr.IsBeacon() 
                && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4) {
                acceptFrame = false;
            }

            // Energy Scan is running, reject any frames
            if (m_scanEnergyEvent.IsRunning()) {
                acceptFrame = false;
            }

            // When Enhanced Active scan is running, accept only Enhanced Beacon frame
            if (acceptFrame && m_scanEvent.IsRunning() 
                && receivedMacHdr.IsBeacon()
                && m_scanParams.m_scanType == MLMESCAN_ENHANCED_ACTIVE_SCAN
                && receivedMacHdr.GetFrameVer() != LrWpanMacHeader::IEEE_802_15_4) {
                acceptFrame = false;
            }

            // When PASSIVE or ACTIVE scan is running, accept only Beacon frame
            if (acceptFrame && m_scanEvent.IsRunning()
                && receivedMacHdr.IsBeacon()
                && m_scanParams.m_scanType != MLMESCAN_ENHANCED_ACTIVE_SCAN
                && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4) {
                acceptFrame = false;
            }

            // Orphan Scan can receive the coordinator realignment command
            if (receivedMacHdr.IsCommand() && m_scanEvent.IsRunning()
                && m_scanParams.m_scanType == MLMESCAN_ORPHAN) {
                CommandPayloadHeader peekedPayloadHdr;
                p->PeekHeader(peekedPayloadHdr);

                if (peekedPayloadHdr.GetCommandFrameType() == CommandPayloadHeader::COOR_REALIGN) {
                    acceptFrame = true;
                }
            }

            // Check device is panCoor with association permit when receiving Association Request
            // Commands.
            // TODO:: Simple coordinators should also be able to receive it (currently only Pan
            // Coordinators are checked)
            // DSME-TODO
            if (acceptFrame && (receivedMacHdr.IsCommand() && receivedMacHdr.IsAckReq()))
            {
                CommandPayloadHeader receivedMacPayload;
                p->PeekHeader(receivedMacPayload);

                if (receivedMacPayload.GetCommandFrameType() ==
                        CommandPayloadHeader::ASSOCIATION_REQ &&
                    !(m_macAssociationPermit == true && m_coord == true)) {
                    acceptFrame = false;
                }

                // DSME
                if (receivedMacPayload.GetCommandFrameType() ==
                        CommandPayloadHeader::DISASSOCIATION_NOTIF &&
                    receivedMacPayload.GetDisassociationReason() ==
                        CommandPayloadHeader::DISASSC_DEV_LEAVE_PAN &&
                    !(m_macAssociationPermit == true && m_coord == true)) {
                    acceptFrame = false;
                }

                if (receivedMacPayload.GetCommandFrameType() ==
                        CommandPayloadHeader::DISASSOCIATION_NOTIF &&
                    receivedMacPayload.GetDisassociationReason() ==
                        CommandPayloadHeader::DISASSC_COORD_WISH_DEV_LEAVE_PAN &&
                    !(GetPanId() == receivedMacHdr.GetDstPanId())) {
                    acceptFrame = false;
                }

                // Although ACKs do not use CSMA to to be transmitted, we need to make sure
                // that the transmitted ACK will not collide with the transmission of a beacon
                // when beacon-enabled mode is running in the coordinator.
                if (acceptFrame && (m_csmaCa->IsSlottedCsmaCa() 
                    && (m_capEvent.IsRunning() || m_incCapEvent.IsRunning()))) {
                    Time timeLeftInCap;

                    if (m_capEvent.IsRunning()) {
                        timeLeftInCap = Simulator::GetDelayLeft(m_capEvent);
                    } else if (m_incCapEvent.IsRunning()) {
                        timeLeftInCap = Simulator::GetDelayLeft(m_incCapEvent);
                    }
                    
                    uint64_t ackSymbols = m_phy->aTurnaroundTime + m_phy->GetPhySHRDuration() +
                                          ceil(6 * m_phy->GetPhySymbolsPerOctet());
                    Time ackTime = Seconds((double)ackSymbols / symbolRate);

                    if (ackTime >= timeLeftInCap)
                    {
                        NS_LOG_DEBUG("Command frame received but not enough time to transmit ACK "
                                     "before the end of CAP ");
                        acceptFrame = false;
                    }
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
                    ) {
                    // If this is a data or mac command frame, which is not a broadcast or
                    // multicast, with ack req set, generate and send an ack frame. If there is a
                    // CSMA medium access in progress we cancel the medium access for sending the
                    // ACK frame. A new transmission attempt will be started after the ACK was send.
                    if (m_lrWpanMacState == MAC_ACK_PENDING)
                    {
                        m_ackWaitTimeout.Cancel();
                        PrepareRetransmission();
                    }
                    else if (m_lrWpanMacState == MAC_CSMA)
                    {   
                        // DSME-TODO
                        // \todo: If we receive a packet while doing CSMA/CA, should  we drop the
                        // packet because of channel busy,
                        //        or should we restart CSMA/CA for the packet after sending the ACK?
                        // Currently we simply restart CSMA/CA after sending the ACK.
                        NS_LOG_DEBUG("Received a packet with ACK required while in CSMA. Cancel "
                                     "current CSMA-CA");
                        m_csmaCa->Cancel();
                    }

                    // Cancel any pending MAC state change, ACKs have higher priority.
                    m_setMacState.Cancel();
                    ChangeMacState(MAC_IDLE);

                    // save received packet to process the appropriate indication/response after
                    // sending ACK (PD-DATA.confirm)
                    m_rxPkt = originalPkt->Copy();

                    // LOG Commands with ACK required.
                    CommandPayloadHeader receivedMacPayload;
                    p->PeekHeader(receivedMacPayload);

                    MlmeSyncRequestParams syncParams;

                    switch (receivedMacPayload.GetCommandFrameType()) {
                        case CommandPayloadHeader::DATA_REQ:
                            NS_LOG_DEBUG("Data Request Command Received; processing ACK");
                            break;

                        case CommandPayloadHeader::ASSOCIATION_REQ:
                            NS_LOG_DEBUG("Association Request Command Received; processing ACK");
                            break;

                        case CommandPayloadHeader::ASSOCIATION_RESP:
                            m_assocResCmdWaitTimeout.Cancel(); // cancel event to a lost assoc resp cmd.
                            NS_LOG_DEBUG("Association Response Command Received; processing ACK");
                            break;

                        case CommandPayloadHeader::DSME_ASSOCIATION_REQ:
                            NS_LOG_DEBUG("Dsme Association Request Command Received; processing ACK");
                            break;
                        
                        case CommandPayloadHeader::DSME_ASSOCIATION_RESP:
                            m_assocResCmdWaitTimeout.Cancel(); // cancel event to a lost assoc resp cmd.
                            NS_LOG_DEBUG("Dsme Association Response Command Received; processing ACK");
                            break;
                        
                        case CommandPayloadHeader::DISASSOCIATION_NOTIF:
                            NS_LOG_DEBUG("Disassociation Notification Command Received; processing ACK");

                            if (receivedMacPayload.GetDisassociationReason() 
                                == CommandPayloadHeader::DISASSC_COORD_WISH_DEV_LEAVE_PAN) {
                                RemoveReferencesToPAN();
                            }
                            
                            break;

                        // DSME-TOOD
                        // only the coordinator realignment command in response to an orphan scan would meet this
                        case CommandPayloadHeader::COOR_REALIGN:
                            NS_LOG_DEBUG("Coordinator realignment command Received; processing ACK");

                            if (m_scanEvent.IsRunning() && m_scanParams.m_scanType == MLMESCAN_ORPHAN) {
                                realignmentRecevied = true;
                            }
                            
                            m_macPanId = receivedMacPayload.GetPanId();
                            m_shortAddress = receivedMacPayload.GetShortAddr();
                            m_macCoordShortAddress = receivedMacPayload.GetCoordinatorShortAddress();
                            m_macCoordExtendedAddress = receivedMacHdr.GetExtSrcAddr();

                            syncParams.m_logCh = receivedMacPayload.GetChannelNum(); 
                            syncParams.m_logChPage = receivedMacPayload.GetChannelPage(); 
                            syncParams.m_trackBcn = true; 

                            Simulator::ScheduleNow(&LrWpanMac::MlmeSyncRequest,
                                                    this,
                                                    syncParams);  

                            break;

                        case CommandPayloadHeader::DSME_GTS_REQ:
                            NS_LOG_DEBUG("Dsme GTS Request Command Received; processing ACK");
          
                            break;       

                        case CommandPayloadHeader::DSME_GTS_REPLY:
                            NS_LOG_DEBUG("Dsme GTS Response/Reply Command Received; processing ACK");
                            m_dsmeGtsRespTimeout.Cancel();
                            break;
                        
                        case CommandPayloadHeader::DSME_GTS_NOTIFY:
                            NS_LOG_DEBUG("Dsme GTS Notify Command Received; processing ACK");
                            m_dsmeGtsNotifyTimeout.Cancel();
                            break;

                        case CommandPayloadHeader::DSME_INFO_REQ:
                            NS_LOG_DEBUG("Dsme Information Request Command Received; processing ACK");

                            break;

                        case CommandPayloadHeader::DSME_INFO_REPLY:
                            NS_LOG_DEBUG("Dsme Inforamtion Reply Command Received; processing ACK");
                            m_dsmeInfoReplyTimeout.Cancel();
                            break;

                        default:
                            break;
                    }

                    if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DATA_REQ) {
                        m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SendAckAfterDataReq,
                                                                this,
                                                                receivedMacHdr);

                    } else {
                        m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SendAck,
                                                           this,
                                                           receivedMacHdr.GetSeqNum());
                        // if (!receivedMacHdr.IsData()) {
                        //     m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SendAck,
                        //                                             this,
                        //                                             receivedMacHdr.GetSeqNum());
                        // }
                    }
                    
                    // 應該是要 coordinator 有想要挑 beacon timeslot 就要發送
                    if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_ASSOCIATION_RESP
                        && m_coord) {
                        m_sendDsmeBcnAllocNotifiCmd = Simulator::ScheduleNow(&LrWpanMac::SendDsmeBeaconAllocNotifyCommand
                                                                             , this);   
                    }                                                 
                }

                // DSME-TODO
                // Send a enhanced acknowledgment
                // if ((receivedMacHdr.IsData() || receivedMacHdr.IsCommand()) &&
                //     receivedMacHdr.IsAckReq() 
                //     && (receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4)
                //     && !(receivedMacHdr.GetDstAddrMode() == SHORT_ADDR &&
                //       (receivedMacHdr.GetShortDstAddr().IsBroadcast() ||
                //        receivedMacHdr.GetShortDstAddr().IsMulticast()))) {
                //     // DSME-TODO

                //     m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SendEnhancedAck,
                //                                            this,
                //                                            receivedMacHdr.GetSeqNum());
                // }

                if (receivedMacHdr.GetSrcAddrMode() == SHORT_ADDR) {
                    NS_LOG_DEBUG("Packet from " << params.m_srcAddr);
                } else if (receivedMacHdr.GetSrcAddrMode() == EXT_ADDR) {
                    NS_LOG_DEBUG("Packet from " << params.m_srcExtAddr);
                }

                if (receivedMacHdr.GetDstAddrMode() == SHORT_ADDR) {
                    NS_LOG_DEBUG("Packet to " << params.m_dstAddr);
                } else if (receivedMacHdr.GetDstAddrMode() == EXT_ADDR) {
                    NS_LOG_DEBUG("Packet to " << params.m_dstExtAddr);
                }

                if (receivedMacHdr.IsBeacon()) {
                    // DSME-TODO
                    // The received beacon size in symbols
                    // Beacon = 5 bytes Sync Header (SHR) +  1 byte PHY header (PHR) + PSDU (default
                    // 17 bytes)
                    m_rxBeaconSymbols = m_phy->GetPhySHRDuration() +
                                        1 * m_phy->GetPhySymbolsPerOctet() +
                                        (originalPkt->GetSize() * m_phy->GetPhySymbolsPerOctet());

                    // The start of Rx beacon time and start of the Incoming superframe Active
                    // Period
                    m_macBeaconRxTime =
                        Simulator::Now() - MilliSeconds(double(m_rxBeaconSymbols) / symbolRate);
                        
                    if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4) {
                        NS_LOG_DEBUG("Enhanced Beacon Received; forwarding up (m_macBeaconRxTime: "
                                 << m_macBeaconRxTime.As(Time::S) << ")");
                    } else {
                        NS_LOG_DEBUG("Beacon Received; forwarding up (m_macBeaconRxTime: "
                                 << m_macBeaconRxTime.As(Time::S) << ")");
                    }

                    // Fill the PAN descriptor
                    PanDescriptor panDescriptor;

                    DsmePANDescriptorIE receivedDsmePANDescriptorIEHeaderIE;

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
                        panDescriptor.m_bcnBitmap = receivedDsmePANDescriptorIEHeaderIE.GetBeaconBitmap();
                        panDescriptor.m_channelHoppingSpec = receivedDsmePANDescriptorIEHeaderIE.GetChannelHopping();
                        panDescriptor.m_gACKSpec = receivedDsmePANDescriptorIEHeaderIE.GetGroupACK();

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

                        // DSME-TODO
                        // 這個應該是要從 m_incomingSDBitmap 取出來
                        m_incSDindex = panDescriptor.m_bcnBitmap.GetSDIndex();

                        // incoming multi-superframe duration
                        m_incomingMultisuperframeDuration = 
                            (static_cast<uint32_t>(1 << m_incomingMultisuperframeOrder)) * aBaseSuperframeDuration;

                        // Channel Hopping Specification
                        // DSME-TODO

                        // Group ACK Specification
                        // DSME-TODO

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
                    } else {
                        panDescriptor.m_coorAddrMode = EXT_ADDR;
                        panDescriptor.m_coorExtAddr = receivedMacHdr.GetExtSrcAddr();
                    }

                    panDescriptor.m_coorPanId = receivedMacHdr.GetSrcPanId();
                    panDescriptor.m_gtsPermit = receivedMacPayload.GetGtsFields().GetGtsPermit();
                    panDescriptor.m_linkQuality = lqi;
                    panDescriptor.m_logChPage = m_phy->GetCurrentPage();
                    panDescriptor.m_logCh = m_phy->GetCurrentChannelNum();

                    // DSME
                    if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4
                        && receivedMacHdr.IsIEListPresent()) {
                        panDescriptor.m_superframeSpec = receivedDsmePANDescriptorIEHeaderIE.GetSuperframeField();
                    } else {
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
                        if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4
                        && receivedMacHdr.IsIEListPresent()) {
                            incomingSuperframe = receivedDsmePANDescriptorIEHeaderIE.GetSuperframeField();
                        } else {
                            incomingSuperframe = receivedMacPayload.GetSuperframeSpecField();
                        }

                        m_incomingBeaconOrder = incomingSuperframe.GetBeaconOrder();
                        m_incomingSuperframeOrder = incomingSuperframe.GetFrameOrder();
                        m_incomingFnlCapSlot = incomingSuperframe.GetFinalCapSlot();

                        m_incomingBeaconInterval =
                            (static_cast<uint32_t>(1 << m_incomingBeaconOrder)) *
                            aBaseSuperframeDuration;
                        m_incomingSuperframeDuration =
                            aBaseSuperframeDuration *
                            (static_cast<uint32_t>(1 << m_incomingSuperframeOrder));

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

                        if (m_macDSMEenabled 
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

                        if (m_macDSMEenabled && m_coord && !m_panCoor && m_sendBcn) {
                            Time scheduleBcnTime = Seconds(((double)m_incomingSuperframeDuration * m_choosedSDIndexToSendBcn) 
                                                           / symbolRate)
                                                           - (Simulator::Now() - m_startOfBcnSlotOfSyncParent);
                            
                            // NS_LOG_DEBUG(Simulator::Now() - m_startOfBcnSlotOfSyncParent); // debug
                            // std::cout << scheduleBcnTime.As(Time::NS) << std::endl; // debug

                            m_setMacState = Simulator::Schedule(scheduleBcnTime
                                                                , &LrWpanMac::SetLrWpanMacState
                                                                , this
                                                                , MAC_IDLE);

                            m_beaconEvent = Simulator::Schedule(scheduleBcnTime
                                                                , &LrWpanMac::SendOneEnhancedBeacon
                                                                , this);
                            
                            m_sendBcn = false;
                        }

                        PurgeDsmeACT();

                        // DSME-TODO
                        // For Dsme net device setting use only 
                        // ScheduleGtsSyncToCoord(m_incSDindex);

                        // if (receivedMacHdr.GetExtSrcAddr() == GetCoordExtAddress()
                        //     || receivedMacHdr.GetShortSrcAddr() == GetCoordShortAddress()) {
                        //     ScheduleGts(true);
                        // }

                        // CAP reduction 
                        // DSME-TODO
                        // 應該從 dsme pan descriptor 拿出資訊來檢查 CAP reduction
                        if (m_macDSMEenabled && m_incomingCAPReductionFlag) {
                            if (m_incSDindex % (m_incomingMultisuperframeDuration / m_incomingSuperframeDuration)) {
                                NS_LOG_DEBUG("Incoming superframe Active Portion (Beacon + CFP + CFP): "
                                            << m_incomingSuperframeDuration << " symbols");
                                m_incomingFirstCFP = true;
                                m_incCfpEvent = Simulator::ScheduleNow(&LrWpanMac::StartCFP,
                                                                    this,
                                                                    SuperframeType::INCOMING);
                            } else {
                                m_incCapEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                            this,
                                                            SuperframeType::INCOMING);  
                                                            
                                m_setMacState =
                                    Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);  
                            }

                        } else {
                            m_incCapEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                            this,
                                                            SuperframeType::INCOMING);  
                            m_setMacState =
                                Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);                         
                        }                

                        // Begin CAP on the current device using info from the Incoming superframe
                        // NS_LOG_DEBUG("Incoming superframe Active Portion (Beacon + CAP + CFP): "
                        //              << m_incomingSuperframeDuration << " symbols");
                        // m_incCapEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                        //                                        this,
                        //                                        SuperframeType::INCOMING);
                        // m_setMacState =
                        //     Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);

                    } else if (!m_scanEvent.IsRunning() && m_macPanId == 0xFFFF) {
                        NS_LOG_DEBUG(this << " Device not associated, cannot process beacon");
                        return;
                    }

                    if (m_macAutoRequest) {
                        if (p->GetSize() > 0) {      // the beacon contains any beacon payload
                            if (!m_mlmeBeaconNotifyIndicationCallback.IsNull()) {
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

                        if (m_scanEvent.IsRunning()) {
                            // Channel scanning is taking place, save only unique PAN descriptors
                            bool descriptorExists = false;

                            for (const auto& descriptor : m_panDescriptorList)
                            {
                                if (descriptor.m_coorAddrMode == SHORT_ADDR)
                                {
                                    // Found a coordinator in PAN descriptor list with the same
                                    // registered short address
                                    descriptorExists =
                                        (descriptor.m_coorShortAddr ==
                                             panDescriptor.m_coorShortAddr &&
                                         descriptor.m_coorPanId == panDescriptor.m_coorPanId);
                                }
                                else
                                {
                                    // Found a coordinator in PAN descriptor list with the same
                                    // registered extended address
                                    descriptorExists =
                                        (descriptor.m_coorExtAddr == panDescriptor.m_coorExtAddr &&
                                         descriptor.m_coorPanId == panDescriptor.m_coorPanId);
                                }

                                if (descriptorExists)
                                {
                                    break;
                                }
                            }

                            if (!descriptorExists) {
                                m_panDescriptorList.push_back(panDescriptor);
                            }

                            return;

                        } else if (m_trackingEvent.IsRunning()) {  // currently synchronizing with a coordinator
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
                            
                            if (GetShortAddress() != Mac16Address("ff:ff")) {
                                if (pndAddrFields.SearchAddress(GetShortAddress())) {
                                    NS_LOG_DEBUG("Extract the Pending Address List From Beacon/Enhanced Beacon, then Send Data Request Command to the Coordinator");
                                    SendDataRequestCommand();
                                }
                            }

                            if (GetExtendedAddress() != Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed")) {
                                if (pndAddrFields.SearchAddress(GetExtendedAddress())) {
                                    NS_LOG_DEBUG("Extract the Pending Address List From Beacon/Enhanced Beacon, then Send Data Request Command to the Coordinator");
                                    SendDataRequestCommand();
                                }
                            }

                            // TODO: Ignore pending data, and do not send data command request if
                            // the address is in the GTS list.s
                            //       If the address is not in the GTS list, then  check if the
                            //       address is in the short address pending list or in the extended
                            //       address pending list and send a data command request.

                        } 

                    } else {
                        // m_macAutoRequest is FALSE
                        // Data command request are not send, only the beacon notification.
                        // see IEEE 802.15.4-2011 Section 6.2.4.1
                        if (!m_mlmeBeaconNotifyIndicationCallback.IsNull()) {
                            MlmeBeaconNotifyIndicationParams beaconParams;

                            if (m_macDSMEenabled && receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4) {
                                beaconParams.m_ebsn = receivedMacHdr.GetSeqNum();
                                beaconParams.m_beaconType = 0x01;
                                
                            } else {
                                beaconParams.m_bsn = receivedMacHdr.GetSeqNum();
                                beaconParams.m_beaconType = 0x00;
                            }
                            
                            beaconParams.m_panDescriptor = panDescriptor;
                            m_mlmeBeaconNotifyIndicationCallback(beaconParams, originalPkt);
                        }
                    }

                } else if (receivedMacHdr.IsCommand()) {
                    // Handle the reception of frame commands that do not require ACK (i.e. Beacon
                    // Request Command)
                    HeaderIETermination termination;
                    PayloadIETermination termination2;
                    EBFilterIE filterIE;
                    bool respTheEBR = true;

                    if (receivedMacHdr.IsIEListPresent()) {
                        p->RemoveHeader(termination);
                        p->RemoveHeader(filterIE);
                        p->RemoveHeader(termination2);

                        if (filterIE.IsPermitJoiningOn() && !m_macAssociationPermit) {
                            respTheEBR = false;
                        }
                    }

                    CommandPayloadHeader receivedMacPayload;
                    p->PeekHeader(receivedMacPayload);

                    if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::BEACON_REQ) {
                        // TODO: check that node is any coordinator not just pan coordinator
                        if (m_csmaCa->IsUnSlottedCsmaCa() && m_coord) { // non-beacon enabled PAN
                            if (receivedMacHdr.GetFrameVer() == LrWpanMacHeader::IEEE_802_15_4 && m_macDSMEenabled) {
                                NS_LOG_DEBUG("Enhanced Beacon Request Command Received;");

                                if (respTheEBR) {
                                    NS_LOG_DEBUG("Response with Enhanced Beacon.");
                                    SendOneEnhancedBeacon();
                                }

                            } else {
                                NS_LOG_DEBUG("Beacon Request Command Received; Response with Beacon.");
                                SendOneBeacon();
                            }

                        } else {
                            m_macRxDropTrace(originalPkt);
                        }

                    } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_BEACON_ALLOC_NOTIF) {
                        NS_LOG_DEBUG("Received Dsme beacon allocation notification with SD index: " 
                                    << receivedMacPayload.GetAllocationBcnSDIndex());

                        // DSME-TODO
                        // Update beacon bitmap?
                        m_macSDBitmap.SetSDBitmap(receivedMacPayload.GetAllocationBcnSDIndex());

                        NS_LOG_DEBUG("Current mac SD Bitmap is updated as: " << m_macSDBitmap);

                    } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::COOR_REALIGN) {
                        // DSME-TODO:
                        // the device shall ensure that its own beacons do not overlap 
                        // with the beacons transmitted by the coordinator.
                        if (GetCoordShortAddress() == receivedMacPayload.GetCoordinatorShortAddress()
                            && receivedMacHdr.GetShortDstAddr().IsBroadcast()) {
                            if (!m_mlmeSyncLossIndicationCallback.IsNull()) {
                                MlmeSyncLossIndicationParams syncLossParams;
                                syncLossParams.m_panId = receivedMacPayload.GetPanId();
                                syncLossParams.m_logCh = receivedMacPayload.GetChannelNum();

                                if (false) {
                                    syncLossParams.m_lossReason = MLMESYNCLOSS_SUPERFRAME_OVERLAP;
                                } else {
                                    syncLossParams.m_lossReason = MLMESYNCLOSS_REALIGMENT;
                                }

                                m_mlmeSyncLossIndicationCallback(syncLossParams);

                                m_beaconTrackingOn = false;

                                // DSME-TODO
                                m_trackingEvent.Cancel();
                                m_beaconEvent.Cancel();
                                m_capEvent.Cancel();
                                m_cfpEvent.Cancel();
                                m_incCapEvent.Cancel();
                                m_incCfpEvent.Cancel();
                                
                                m_macPanId = receivedMacPayload.GetPanId();
                            }
                        }

                    } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::ORPHAN_NOTIF) {
                        NS_LOG_DEBUG("Orphan Notification Received; forwarding up");    

                        if (!m_mlmeOrphanIndicationCallback.IsNull()) {
                            MlmeOrphanIndicationPararms OrphanParams;
                            OrphanParams.m_orphanAddress = receivedMacHdr.GetExtSrcAddr();
                            m_mlmeOrphanIndicationCallback(OrphanParams);
                        }

                    } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_INFO_REQ) {
                        if (!m_mlmeDsmeInfoIndicationCallback.IsNull()) {
                            MlmeDsmeInfoIndicationParams indicationParams;
                            indicationParams.m_devAddr = receivedMacHdr.GetExtSrcAddr();
                            indicationParams.m_info = static_cast<InfoType>(receivedMacPayload.GetDsmeInfoType());

                            m_mlmeDsmeInfoIndicationCallback(indicationParams);
                        }

                        // the device shall determine whether it has an allocated DSME-GTS 
                        // to the requesting device.

                        // If it has an allocated DSME-GTS, the
                        // MLME of the Destination device shall send a DSME Information 
                        // reply command frame in the DSME-GTS.

                        uint16_t superframeID = 0;
                        uint8_t slotID = 0;

                        if (receivedMacPayload.GetDsmeInfoType() == MLMEDSMEINFO_TIMESTAMP) {
                            if (SearchDsmeACTForAllocatedGTS(receivedMacHdr.GetShortSrcAddr(), superframeID, slotID)) {
                                SendDsmeInfoResponseCommand(m_rxPkt->Copy(), superframeID, slotID);
                            }

                        } else {
                            SendDsmeInfoResponseCommand(m_rxPkt->Copy(), superframeID, slotID);
                        }

                    } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_INFO_REPLY) {

                        if (!m_mlmeDsmeInfoConfirmCallback.IsNull()) {
                            MlmeDsmeInfoConfirmParams confirmParams;
                            confirmParams.m_info = static_cast<InfoType>(receivedMacPayload.GetDsmeInfoType());

                            if (confirmParams.m_info == MLMEDSMEINFO_TIMESTAMP) {
                                confirmParams.m_timestamp = receivedMacPayload.GetDsmeInfoTimestamp();
                                confirmParams.m_superframeID = receivedMacPayload.GetDsmeInfoSuperframeID();
                                confirmParams.m_slotID = receivedMacPayload.GetDsmeInfoSlotID();

                            } else if (confirmParams.m_info == MLMEDSMEINFO_DSME_SAB_SPECIFICATION) {
                                confirmParams.m_dsmeSABSpec = receivedMacPayload.GetDsmeGtsSABSpec();

                            } else {
                                confirmParams.m_panDescriptor = receivedMacPayload.GetDsmeInfoPANDescriptor();
                            }

                            confirmParams.m_status = MLMEDSMEINFO_SUCCESS;

                            m_mlmeDsmeInfoConfirmCallback(confirmParams);
                        }
                        
                        m_gtsRetrieve = false;
                    }

                } else if (receivedMacHdr.IsData() && !m_mcpsDataIndicationCallback.IsNull()) {
                    // If it is a data frame, push it up the stack.
                    // Fow hilow
                    if (!m_acceptAllHilowPkt) {
                        NS_LOG_DEBUG("Data Packet is for me; forwarding up");
                        m_mcpsDataIndicationCallback(params, p);
                    }

                    // NS_LOG_DEBUG("Data Packet is for me; forwarding up");
                    // m_mcpsDataIndicationCallback(params, p);

                    if (m_incGtsEvent.IsRunning()) {
                        m_macDsmeACT[m_curGTSSuperframeID][m_curGTSIdx].m_cnt = 0;
                    }

                } else if (receivedMacHdr.IsAcknowledgment() && m_txPkt &&
                        m_lrWpanMacState == MAC_ACK_PENDING) {
                    LrWpanMacHeader peekedMacHdr;
                    m_txPkt->PeekHeader(peekedMacHdr);          // the frame this device previously sent

                    // If it is an ACK with the expected sequence number, finish the transmission
                    if (receivedMacHdr.GetSeqNum() == peekedMacHdr.GetSeqNum()) {
                        m_ackWaitTimeout.Cancel();
                        m_macTxOkTrace(m_txPkt);

                        // TODO: check  if the IFS is the correct size after ACK.
                        Time ifsWaitTime = Seconds((double)GetIfsSize() / symbolRate);

                        // We received an ACK to a command
                        if (peekedMacHdr.IsCommand()) {
                            // check the original sent command frame which belongs to this received
                            // ACK
                            Ptr<Packet> pkt = m_txPkt->Copy();
                            LrWpanMacHeader macHdr;
                            CommandPayloadHeader cmdPayload;
                            pkt->RemoveHeader(macHdr);
                            pkt->RemoveHeader(cmdPayload);

                            switch (cmdPayload.GetCommandFrameType()) {
                                case CommandPayloadHeader::ASSOCIATION_REQ: {
                                    double symbolRate = m_phy->GetDataOrSymbolRate(false);
                                    // macResponeWaitTime
                                    Time waitTime = Seconds(static_cast<double>(m_macResponseWaitTime) /
                                                            symbolRate);

                                    if (!m_beaconTrackingOn) {
                                        m_respWaitTimeout =
                                            Simulator::Schedule(waitTime,
                                                                &LrWpanMac::SendDataRequestCommand,
                                                                this);
                                    } else {
                                        // TODO: The data must be extracted by the coordinator within
                                        // macResponseWaitTime on timeout, MLME-ASSOCIATE.confirm is set
                                        // with status NO_DATA, and this should trigger the cancellation
                                        // of the beacon tracking (MLME-SYNC.request  trackBeacon
                                        // =FALSE)
                                    }

                                    break;
                                }

                                case CommandPayloadHeader::ASSOCIATION_RESP: {
                                    // MLME-comm-status.Indication generated as a result of an
                                    // association response command, therefore src and dst address use
                                    // extended mode (see 5.3.2.1)
                                    if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
                                        MlmeCommStatusIndicationParams commStatusParams;
                                        commStatusParams.m_panId = m_macPanId;
                                        commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                                        commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                                        commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                                        commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                                        commStatusParams.m_status =
                                            LrWpanMlmeCommStatus::MLMECOMMSTATUS_SUCCESS;
                                        m_mlmeCommStatusIndicationCallback(commStatusParams);
                                    }

                                    // Remove element from Pending Transaction List
                                    RemovePendTxQElement(m_txPkt->Copy());
                                    break;
                                }

                                // DSME
                                case CommandPayloadHeader::DSME_ASSOCIATION_REQ: {
                                    double symbolRate = m_phy->GetDataOrSymbolRate(false);

                                    Time waitTime = Seconds(static_cast<double>(m_macResponseWaitTime) /
                                                            symbolRate);
                                    if (!m_associateParams.m_capabilityInfo.IsFastAOn()) {
                                        if (!m_beaconTrackingOn) {
                                            // DSME: within the macResponseWait Time, a data request command must be sent to coordinator
                                            // To request the dsme association reponse command in the pending transaction list 
                                            // in the coordinator
                                            m_respWaitTimeout = Simulator::Schedule(waitTime,
                                                                    &LrWpanMac::SendDataRequestCommand,
                                                                    this);
                                        } else {
                                            // DSME-TODO: The data must be extracted by the coordinator within
                                            // macResponseWaitTime on timeout, MLME-ASSOCIATE.confirm is set
                                            // with status NO_DATA, and this should trigger the cancellation
                                            // of the beacon tracking (MLME-SYNC.request  trackBeacon
                                            // =FALSE)
                                        }

                                    } else { // FastA
                                        // waitTime = Seconds(
                                        //     static_cast<double>(m_assocRespCmdWaitTime) / symbolRate);

                                        // m_assocResCmdWaitTimeout =
                                        //     Simulator::Schedule(waitTime,
                                        //                         &LrWpanMac::LostAssocRespCommand,
                                        //                         this);
                                    }                        
                                    
                                    break;
                                }

                                // DSME
                                case CommandPayloadHeader::DSME_ASSOCIATION_RESP: {
                                    if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
                                        MlmeCommStatusIndicationParams commStatusParams;
                                        commStatusParams.m_panId = m_macPanId;
                                        commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                                        commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                                        commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                                        commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                                        commStatusParams.m_status =
                                            LrWpanMlmeCommStatus::MLMECOMMSTATUS_SUCCESS;
                                        m_mlmeCommStatusIndicationCallback(commStatusParams);
                                    }

                                    // Remove element from Pending Transaction List
                                    RemovePendTxQElement(m_txPkt->Copy());
                                    break;
                                }

                                case CommandPayloadHeader::DISASSOCIATION_NOTIF: {
                                    if (!m_mlmeDisassociateConfirmCallback.IsNull()) {
                                        MlmeDisassociateConfirmParams disassociateParams;
                                        disassociateParams.m_status = DISASSOCIATE_SUCCESS;

                                        if (macHdr.GetDstAddrMode() == LrWpanMacHeader::SHORTADDR) {
                                            disassociateParams.m_devAddrMode = SHORT_ADDR;
                                            disassociateParams.m_shortDevAddr = macHdr.GetShortDstAddr();
                                        } else {
                                            disassociateParams.m_devAddrMode = EXT_ADDR;
                                            disassociateParams.m_extDevAddr = macHdr.GetExtDstAddr();
                                        }

                                        disassociateParams.m_devPanId = macHdr.GetDstPanId();
                                        m_mlmeDisassociateConfirmCallback(disassociateParams);
                                    }

                                    if (m_disassociateParams.m_txIndirect) {
                                        RemovePendTxQElement(m_txPkt->Copy());
                                    } 

                                    break;
                                }

                                case CommandPayloadHeader::DATA_REQ: {
                                    if (m_macPanId == 0xffff) {    // is not associated yet
                                        // Schedule an event in case the Association Response Command never
                                        // reached this device during an association process.
                                        double symbolRate = m_phy->GetDataOrSymbolRate(false);
                                        Time waitTime = Seconds(
                                            static_cast<double>(m_assocRespCmdWaitTime) / symbolRate);
                                        m_assocResCmdWaitTimeout =
                                            Simulator::Schedule(waitTime,
                                                                &LrWpanMac::LostAssocRespCommand,
                                                                this);

                                        if (!m_mlmePollConfirmCallback.IsNull()) {
                                            MlmePollConfirmParams pollConfirmParams;
                                            pollConfirmParams.m_status =
                                                LrWpanMlmePollConfirmStatus::MLMEPOLL_SUCCESS;
                                            m_mlmePollConfirmCallback(pollConfirmParams);
                                        }

                                    } else { 
                                        // the data request sent previously is intiated by MLME-POLL.request
                                        if (!m_mlmePollConfirmCallback.IsNull()) {
                                            MlmePollConfirmParams pollConfirmParams;
                                            pollConfirmParams.m_status =
                                                LrWpanMlmePollConfirmStatus::MLMEPOLL_SUCCESS;
                                            m_mlmePollConfirmCallback(pollConfirmParams);
                                        }
                                    }
                                    
                                    break;
                                }

                                // DSME-TODO
                                // Only the coordinator realignment command will meet this
                                case CommandPayloadHeader::COOR_REALIGN: {
                                    if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
                                        MlmeCommStatusIndicationParams commStatusParams;
                                        commStatusParams.m_panId = m_macPanId;
                                        commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                                        commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                                        commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                                        commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                                        commStatusParams.m_status =
                                            LrWpanMlmeCommStatus::MLMECOMMSTATUS_SUCCESS;
                                        m_mlmeCommStatusIndicationCallback(commStatusParams);
                                    }

                                    break;
                                }              

                                case CommandPayloadHeader::DSME_GTS_REQ: {
                                    m_dsmeGtsAckWaitTimeout.Cancel();

                                    double symbolRate = m_phy->GetDataOrSymbolRate(false);
                                    // DSME-TODO: macMaxFrameTotalWaitTime CAP symbols?
                                    Time waitTime = Seconds(static_cast<double>(m_macResponseWaitTime) /
                                                            symbolRate);

                                    m_dsmeGtsRespTimeout = Simulator::Schedule(waitTime,
                                                                              &LrWpanMac::DsmeGtsRespWaitTimeout,
                                                                              this);
                                    break;
                                }    

                                case CommandPayloadHeader::DSME_GTS_REPLY: {
                                    double symbolRate = m_phy->GetDataOrSymbolRate(false);
                                    // DSME-TODO: macMaxFrameTotalWaitTime CAP symbols?
                                    Time waitTime = Seconds(static_cast<double>(m_macResponseWaitTime) /
                                                            symbolRate);

                                    m_dsmeGtsNotifyTimeout = Simulator::Schedule(waitTime,
                                                                &LrWpanMac::DsmeGtsNotifyWaitTimeout,
                                                                this);

                                    if (cmdPayload.GetDsmeGtsManagementField().GetManagementType() == 0b001
                                        && cmdPayload.GetDsmeGtsManagementField().GetStatus() == 0b000) {
                                        m_gtsDirections.push_back(!cmdPayload.GetDsmeGtsManagementField()
                                                                             .IsDirectionRX());
                                    
                                        CheckDsmeGtsSABFromReplyCmd(cmdPayload.GetDsmeGtsSABSpec());

                                    } else if (cmdPayload.GetDsmeGtsManagementField().GetManagementType() == 0b000
                                               && cmdPayload.GetDsmeGtsManagementField().GetStatus() == 0b000) {
                                        UpdateDsmeACTAndDeallocate(cmdPayload.GetDsmeGtsSABSpec());

                                        // DSME-TODO
                                        DSMESABSpecificationField partialSAB = cmdPayload.GetDsmeGtsSABSpec();
                                        
                                        if (partialSAB.isCAPReduction()) {
                                            std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                            for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                                m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                                    m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] ^ subBlk[i];                                 
                                            }

                                        } else {
                                            std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                            for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                                m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                                    m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] ^ subBlkCapOff[i];   
                                            }
                                        }
                                    }
                                      
                                    // NS_LOG_DEBUG(" GTS Superframe ID: " << m_gtsSuperframeIDs.back() << std::endl
                                    //             << " Direction: " << m_gtsDirections.back() << std::endl
                                    //             << " Slot ID: " << std::get<0>(m_gtsStartAndLens.back()) << std::endl
                                    //             << " GTS Length:  " << std::get<1>(m_gtsStartAndLens.back()) << std::endl);

                                    break;
                                }     

                                case CommandPayloadHeader::DSME_GTS_NOTIFY: {
                                    // nothing to do here
                                    break;
                                }   

                                case CommandPayloadHeader::DSME_INFO_REQ: {
                                    m_dsmeInfoAckWaitTimeout.Cancel();

                                    double symbolRate = m_phy->GetDataOrSymbolRate(false);
                                    // DSME-TODO: macMaxFrameTotalWaitTime CAP symbols?
                                    Time waitTime = Seconds(static_cast<double>(m_macResponseWaitTime) /
                                                            symbolRate);

                                    m_dsmeInfoReplyTimeout = Simulator::Schedule(waitTime,
                                                                              &LrWpanMac::DsmeGtsReplyWaitTimeout,
                                                                              this);
                                    
                                    m_gtsRetrieve = true;

                                    break;
                                }  

                                default: {
                                    // TODO: add response to other request commands (e.g. Orphan)
                                    break;
                                }
                            }

                        } else {
                            if (!m_mcpsDataConfirmCallback.IsNull()) {
                                if (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()) {
                                    // For dsme-net-device-throughput... testing usage, So comment it
                                    // McpsDataConfirmParams confirmParams;
                                    // confirmParams.m_msduHandle = m_mcpsDataRequestParams.m_msduHandle;
                                    // confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                                    // m_mcpsDataConfirmCallback(confirmParams);

                                } else {
                                    Ptr<TxQueueElement> txQElement = m_txQueue.front();
                                    McpsDataConfirmParams confirmParams;
                                    confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                                    confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                                    m_mcpsDataConfirmCallback(confirmParams);
                                }
                            }
                        }

                        // Ack was successfully received, wait for the Interframe Space (IFS) and
                        // then proceed
                        if (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()) {
                            m_txPkt = nullptr;
                            m_setMacState =
                                Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_GTS);
                            
                            m_ifsEvent = Simulator::Schedule(ifsWaitTime,
                                                            &LrWpanMac::IfsWaitTimeout,
                                                            this,
                                                            ifsWaitTime);
                                                            
                        } else {
                            RemoveFirstTxQElement();
                            m_setMacState.Cancel();
                            m_setMacState =
                                Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);

                            m_ifsEvent = Simulator::Schedule(ifsWaitTime,
                                                            &LrWpanMac::IfsWaitTimeout,
                                                            this,
                                                            ifsWaitTime);
                        }
                        
                    } else {
                        // If it is an ACK with an unexpected sequence number, mark the current
                        // transmission as failed and start a retransmit. (cf 7.5.6.4.3)
                        m_ackWaitTimeout.Cancel();

                        if (!PrepareRetransmission()) {
                            m_setMacState.Cancel();
                            m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState,
                                                                   this,
                                                                   MAC_IDLE);

                        } else {
                            m_setMacState.Cancel();

                            if (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning()) {
                                m_txPkt = m_txPktGts;
                                ChangeMacState(MAC_GTS_SENDING);
                                m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);

                            } else {
                                m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState,
                                                                    this,
                                                                    MAC_CSMA);
                            }
                        }
                    }
                }
            }
            else
            {
                m_macRxDropTrace(originalPkt);
            }
        }
    }
}

void
LrWpanMac::SendAck(uint8_t seqno)
{
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(seqno));

    NS_ASSERT(m_lrWpanMacState == MAC_IDLE);

    // Generate a corresponding ACK Frame.
    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT, seqno);
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> ackPacket = Create<Packet>(0);
    ackPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(ackPacket);
    }

    ackPacket->AddTrailer(macTrailer);

    // Enqueue the ACK packet for further processing
    // when the transmitter is activated.
    m_txPkt = ackPacket;

    // Switch transceiver to TX mode. Proceed sending the Ack on confirm.

    if (m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) { 
        ChangeMacState(MAC_GTS_SENDING);
    } else {
        ChangeMacState(MAC_SENDING);
    }
    
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

void LrWpanMac::SendAckAfterDataReq(LrWpanMacHeader receivedMacHdr) {
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(receivedMacHdr.GetSeqNum()));

    NS_ASSERT(m_lrWpanMacState == MAC_IDLE);

    // Generate a corresponding ACK Frame.
    LrWpanMacHeader macHdr(LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT, receivedMacHdr.GetSeqNum());
    LrWpanMacTrailer macTrailer;
    Ptr<Packet> ackPacket = Create<Packet>(0);

    Ptr<IndTxQueueElement> indTxQElement = Create<IndTxQueueElement>();
    bool elementFound = false;

    if (receivedMacHdr.GetDstAddrMode() == EXT_ADDR) {
        elementFound = SearchIndQueueElement(receivedMacHdr.GetExtSrcAddr(), indTxQElement);
    } else {
        elementFound = SearchIndQueueElement(receivedMacHdr.GetShortSrcAddr(), indTxQElement);
    }

    if (elementFound) {
        macHdr.SetFrmPend();
    } else {
        macHdr.SetNoFrmPend();
    }

    ackPacket->AddHeader(macHdr);

    // Calculate FCS if the global attribute ChecksumEnable is set.
    if (Node::ChecksumEnabled()) {
        macTrailer.EnableFcs(true);
        macTrailer.SetFcs(ackPacket);
    }

    ackPacket->AddTrailer(macTrailer);

    // Enqueue the ACK packet for further processing
    // when the transmitter is activated.
    m_txPkt = ackPacket;

    // Switch transceiver to TX mode. Proceed sending the Ack on confirm.

    if (m_incGtsEvent.IsRunning() || m_gtsEvent.IsRunning()) { 
        ChangeMacState(MAC_GTS_SENDING);
    } else {
        ChangeMacState(MAC_SENDING);
    }
    
    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
}

void LrWpanMac::SendEnhancedAck(uint8_t seqno) {
    // DSME-TODO
}

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

void
LrWpanMac::AckWaitTimeout()
{
    NS_LOG_FUNCTION(this);

    // TODO: If we are a PAN coordinator and this was an indirect transmission,
    //       we will not initiate a retransmission. Instead we wait for the data
    //       being extracted after a new data request command.

    if (!PrepareRetransmission()) {
        if (!m_gtsEvent.IsRunning() && !m_incGtsEvent.IsRunning() ) {
            SetLrWpanMacState(MAC_IDLE);
        } else {
            ChangeMacState(MAC_GTS_SENDING);
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
        }
        
    } else {
        if (!m_gtsEvent.IsRunning() && !m_incGtsEvent.IsRunning() ) {
            SetLrWpanMacState(MAC_CSMA);
        } else {
            ChangeMacState(MAC_GTS_SENDING);
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
        }
    }
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
    }
    else if (ifsTime == sifsTime)
    {
        NS_LOG_DEBUG("SIFS of " << m_macSIFSPeriod << " symbols (" << ifsTime.As(Time::S)
                                << ") completed ");
    }
    else
    {
        NS_LOG_DEBUG("Unknown IFS size (" << ifsTime.As(Time::S) << ") completed ");
    }

    m_macIfsEndTrace(ifsTime);
    CheckQueue();
}

void LrWpanMac::DsmeGtsRespWaitTimeout() {
    NS_LOG_FUNCTION(this);

    if (!m_mlmeDsmeGtsConfirmCallback.IsNull()) {
        MlmeDsmeGtsConfirmParams confirmParams;
        confirmParams.m_status = MLMEDSMEGTS_REQ_NO_DATA;

        m_mlmeDsmeGtsConfirmCallback(confirmParams);
    }
}

void LrWpanMac::DsmeGtsNotifyWaitTimeout() {
    NS_LOG_FUNCTION(this);

    if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
        MlmeCommStatusIndicationParams commStatusParams;
        commStatusParams.m_panId = GetPanId();
        commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
        // DSME-TODO: How to get the src ExtAddr
        // commStatusParams.m_srcExtAddr = GetExtendedAddress();
        commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
        commStatusParams.m_dstExtAddr = GetExtendedAddress();
        commStatusParams.m_status = MLMECOMMSTATUS_TRANSACTION_EXPIRED;

        m_mlmeCommStatusIndicationCallback(commStatusParams);
    }
}

void LrWpanMac::DsmeGtsReplyWaitTimeout() {
    NS_LOG_FUNCTION(this);

    if (!m_mlmeDsmeInfoConfirmCallback.IsNull()) {
        MlmeDsmeInfoConfirmParams confirmParams;
        confirmParams.m_status = MLMEDSMEINFO_NO_DATA;

        m_mlmeDsmeInfoConfirmCallback(confirmParams);
    }
}

bool
LrWpanMac::PrepareRetransmission()
{
    NS_LOG_FUNCTION(this);

    if (m_txPkt == nullptr) {
        return false;
    }

    // LrWpanMacHeader peekedMacHdr;
    // m_txPkt->PeekHeader(peekedMacHdr);

    // // DSME-TODO
    // if (peekedMacHdr.IsData() && (m_gtsEvent.IsRunning())) {
    //     return false;
    // }

    // Max retransmissions reached without receiving ACK,
    // send the proper indication/confirmation
    // according to the frame type and call drop trace.
    if (m_retransmission >= m_macMaxFrameRetries) {
        LrWpanMacHeader peekedMacHdr;
        m_txPkt->PeekHeader(peekedMacHdr);

        if (peekedMacHdr.IsCommand()) {
            m_macTxDropTrace(m_txPkt);

            Ptr<Packet> pkt = m_txPkt->Copy();
            LrWpanMacHeader macHdr;
            CommandPayloadHeader cmdPayload;
            pkt->RemoveHeader(macHdr);
            pkt->RemoveHeader(cmdPayload);

            switch (cmdPayload.GetCommandFrameType()) {
                case CommandPayloadHeader::ASSOCIATION_REQ: {
                    m_macPanId = 0xffff;
                    m_macCoordShortAddress = Mac16Address("FF:FF");
                    m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                    m_incCapEvent.Cancel();
                    m_incCfpEvent.Cancel();
                    m_csmaCa->SetUnSlottedCsmaCa();
                    m_incomingBeaconOrder = 15;
                    m_incomingSuperframeOrder = 15;

                    if (!m_mlmeAssociateConfirmCallback.IsNull())
                    {
                        MlmeAssociateConfirmParams confirmParams;
                        confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
                        confirmParams.m_status = MLMEASSOC_NO_ACK;
                        m_mlmeAssociateConfirmCallback(confirmParams);
                    }
                    break;
                }
                case CommandPayloadHeader::ASSOCIATION_RESP: {
                    // IEEE 802.15.4-2006 (Section 7.1.3.3.3 and 7.1.8.2.3)
                    if (!m_mlmeCommStatusIndicationCallback.IsNull())
                    {
                        MlmeCommStatusIndicationParams commStatusParams;
                        commStatusParams.m_panId = m_macPanId;
                        commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                        commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                        commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                        commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                        commStatusParams.m_status = LrWpanMlmeCommStatus::MLMECOMMSTATUS_NO_ACK;
                        m_mlmeCommStatusIndicationCallback(commStatusParams);
                    }
                    RemovePendTxQElement(m_txPkt->Copy());
                    break;
                }

                case CommandPayloadHeader::DSME_ASSOCIATION_REQ: {
                    // DSME-TODO
                }

                case CommandPayloadHeader::DSME_ASSOCIATION_RESP: {
                    // DSME-TODO
                }

                case CommandPayloadHeader::COOR_REALIGN: {
                    // DSME-TODO
                }

                case CommandPayloadHeader::DATA_REQ: {
                    // DSME-TODO
                    // IEEE 802.15.4-2006 (Section 7.1.16.1.3)
                    m_macPanId = 0xffff;
                    m_macCoordShortAddress = Mac16Address("FF:FF");
                    m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                    m_incCapEvent.Cancel();
                    m_incCfpEvent.Cancel();
                    m_csmaCa->SetUnSlottedCsmaCa();
                    m_incomingBeaconOrder = 15;
                    m_incomingSuperframeOrder = 15;

                    if (!m_mlmePollConfirmCallback.IsNull())
                    {
                        MlmePollConfirmParams pollConfirmParams;
                        pollConfirmParams.m_status = LrWpanMlmePollConfirmStatus::MLMEPOLL_NO_ACK;
                        m_mlmePollConfirmCallback(pollConfirmParams);
                    }
                    break;
                }

                case CommandPayloadHeader::DSME_INFO_REQ: {
                    
                    break;
                }

                case CommandPayloadHeader::DSME_INFO_REPLY: {

                    break;
                }
                
                default: {
                    // TODO: Specify other indications according to other commands
                    break;
                }
            }

        } else if (peekedMacHdr.IsData()) {
            // For lost data then trigger lost sync usage
            // m_macPanId = 0xffff;
            // m_macCoordShortAddress = Mac16Address("FF:FF");
            // m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
            // m_shortAddress = Mac16Address("00:00");

            // m_incCapEvent.Cancel();
            // m_incCfpEvent.Cancel();
            // m_csmaCa->SetUnSlottedCsmaCa();
            // m_incomingBeaconOrder = 15;
            // m_incomingSuperframeOrder = 15;

            // m_beaconTrackingOn = false;
            // m_trackingEvent.Cancel();
            
            if (!m_mcpsDataConfirmCallback.IsNull()) {
                McpsDataConfirmParams confirmParams;
                confirmParams.m_status = IEEE_802_15_4_NO_ACK;
                m_mcpsDataConfirmCallback(confirmParams);
            }

        } else {
            // Maximum number of retransmissions has been reached.
            // remove the copy of the DATA packet that was just sent
            Ptr<TxQueueElement> txQElement = m_txQueue.front();
            m_macTxDropTrace(txQElement->txQPkt);
            if (!m_mcpsDataConfirmCallback.IsNull())
            {
                McpsDataConfirmParams confirmParams;
                confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                confirmParams.m_status = IEEE_802_15_4_NO_ACK;
                m_mcpsDataConfirmCallback(confirmParams);
            }
        }

        if (!m_gtsEvent.IsRunning() && !m_incGtsEvent.IsRunning()) {
            RemoveFirstTxQElement();
        }
        
        return false;
    }
    else
    {
        m_retransmission++;

        if (m_lrWpanMacState != MAC_GTS_SENDING) {
            m_numCsmacaRetry += m_csmaCa->GetNB() + 1;        
            // Start next CCA process for this packet.
        }

        return true;
    }
}

void
LrWpanMac::EnqueueInd(Ptr<Packet> p)
{
    Ptr<IndTxQueueElement> indTxQElement = Create<IndTxQueueElement>();
    LrWpanMacHeader peekedMacHdr;
    p->PeekHeader(peekedMacHdr);

    PurgeInd();

    NS_ASSERT(peekedMacHdr.GetDstAddrMode() == SHORT_ADDR ||
              peekedMacHdr.GetDstAddrMode() == EXT_ADDR);

    if (peekedMacHdr.GetDstAddrMode() == SHORT_ADDR)
    {
        indTxQElement->dstShortAddress = peekedMacHdr.GetShortDstAddr();
    }
    else
    {
        indTxQElement->dstExtAddress = peekedMacHdr.GetExtDstAddr();
    }

    indTxQElement->seqNum = peekedMacHdr.GetSeqNum();

    // See IEEE 802.15.4-2006, Table 86
    uint32_t unit = 0; // The persistence time in symbols
    if (m_macBeaconOrder == 15)
    {
        // Non-beacon enabled mode
        unit = aBaseSuperframeDuration * m_macTransactionPersistenceTime;
    }
    else
    {
        // Beacon-enabled mode
        unit = ((static_cast<uint32_t>(1) << m_macBeaconOrder) * aBaseSuperframeDuration) *
               m_macTransactionPersistenceTime;
    }

    if (m_indTxQueue.size() < m_maxIndTxQueueSize)
    {
        double symbolRate = m_phy->GetDataOrSymbolRate(false);
        Time expireTime = Seconds(unit / symbolRate);
        expireTime += Simulator::Now();
        indTxQElement->expireTime = expireTime;
        indTxQElement->txQPkt = p;
        m_indTxQueue.emplace_back(indTxQElement);
        m_macIndTxEnqueueTrace(p);
    }
    else
    {
        if (!m_mlmeCommStatusIndicationCallback.IsNull())
        {
            LrWpanMacHeader peekedMacHdr;
            indTxQElement->txQPkt->PeekHeader(peekedMacHdr);
            MlmeCommStatusIndicationParams commStatusParams;
            commStatusParams.m_panId = m_macPanId;
            commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
            commStatusParams.m_srcExtAddr = peekedMacHdr.GetExtSrcAddr();
            commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
            commStatusParams.m_dstExtAddr = peekedMacHdr.GetExtDstAddr();
            commStatusParams.m_status = MLMECOMMSTATUS_TRANSACTION_OVERFLOW;
            m_mlmeCommStatusIndicationCallback(commStatusParams);
        }
        m_macIndTxDropTrace(p);
    }
}

bool
LrWpanMac::DequeueInd(Mac16Address dst, Ptr<IndTxQueueElement> entry)
{
    PurgeInd();

    for (auto iter = m_indTxQueue.begin(); iter != m_indTxQueue.end(); iter++)
    {
        if ((*iter)->dstShortAddress == dst)
        {
            *entry = **iter;
            m_macIndTxDequeueTrace((*iter)->txQPkt->Copy());
            m_indTxQueue.erase(iter);
            return true;
        }
    }
    return false;
}

bool
LrWpanMac::DequeueInd(Mac64Address dst, Ptr<IndTxQueueElement> entry)
{
    PurgeInd();

    for (auto iter = m_indTxQueue.begin(); iter != m_indTxQueue.end(); iter++)
    {
        if ((*iter)->dstExtAddress == dst)
        {
            *entry = **iter;
            m_macIndTxDequeueTrace((*iter)->txQPkt->Copy());
            m_indTxQueue.erase(iter);
            return true;
        }
    }
    return false;
}

bool LrWpanMac::SearchIndQueueElement(Mac16Address dst, const Ptr<IndTxQueueElement> entry) {
    for (auto iter = m_indTxQueue.begin(); iter != m_indTxQueue.end(); iter++) {
        if ((*iter)->dstShortAddress == dst) {
            *entry = **iter;
            return true;
        }
    }

    return false;
}

bool LrWpanMac::SearchIndQueueElement(Mac64Address dst, const Ptr<IndTxQueueElement> entry) {
    for (auto iter = m_indTxQueue.begin(); iter != m_indTxQueue.end(); iter++) {
        if ((*iter)->dstExtAddress == dst) {
            *entry = **iter;
            return true;
        }
    }

    return false;
}

void
LrWpanMac::PurgeInd()
{
    for (uint32_t i = 0; i < m_indTxQueue.size();)
    {
        if (Simulator::Now() > m_indTxQueue[i]->expireTime)
        {
            // Transaction expired, remove and send proper confirmation/indication to a higher layer
            LrWpanMacHeader peekedMacHdr;
            m_indTxQueue[i]->txQPkt->PeekHeader(peekedMacHdr);

            if (peekedMacHdr.IsCommand())
            {
                // IEEE 802.15.4-2006 (Section 7.1.3.3.3)
                if (!m_mlmeCommStatusIndicationCallback.IsNull())
                {
                    MlmeCommStatusIndicationParams commStatusParams;
                    commStatusParams.m_panId = m_macPanId;
                    commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                    commStatusParams.m_srcExtAddr = peekedMacHdr.GetExtSrcAddr();
                    commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                    commStatusParams.m_dstExtAddr = peekedMacHdr.GetExtDstAddr();
                    commStatusParams.m_status =
                        LrWpanMlmeCommStatus::MLMECOMMSTATUS_TRANSACTION_EXPIRED;
                    m_mlmeCommStatusIndicationCallback(commStatusParams);
                }
            }
            else if (peekedMacHdr.IsData())
            {
                // IEEE 802.15.4-2006 (Section 7.1.1.1.3)
                if (!m_mcpsDataConfirmCallback.IsNull())
                {
                    McpsDataConfirmParams confParams;
                    confParams.m_status = IEEE_802_15_4_TRANSACTION_EXPIRED;
                    m_mcpsDataConfirmCallback(confParams);
                }
            }
            m_macIndTxDropTrace(m_indTxQueue[i]->txQPkt->Copy());
            m_indTxQueue.erase(m_indTxQueue.begin() + i);
        }
        else
        {
            i++;
        }
    }
}

void
LrWpanMac::PrintPendTxQ(std::ostream& os) const
{
    LrWpanMacHeader peekedMacHdr;

    os << "Pending Transaction List [" << GetShortAddress() << " | " << GetExtendedAddress()
       << "] | CurrentTime: " << Simulator::Now().As(Time::S) << "\n"
       << "       Destination        | Sequence Number |   Frame type    | Expire time\n";

    for (uint32_t i = 0; i < m_indTxQueue.size(); i++)
    {
        m_indTxQueue[i]->txQPkt->PeekHeader(peekedMacHdr);
        os << m_indTxQueue[i]->dstExtAddress << "           "
           << static_cast<uint32_t>(m_indTxQueue[i]->seqNum) << "          ";

        if (peekedMacHdr.IsCommand())
        {
            os << "Cmd Frame    ";
        }
        else if (peekedMacHdr.IsData())
        {
            os << "Data Frame   ";
        }
        else
        {
            os << "Unk Frame    ";
        }

        os << m_indTxQueue[i]->expireTime.As(Time::S) << "\n";
    }
}

void
LrWpanMac::PrintTxQueue(std::ostream& os) const
{
    LrWpanMacHeader peekedMacHdr;

    os << "\nTx Queue [" << GetShortAddress() << " | " << GetExtendedAddress()
       << "] | CurrentTime: " << Simulator::Now().As(Time::S) << "\n"
       << "       Destination                 | Sequence Number |  Dst PAN id | Frame type    |\n";

    for (uint32_t i = 0; i < m_txQueue.size(); i++)
    {
        m_txQueue[i]->txQPkt->PeekHeader(peekedMacHdr);

        os << "[" << peekedMacHdr.GetShortDstAddr() << "]"
           << ", [" << peekedMacHdr.GetExtDstAddr() << "]        "
           << static_cast<uint32_t>(peekedMacHdr.GetSeqNum()) << "               "
           << peekedMacHdr.GetDstPanId() << "          ";

        if (peekedMacHdr.IsCommand())
        {
            os << "Cmd Frame    ";
        }
        else if (peekedMacHdr.IsData())
        {
            os << "Data Frame   ";
        }
        else
        {
            os << "Unk Frame    ";
        }

        os << "\n";
    }
    os << "\n";
}

void
LrWpanMac::RemovePendTxQElement(Ptr<Packet> p)
{
    LrWpanMacHeader peekedMacHdr;
    p->PeekHeader(peekedMacHdr);

    for (auto it = m_indTxQueue.begin(); it != m_indTxQueue.end(); it++)
    {
        if (peekedMacHdr.GetDstAddrMode() == EXT_ADDR)
        {
            if (((*it)->dstExtAddress == peekedMacHdr.GetExtDstAddr()) &&
                ((*it)->seqNum == peekedMacHdr.GetSeqNum()))
            {
                m_macIndTxDequeueTrace(p);
                m_indTxQueue.erase(it);
                break;
            }
        }
        else if (peekedMacHdr.GetDstAddrMode() == SHORT_ADDR)
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

    LrWpanMacHeader macHdr;
    Time ifsWaitTime;
    double symbolRate;

    // std::cout << "PdDataConfirm" << std::endl;

    symbolRate = m_phy->GetDataOrSymbolRate(false); // symbols per second

    m_txPkt->PeekHeader(macHdr);

    if (status == IEEE_802_15_4_PHY_SUCCESS) {
        if (!macHdr.IsAcknowledgment()) {
            if (macHdr.IsBeacon()) {
                // Start CAP only if we are in beacon mode (i.e. if slotted csma-ca is running)
                if (m_csmaCa->IsSlottedCsmaCa()) {
                    // The Tx Beacon in symbols
                    // Beacon = 5 bytes Sync Header (SHR) +  1 byte PHY header (PHR) + PSDU (default
                    // 17 bytes)
                    uint64_t beaconSymbols = m_phy->GetPhySHRDuration() +
                                             1 * m_phy->GetPhySymbolsPerOctet() +
                                             (m_txPkt->GetSize() * m_phy->GetPhySymbolsPerOctet());

                    // The beacon Tx time and start of the Outgoing superframe Active Period
                    m_macBeaconTxTime =
                        Simulator::Now() - MilliSeconds(static_cast<double>(beaconSymbols) / symbolRate);
                    
                    PurgeDsmeACT();

                    // DSME-TODO
                    // For Dsme net device setting use only 
                    // ScheduleGtsSyncToCoord(m_choosedSDIndexToSendBcn);

                    // ScheduleGts(false);

                    if (m_macDSMEenabled && m_macCAPReductionFlag) {
                        if (m_macSDindex % (m_multiSuperframeDuration / m_superframeDuration) == 0) {
                            m_capEvent = Simulator::ScheduleNow(&LrWpanMac::StartCAP,
                                                                this,
                                                                SuperframeType::OUTGOING); 
                        } else {
                            m_firstCFP = true;
                            m_cfpEvent = Simulator::ScheduleNow(&LrWpanMac::StartCFP,
                                                                this,
                                                                SuperframeType::OUTGOING);      
                        }

                    } else {
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

            } else if (macHdr.IsAckReq()) { 
                // We have sent a regular data packet, check if we have to
                // wait  for an ACK.
            
                // we sent a regular data frame or command frame (e.g. AssocReq command) that
                // require ACK wait for the ack or the next retransmission timeout start
                // retransmission timer

                Time waitTime = Seconds(static_cast<double>(GetMacAckWaitDuration()) / symbolRate);

                // DSME
                if (macHdr.IsCommand()) {
                    Ptr<Packet> txOriginalPkt = m_txPkt->Copy();
                    LrWpanMacHeader txMacHdr;
                    txOriginalPkt->RemoveHeader(txMacHdr);
                    CommandPayloadHeader txMacPayload;
                    txOriginalPkt->RemoveHeader(txMacPayload);

                    if (txMacPayload.GetCommandFrameType() == CommandPayloadHeader::DISASSOCIATION_NOTIF) {
                        if (m_disassociateParams.m_disassociateReason == CommandPayloadHeader::DISASSC_DEV_LEAVE_PAN) {
                            RemoveReferencesToPAN();
                        }

                    } else if (txMacPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_GTS_REQ) {
                        NS_ASSERT(m_dsmeGtsAckWaitTimeout.IsExpired());
                        m_dsmeGtsAckWaitTimeout = Simulator::Schedule(waitTime
                                                                      , &LrWpanMac::DsmeGtsAckWaitTimeout
                                                                      , this);

                    } else if (txMacPayload.GetCommandFrameType() == CommandPayloadHeader::COOR_REALIGN) {
                        // DSME-TODO
                        // the MLME updates the PIB attributes BeaconOrder
                        // , SuperframeOrder, PANId, ChannelPage, and ChannelNumber parameters.z

                    } else if (txMacPayload.GetCommandFrameType() == CommandPayloadHeader::DSME_INFO_REQ) {
                        NS_ASSERT(m_dsmeInfoAckWaitTimeout.IsExpired());
                        m_dsmeInfoAckWaitTimeout = Simulator::Schedule(waitTime
                                                                       , &LrWpanMac::DsmeInfoAckWaitTimeout
                                                                       , this);
                    }
                
                // For dsme-net-device-throughput... testing usage
                } else if (macHdr.IsData() && (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning())) {
                    NS_LOG_DEBUG("Successfully sent a data packet during a GTS period.");

                    if (!m_mcpsDataConfirmCallback.IsNull()) {
                        McpsDataConfirmParams confirmParams;
                        confirmParams.m_msduHandle = m_mcpsDataRequestParams.m_msduHandle;
                        confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                        m_mcpsDataConfirmCallback(confirmParams);
                    }

                } else if (macHdr.IsData()) {
                    std::cout << Simulator::Now().GetNanoSeconds() << ", " << GetShortAddress() << " successfully sent the data packet" << std::endl;

                    // (*m_record)[GetShortAddress()] = {macHdr.GetShortDstAddr(), {Simulator::Now().GetNanoSeconds()}};
                    if (m_record != nullptr) {
                        (*m_record)[GetShortAddress()] = {macHdr.GetShortDstAddr(), {}};
                        (*m_record)[GetShortAddress()].second.push_back(Simulator::Now().GetNanoSeconds());
                    }
                }
                
                // Time waitTime = Seconds(static_cast<double>(GetMacAckWaitDuration()) / symbolRate);
                NS_ASSERT(m_ackWaitTimeout.IsExpired());
                m_ackWaitTimeout = Simulator::Schedule(waitTime, &LrWpanMac::AckWaitTimeout, this);
                m_setMacState.Cancel();
                m_setMacState =
                    Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_ACK_PENDING);

                return;
                
            } else if (macHdr.IsCommand()) {
                // We handle commands that do not require ACK (e.g. BeaconReq command)
                // other command are handle by the previous if statement.
                // Broadcasted coordinator realignment command would meet this.
                Ptr<Packet> txOriginalPkt = m_txPkt->Copy();
                LrWpanMacHeader txMacHdr;
                txOriginalPkt->RemoveHeader(txMacHdr);
                CommandPayloadHeader txMacPayload;
                txOriginalPkt->RemoveHeader(txMacPayload);

                if (txMacPayload.GetCommandFrameType() == CommandPayloadHeader::COOR_REALIGN) {
                    // DSME-TODO
                    // the new superframe configuration and channel parameters 
                    // shall be put into operation
                    m_macPanId = m_startParams.m_PanId;

                    m_pendPrimitive = MLME_START_REALGN;

                    LrWpanPhyPibAttributes pibAttr;
                    pibAttr.phyCurrentPage = m_startParams.m_logChPage;
                    m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentPage, &pibAttr);

                    if (!m_panCoor) {
                        m_sendBcn = true;
                    }

                    if (!m_mlmeStartConfirmCallback.IsNull()) {
                        MlmeStartConfirmParams mlmeConfirmParams;
                        mlmeConfirmParams.m_status = MLMESTART_SUCCESS;
                        m_mlmeStartConfirmCallback(mlmeConfirmParams);
                    }
                }
                
                RemoveFirstTxQElement();
            
            } else if (macHdr.IsData() && (m_gtsEvent.IsRunning() || m_incGtsEvent.IsRunning())) {
                // Do nothing because Acknowledgment is not required
                NS_LOG_DEBUG("Successfully sent a data packet during a GTS period.");

            } else {
                m_macTxOkTrace(m_txPkt);
                // remove the copy of the packet that was just sent
                if (!m_mcpsDataConfirmCallback.IsNull())
                {
                    McpsDataConfirmParams confirmParams;
                    NS_ASSERT_MSG(m_txQueue.size() > 0, "TxQsize = 0");
                    Ptr<TxQueueElement> txQElement = m_txQueue.front();
                    confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                    confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                    m_mcpsDataConfirmCallback(confirmParams);
                }
                ifsWaitTime = Seconds(static_cast<double>(GetIfsSize()) / symbolRate);
                RemoveFirstTxQElement();
            }

        } else {
            // The packet sent was a successful ACK

            // Check the received frame before the transmission of the ACK,
            // and send the appropriate Indication or Confirmation
            Ptr<Packet> recvOriginalPkt = m_rxPkt->Copy();
            LrWpanMacHeader receivedMacHdr;
            recvOriginalPkt->RemoveHeader(receivedMacHdr);

            if (receivedMacHdr.IsCommand())
            {
                CommandPayloadHeader receivedMacPayload;
                recvOriginalPkt->RemoveHeader(receivedMacPayload);
                
                if (receivedMacPayload.GetCommandFrameType() ==
                    CommandPayloadHeader::ASSOCIATION_REQ)
                {
                    if (!m_mlmeAssociateIndicationCallback.IsNull())
                    {
                        MlmeAssociateIndicationParams associateParams;
                        associateParams.capabilityInfo = receivedMacPayload.GetCapabilityField();
                        associateParams.m_extDevAddr = receivedMacHdr.GetExtSrcAddr();
                        m_mlmeAssociateIndicationCallback(associateParams);
                    }

                    // Clear the packet buffer for the packet request received.
                    m_rxPkt = nullptr;

                } else if (receivedMacPayload.GetCommandFrameType() ==
                    CommandPayloadHeader::DSME_ASSOCIATION_REQ) {
                    // DSME
                    if (!m_mlmeAssociateIndicationCallback.IsNull()) {
                        MlmeAssociateIndicationParams associateParams;
                        associateParams.capabilityInfo = receivedMacPayload.GetCapabilityField();
                        associateParams.m_extDevAddr = receivedMacHdr.GetExtSrcAddr();
                        associateParams.m_channelOfs = receivedMacPayload.GetChannelOfs();
                        associateParams.m_hoppingSeqID = receivedMacPayload.GetHoppingSeqID();

                        m_mlmeAssociateIndicationCallback(associateParams);
                    }

                } else if (receivedMacPayload.GetCommandFrameType() == 
                            CommandPayloadHeader::ASSOCIATION_RESP 
                         || receivedMacPayload.GetCommandFrameType() ==
                         CommandPayloadHeader::DSME_ASSOCIATION_RESP) {
                    // DSME
                    MlmeAssociateConfirmParams confirmParams;

                    switch (receivedMacPayload.GetAssociationStatus()) {
                        case CommandPayloadHeader::SUCCESSFUL:
                            confirmParams.m_status =
                                LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_SUCCESS;
                            confirmParams.m_assocShortAddr =
                                GetShortAddress(); // the original short address used in the association
                                                // request
                            SetShortAddress(
                                receivedMacPayload
                                    .GetShortAddr()); // the assigned short address by the coordinator

                            m_macPanId = receivedMacHdr.GetSrcPanId();
                            m_macChannelOfs = receivedMacPayload.GetChannelOfs();

                            // DSME: Dsme association response command
                            // DSME-TODO
                            if (m_macDSMEenabled && m_macHoppingEnabled) {
                                if (receivedMacPayload.GetHoppingSeqLen() > 0) {
                                    SetHoppingSeq(receivedMacPayload.GetHoppingSequnce());
                                }

                                SetHoppingSeqLen(receivedMacPayload.GetHoppingSeqLen());
                            }

                            break;

                        case CommandPayloadHeader::FULL_CAPACITY:
                            confirmParams.m_status =
                                LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_FULL_CAPACITY;
                            m_macPanId = 0xffff;
                            m_macCoordShortAddress = Mac16Address("FF:FF");
                            m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                            m_incCapEvent.Cancel();
                            m_incCfpEvent.Cancel();
                            m_csmaCa->SetUnSlottedCsmaCa();
                            m_incomingBeaconOrder = 15;
                            m_incomingSuperframeOrder = 15;
                            break;

                        case CommandPayloadHeader::ACCESS_DENIED:
                            confirmParams.m_status =
                                // LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_ACCESS_DENIED;
                                LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_CHANNEL_ACCESS_FAILURE;
                            m_macPanId = 0xffff;
                            m_macCoordShortAddress = Mac16Address("FF:FF");
                            m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                            m_incCapEvent.Cancel();
                            m_incCfpEvent.Cancel();
                            m_csmaCa->SetUnSlottedCsmaCa();
                            m_incomingBeaconOrder = 15;
                            m_incomingSuperframeOrder = 15;
                            break;
                        
                        case CommandPayloadHeader::HOPPING_SEQ_OFS_DUPLICATION:
                            confirmParams.m_status =
                                LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_INVALID_PARAMETER;
                            m_macPanId = 0xffff;
                            m_macCoordShortAddress = Mac16Address("FF:FF");
                            m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                            m_incCapEvent.Cancel();
                            m_incCfpEvent.Cancel();
                            m_csmaCa->SetUnSlottedCsmaCa();
                            m_incomingBeaconOrder = 15;
                            m_incomingSuperframeOrder = 15;     
                            break;
                                
                        case CommandPayloadHeader::FASTA_SUCCESSFUL:
                            confirmParams.m_status =
                                LrWpanMlmeAssociateConfirmStatus::MLMEASSOC_SUCCESS;
                            confirmParams.m_assocShortAddr =
                                GetShortAddress(); // the original short address used in the association
                                                // request
                            SetShortAddress(
                                receivedMacPayload
                                    .GetShortAddr()); // the assigned short address by the coordinator

                            m_macPanId = receivedMacHdr.GetSrcPanId();
                            m_macChannelOfs = receivedMacPayload.GetChannelOfs();

                            if (m_macDSMEenabled && m_macHoppingEnabled) {
                                if (receivedMacPayload.GetHoppingSeqLen() > 0) {
                                    SetHoppingSeq(receivedMacPayload.GetHoppingSequnce());
                                }

                                SetHoppingSeqLen(receivedMacPayload.GetHoppingSeqLen());
                            }

                            break;

                        case CommandPayloadHeader::RESERVED:
                            NS_LOG_ERROR("CommandPayloadHeader::RESERVED encounterd, which is not legit");
                            break;
                    }

                    if (!m_mlmeAssociateConfirmCallback.IsNull())
                    {
                        m_mlmeAssociateConfirmCallback(confirmParams);
                    }

                } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DISASSOCIATION_NOTIF) {
                    if (!m_mlmeDisassociateIndicationCallback.IsNull()) {
                        MlmeDisassociateIndicationParams disassociateParams;
                        disassociateParams.m_extDevAddr = receivedMacHdr.GetExtSrcAddr();
                        disassociateParams.m_disassociateReason = receivedMacPayload.GetDisassociationReason();
                        m_mlmeDisassociateIndicationCallback(disassociateParams);
                    }
                    
                } else if (receivedMacPayload.GetCommandFrameType() == CommandPayloadHeader::DATA_REQ) {
                    // We enqueue the the Assoc Response command frame in the Tx queue
                    // and the packet is transmitted as soon as the PHY is free and the IFS have
                    // taken place.
                    // SendAssocResponseCommand(m_rxPkt->Copy());

                    /**
                     * 應該先檢查 pending transcation list 有沒有相對應的 address 
                     * 而不是直接認定是要回傳 association response command frame.
                     */
                    // DSME-TODO
                    Ptr<IndTxQueueElement> indTxQElement = Create<IndTxQueueElement>();
                    bool elementFound = true;
                    // std::cout << receivedMacHdr.GetExtSrcAddr() << std::endl;
                    // PrintPendTxQ(std::cout);
                    if (receivedMacHdr.GetSrcAddrMode() == LrWpanMacHeader::EXTADDR) {
                        elementFound = SearchIndQueueElement(receivedMacHdr.GetExtSrcAddr(), indTxQElement);
                    } else {
                        elementFound = SearchIndQueueElement(receivedMacHdr.GetShortSrcAddr(), indTxQElement);
                    }

                    // PrintPendTxQ(std::cout); // debug

                    if (elementFound) {
                        LrWpanMacTrailer peekedTrailer;
                        LrWpanMacHeader peekedMacHdr;
                        CommandPayloadHeader peekedPayloadHdr;

                        indTxQElement->txQPkt->RemoveTrailer(peekedTrailer);
                        indTxQElement->txQPkt->RemoveHeader(peekedMacHdr);
                        indTxQElement->txQPkt->PeekHeader(peekedPayloadHdr);

                        indTxQElement->txQPkt->AddHeader(peekedMacHdr);
                        indTxQElement->txQPkt->AddTrailer(peekedTrailer);
                        
                        if (peekedPayloadHdr.GetCommandFrameType() == CommandPayloadHeader::ASSOCIATION_RESP) {
                            SendAssocResponseCommand(m_rxPkt->Copy());

                        } else if (peekedPayloadHdr.GetCommandFrameType() == CommandPayloadHeader::DSME_ASSOCIATION_RESP
                                  && m_macDSMEenabled) {
                            SendDsmeAssocResponseCommand(m_rxPkt->Copy());

                        } else if (peekedPayloadHdr.GetCommandFrameType() == CommandPayloadHeader::DISASSOCIATION_NOTIF) {
                            SendDisassocNotificationCommandIndirect(m_rxPkt->Copy());

                        } else if (peekedMacHdr.IsData()) {
                            // DSME-TODO
                            if (receivedMacHdr.GetSrcAddrMode() == LrWpanMacHeader::EXTADDR) {
                                elementFound = DequeueInd(receivedMacHdr.GetExtSrcAddr(), indTxQElement);
                            } else {
                                elementFound = DequeueInd(receivedMacHdr.GetShortSrcAddr(), indTxQElement);
                            }

                            if (elementFound) {
                                Ptr<TxQueueElement> txQElement = Create<TxQueueElement>();
                                txQElement->txQPkt = indTxQElement->txQPkt;
                                EnqueueTxQElement(txQElement);

                            } else {
                                NS_LOG_DEBUG("Requested element not found in pending list");
                            }
                        }

                    } else {
                        NS_LOG_DEBUG("Requested element not found in pending list");
                    }          

                } else if (receivedMacPayload.GetCommandFrameType() 
                            == CommandPayloadHeader::COOR_REALIGN) {
                    // DSME-TODO

                } else if (receivedMacPayload.GetCommandFrameType()
                            == CommandPayloadHeader::DSME_GTS_REQ) {
                    if (receivedMacPayload.GetDsmeGtsManagementField().GetManagementType()
                        == 0b000) {
                        if (!CheckDsmeGtsSABFromReqCmdWithDsmeACT(receivedMacPayload.GetDsmeGtsSABSpec())) {
                            // Send DSME-GTS reply command frame with management type = zero
                            // and the Status field shall be set to two (INVALID_PARAMETER)
                            SendDsmeGtsReplyWithInvalidParam(receivedMacHdr.GetShortSrcAddr(), 
                                                            receivedMacPayload.GetDsmeGtsSABSpec());

                            // Clear the packet buffer for the ACK packet sent.
                            m_txPkt = nullptr;   

                            return;
                        } 
                    }
                
                    if (!m_mlmeDsmeGtsIndicationCallback.IsNull()) {
                        MlmeDsmeGtsIndicationParams gtsIndicationParams;
                        
                        gtsIndicationParams.m_devAddr = receivedMacHdr.GetShortSrcAddr();
                        gtsIndicationParams.m_manageType = static_cast<ManagementType>(receivedMacPayload.GetDsmeGtsManagementField()
                                                                             .GetManagementType());

                        gtsIndicationParams.m_direction = receivedMacPayload.GetDsmeGtsManagementField()
                                                                            .IsDirectionRX();

                        gtsIndicationParams.m_prioritizedChAccess = receivedMacPayload.GetDsmeGtsManagementField()
                                                                                      .IsHighPriority();

                        if (receivedMacPayload.GetDsmeGtsManagementField().GetManagementType()
                            == 0b001) {
                            gtsIndicationParams.m_numSlot = receivedMacPayload.GetDsmeGtsNumOfSlot();
                            gtsIndicationParams.m_preferredSuperframeID = receivedMacPayload.GetDsmeGtsPreferredSuperframeId();
                            gtsIndicationParams.m_preferredSlotID = receivedMacPayload.GetDsmeGtsPreferredSlotId();
                        }
                        
                        gtsIndicationParams.m_dsmeSABSpec = receivedMacPayload.GetDsmeGtsSABSpec();

                        // gtsIndicationParams.m_dsmeSABSpec.Print(std::cout); // debug

                        // The higher layer will make the decision on the allocation//deallocation 
                        // using the value of DsmeSabSpecification parameter 
                        // as the slot availability information.
                        m_mlmeDsmeGtsIndicationCallback(gtsIndicationParams);
                    }

                } else if (receivedMacPayload.GetCommandFrameType()
                            == CommandPayloadHeader::DSME_GTS_NOTIFY) {   
                    // Deallocation
                    if (receivedMacPayload.GetDsmeGtsManagementField().GetManagementType() == 0b000) {
                        if (receivedMacPayload.GetDsmeGtsDestAddress() == GetShortAddress()) {
                            if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
                                MlmeCommStatusIndicationParams commStatusParams;
                                commStatusParams.m_panId = m_macPanId;
                                commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                                commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                                commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                                commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                                commStatusParams.m_status = MLMECOMMSTATUS_SUCCESS;

                                m_mlmeCommStatusIndicationCallback(commStatusParams);           
                            }

                        } else {
                            // he device shall update macDSMESAB according to the DSMESABSpecification 
                            // in this command frame to reflect the neighbor’s m_deallocated DSME-GTSs.

                            // DSME-TODO
                            DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();

                            if (partialSAB.isCAPReduction()) {
                                std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                    m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                        m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] ^ subBlk[i];                                 
                                }

                            } else {
                                std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                    m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                        m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] ^ subBlkCapOff[i];   
                                }
                            }
                        }

                    } else if (receivedMacPayload.GetDsmeGtsManagementField().GetManagementType() == 0b001) {
                        if (receivedMacPayload.GetDsmeGtsDestAddress() == GetShortAddress()) {
                            if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
                                MlmeCommStatusIndicationParams commStatusParams;
                                commStatusParams.m_panId = m_macPanId;
                                commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                                commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                                commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                                commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                                commStatusParams.m_status = MLMECOMMSTATUS_SUCCESS;

                                m_mlmeCommStatusIndicationCallback(commStatusParams);           
                            }

                            // The device shall update macDsmeAct according the value of 
                            // DSME SAB Specification field of the DSME GTS Notify command.

                            // DSME-TODO
                            // Update()
                            // The device shall update macDsmeAct according the value of DSME SAB Specification 
                            // field of the DSME GTS Notify command.
                            macDSMEACTEntity entity;
                            entity.m_superframeID = m_gtsSuperframeIDs.back();
                            entity.m_slotID = std::get<0>(m_gtsStartAndLens.back());
                            entity.m_numSlot = std::get<1>(m_gtsStartAndLens.back());

                            // DSME-TODO
                            // entity.m_channelID = ;

                            entity.m_direction = !receivedMacPayload.GetDsmeGtsManagementField()
                                                                                .IsDirectionRX();
                            entity.m_type = 0x00;

                            entity.m_prioritizedChAccess = receivedMacPayload.GetDsmeGtsManagementField()
                                                                                        .IsHighPriority();
                            if (entity.m_direction) {
                                entity.m_dstAddr = receivedMacHdr.GetShortSrcAddr();
                            } else {
                                entity.m_srcAddr = receivedMacHdr.GetShortSrcAddr();
                            }                                                 
                            
                            entity.m_cnt = 0;

                            // DSME-TODO
                            // entity.m_linkQuality = ;
                            m_macDsmeACT[m_gtsSuperframeIDs.back()].push_back(entity);

                            // DSME-TODO      
                            DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();

                            if (partialSAB.isCAPReduction()) {
                                std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                    m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                        m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] | subBlk[i];                                 
                                }  

                            } else {
                                std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                    m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                        m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] | subBlkCapOff[i];   
                                }
                            }

                        } else {
                            // device shall check if the slots marked as one in this command 
                            // is conflicting with the readily allocated slots in macDsmeAct

                            if (CheckDsmeGtsSABAndDsmeACTConflict(receivedMacPayload.GetDsmeGtsSABSpec())) {
                                // the device shall send a DSME GTS Request command 
                                // with Management Type field set to duplicated allocation notification
                                // DSME-TODO

                            } else {
                                // the device shall update macDSMESAB according to the DSMESABSpecification 
                                // in this command frame to reflect the neighbor’s allocated DSME-GTSs.
                                
                                // DSME-TODO      
                                DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();

                                if (partialSAB.isCAPReduction()) {
                                    std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] | subBlk[i];                                 
                                    }  

                                } else {
                                    std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] | subBlkCapOff[i];   
                                    }
                                }
                            }
                        }
                    }
                                   
                } else if (receivedMacPayload.GetCommandFrameType()
                            == CommandPayloadHeader::DSME_GTS_REPLY) {
                    // Deallocation
                    if (receivedMacPayload.GetDsmeGtsManagementField().GetManagementType() == 0b000) {
                        if (receivedMacPayload.GetDsmeGtsDestAddress() == GetShortAddress()) {
                            if (!m_mlmeDsmeGtsConfirmCallback.IsNull()) {
                                MlmeDsmeGtsConfirmParams confirmParams;
    
                                confirmParams.m_devAddr = receivedMacHdr.GetShortSrcAddr();
                                confirmParams.m_manageType = static_cast<ManagementType>(receivedMacPayload.GetDsmeGtsManagementField()
                                                                            .GetManagementType());

                                confirmParams.m_direction = receivedMacPayload.GetDsmeGtsManagementField()
                                                                            .IsDirectionRX();

                                confirmParams.m_prioritizedChAccess = receivedMacPayload.GetDsmeGtsManagementField()
                                                                                        .IsHighPriority();
                                                                                        
                                confirmParams.m_channelOfs = receivedMacPayload.GetChannelOfs();
                                confirmParams.m_dsmeSABSpec = receivedMacPayload.GetDsmeGtsSABSpec();
                                confirmParams.m_status = static_cast<LrWpanMlmeDsmeGtsRequestStatus>(receivedMacPayload
                                                                                                    .GetDsmeGtsManagementField()
                                                                                                    .GetStatus());

                                m_mlmeDsmeGtsConfirmCallback(confirmParams);
                            }  

                            if (receivedMacPayload.GetDsmeGtsManagementField().GetStatus() == 0b000) {
                                SendDsmeGtsNotifyCommand(receivedMacHdr.GetShortSrcAddr(), receivedMacPayload);
                                UpdateDsmeACTAndDeallocate(receivedMacPayload.GetDsmeGtsSABSpec());

                                // DSME-TODO
                                DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();

                                if (partialSAB.isCAPReduction()) {
                                    std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] ^ subBlk[i];                                 
                                    }

                                } else {
                                    std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] ^ subBlkCapOff[i];   
                                    }
                                }
                            }

                        } else {
                            if (receivedMacPayload.GetDsmeGtsManagementField().GetStatus() == 0b000){
                                // the device shall update macDSMESAB according to the DSMESABSpecification 
                                // in this command frame to reflect the neighbor’s m_deallocated DSME-GTSs

                                // DSME-TODO
                                DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();

                                if (partialSAB.isCAPReduction()) {
                                    std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] ^ subBlk[i];                                 
                                    }
                                    
                                } else {
                                    std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] ^ subBlkCapOff[i];   
                                    }    
                                }
                            }
                        }
                    // allocation
                    } else if (receivedMacPayload.GetDsmeGtsManagementField().GetManagementType() == 0b001) {
                        if (receivedMacPayload.GetDsmeGtsDestAddress() == GetShortAddress()) {
                            if (!m_mlmeDsmeGtsConfirmCallback.IsNull()) {
                                MlmeDsmeGtsConfirmParams confirmParams;
    
                                confirmParams.m_devAddr = receivedMacHdr.GetShortSrcAddr();
                                confirmParams.m_manageType = static_cast<ManagementType>(receivedMacPayload.GetDsmeGtsManagementField()
                                                                            .GetManagementType());

                                confirmParams.m_direction = receivedMacPayload.GetDsmeGtsManagementField()
                                                                            .IsDirectionRX();

                                confirmParams.m_prioritizedChAccess = receivedMacPayload.GetDsmeGtsManagementField()
                                                                                        .IsHighPriority();
                                                                                        
                                confirmParams.m_channelOfs = receivedMacPayload.GetChannelOfs();
                                confirmParams.m_dsmeSABSpec = receivedMacPayload.GetDsmeGtsSABSpec();
                                confirmParams.m_status = static_cast<LrWpanMlmeDsmeGtsRequestStatus>(receivedMacPayload
                                                                                                    .GetDsmeGtsManagementField()
                                                                                                    .GetStatus());

                                m_mlmeDsmeGtsConfirmCallback(confirmParams);
                            }  

                            if (receivedMacPayload.GetDsmeGtsManagementField().GetStatus() == 0b000) {
                                SendDsmeGtsNotifyCommand(receivedMacHdr.GetShortSrcAddr(), receivedMacPayload);

                                m_gtsDirections.push_back(receivedMacPayload.GetDsmeGtsManagementField()
                                                                            .IsDirectionRX());

                                CheckDsmeGtsSABFromReplyCmd(receivedMacPayload.GetDsmeGtsSABSpec());

                                // NS_LOG_DEBUG(" GTS Superframe ID: " << m_gtsSuperframeIDs.back() << std::endl
                                //             << " Direction: " << m_gtsDirections.back() << std::endl
                                //             << " Slot ID: " << std::get<0>(m_gtsStartAndLens.back()) << std::endl
                                //             << " GTS Length:  " << std::get<1>(m_gtsStartAndLens.back()) << std::endl);

                                // the device shall update macDsmeAct according to 
                                // the DSME SAB Specification field in the received command.
                                // DSME-TODO
                                macDSMEACTEntity entity;
                                entity.m_superframeID = m_gtsSuperframeIDs.back();
                                entity.m_slotID = std::get<0>(m_gtsStartAndLens.back());
                                entity.m_numSlot = std::get<1>(m_gtsStartAndLens.back());

                                // DSME-TODO
                                // entity.m_channelID = ;

                                entity.m_direction = receivedMacPayload.GetDsmeGtsManagementField()
                                                                                .IsDirectionRX();
                                entity.m_type = 0x00;

                                entity.m_prioritizedChAccess = receivedMacPayload.GetDsmeGtsManagementField()
                                                                                            .IsHighPriority();
                                if (entity.m_direction) {
                                    entity.m_srcAddr = receivedMacHdr.GetShortSrcAddr();
                                } else {
                                    entity.m_dstAddr = receivedMacHdr.GetShortSrcAddr();
                                }                                                 
                                
                                entity.m_cnt = 0;

                                // DSME-TODO
                                // entity.m_linkQuality = ;
                                m_macDsmeACT[m_gtsSuperframeIDs.back()].push_back(entity);
                                
                                m_macChannelOfs = receivedMacPayload.GetChannelOfs();

                                // DSME-TODO      
                                DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();
                                                  
                                if (partialSAB.isCAPReduction()) {
                                    std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();
                    
                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] | subBlk[i];                                 
                                    }

                                } else {
                                    std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                    for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                        m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                            m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] | subBlkCapOff[i];   
                                    }
                                }
                            } 

                        } else {
                            if (receivedMacPayload.GetDsmeGtsManagementField().GetStatus() == 0b000) {
                                // the device shall check if the slots marked as one in the command 
                                // is conflicting with the readily allocated slots in macDsmeAct

                                // DSME-TODO
                                if (CheckDsmeGtsSABAndDsmeACTConflict(receivedMacPayload.GetDsmeGtsSABSpec())) {
                                    // the device shall send a DSME GTS Request command 
                                    // with Management Type field set to duplicated allocation notification

                                } else {    
                                    // update macDsmeSab
                                    DSMESABSpecificationField partialSAB = receivedMacPayload.GetDsmeGtsSABSpec();
                                                               
                                    if (partialSAB.isCAPReduction()) {
                                        std::vector<uint16_t> subBlk = partialSAB.GetSABSubBlk();

                                        for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                            m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] = 
                                                m_macDSMESAB[partialSAB.GetSABSubBlkIdx() + i] | subBlk[i];                                 
                                        }

                                    } else {
                                        std::vector<uint8_t> subBlkCapOff = partialSAB.GetSABSubBlkCapOff();

                                        for (int i = 0; i < partialSAB.GetSABSubBlkLen(); ++i) {
                                            m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] = 
                                                m_macDSMESABCapOff[partialSAB.GetSABSubBlkIdx() + i] | subBlkCapOff[i];   
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Clear the packet buffer for the ACK packet sent.
            m_txPkt = nullptr;
        }
    }
    else if (status == IEEE_802_15_4_PHY_UNSPECIFIED)
    {
        if (!macHdr.IsAcknowledgment())
        {
            NS_ASSERT_MSG(m_txQueue.size() > 0, "TxQsize = 0");
            Ptr<TxQueueElement> txQElement = m_txQueue.front();
            m_macTxDropTrace(txQElement->txQPkt);
            if (!m_mcpsDataConfirmCallback.IsNull())
            {
                McpsDataConfirmParams confirmParams;
                confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                confirmParams.m_status = IEEE_802_15_4_FRAME_TOO_LONG;
                m_mcpsDataConfirmCallback(confirmParams);
            }
            RemoveFirstTxQElement();
        }
        else
        {
            NS_LOG_ERROR("Unable to send ACK");
        }
    }
    else
    {
        // Something went really wrong. The PHY is not in the correct state for
        // data transmission.
        NS_FATAL_ERROR("Transmission attempt failed with PHY status " << status);
    }

    if (!ifsWaitTime.IsZero())
    {
        m_ifsEvent =
            Simulator::Schedule(ifsWaitTime, &LrWpanMac::IfsWaitTimeout, this, ifsWaitTime);
    }

    m_setMacState.Cancel();
    m_setMacState = Simulator::ScheduleNow(&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);
}

void
LrWpanMac::PlmeCcaConfirm(LrWpanPhyEnumeration status)
{
    NS_LOG_FUNCTION(this << status);
    // Direct this call through the csmaCa object
    m_csmaCa->PlmeCcaConfirm(status);
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

    if (m_lrWpanMacState == MAC_SENDING &&
        (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
        NS_ASSERT(m_txPkt);

        // Start sending if we are in state SENDING and the PHY transmitter was enabled.
        m_promiscSnifferTrace(m_txPkt);
        m_snifferTrace(m_txPkt);
        m_macTxTrace(m_txPkt);
        
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
    if (id == LrWpanPibAttributeIdentifier::phyCurrentPage && m_pendPrimitive == MLME_SCAN_REQ)
    {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS)
        {
            // get the first channel to scan from scan channel list
            bool channelFound = false;
            for (int i = m_channelScanIndex; i <= 26; i++)
            {
                if ((m_scanParams.m_scanChannels & (1 << m_channelScanIndex)) != 0)
                {
                    channelFound = true;
                    break;
                }
                m_channelScanIndex++;
            }

            if (channelFound)
            {
                LrWpanPhyPibAttributes pibAttr;
                pibAttr.phyCurrentChannel = m_channelScanIndex;
                m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel,
                                               &pibAttr);
            }
        }
        else
        {
            if (!m_mlmeScanConfirmCallback.IsNull())
            {
                MlmeScanConfirmParams confirmParams;
                confirmParams.m_scanType = m_scanParams.m_scanType;
                confirmParams.m_chPage = m_scanParams.m_chPage;
                confirmParams.m_status = MLMESCAN_INVALID_PARAMETER;
                m_mlmeScanConfirmCallback(confirmParams);
            }
            NS_LOG_ERROR(this << "Channel Scan: Invalid channel page");
        }
    }
    else if (id == LrWpanPibAttributeIdentifier::phyCurrentChannel &&
             m_pendPrimitive == MLME_SCAN_REQ)
    {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS)
        {
            uint64_t symbolRate = static_cast<uint64_t>(m_phy->GetDataOrSymbolRate(false));
            uint64_t scanDuration = aBaseSuperframeDuration *
                                    ((static_cast<uint32_t>(1 << m_scanParams.m_scanDuration)) + 1);
            Time nextScanTime = Seconds(static_cast<double>(scanDuration /(double) symbolRate));
            Time respWaitTime = Seconds(static_cast<double>(m_macResponseWaitTime) /
                                                            (double) symbolRate);
            
            switch (m_scanParams.m_scanType) {
                case MLMESCAN_ED:
                    m_maxEnergyLevel = 0;
                    m_scanEnergyEvent =
                        Simulator::Schedule(nextScanTime, &LrWpanMac::EndChannelEnergyScan, this);
                    // set phy to RX_ON and kick start  the first PLME-ED.request
                    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
                    break;

                case MLMESCAN_ACTIVE:
                    m_scanEvent = Simulator::Schedule(nextScanTime, &LrWpanMac::EndChannelScan, this);
                    SendBeaconRequestCommand();
                    break;

                case MLMESCAN_PASSIVE:
                    m_scanEvent = Simulator::Schedule(nextScanTime, &LrWpanMac::EndChannelScan, this);
                    // turn back the phy to RX_ON after setting Page/channel
                    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
                    break;

                case MLMESCAN_ORPHAN:
                    // DSME-TODO: add orphan scan support
                    SendOrphanNotificationCmd();
                    m_scanEvent = Simulator::Schedule(respWaitTime, &LrWpanMac::EndOrphanScan, this);
                    break;

                case MLMESCAN_ENHANCED_ACTIVE_SCAN:
                    m_scanEvent = Simulator::Schedule(nextScanTime, &LrWpanMac::EndChannelScan, this);
                    SendEnhancedBeaconRequestCommand();
                    break;

                default:
                    MlmeScanConfirmParams confirmParams;
                    confirmParams.m_scanType = m_scanParams.m_scanType;
                    confirmParams.m_chPage = m_scanParams.m_chPage;
                    confirmParams.m_status = MLMESCAN_INVALID_PARAMETER;

                    if (!m_mlmeScanConfirmCallback.IsNull()) {
                        m_mlmeScanConfirmCallback(confirmParams);
                    }

                    NS_LOG_ERROR("Scan Type currently not supported");
                    return;
            }
        }
        else
        {
            if (!m_mlmeScanConfirmCallback.IsNull())
            {
                MlmeScanConfirmParams confirmParams;
                confirmParams.m_scanType = m_scanParams.m_scanType;
                confirmParams.m_chPage = m_scanParams.m_chPage;
                confirmParams.m_status = MLMESCAN_INVALID_PARAMETER;
                m_mlmeScanConfirmCallback(confirmParams);
            }
            NS_LOG_ERROR("Channel " << m_channelScanIndex
                                    << " could not be set in the current page");
        }
    }
    else if (id == LrWpanPibAttributeIdentifier::phyCurrentPage &&
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
        else
        {
            if (!m_mlmeStartConfirmCallback.IsNull())
            {
                MlmeStartConfirmParams confirmParams;
                confirmParams.m_status = MLMESTART_INVALID_PARAMETER;
                m_mlmeStartConfirmCallback(confirmParams);
            }
            NS_LOG_ERROR("Invalid channel parameter in MLME-start");
        }
    }
    else if (id == LrWpanPibAttributeIdentifier::phyCurrentPage &&
             m_pendPrimitive == MLME_ASSOC_REQ)
    {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS)
        {
            LrWpanPhyPibAttributes pibAttr;
            pibAttr.phyCurrentChannel = m_associateParams.m_chNum;
            m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel,
                                           &pibAttr);
        }
        else
        {
            m_macPanId = 0xffff;
            m_macCoordShortAddress = Mac16Address("FF:FF");
            m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
            m_incCapEvent.Cancel();
            m_incCfpEvent.Cancel();
            m_csmaCa->SetUnSlottedCsmaCa();
            m_incomingBeaconOrder = 15;
            m_incomingSuperframeOrder = 15;

            if (!m_mlmeAssociateConfirmCallback.IsNull())
            {
                MlmeAssociateConfirmParams confirmParams;
                confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
                confirmParams.m_status = MLMEASSOC_INVALID_PARAMETER;
                m_mlmeAssociateConfirmCallback(confirmParams);
            }
            NS_LOG_ERROR("Invalid page parameter in MLME-associate");
        }
    }
    else if (id == LrWpanPibAttributeIdentifier::phyCurrentChannel &&
             m_pendPrimitive == MLME_ASSOC_REQ)
    {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS)
        {
            m_originalChannelInCAP = m_associateParams.m_chNum;
            EndAssociateRequest();
        }
        else
        {
            m_macPanId = 0xffff;
            m_macCoordShortAddress = Mac16Address("FF:FF");
            m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
            m_incCapEvent.Cancel();
            m_incCfpEvent.Cancel();
            m_csmaCa->SetUnSlottedCsmaCa();
            m_incomingBeaconOrder = 15;
            m_incomingSuperframeOrder = 15;

            if (!m_mlmeAssociateConfirmCallback.IsNull())
            {
                MlmeAssociateConfirmParams confirmParams;
                confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
                confirmParams.m_status = MLMEASSOC_INVALID_PARAMETER;
                m_mlmeAssociateConfirmCallback(confirmParams);
            }
            NS_LOG_ERROR("Invalid channel parameter in MLME-associate");
        }
    }    
    else if (id == LrWpanPibAttributeIdentifier::phyCurrentPage &&
             m_pendPrimitive == MLME_START_REALGN) {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS) {
            LrWpanPhyPibAttributes pibAttr;
            pibAttr.phyCurrentChannel = m_startParams.m_logCh;
            m_phy->PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier::phyCurrentChannel,
                                           &pibAttr);

        } else {
            if (!m_mlmeStartConfirmCallback.IsNull()) {
                MlmeStartConfirmParams confirmParams;
                confirmParams.m_status = MLMESTART_INVALID_PARAMETER;
                m_mlmeStartConfirmCallback(confirmParams);
            }

            NS_LOG_ERROR("Invalid page parameter in MLME-start");
        }        
    }

    else if (id == LrWpanPibAttributeIdentifier::phyCurrentChannel &&
             m_pendPrimitive == MLME_START_REALGN) {
        if (status == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS) {
            m_originalChannelInCAP = m_startParams.m_logCh;

        } else {
            if (!m_mlmeStartConfirmCallback.IsNull()) {
                MlmeStartConfirmParams confirmParams;
                confirmParams.m_status = MLMESTART_INVALID_PARAMETER;
                m_mlmeStartConfirmCallback(confirmParams);
            }

            NS_LOG_ERROR("Invalid channel parameter in MLME-start");
        }        
    }
}

void LrWpanMac::SetLrWpanMacStateToGTS(uint16_t superframeID, int idx) {
    ChangeMacState(MAC_GTS);

    if (m_gtsRetrieve) {
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
        return;
    }

    // if (m_macDsmeACT[superframeID][idx].m_direction) {
    //     m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
    // } else {
    //     m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);
    // }

    m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
}

void
LrWpanMac::SetLrWpanMacState(LrWpanMacState macState)
{
    NS_LOG_FUNCTION(this << "mac state = " << macState);

    if (macState == MAC_IDLE) {
        ChangeMacState(MAC_IDLE);

        if (m_macRxOnWhenIdle)
        {
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
        ChangeMacState(MAC_CSMA);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);

    } else if (m_lrWpanMacState == MAC_CSMA && macState == CHANNEL_IDLE) {
        // Channel is idle, set transmitter to TX_ON
        ChangeMacState(MAC_SENDING);
        m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TX_ON);

    } else if (m_lrWpanMacState == MAC_CSMA && macState == CHANNEL_ACCESS_FAILURE) {
        NS_ASSERT(m_txPkt);

        // Cannot find a clear channel, drop the current packet
        // and send the proper confirm/indication according to the packet type
        NS_LOG_DEBUG(this << " cannot find clear channel");

        m_macTxDropTrace(m_txPkt);

        Ptr<Packet> pkt = m_txPkt->Copy();
        LrWpanMacHeader macHdr;
        pkt->RemoveHeader(macHdr);

        if (macHdr.IsCommand())
        {
            CommandPayloadHeader cmdPayload;
            pkt->RemoveHeader(cmdPayload);

            switch (cmdPayload.GetCommandFrameType())
            {
            case CommandPayloadHeader::ASSOCIATION_REQ: {
                m_macPanId = 0xffff;
                m_macCoordShortAddress = Mac16Address("FF:FF");
                m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                m_incCapEvent.Cancel();
                m_incCfpEvent.Cancel();
                m_csmaCa->SetUnSlottedCsmaCa();
                m_incomingBeaconOrder = 15;
                m_incomingSuperframeOrder = 15;

                if (!m_mlmeAssociateConfirmCallback.IsNull())
                {
                    MlmeAssociateConfirmParams confirmParams;
                    confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
                    confirmParams.m_status = MLMEASSOC_CHANNEL_ACCESS_FAILURE;
                    m_mlmeAssociateConfirmCallback(confirmParams);
                }
                break;
            }
            case CommandPayloadHeader::ASSOCIATION_RESP: {
                if (!m_mlmeCommStatusIndicationCallback.IsNull())
                {
                    MlmeCommStatusIndicationParams commStatusParams;
                    commStatusParams.m_panId = m_macPanId;
                    commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                    commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                    commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                    commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                    commStatusParams.m_status =
                        LrWpanMlmeCommStatus::MLMECOMMSTATUS_CHANNEL_ACCESS_FAILURE;
                    m_mlmeCommStatusIndicationCallback(commStatusParams);
                }
                RemovePendTxQElement(m_txPkt->Copy());
                break;
            }

            // DSME
            case CommandPayloadHeader::DSME_ASSOCIATION_REQ: {
                m_macPanId = 0xffff;
                m_macCoordShortAddress = Mac16Address("FF:FF");
                m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                m_incCapEvent.Cancel();
                m_incCfpEvent.Cancel();
                m_csmaCa->SetUnSlottedCsmaCa();
                m_incomingBeaconOrder = 15;
                m_incomingSuperframeOrder = 15;
                // m_incomingMultisuperframeOrder = 15;

                if (!m_mlmeAssociateConfirmCallback.IsNull()) {
                    MlmeAssociateConfirmParams confirmParams;
                    confirmParams.m_assocShortAddr = Mac16Address("FF:FF");
                    confirmParams.m_status = MLMEASSOC_CHANNEL_ACCESS_FAILURE;
                    m_mlmeAssociateConfirmCallback(confirmParams);
                }

                break;
            }
            // DSME
            case CommandPayloadHeader::DSME_ASSOCIATION_RESP: {
                if (!m_mlmeCommStatusIndicationCallback.IsNull()) {
                    MlmeCommStatusIndicationParams commStatusParams;
                    commStatusParams.m_panId = m_macPanId;
                    commStatusParams.m_srcAddrMode = LrWpanMacHeader::EXTADDR;
                    commStatusParams.m_srcExtAddr = macHdr.GetExtSrcAddr();
                    commStatusParams.m_dstAddrMode = LrWpanMacHeader::EXTADDR;
                    commStatusParams.m_dstExtAddr = macHdr.GetExtDstAddr();
                    commStatusParams.m_status =
                        LrWpanMlmeCommStatus::MLMECOMMSTATUS_CHANNEL_ACCESS_FAILURE;
                    m_mlmeCommStatusIndicationCallback(commStatusParams);
                }

                RemovePendTxQElement(m_txPkt->Copy());
                break;
            }
            // DSME
            case CommandPayloadHeader::COOR_REALIGN: {
                // DSME-TODO
            }         

            case CommandPayloadHeader::DATA_REQ: {
                m_macPanId = 0xffff;
                m_macCoordShortAddress = Mac16Address("FF:FF");
                m_macCoordExtendedAddress = Mac64Address("ff:ff:ff:ff:ff:ff:ff:ed");
                m_incCapEvent.Cancel();
                m_incCfpEvent.Cancel();
                m_csmaCa->SetUnSlottedCsmaCa();
                m_incomingBeaconOrder = 15;
                m_incomingSuperframeOrder = 15;

                if (!m_mlmePollConfirmCallback.IsNull())
                {
                    MlmePollConfirmParams pollConfirmParams;
                    pollConfirmParams.m_status =
                        LrWpanMlmePollConfirmStatus::MLMEPOLL_CHANNEL_ACCESS_FAILURE;
                    m_mlmePollConfirmCallback(pollConfirmParams);
                }
                break;
            }
            default: {
                // TODO: Other commands(e.g. Orphan Request)
                break;
            }
            }
            RemoveFirstTxQElement();
        }
        else if (macHdr.IsData())
        {
            if (!m_mcpsDataConfirmCallback.IsNull())
            {
                McpsDataConfirmParams confirmParams;
                confirmParams.m_msduHandle = m_txQueue.front()->txQMsduHandle;
                confirmParams.m_status = IEEE_802_15_4_CHANNEL_ACCESS_FAILURE;
                m_mcpsDataConfirmCallback(confirmParams);
            }
            // remove the copy of the packet that was just sent
            RemoveFirstTxQElement();
        }
        else
        {
            // TODO:: specify behavior for other packets
            m_txPkt = nullptr;
            m_retransmission = 0;
            m_numCsmacaRetry = 0;
        }

        ChangeMacState(MAC_IDLE);
        if (m_macRxOnWhenIdle)
        {
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
        }
        else
        {
            m_phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_TRX_OFF);
        }

    } else if (m_lrWpanMacState == MAC_CSMA && macState == MAC_CSMA_DEFERRED) {
        ChangeMacState(MAC_IDLE);
        m_txPkt = nullptr;
        // The MAC is running on beacon mode and the current packet could not be sent in the
        // current CAP. The packet will be send on the next CAP after receiving the beacon.
        // The PHY state does not change from its current form. The PHY change (RX_ON) will be
        // triggered by the scheduled beacon event.

        NS_LOG_DEBUG("****** PACKET DEFERRED to the next superframe *****");
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

Mac64Address
LrWpanMac::GetCoordExtAddress() const
{
    return m_macCoordExtendedAddress;
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
    NS_LOG_LOGIC(this << " change lrwpan mac state from " << m_lrWpanMacState << " to "
                      << newState);
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

void LrWpanMac::SetMultisuperframeOrder(uint8_t multisuperfmOrder) {
    m_macMultisuperframeOrder = multisuperfmOrder;
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

bool
LrWpanMac::isCoordDest()
{
    NS_ASSERT(m_txPkt);
    LrWpanMacHeader macHdr;
    m_txPkt->PeekHeader(macHdr);

    if (m_panCoor) {
        // The device is the PAN coordinator and the packet is not to itself
        return false;

    } else if (m_macCoordShortAddress == macHdr.GetShortDstAddr() 
                || m_macCoordExtendedAddress == macHdr.GetExtDstAddr()) {
        return true;

    // DSME-TODO: For Dsme Gts Notify Command frame.
    } else if (macHdr.GetShortDstAddr() == Mac16Address("ff:ff")) {
        return true;    

    } else if (m_coord) {
        return true;
        
    } else {
        NS_LOG_DEBUG("ERROR: Packet not for the coordinator!");
        return false;
    }
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

void LrWpanMac::SetAcceptAllHilowPkt(bool on) {
    m_acceptAllHilowPkt = on;
}

void LrWpanMac::SetGtsContinuePktSendingFromCap(bool on) {
    m_gtsContinuePktSendingFromCap = on;
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

void
LrWpanMac::SetAssociatedCoor(Mac64Address mac)
{
    m_macCoordExtendedAddress = mac;
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

uint16_t LrWpanMac::GetDsmeCurrentHoppingChannel() {
    // m_macChannelOfs;
    // m_macHoppingSeqList.m_len;
    m_macHoppingSeqList.GetChannel(10);
    // m_macPANCoordinatorBSN;

    if (m_macCAPReductionFlag) {

    }

    return 10;
}

} // namespace ns3
