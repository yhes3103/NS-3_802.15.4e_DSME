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
 *  Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

#ifndef LR_WPAN_MAC_H
#define LR_WPAN_MAC_H

#include <ns3/event-id.h>
#include <ns3/lr-wpan-fields.h>
#include <ns3/lr-wpan-phy.h>
#include <ns3/lr-wpan-mac-pl-headers.h>
#include <ns3/lr-wpan-mac-header.h>

#include <ns3/mac16-address.h>
#include <ns3/mac64-address.h>
#include <ns3/object.h>
#include <ns3/sequence-number.h>
#include <ns3/traced-callback.h>
#include <ns3/traced-value.h>

#include <deque>
#include <memory>
#include <tuple>
#include <map>
#include <bitset>

/**
 * Self-Designed enhanced group ack flags
*/ 
#define ENHANCED_GROUP_ACK_FIRST_SLOT 6
#define ENHANCED_GROUP_ACK_SECOND_SLOT 14
#define DSME_GTS_GROUP_ACK_SLOT 5
namespace ns3
{

class Packet;
class LrWpanCsmaCa;

typedef enum
{
    GROUP_ACK_DISABLED = 0,     //!< GROUP_ACK_DISABLED (disable group ack feature)
    GROUP_ACK_LEGACY = 1,       //!< GROUP_ACK_LEGACY
    GROUP_ACK_ENHANCED = 2,     //!< GROUP_ACK_ENHANCED (Self-designed)
    GROUP_ACK_DSME_GTS = 3,     //!< GROUP_ACK_DSME_GTS
    GROUP_ACK_RESERVED = 4      //!< GROUP_ACK_RESERVED

} LrWpanGroupAckPolicy;



class BeaconSchedulingPerformance
{
    public:
        // TODO : beacon scheduling
        double m_bcnSchedulingSuccessRatio = 0;
        double m_bcnSchedulingPeriod = 0;
        double m_bcnSchedulingTotalPktCount = 0;
        
    private:
};

template <typename T>
class ListNode
{
  public:
    T data;
    ListNode* next;

    ListNode(T data)
    {
        this->data = data;
        this->next = nullptr;
    }
};

template <typename T>
class LinkedList
{
  private:
    ListNode<T>* head;

  public:

    LinkedList();

    void insertAtBeginning(T data);
    void insertAtEnd(T data);
    void deleteAtBeginning();
    void deleteAtEnd();
    void printList();
    uint16_t GetHeadNodeData();
    bool isHeadNull() const {
        return head == nullptr;
    }
};

/**
 * \defgroup lr-wpan LR-WPAN models
 *
 * This section documents the API of the IEEE 802.15.4-related models.  For a generic functional
 * description, please refer to the ns-3 manual.
 */

/**
 * \ingroup lr-wpan
 *
 * Tx options
 */
typedef enum
{
    TX_OPTION_NONE = 0,    //!< TX_OPTION_NONE
    TX_OPTION_ACK = 1,     //!< TX_OPTION_ACK
    TX_OPTION_GTS = 2,     //!< TX_OPTION_GTS
    TX_OPTION_INDIRECT = 4, //!< TX_OPTION_INDIRECT
    TX_OPTION_DIRECT = 8
} LrWpanTxOption;

/**
 * \ingroup lr-wpan
 *
 * MAC states
 */
typedef enum
{
    MAC_IDLE,               //!< MAC_IDLE 0
    MAC_CSMA,               //!< MAC_CSMA 1
    MAC_SENDING,            //!< MAC_SENDING 2 
    MAC_ACK_PENDING,        //!< MAC_ACK_PENDING 3 
    CHANNEL_ACCESS_FAILURE, //!< CHANNEL_ACCESS_FAILURE 4
    CHANNEL_IDLE,           //!< CHANNEL_IDLE 5
    SET_PHY_TX_ON,          //!< SET_PHY_TX_ON 6
    MAC_GTS,                //!< MAC_GTS 7
    MAC_GTS_SENDING,        //!< 8
    MAC_INACTIVE,           //!< MAC_INACTIVE 9
    MAC_CSMA_DEFERRED       //!< MAC_CSMA_DEFERRED 10
} LrWpanMacState;

/**
 * \ingroup lr-wpan
 *
 * Superframe status
 */
typedef enum
{
    BEACON,     //!< The Beacon transmission or reception Period
    CAP,        //!< Contention Access Period
    CFP,        //!< Contention Free Period
    INACTIVE,   //!< Inactive Period or unslotted CSMA-CA
    REMAINING   //!< DSME: Remaining Period (after CFP but until the next beacon tx)
} SuperframeStatus;

/**
 * \ingroup lr-wpan
 *
 * Superframe type
 */
typedef enum
{
    OUTGOING = 0, //!< Outgoing Superframe
    INCOMING = 1  //!< Incoming Superframe
} SuperframeType;

/**
 * \ingroup lr-wpan
 *
 * Indicates a pending MAC primitive
 */
typedef enum
{
    MLME_NONE = 0,      //!< No pending primitive
    MLME_START_REQ = 1, //!< Pending MLME-START.request primitive
    MLME_SCAN_REQ = 2,  //!< Pending MLME-SCAN.request primitive
    MLME_ASSOC_REQ = 3, //!< Pending MLME-ASSOCIATION.request primitive
    MLME_SYNC_REQ = 4,  //!< Pending MLME-SYNC.request primitive

    MLME_DISASSOCIATE_REQ = 5,
    MLME_DSME_GTS_REQ = 6,
    MLME_DSME_INFO_REQ = 7,
    MLME_DSME_LINKSTATUSRPT_REQ = 8,    

    MLME_START_REALGN = 9,

    // MLME-DISASSOCIATE-GET-REQ
    // MLME-DISASSOCIATE-GTS-REQ
    // MLME-DISASSOCIATE-RESET-REQ
    // MLME-RX-ENABLE-REQ
    // MLME-POLL-REQ
    // MLME-DPS-REQ
    // MLME-SOUNDING-REQ
    // MLME-CALIBRATE-REQ
} PendingPrimitiveStatus;

namespace TracedValueCallback
{

/**
 * \ingroup lr-wpan
 * TracedValue callback signature for LrWpanMacState.
 *
 * \param [in] oldValue original value of the traced variable
 * \param [in] newValue new value of the traced variable
 */
typedef void (*LrWpanMacState)(LrWpanMacState oldValue, LrWpanMacState newValue);

/**
 * \ingroup lr-wpan
 * TracedValue callback signature for SuperframeStatus.
 *
 * \param [in] oldValue original value of the traced variable
 * \param [in] newValue new value of the traced variable
 */
typedef void (*SuperframeStatus)(SuperframeStatus oldValue, SuperframeStatus newValue);

} // namespace TracedValueCallback

/**
 * \ingroup lr-wpan
 *
 * Table 3 of 802.15.4e-2012
 */
typedef enum
{
    NO_PANID_ADDR = 0,
    // ADDR_MODE_RESERVED = 1,
    SIMPLE_ADDR = 1,
    SHORT_ADDR = 2,
    EXT_ADDR = 3
} LrWpanAddressMode;

/**
 * \ingroup lr-wpan
 *
 * See 802.15.4e-2012 Section 5.3.2.3 Table 6
 */
typedef enum
{
    ASSOCIATED                          = 0x00,
    PAN_AT_CAPACITY                     = 0x01,
    PAN_ACCESS_DENIED                   = 0x02,
    HOPPING_SEQ_OFS_DUPLICATION         = 0x03,
    ASSOCIATION_STATUS_FIELD_RESERVED   = 0x04,

    /**
     * Add self-designed allocation status - Enhanced Beacon Scheduling (EBS)
     * Note :
     * This status is used when PAN-C sending a association response command which choose EBS as the beacon scheduling policy.
     * It contains a unique, self-designed sequence number (association sequence) for EBS.
     **/ 
    ASSOCIATED_EBS                      = 0x05,

    FASTA_SUCCESSFUL                    = 0x80,
    ASSOCIATED_WITHOUT_ADDRESS          = 0xfe,
    DISASSOCIATED                       = 0xff
} LrWpanAssociationStatus;

/**
 * \ingroup dsme
 *
 * See 802.15.4-2011 Section 5.3.3.2 Table 7
 */
// typedef enum {
//     Reserved                          = 0x00,
//     COORD_WISH_TO_DEV_LEAVE_PAN       = 0x01,
//     DEV_WISH_TO_LEAVE_PAN             = 0x02,
// } LrWpan2012DisassociationReason;

/**
 * \ingroup lr-wpan
 *
 * See 802.15.4-2011 Section 6.2.3.3 Table 15
 */
typedef enum {
    DISASSOCIATE_SUCCESS                 = 0, 
    DISASSOCIATE_NO_ACK                  = 1,
    DISASSOCIATE_TRANSACTION_OVERFLOW    = 2,
    DISASSOCIATE_TRANSACTION_EXPIRED     = 3,
    DISASSOCIATE_CHANNEL_ACCESS_FAILURE  = 4,
    DISASSOCIATE_COUNTER_ERROR           = 5,
    DISASSOCIATE_FRAME_TOO_LONG          = 6,
    DISASSOCIATE_UNAVAILABLE_KEY         = 7,
    DISASSOCIATE_UNSUPPORTED_SECURITY    = 8,
    DISASSOCIATE_INVALID_PARAMETER       = 9
} LrWpanDisassociationStatus;

/**
 * \ingroup lr-wpan
 *
 * See Section 5.1.2.1 Table 30 of IEEE 802.15.4e-2012
 */
typedef enum
{
    MLMESCAN_ED = 0x00,
    MLMESCAN_ACTIVE = 0x01,
    MLMESCAN_PASSIVE = 0x02,
    MLMESCAN_ORPHAN = 0x03,
    MLMESCAN_ASYMMETRIC_MULTICHANNEL_ACTIVE  = 0x04,
    MLMESCAN_CHANNEL_PROBE                   = 0x05,
    MLMESCAN_MULTICHANNEL_HELLO              = 0x06,
    MLMESCAN_ENHANCED_ACTIVE_SCAN            = 0x07
} LrWpanMlmeScanType;

/**
 * \ingroup lr-wpan
 *
 * Table 47 of 802.15.4e-2012
 */
typedef enum
{
    IEEE_802_15_4_SUCCESS = 0,
    IEEE_802_15_4_TRANSACTION_OVERFLOW = 1,
    IEEE_802_15_4_TRANSACTION_EXPIRED = 2,
    IEEE_802_15_4_CHANNEL_ACCESS_FAILURE = 3,
    IEEE_802_15_4_INVALID_ADDRESS = 4,
    IEEE_802_15_4_INVALID_GTS = 5,
    IEEE_802_15_4_NO_ACK = 6,
    IEEE_802_15_4_COUNTER_ERROR = 7,
    IEEE_802_15_4_FRAME_TOO_LONG = 8,
    IEEE_802_15_4_UNAVAILABLE_KEY = 9,
    IEEE_802_15_4_UNSUPPORTED_SECURITY = 10,
    IEEE_802_15_4_INVALID_PARAMETER = 11,
    IEEE_802_15_4_ACK_RCVD_NODSN_NOSA = 12
} LrWpanMcpsDataConfirmStatus;

/**
 * Self-designed enum for beacon scheduling
 */
typedef enum
{
    LEGACY = 0,     // IEEE 802.15.4e legacy beacon scheduling (Random)
    LSB = 1,        // Least significant bit first policy
    MSB = 2,        // Most significant bit first policy
    EBS = 3         // Enhanced Beacon scheduling (use association sequence & vacant list)
} LrWpanBeaconSchedulingPolicy;

/**
 * \ingroup lr-wpan
 *
 * PAN Descriptor, Table 17 IEEE 802.15.4-2011 and IEEE 802.15.4e-2012
 */
struct PanDescriptor {
    LrWpanAddressMode m_coorAddrMode{SHORT_ADDR};   //!< The coordinator addressing mode corresponding
                                                        //!< to the received beacon frame.
    uint16_t m_coorPanId{0xffff};                       //!< The PAN ID of the coordinator as specified in the received beacon frame.
    Mac16Address m_coorShortAddr;                       //!< The coordinator short address as specified in the coordinator
                                                        //!< address mode.
    Mac64Address m_coorExtAddr;                         //!< The coordinator extended address as specified in the
                                                        //!< coordinator address mode.
    uint8_t m_logCh{11};                                //!< The current channel number occupied by the network.
    uint8_t m_logChPage{0};                             //!< The current channel page occupied by the network.
    SuperframeField m_superframeSpec;                   //!< The superframe specification as specified in the received
                                                        //!< beacon frame.
    bool m_gtsPermit{false};                            //!< TRUE if the beacon is from the PAN coordinator that is accepting GTS requests.
    uint8_t m_linkQuality{0};                           //!< The LQI at which the network beacon was received. Lower values represent lower LQI.
    Time m_timeStamp;                                   //!< Beacon frame reception time. Used as Time data type in ns-3 to avoid
                                                        //!< precision problems.
    DsmeSuperFrameField m_dsmeSuperframeSpec;
    TimeSync m_timeSyncSpec;
    BeaconBitmap m_bcnBitmap;
    ChannelHopping m_channelHoppingSpec;
    GroupACK m_gACKSpec;
};

/**
 * \ingroup lr-wpan
 *
 * Management Type, Table 44q IEEE 802.15.4e-2012
 */
typedef enum {
    GTS_DEALLOCATION                = 0b000,
    GTS_ALLOCATION                  = 0b001,
    GTS_DUPLICATED_ALLOCATION_NOTIF = 0b010,
    GTS_REDUCE                      = 0b011,
    GTS_RESTART                     = 0b100,
    GTS_EXPIRATION                  = 0b101
} ManagementType;

/**
 * \ingroup lr-wpan
 *
 * DSME-GTS response status, Table 44s IEEE 802.15.4e-2012 Section 6.2.21.1.3
 */
typedef enum {
    MLMEDSMEGTS_SUCCESS           = 0,
    MLMEDSMEGTS_DENIED            = 1,
    MLMEDSMEGTS_INVALID_PARAMETER = 2,
} LrWpanMlmeDsmeGtsResponseStatus;

/**
 * \ingroup lr-wpan
 * 
 * DSME-GTS request status, Table 44t IEEE 802.15.4e-2012 Section 6.2.21.1.4
 */
typedef enum {
    MLMEDSMEGTS_REQ_SUCCESS                 = 0,
    MLMEDSMEGTS_REQ_DENIED                  = 1,
    MLMEDSMEGTS_REQ_INVALID_PARAMETER       = 2,
    MLMEDSMEGTS_REQ_NO_ACK                  = 3,
    MLMEDSMEGTS_REQ_NO_DATA                 = 4,
    MLMEDSMEGTS_REQ_CHANNEL_ACCESS_FAILURE  = 5,
} LrWpanMlmeDsmeGtsRequestStatus;

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-INFO.request parameters, Table 44u IEEE 802.15.4e-2012 Section 6.2.21.1.3
 */
typedef enum {
    MLMEDSMEINFO_TIMESTAMP              = 0x00,
    MLMEDSMEINFO_DSME_SAB_SPECIFICATION = 0x01,
    MLMEDSMEINFO_DSME_PAN_DESCRIPTOR    = 0x02,
} InfoType;

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-INFO.response status, Table 44w IEEE 802.15.4e-2012 Section 6.2.21.2.3
 */
typedef enum {
    MLMEDSMEINFO_SUCCESS                = 0,
    MLMEDSMEINFO_CHANNEL_ACCESS_FAILURE = 1,
    MLMEDSMEINFO_NO_ACK                 = 2,
    MLMEDSMEINFO_NO_DATA                = 3,
    MLMEDSMEINFO_COUNTER_ERROR          = 4,
    MLMEDSMEINFO_FRAME_TOO_LONG         = 5,
    MLMEDSMEINFO_UNAVAILABLE_KEY        = 6,
    MLMEDSMEINFO_UNSUPPORTED_SECURITY   = 7,
    MLMEDSMEINFO_INVALID_PARAMETER      = 8,
} LrWpanMlmeDsmeInfoResponseStatus;

/**
 * \ingroup lr-wpan
 *
 * Table 35 of IEEE 802.15.4-2011
 */
typedef enum
{
    MLMESTART_SUCCESS = 0,
    MLMESTART_NO_SHORT_ADDRESS = 1,
    MLMESTART_SUPERFRAME_OVERLAP = 2,
    MLMESTART_TRACKING_OFF = 3,
    MLMESTART_INVALID_PARAMETER = 4,
    MLMESTART_COUNTER_ERROR = 5,
    MLMESTART_FRAME_TOO_LONG = 6,
    MLMESTART_UNAVAILABLE_KEY = 7,
    MLMESTART_UNSUPPORTED_SECURITY = 8,
    MLMESTART_CHANNEL_ACCESS_FAILURE = 9
} LrWpanMlmeStartConfirmStatus;

/**
 * \ingroup lr-wpan
 *
 * Table 31 of IEEE 802.15.4e-2012
 */
typedef enum
{
    MLMESCAN_SUCCESS = 0,
    MLMESCAN_LIMIT_REACHED = 1,
    MLMESCAN_NO_BEACON = 2,
    MLMESCAN_SCAN_IN_PROGRESS = 3,
    MLMESCAN_COUNTER_ERROR = 4,
    MLMESCAN_FRAME_TOO_LONG = 5,
    MLMESCAN_UNAVAILABLE_KEY = 6,
    MLMESCAN_UNSUPPORTED_SECURITY = 7,
    // MLMESCAN_INVALID_PARAMETER = 8
    MLMESCAN_BAD_CHANNEL                = 8,
    MLMESCAN_INVALID_PARAMETER          = 9
} LrWpanMlmeScanConfirmStatus;

/**
 * \ingroup lr-wpan
 *
 * Table 12 of IEEE 802.15.4-2011
 */
typedef enum
{
    MLMEASSOC_SUCCESS                   = 0,
    MLMEASSOC_FULL_CAPACITY             = 1,
    MLMEASSOC_CHANNEL_ACCESS_FAILURE    = 2,
    MLMEASSOC_NO_ACK                    = 3,
    MLMEASSOC_NO_DATA                   = 4,
    MLMEASSOC_COUNTER_ERROR             = 5,
    MLMEASSOC_FRAME_TOO_LONG            = 6,
    MLMEASSOC_IMPROPER_KEY_TYPE         = 7,
    MLMEASSOC_IMPROPER_SECURITY_LEVEL   = 8,
    MLMEASSOC_SECURITY_ERROR            = 9,
    MLMEASSOC_UNAVAILABLE_KEY           = 10,

    // MLMEASSOC_FULL_CAPACITY = 1,
    // MLMEASSOC_ACCESS_DENIED = 2,

    MLMEASSOC_UNSUPPORTED_LEGACY        = 11,
    MLMEASSOC_UNSUPPORTED_SECURITY      = 12,
    MLMEASSOC_UNSUPPORTED_FEATURE       = 13,
    MLMEASSOC_INVALID_PARAMETER         = 14
} LrWpanMlmeAssociateConfirmStatus;

/**
 * \ingroup lr-wpan
 *
 * Table 37 of IEEE 802.15.4-2011
 */
typedef enum
{
    MLMESYNCLOSS_PAN_ID_CONFLICT = 0,
    MLMESYNCLOSS_REALIGMENT = 1,
    MLMESYNCLOSS_BEACON_LOST = 2,
    MLMESYNCLOSS_SUPERFRAME_OVERLAP = 3
} LrWpanSyncLossReason;

/**
 * \ingroup lr-wpan
 *
 * Table 18 of IEEE 802.15.4-2011
 */
typedef enum
{
    MLMECOMMSTATUS_SUCCESS = 0,
    MLMECOMMSTATUS_TRANSACTION_OVERFLOW = 1,
    MLMECOMMSTATUS_TRANSACTION_EXPIRED = 2,
    MLMECOMMSTATUS_CHANNEL_ACCESS_FAILURE = 3,
    MLMECOMMSTATUS_NO_ACK = 4,
    MLMECOMMSTATUS_COUNTER_ERROR = 5,
    MLMECOMMSTATUS_FRAME_TOO_LONG = 6,
    MLMECOMMSTATUS_INVALID_PARAMETER = 7
} LrWpanMlmeCommStatus;

/**
 * \ingroup lr-wpan
 *
 * Table 39 of IEEE 802.15.4-2011
 */
typedef enum
{
    MLMEPOLL_SUCCESS = 0,
    MLMEPOLL_CHANNEL_ACCESS_FAILURE = 2,
    MLMEPOLL_NO_ACK = 3,
    MLMEPOLL_NO_DATA = 4,
    MLMEPOLL_COUNTER_ERROR = 5,
    MLMEPOLL_FRAME_TOO_LONG = 6,
    MLMEPOLL_UNAVAILABLE_KEY = 7,
    MLMEPOLL_UNSUPPORTED_SECURITY = 8,
    MLMEPOLL_INVALID_PARAMETER = 9
} LrWpanMlmePollConfirmStatus;

/**
 * \ingroup lr-wpan
 * Self-define enum for beacon scheduling timeslot status
 */
typedef enum 
{
    SLOT_VACANT = 0,
    SLOT_ALLOCATED = 1,
    SLOT_UNDIFINED = 2
} LrWpanBeaconSchedulingTimeslotStatus;

typedef enum 
{
    ALLOC_COLLISION = 0,
    ALLOC_SUCCESS = 1,
    ALLOC_TO_BE_DONE = 2
} LrWpanBeaconSchedulingAllocStatus;

/**
 * \ingroup lr-wpan
 *
 * MCPS-DATA.request params. See 802.15.4-2011 Section 6.3.1 and 802.15.4e-2012 Section 6.3.1
 */
struct McpsDataRequestParams
{
    LrWpanAddressMode m_srcAddrMode{SHORT_ADDR}; //!< Source address mode
    LrWpanAddressMode m_dstAddrMode{SHORT_ADDR}; //!< Destination address mode
    uint16_t m_dstPanId{0};                      //!< Destination PAN identifier
    Mac16Address m_dstAddr;                      //!< Destination address
    Mac64Address m_dstExtAddr;                   //!< Destination extended address
    // msduLength,
    // msdu,
    uint8_t m_msduHandle{0};                     //!< MSDU handle
    uint8_t m_txOptions{0};                      //!< Tx Options (bitfield)

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex

    // 802.15.4e-2012
    std::vector<bool> m_frameCtrlOptions;           //!< Corresponding to the options in the multipurpose frame control 
                                                    //!< or general frame control for frame version 0b10.
    std::vector<HeaderElementIDs> m_headerIElist;   //!< Determines which Header IEs are sent.
                                                    //!< IES_INCLUDED shall be set in frameControlOptions if present.
    std::vector<PayloadIEGroupIDs> m_payloadIElist; //!< Determines which Payload IEs are sent in the frame.
                                                    //!< IES_INCLUDED shall be set in frameControlOptions if present.
    bool m_sendMultipurpose{false};
};

/**
 * \ingroup lr-wpan
 *
 * MCPS-DATA.confirm params. See 802.15.4e-2012 and 802.15.4-2011 Section 6.3.2, Table 47
 */
struct McpsDataConfirmParams
{
    uint8_t m_msduHandle{0}; //!< MSDU handle
    Time m_timestamp;
    
    bool m_rangingReceived;
    uint32_t m_rangingCntStart;
    uint32_t m_rangingCntStop;
    int32_t m_rangingTrackingInterval;
    int32_t m_rangingOfs;
    int8_t m_rangingFOM;

    uint8_t m_numBackoffs;
    std::vector<uint8_t> m_ackPayload;
    LrWpanMcpsDataConfirmStatus m_status{
        IEEE_802_15_4_INVALID_PARAMETER}; //!< The status of the last MSDU transmission
};

/**
 * \ingroup lr-wpan
 *
 * MCPS-DATA.indication params. 802.15.4e-2012 and 802.15.4-2011 Section 6.3.3
 */
struct McpsDataIndicationParams
{
    uint8_t m_srcAddrMode{SHORT_ADDR}; //!< Source address mode
    uint16_t m_srcPanId{0};            //!< Source PAN identifier
    Mac8Address m_srcSimpleAddr;
    Mac16Address m_srcAddr;            //!< Source address
    Mac64Address m_srcExtAddr;         //!< Source extended address

    uint8_t m_dstAddrMode{SHORT_ADDR}; //!< Destination address mode
    uint16_t m_dstPanId{0};            //!< Destination PAN identifier
    Mac8Address m_dstSimpleAddr;
    Mac16Address m_dstAddr;            //!< Destination address
    Mac64Address m_dstExtAddr;         //!< Destination extended address

    // msduLength
    // msdu

    uint8_t m_mpduLinkQuality{0};      //!< LQI value measured during reception of the MPDU
    uint8_t m_dsn{0};                  //!< The DSN of the received data frame
    Time m_timestamp;

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex
    // UWBPRF
    // UWBPreamble Symbol Repetitions
    uint8_t m_dataRate;
    
    bool m_rangingReceived;
    uint32_t m_rangingCntStart;
    uint32_t m_rangingCntStop;
    int32_t m_rangingTrackingInterval;
    int32_t m_rangingOfs;
    int8_t m_rangingFOM;
};

/**
 * \ingroup lr-wpan
 *
 * MLME-ASSOCIATE.indication params. See 802.15.4-2011 and 802.15.4e-2012 6.2.2.2.
 */
struct MlmeAssociateIndicationParams
{
    Mac64Address m_extDevAddr; //!< The extended address of the device requesting association
    CapabilityField
        capabilityInfo; //!< The operational capabilities of the device requesting association.
    
    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex,
    // LowLatencyNetworkInfo,

    uint16_t m_channelOfs;
    uint8_t m_hoppingSeqID;
};

/**
 * \ingroup lr-wpan
 *
 * MLME-ASSOCIATE.response params. See 802.15.4-2011 and 802.15.4e-2012 6.2.2.3.
 */
struct MlmeAssociateResponseParams
{
    Mac64Address m_extDevAddr;     //!< The extended address of the device requesting association
    Mac16Address m_assocShortAddr; //!< The short address allocated by the coordinator on successful
                                   //!< assoc. FF:FF = Unsuccessful
    LrWpanAssociationStatus m_status{DISASSOCIATED}; //!< The status of the association attempt (As
                                                     //!< defined on Table 83 IEEE 802.15.4-2006)

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex,
    // LowLatencyNetworkInfo,   

    uint32_t m_channelOfs;

    uint16_t m_hoppingSeqLen;               //!< Hopping Sequence Length (DSME-Association response command)
    HoppingSequence m_hoppingSeq;           //!< Hopping Sequence (DSME-Association response command)                                                    
};

struct HoppingDescriptor {
    uint8_t m_HoppingSequenceID;               // hopping sequence ID (1 Octets)
    // uint8_t m_panCoordinatorBSN;
    uint16_t m_channelOfs;                     // Channel hopping sepcification channel offset (2 Octets)
    uint8_t m_channelOfsBitmapLen;             // Channel hopping sepcification channel offset bitmap length (1 Octets)
    std::vector<uint16_t> m_channelOfsBitmap;  // Channel hopping sepcification channel offset bitmap (variable)
    uint16_t m_hoppingSeqLen;                  // Hopping Sequence Length 
    HoppingSequence m_hoppingSeq;
};

/**
 * \ingroup lr-wpan
 *
 * MLME-BEACON.request params. See 802.15.4e-2012 Section 6.2.18.1 and Table 44a
 */
struct MlmeBeaconRequestParams {

};

/**
 * \ingroup lr-wpan 
 *
 * MLME-START.request params. See 802.15.4-2011 and 802.15.4e-2012 Section 6.2.12.1 and Table 34a
 */
struct MlmeStartRequestParams
{
    uint16_t m_PanId{0}; //!< Pan Identifier used by the device.
    uint8_t m_logCh{
        11}; //!< Logical channel on which to start using the new superframe configuration.
    uint32_t m_logChPage{
        0}; //!< Logical channel page on which to start using the new superframe configuration.
    uint32_t m_startTime{0}; //!< Time at which to begin transmitting beacons (Used by Coordinator
                             //!< not PAN Coordinators). The time is specified in symbols.
    uint8_t m_bcnOrd{15};    //!< Beacon Order, Used to calculate the beacon interval, a value of 15
                             //!< indicates no periodic beacons will be transmitted.
    uint8_t m_sfrmOrd{15};   //!< Superframe Order, indicates the length of the CAP in time slots.
    bool m_panCoor{false};   //!< On true this device will become coordinator.
    bool m_battLifeExt{false}; //!< Flag indicating whether or not the Battery life extension (BLE)
                               //!< features are used.
    bool m_coorRealgn{false};  //!< True if a realignment request command is to be transmitted prior
                               //!< changing the superframe.

    // CoordRealignSecurityLevel,
    // CoordRealignKeyIdMode,
    // CoordRealignKeySource,
    // CoordRealignKeyIndex,
    // BeaconSecurityLevel,
    // BeaconKeyIdMode,
    // BeaconKeySource,
    // BeaconKeyIndex             

    DsmeSuperFrameField m_dsmeSuperframeSpec;   //!< Specifies the superframe configuration in the DSME-enabled
                                                //!< PAN. Refer to IEEE 802.15.4e-2012 5.2.4.9.1.

    BeaconBitmap m_bcnBitmap;                   //!< Specifies beacon bitmap. 
                                                //!< Refer to IEEE 802.15.4e-2012 5.2.4.9.3.
                                                
    HoppingDescriptor m_hoppingDescriptor;      //!< Specifies channel hopping information.
                                                //!< Refer to IEEE 802.15.4e-2012 Table 34a.
};

/**
 * \ingroup lr-wpan
 *
 * MLME-SYNC.request params. See 802.15.4-2011  Section 6.2.13.1
 */
struct MlmeSyncRequestParams
{
    uint8_t m_logCh{11};    //!< The channel number on which to attempt coordinator synchronization.
    uint32_t m_logChPage{0};    //!< The channel page on which to attempt coordinator synchronization.
    bool m_trackBcn{false}; //!< True if the mlme sync with the next beacon and attempts to track
                            //!< future beacons. False if mlme sync only the next beacon.
};

// 6.2.14 Primitives for requesting data from a coordinator

/**
 * \ingroup lr-wpan
 *
 * MLME-POLL.request params. See 802.15.4-2011  Section 6.2.14.1
 */
struct MlmePollRequestParams
{
    LrWpanAddressMode m_coorAddrMode{
        SHORT_ADDR}; //!< The addressing mode of the coordinator to which the pool is intended.
    uint16_t m_coorPanId{0};      //!< The PAN id of the coordinator to which the poll is intended.
    Mac16Address m_coorShortAddr; //!< Coordinator short address.
    Mac64Address m_coorExtAddr;   //!< Coordinator extended address.

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex,
};

/**
 * \ingroup lr-wpan
 *
 * MLME-SCAN.request params. See IEEE 802.15.4-2011 and 802.15.4e-2012 Section 6.2.10.1 Table 30
 */
struct MlmeScanRequestParams
{
    LrWpanMlmeScanType m_scanType{MLMESCAN_PASSIVE}; //!< Indicates the type of scan performed as
                                                     //!< described in IEEE 802.15.4-2011 (5.1.2.1).
    uint32_t m_scanChannels{0x7FFFFFF};              //!< The channel numbers to be scanned.
    uint8_t m_scanDuration{14}; //!< A value used to calculate the length of time to spend scanning
                                //!< [aBaseSuperframeDuration * (2^m_scanDuration +)].
    uint32_t m_chPage{0};       //!< The channel page on which to perform scan.

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex,  

    bool m_linkQualityScan{false};                          //!< If TRUE, Link Quality Scan should be enabled, otherwise FALSE.
    std::vector<bool> m_frameCtrlOptions;                   //!< Corresponding to the options in frame control for
                                                            //!< frame version b10. Used with enhanced active scan.
    std::vector<HeaderElementIDs> m_headerIElist;           //!< Determines which Header IEs are sent in the
                                                            //!< frame. IES_INCLUDED shall be set in frameControlOptions if present.
    std::vector<PayloadIEGroupIDs> m_payloadIElist;         //!< Determines which Payload IEs are sent in the
                                                            //!< frame. IES_INCLUDED shall be set in　frameControlOptions if present.  
};

/**
 * \ingroup lr-wpan
 *
 * MLME-SCAN.confirm params. See IEEE 802.15.4-2011 and 802.15.4e-2012 Section 6.2.10.2 Table 31
 */
struct MlmeScanConfirmParams
{
    LrWpanMlmeScanConfirmStatus m_status{
        MLMESCAN_INVALID_PARAMETER}; //!< The status of the scan request.
    LrWpanMlmeScanType m_scanType{
        MLMESCAN_PASSIVE}; //!< Indicates the type of scan performed (ED,ACTIVE,PASSIVE,ORPHAN).
    uint32_t m_chPage{0};  //!< The channel page on which the scan was performed.
    std::vector<uint8_t> m_unscannedCh; //!< A list of channels given in the request which were not
                                        //!< scanned (Not valid for ED scans).
    std::vector<uint8_t>
        m_energyDetList; //!< A list of energy measurements, one for each channel searched during ED
                         //!< scan (Not valid for Active, Passive or Orphan Scans)
    std::vector<PanDescriptor> m_panDescList; //!< A list of PAN descriptor, one for each beacon
                                              //!< found (Not valid for ED and Orphan scans).

    // DetectedCategory
    // UWBEnergyDetectList                                        
};

/**
 * \ingroup lr-wpan
 *
 * MLME-ASSOCIATE.request params. See 802.15.4-2011 and 802.15.4e-2012 Section 6.2.2.1
 */
struct MlmeAssociateRequestParams
{
    uint8_t m_chNum{11};  //!< The channel number on which to attempt association.
    uint32_t m_chPage{0}; //!< The channel page on which to attempt association.
    uint8_t m_coordAddrMode{
        SHORT_ADDR}; //!< The coordinator addressing mode for this primitive and subsequent MPDU.
    uint16_t m_coordPanId{0}; //!< The identifier of the PAN with which to associate.
    Mac16Address
        m_coordShortAddr; //!< The short address of the coordinator with which to associate.
    Mac64Address
        m_coordExtAddr; //!< The extended address of the coordinator with which to associate.
    CapabilityField
        m_capabilityInfo; //!< Specifies the operational capabilities of the associating device.
    
    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex
    // LowLatencyNetworkInfo,

    uint16_t m_channelOfs;                  //!< Specifies the offset value of Hopping Sequence.
    uint8_t m_hoppingSeqID;                 //!< Indicate the ID of channel hopping sequence in use:
                                            //!< 0x00: a default hopping sequence
                                            //!< 0x01: a hopping sequence generated by PAN coordinator
                                            //!< 0x02−0x0f: a hopping sequence set by NHL
                                            //!< If a coordinator receives an association
                                            //!< request command with HoppingSequenceID of 1, 
                                            //!< it replies with a channel hopping sequence in an association
                                            //!< response command.    
};

/**
 * \ingroup lr-wpan
 *
 * MLME-ASSOCIATE.confirm params. See 802.15.4-2011  Section 6.2.2.4
 */
struct MlmeAssociateConfirmParams
{
    Mac16Address m_assocShortAddr; //!< The short address used in the association request
    LrWpanMlmeAssociateConfirmStatus m_status{
        MLMEASSOC_INVALID_PARAMETER}; //!< The status of a MLME-associate.request

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex,
    // LowLatencyNetworkInfo,

    uint16_t m_channelOfs;
    uint16_t m_hoppingSeqLen;               //!< Hopping Sequence Length (DSME-Association response command)
    HoppingSequence m_hoppingSeq;           //!< Hopping Sequence (DSME-Association response command)
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DISASSOCIATE.request params. See 802.15.4-2011 Section 6.2.3.1 and Table 13, Table 7
 */
struct MlmeDisassociateRequestParams {
    uint8_t m_devAddrMode{SHORT_ADDR};                                          //!< The addressing mode of the device to which
                                                                                //!< to send the disassociation notification command.
    uint16_t m_devPanId{0};                                                     //!< The PAN identifier of the device to which
                                                                                //!< to send the disassociation notification command.
    Mac16Address m_shortDevAddr;                                                //!< The short address of the device to which to send
                                                                                //!< the disassociation notification command.
    Mac64Address m_extDevAddr;                                                  //!< The extended address of the device to which to send
                                                                                //!< the disassociation notification command.
    CommandPayloadHeader::DisassociationReason m_disassociateReason{CommandPayloadHeader::DISASSC_DEV_LEAVE_PAN};
    bool m_txIndirect;                                                          //!< TRUE if the disassociation notification
                                                                                //!< command is to be sent indirectly.
    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DISASSOCIATE.indication params. See 802.15.4-2011 Section 6.2.3.2 and Table 14
 */
struct MlmeDisassociateIndicationParams {
    Mac64Address m_extDevAddr;
    CommandPayloadHeader::DisassociationReason m_disassociateReason{CommandPayloadHeader::DISASSC_DEV_LEAVE_PAN};

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DISASSOCIATE.confirm params. See 802.15.4-2011 Section 6.2.3.3 and Table 15
 */
struct MlmeDisassociateConfirmParams {
    LrWpanDisassociationStatus m_status{DISASSOCIATE_SUCCESS};   //! <The status of the disassociation attempt.
    uint8_t m_devAddrMode{SHORT_ADDR};
    uint16_t m_devPanId{0};
    Mac16Address m_shortDevAddr;                        //!< The short address of the device to which to send
                                                        //!< the disassociation notification command.
    Mac64Address m_extDevAddr;                          //!< The extended address of the device to which to send
                                                        //!< the disassociation notification command.
};

/**
 * \ingroup lr-wpan
 *
 * MLME-START.confirm params. See  802.15.4-2011   Section 6.2.12.2
 */
struct MlmeStartConfirmParams
{
    LrWpanMlmeStartConfirmStatus m_status{
        MLMESTART_INVALID_PARAMETER}; //!< The status of a MLME-start.request
};

/**
 * \ingroup lr-wpan
 *
 * MLME-BEACON-NOTIFY.indication params. See  802.15.4-2011 and 802.15.4e-2012   Section 6.2.4.1, Table 16
 */
struct MlmeBeaconNotifyIndicationParams
{
    uint8_t m_bsn{0};              //!< The beacon sequence number.
    PanDescriptor m_panDescriptor; //!< The PAN descriptor for the received beacon.
    PendingAddrFields m_pendAddrSpec;       //!< The beacon pending address specification.
    std::vector<Mac64Address> m_addrList;   //!< The addresses of the devices for which
                                            //!< the beacon source has data.
    uint32_t m_sduLength{0};       //!< The number of octets contained in the beacon payload.
    Ptr<Packet> m_sdu;             //!< The set of octets comprising the beacon payload.

    uint8_t m_ebsn{0};                      //!< Beacon sequence number used for enhanced beacon frames
    bool m_beaconType{0};                   //!< Indicates a beacon (0x00) or enhanced beacon (0x01) was received
};

/**
 * \ingroup lr-wpan
 *
 * MLME-SYNC-LOSS.indication params. See  802.15.4-2011   Section 6.2.13.2, Table 37
 */
struct MlmeSyncLossIndicationParams
{
    LrWpanSyncLossReason m_lossReason{
        MLMESYNCLOSS_PAN_ID_CONFLICT}; //!< The reason for the lost of synchronization.
    uint16_t m_panId{0}; //!< The PAN identifier with which the device lost synchronization or to
                         //!< which it was realigned.
    uint8_t m_logCh{11}; //!< The channel number on which the device lost synchronization or to
                         //!< which it was realigned.
    
    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex   
};

/**
 * \ingroup lr-wpan
 *
 * MLME-COMM-STATUS.indication params. See  802.15.4-2011   Section 6.2.4.2 Table 18
 */
struct MlmeCommStatusIndicationParams
{
    uint16_t m_panId{0}; //!< The PAN identifier of the device from which the frame was received or
                         //!< to which the frame was being sent.
    uint8_t m_srcAddrMode{SHORT_ADDR}; //!< The source addressing mode for this primitive
    Mac16Address m_srcShortAddr; //!< The short address of the entity from which the frame causing
                                 //!< the error originated.
    Mac64Address m_srcExtAddr; //!< The extended address of the entity from which the frame causing
                               //!< the error originated.
    uint8_t m_dstAddrMode{SHORT_ADDR}; //!< The destination addressing mode for this primitive.
    Mac16Address
        m_dstShortAddr; //!< The short address of the device for which the frame was intended.
    Mac64Address
        m_dstExtAddr; //!< The extended address of the device for which the frame was intended.
    LrWpanMlmeCommStatus m_status{MLMECOMMSTATUS_INVALID_PARAMETER}; //!< The communication status
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-GTS.request params. See  802.15.4e-2012   Section 6.2.21.1.1 Table 44q
 */
struct MlmeDsmeGtsRequestParams {
    Mac16Address m_devAddr;                     //<! The short address of the neighboring device
                                                //<! to request the management of DSME-GTSs.
    ManagementType m_manageType{GTS_ALLOCATION};    
    uint8_t m_direction{0};                     //<! 0x00: TX (Transmission), 0x01: RX (Reception)
    uint8_t m_prioritizedChAccess{0};           //<! 0x00: low priority, 0x01: high priority
    uint8_t m_numSlot;                          //<! The number of slots to be requested for
                                                //<! allocation.

    uint16_t m_preferredSuperframeID;           //<! 0x0000-0xffff, The superframe ID is the sequence 
                                                //<! number of the superframe in a multi-superframe 
                                                //<! beginning from zero.

    uint8_t m_preferredSlotID;                  //<! 0x00-0x0e, The slot ID is the sequence number of the DSME-GTSs
                                                //<! (not including beacon or CAP slots) in a
                                                //<! superframe beginning from zero.
    
    DSMESABSpecificationField m_dsmeSABSpec;    //<! SAB of GTS infos when GTS request

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex    
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-GTS.indication params. See  802.15.4e-2012   Section 6.2.21.1.2 Table 44r
 */
struct MlmeDsmeGtsIndicationParams {
    Mac16Address m_devAddr;                     //<! The short address of the neighboring device
                                                //<! to request the management of DSME-GTSs.
    ManagementType m_manageType{GTS_ALLOCATION};    
    uint8_t m_direction{0};                     //<! 0x00: TX (Transmission), 0x01: RX (Reception)
    uint8_t m_prioritizedChAccess{0};           //<! 0x00: low priority, 0x01: high priority
    uint8_t m_numSlot;                          //<! The number of slots to be requested for
                                                //<! allocation. This
    uint16_t m_preferredSuperframeID;           //<! 0x0000-0xffff
    uint8_t m_preferredSlotID;                  //<! 0x00-0x0e
    DSMESABSpecificationField m_dsmeSABSpec;
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-GTS.response params. See  802.15.4e-2012   Section 6.2.21.1.3 Table 44s
 */
struct MlmeDsmeGtsResponseParams {
    Mac16Address m_devAddr;                     //<! The short address of the neighboring device
                                                //<! to request the management of DSME-GTSs.
    ManagementType m_manageType{GTS_ALLOCATION};    
    uint8_t m_direction{0};                     //<! 0x00: TX (Transmission), 0x01: RX (Reception)
    uint8_t m_prioritizedChAccess{0};           //<! 0x00: low priority, 0x01: high priority
    uint16_t m_channelOfs;                      //<! This parameter specifies the offset value of
                                                //<! Hopping Sequence.
    DSMESABSpecificationField m_dsmeSABSpec;
    LrWpanMlmeDsmeGtsResponseStatus m_status{MLMEDSMEGTS_SUCCESS};
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-GTS.confirm params. See  802.15.4e-2012   Section 6.2.21.1.4 Table 44t
 */
struct MlmeDsmeGtsConfirmParams {
    Mac16Address m_devAddr;                         //<! The 16-bit short address of the device that has
                                                    //<! transmitted the received DSME-GTS reply command.
    ManagementType m_manageType{GTS_ALLOCATION};    //<! The type of the management request
    uint8_t m_direction{0};                         //<! The direction of DSME-GTSs.
    uint8_t m_prioritizedChAccess{0};               //<! The direction of DSME-GTSs.
    uint16_t m_channelOfs;
    DSMESABSpecificationField m_dsmeSABSpec;
    LrWpanMlmeDsmeGtsRequestStatus m_status{MLMEDSMEGTS_REQ_SUCCESS};
};

/**
 * \ingroup lr-wpan
 * 
 * MLME-ORPHAN.indication params. See 802.15.4-2011 Section 6.2.7.1
 */
struct MlmeOrphanIndicationPararms {
    Mac64Address m_orphanAddress;

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex    
};

/**
 * \ingroup lr-wpan
 * 
 * MLME-ORPHAN.reponse params. See 802.15.4-2011 Section 6.2.7.2
 */
struct MlmeOrphanResponsePararms {
    Mac64Address m_orphanAddress;
    Mac16Address m_shortAddr;
    bool m_associateMember;

    // SecurityLevel,
    // KeyIdMode,
    // KeySource,
    // KeyIndex    
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-INFO.request params. See  802.15.4e-2012   Section 6.2.21.2.1 Table 44u
 */
struct MlmeDsmeInfoRequestParams {
    uint8_t m_dstAddrMode{SHORT_ADDR};
    Mac16Address m_dstShortAddr;                                           
    Mac64Address m_dstExtAddr;
    InfoType m_info{MLMEDSMEINFO_TIMESTAMP};
    uint8_t m_dsmeSABSubBlkLen;                 // 0x00-0x0ff
    uint16_t m_dsmeSABSubBlkIdx;                // 0x00-0xff
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-INFO.indication params. See  802.15.4e-2012   Section 6.2.21.2.2 Table 44v
 */
struct MlmeDsmeInfoIndicationParams {
    Mac64Address m_devAddr;
    InfoType m_info{MLMEDSMEINFO_TIMESTAMP};
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-INFO.confirm params. See  802.15.4e-2012   Section 6.2.21.2.3 Table 44w
 */
struct MlmeDsmeInfoConfirmParams {
    InfoType m_info{MLMEDSMEINFO_TIMESTAMP};
    uint64_t m_timestamp;
    uint16_t m_superframeID;                                            //!< The ID of the superframe slot in which the DSMEInformation
                                                                        //!< reply command was transmitted
    uint8_t m_slotID;
    DSMESABSpecificationField m_dsmeSABSpec;                            //!< The information of the current DSME-GTS
                                                                        //!< allocation status and slot availability in
                                                                        //!< one hop neighborhood of the replying device
    DsmePANDescriptorIE m_panDescriptor;                                      //!< The information of the configurations of the DSME 
                                                                        //!< enabled PAN.
    LrWpanMlmeDsmeInfoResponseStatus m_status{MLMEDSMEINFO_SUCCESS};
};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-LINKSTATUSRPT.request params. See  802.15.4e-2012   Section 6.2.21.3.1 Table 44x
 */
struct MlmeDsmeLinkStatusReportRequestParams {

};

/**
 * \ingroup lr-wpan
 *
 * MLME-DSME-LINKSTATUSRPT.indication params. See  802.15.4e-2012   Section 6.2.21.3.2 Table 44y
 */
struct MlmeDsmeLinkStatusRptIndicationParams {

};

/**
 * \ingroup dsme
 *
 * MLME-DSME-LINKSTATUSRPT.confirm params. See  802.15.4e-2012   Section 6.2.21.3.3 Table 44z
 */
struct MlmeDsmeLinkStatusRptConfirmParams {

};

/**
 * \ingroup lr-wpan
 *
 * MLME-START.confirm params. See  802.15.4-2011   Section 6.2.14.2
 */
struct MlmePollConfirmParams
{
    LrWpanMlmePollConfirmStatus m_status{
        MLMEPOLL_INVALID_PARAMETER}; //!< The confirmation status resulting from a
                                     //!< MLME-poll.request.
};

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a McpsDataRequest has been called from
 * the higher layer.  It returns a status of the outcome of the
 * transmission request
 */
typedef Callback<void, McpsDataConfirmParams> McpsDataConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mcps has successfully received a
 *  frame and wants to deliver it to the higher layer.
 *
 *  \todo for now, we do not deliver all of the parameters in section
 *  802.15.4-2006 7.1.1.3.1 but just send up the packet.
 */
typedef Callback<void, McpsDataIndicationParams, Ptr<Packet>> McpsDataIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeStartRequest has been called from
 * the higher layer.  It returns a status of the outcome of the
 * transmission request
 */
typedef Callback<void, MlmeStartConfirmParams> MlmeStartConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme has successfully received a
 *  beacon frame and wants to deliver it to the higher layer.
 *
 *  \todo for now, we do not deliver all of the parameters in section
 *  802.15.4-2006 6.2.4.1 but just send up the packet.
 */
typedef Callback<void, MlmeBeaconNotifyIndicationParams, Ptr<Packet>>
    MlmeBeaconNotifyIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called to indicate the loss of synchronization with
 * a coordinator.
 *
 * \todo for now, we do not deliver all of the parameters in section
 *  See IEEE 802.15.4-2011 6.2.13.2.
 */
typedef Callback<void, MlmeSyncLossIndicationParams> MlmeSyncLossIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme-Poll.Request has been called from
 * the higher layer.  It returns a status of the outcome of the
 * transmission request
 */
typedef Callback<void, MlmePollConfirmParams> MlmePollConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeScanRequest has been called from
 * the higher layer.  It returns a status of the outcome of the scan.
 */
typedef Callback<void, MlmeScanConfirmParams> MlmeScanConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeAssociateRequest has been called from
 * the higher layer. It returns a status of the outcome of the
 * association request
 */
typedef Callback<void, MlmeAssociateConfirmParams> MlmeAssociateConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme has successfully received a command
 *  frame and wants to deliver it to the higher layer.
 *
 *  Security related parameters and not handle.
 *  See 802.15.4-2011 6.2.2.2.
 */
typedef Callback<void, MlmeAssociateIndicationParams> MlmeAssociateIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called by the MLME and issued to its next higher layer following
 * a transmission instigated through a response primitive.
 *
 *  Security related parameters and not handle.
 *  See 802.15.4-2011 6.2.4.2
 */
typedef Callback<void, MlmeCommStatusIndicationParams> MlmeCommStatusIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called by the MLME when the MLME receive a disassociation
 * notification command.
 *
 *  See 802.15.4-2011 6.2.3.2
 */
typedef Callback<void, MlmeDisassociateIndicationParams> MlmeDisassociateIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeDisassociateRequest has been called from
 * the higher layer. It returns a status of the outcome of the
 * disassociation request
 * 
 *  Security related parameters and not handle.
 *  See 802.15.4-2011 6.2.3.3
 */
typedef Callback<void, MlmeDisassociateConfirmParams> MlmeDisassociateConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme has successfully received a DSME-GTS request 
 * command frame and wants to deliver it to the higher layer.
 * 
 *  Security related parameters and not handle.
 *  See 802.15.4e-2012   Section 6.2.21.1.2
 */
typedef Callback<void, MlmeDsmeGtsIndicationParams> MlmeDsmeGtsIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme has successfully received a DSME-GTS response 
 * command frame and wants to deliver it to the higher layer or Timeout when DSME-GTS response is not received.
 * 
 *  Security related parameters and not handle.
 *  See 802.15.4e-2012   Section 6.2.21.1.4
 */
typedef Callback<void, MlmeDsmeGtsConfirmParams> MlmeDsmeGtsConfirmCallback;

/**
 * \ingroup lr-wpan
 * 
 * This callback is called after a Mlme has successfully received a orphan notification 
 * command frame and to deliver it to the higher layer
 * 
 * Security related parameters and not handle.
 * See 802.15.4e-2011 Section 6.2.7.1
 */
typedef Callback<void, MlmeOrphanIndicationPararms> MlmeOrphanIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme has successfully received a DSME-Infomation request 
 * command frame and wants to deliver it to the higher layer.
 * 
 *  See 802.15.4e-2012   Section 6.2.21.2.2
 */
typedef Callback<void, MlmeDsmeInfoIndicationParams> MlmeDsmeInfoIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeDsmeInfoRequest has been called from
 * the higher layer. It returns a status of the outcome of the
 * MLME-DSME-INFO request
 * 
 *  See 802.15.4e-2012   Section 6.2.21.2.3
 */
typedef Callback<void, MlmeDsmeInfoConfirmParams> MlmeDsmeInfoConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mlme has successfully received a 
 * MLME-DSME-LINKSTATUSRPT request command frame 
 * and wants to deliver it to the higher layer.
 * 
 *  See 802.15.4e-2012   Section 6.2.21.3.2
 */
typedef Callback<void, MlmeDsmeLinkStatusRptIndicationParams> MlmeDsmeLinkStatusReportIndicationCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeDsmeLinkStatusRequest has been called from
 * the higher layer. It returns a status of the outcome of the
 * MLME-DSME-LINKSTATUSRPT request
 * 
 *  See 802.15.4e-2012   Section 6.2.21.3.3
 */
typedef Callback<void, MlmeDsmeLinkStatusRptConfirmParams> MlmeDsmeLinkStatusRptConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a MlmeDsmeLinkStatusRequest has been called from
 * the higher layer. It returns a status of the outcome of the
 * MLME-DSME-LINKSTATUSRPT request
 * 
 *  See 802.15.4e-2012   Section 6.2.21.3.3
 */
typedef Callback<void, MlmeStartRequestParams> MlmeStartRequestCallback;

/**
 * \ingroup lr-wpan
 *
 * Allocation counter table(macDSMEACT)
 * 
 *  See 802.15.4e-2012   Section 5.1.10.5.3, Table 1a
 */
struct macDSMEACTEntity {
    uint16_t m_superframeID;            //<! 0x0000-0xfffd, The superframe ID of the DSME-GTS in a multi-superframe.
    uint8_t m_slotID;                   //<! 0x00-0x0e, The slot ID of the DSME-GTS in the superframe.
    uint8_t m_numSlot;                  //<! The number of slots to be requested for
                                        //<! allocation.

    uint16_t m_channelID;               //<! 0x0000–0xffff, In channel adaptation, this field shall contain the Channel
                                        //<! number of the DSME-GTS. In channel hopping, this field
                                        //<! shall contain the Channel Offset.

    uint8_t m_direction;                //<! The direction of the allocated DSME-GTS.
                                        //<! 0: Transmission (TX), 1: Reception (RX)

    uint8_t m_type;                     //<! The type of the DSME-GTS.
                                        //<! 0x00: (regular) DSME-GTS
                                        //<! 0x01: DSME-GTSR
                                        //<! 0x02: GACK1
                                        //<! 0x03: GACK2

    bool m_prioritizedChAccess;

    Mac16Address m_srcAddr;             //<! The 16-bit short address of the device that is the source (if RX)
                                        //<! or the destination of the allocated DSME-GTS.
    Mac16Address m_dstAddr;

    uint16_t m_cnt;                     //<! An idle counter, in other word, the number of idle multisuperframes
                                        //<! since the allocated DSME-GTS was used.

    uint16_t m_linkQuality;             //<! The link quality of the allocated DSME-GTS.

    bool m_allocated = false;
    bool m_deallocated = false;
    bool m_expired = false;
};


/**
 * \ingroup lr-wpan
 *
 * Class that implements the LR-WPAN MAC state machine
 */
class LrWpanMac : public Object
{
  public:

    /**
     * Default constructor.
     */
    LrWpanMac();
    ~LrWpanMac() override;

    /**
     * Get the type ID.
     *
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    // MAC sublayer constants
    /**
     * The minimum number of octets added by the MAC sublayer to the PSDU.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aMinMPDUOverhead = 9;

    /**
     * Length of a superframe slot in symbols. Defaults to 60 symbols in each
     * superframe slot.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aBaseSlotDuration = 60;

    /**
     * Number of a superframe slots per superframe. Defaults to 16.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aNumSuperframeSlots = 16;

    /**
     * Length of a superframe in symbols. Defaults to
     * aBaseSlotDuration * aNumSuperframeSlots in symbols.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aBaseSuperframeDuration = aBaseSlotDuration * aNumSuperframeSlots;

    /**
     * The number of superframes in which a GTS descriptor
     * exists in the beacon frame of the PAN coordinator.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aGTSDescPersistenceTime = 4;    

    /**
     * The number of consecutive lost beacons that will
     * cause the MAC sublayer of a receiving device to
     * declare a loss of synchronization.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aMaxLostBeacons = 4;

    /**
     * The maximum size of an MPDU, in octets, that can be
     * followed by a Short InterFrame Spacing (SIFS) period.
     * See IEEE 802.15.4-2011, section 6.4.1, Table 51.
     */
    static constexpr uint32_t aMaxSIFSFrameSize = 18;

    /**
     * Check if the receiver will be enabled when the MAC is idle.
     *
     * \return true, if the receiver is enabled during idle periods, false otherwise
     */
    bool GetRxOnWhenIdle();

    /**
     * Set if the receiver should be enabled when the MAC is idle.
     *
     * \param rxOnWhenIdle set to true to enable the receiver during idle periods
     */
    void SetRxOnWhenIdle(bool rxOnWhenIdle);

    // XXX these setters will become obsolete if we use the attribute system
    /**
     * Set the simple address of this MAC.
     *
     * \param address the new address
     */
    void SetSimpleAddress(Mac8Address address);

    /**
     * Get the simple address of this MAC.
     *
     * \return the simple address
     */
    Mac8Address GetSimpleAddress() const;    

    // XXX these setters will become obsolete if we use the attribute system
    /**
     * Set the short address of this MAC.
     *
     * \param address the new address
     */
    void SetShortAddress(Mac16Address address);

    /**
     * Get the short address of this MAC.
     *
     * \return the short address
     */
    Mac16Address GetShortAddress() const;

    /**
     * Set the extended address of this MAC.
     *
     * \param address the new address
     */
    void SetExtendedAddress(Mac64Address address);

    /**
     * Get the extended address of this MAC.
     *
     * \return the extended address
     */
    Mac64Address GetExtendedAddress() const;

    /**
     * Set the PAN id used by this MAC.
     *
     * \param panId the new PAN id.
     */
    void SetPanId(uint16_t panId);

    /**
     * Get the PAN id used by this MAC.
     *
     * \return the PAN id.
     */
    uint16_t GetPanId() const;

    /**
     * Get the coordinator short address currently associated to this device.
     *
     * \return The coordinator short address
     */
    Mac16Address GetCoordShortAddress() const;

    /**
     * Get the coordinator extended address currently associated to this device.
     *
     * \return The coordinator extended address
     */
    Mac64Address GetCoordExtAddress() const;

    /**
     * Get the current channel hopping offset.
     */
    uint16_t GetChannelOffset() const;

    /**
     * IEEE 802.15.4-2006, section 7.1.1.1
     * MCPS-DATA.request
     * Request to transfer a MSDU.
     *
     * \param params the request parameters
     * \param p the packet to be transmitted
     */
    void McpsDataRequest(McpsDataRequestParams params, Ptr<Packet> p);

    /**
     * IEEE 802.15.4e-2012, section 6.2.18.1
     * MLME-BEACON.request
     * Requests the generation of a beacon or enhanced beacon in a nonbeacon-enabled PAN
     * , either in response to a beacon request command when macBeaconAutoRespond is FALSE
     *
     * \param params the request parameters
     */
    void MlmeMlmeBeaconRequest(MlmeBeaconRequestParams params);

    /**
     * IEEE 802.15.4-2006, section 7.1.14.1
     * MLME-START.request
     * Request to allow a PAN coordinator to initiate
     * a new PAN or beginning a new superframe configuration.
     *
     * \param params the request parameters
     */
    void MlmeStartRequest(MlmeStartRequestParams params);

    /**
     * IEEE 802.15.4-2011, section 6.2.10.1
     * MLME-SCAN.request
     * Request primitive used to initiate a channel scan over a given list of channels.
     *
     * \param params the scan request parameters
     */
    void MlmeScanRequest(MlmeScanRequestParams params);

    /**
     * IEEE 802.15.4-2011, section 6.2.2.1
     * MLME-ASSOCIATE.request
     * Request primitive used by a device to request an association with
     * a coordinator.
     *
     * \param params the request parameters
     */
    void MlmeAssociateRequest(MlmeAssociateRequestParams params);

    /**
     * IEEE 802.15.4-2011, section 6.2.2.3
     * MLME-ASSOCIATE.response
     * Primitive used to initiate a response to an MLME-ASSOCIATE.indication
     * primitive.
     *
     * \param params the associate response parameters
     */
    void MlmeAssociateResponse(MlmeAssociateResponseParams params);

    /**
     * IEEE 802.15.4-2011 Section 6.2.3.1 and Table 13, Table 7
     * MLME-ASSOCIATE.request
     * Request primitive used by a device to request an disassociation with
     * a coordinator.
     *
     * \param params the request parameters
     */
    void MlmeDisassociateRequest(MlmeDisassociateRequestParams params);    

    /**
     * IEEE 802.15.4-2011, section 6.2.13.1
     * MLME-SYNC.request
     * Request to synchronize with the coordinator by acquiring and,
     * if specified, tracking beacons.
     *
     * \param params the request parameters
     */
    void MlmeSyncRequest(MlmeSyncRequestParams params);

    /**
     * IEEE 802.15.4-2011, section 6.2.14.2
     * MLME-POLL.request
     * Prompts the device to request data from the coordinator.
     *
     * \param params the request parameters
     */
    void MlmePollRequest(MlmePollRequestParams params);

    /**
     * IEEE 802.15.4e-2012   Section 6.2.21.1.1 Table 44q
     * MLME-DSME-GTS.request
     * Prompts the device to request an allocation of new DSME-GTS 
     * or deallocation, duplicated allocation notification
     * , reduce, or restart of existing DSME-GTSs
     *
     * \param params the request parameters
     */
    void MlmeDsmeGtsRequest(MlmeDsmeGtsRequestParams params);

    /**
     * IEEE 802.15.4e-2012 Section 6.2.21.1.3 Table 44s
     * MLME-DSME-GTS.response
     * Primitive used to initiate a response to an MLME-DSME-GTS.indication
     * primitive.
     *
     * \param params the associate response parameters
     */
    void MlmeDsmeGtsResponse(MlmeDsmeGtsResponseParams params);

    /**
     * 802.15.4e-2012 Section 6.2.7.2 Table 24
     * MLME-ORPHAN.response
     * Prmitive used to initiate a response to an MLME-ORPHAN.indication
     * primitive.
     * 
     * \param params the orphan response parameters
     */
    void MlmeOrphanResponse(MlmeOrphanResponsePararms params);

    /**
     * IEEE 802.15.4e-2012   Section 6.2.21.2.1 Table 44u
     * MLME-DSME-Info.request
     * Prompts the device to request the timestamp and the
     * DSMESABSpecification of the Destination device or 
     * the DSME PAN Descriptor of the Connection device.
     *
     * \param params the request parameters
     */
    void MlmeDsmeInfoRequest(MlmeDsmeInfoRequestParams params);

    /**
     * IEEE 802.15.4e-2012   Section 6.2.21.3.1 Table 44x
     * MLME-DSME-LINKSTATUSRPT.request
     * Prompts the device to request a device start a link quality statistic 
     * and periodically report the statistic results to the destination device.
     *
     * \param params the request parameters
     */
    void MlmeDsmeLinkStatusReportRequest(MlmeDsmeLinkStatusReportRequestParams params);    

    void SendCoordinatorRealignmentCmd(bool orphanOrNot
                                      , bool channelPagePresent
                                      , Mac64Address dst
                                      , Mac16Address storedShortAddr);

    void SendDsmeGtsReplyWithInvalidParam(Mac16Address dst
                                          , DSMESABSpecificationField sAB);

    /**
     * Set the CSMA/CA implementation to be used by the MAC.
     *
     * \param csmaCa the CSMA/CA implementation
     */
    void SetCsmaCa(Ptr<LrWpanCsmaCa> csmaCa);

    /**
     * Set the underlying PHY for the MAC.
     *
     * \param phy the PHY
     */
    void SetPhy(Ptr<LrWpanPhy> phy);

    /**
     * Get the underlying PHY of the MAC.
     *
     * \return the PHY
     */
    Ptr<LrWpanPhy> GetPhy();

    /**
     * Set the callback for the indication of an incoming data packet.
     * The callback implements MCPS-DATA.indication SAP of IEEE 802.15.4-2006,
     * section 7.1.1.3.
     *
     * \param c the callback
     */
    void SetMcpsDataIndicationCallback(McpsDataIndicationCallback c);

    /**
     * Set the callback for the indication of an incoming associate request command.
     * The callback implements MLME-ASSOCIATE.indication SAP of IEEE 802.15.4-2011,
     * section 6.2.2.2.
     *
     * \param c the callback
     */
    void SetMlmeAssociateIndicationCallback(MlmeAssociateIndicationCallback c);

    /**
     * Set the callback for the indication of an incoming disassociate request command.
     * The callback implements MLME-DISASSOCIATE.indication SAP of IEEE 802.15.4-2011,
     * Section 6.2.3.2.
     *
     * \param c the callback
     */
    void SetMlmeDisassociateIndicationCallback(MlmeDisassociateIndicationCallback c);

    /**
     * Set the callback for the indication to a response primitive.
     * The callback implements MLME-COMM-STATUS.indication SAP of IEEE 802.15.4-2011,
     * section 6.2.4.2.
     *
     * \param c the callback
     */
    void SetMlmeCommStatusIndicationCallback(MlmeCommStatusIndicationCallback c);

    void SetMlmeStartRequestCallback(MlmeStartRequestCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MCPS-DATA.confirm SAP of IEEE 802.15.4-2006,
     * section 7.1.1.2.
     *
     * \param c the callback
     */
    void SetMcpsDataConfirmCallback(McpsDataConfirmCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-START.confirm SAP of IEEE 802.15.4-2006,
     * section 7.1.14.2.
     *
     * \param c the callback
     */
    void SetMlmeStartConfirmCallback(MlmeStartConfirmCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-SCAN.confirm SAP of IEEE 802.15.4-2011,
     * section 6.2.10.2.
     *
     * \param c the callback
     */
    void SetMlmeScanConfirmCallback(MlmeScanConfirmCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-ASSOCIATE.confirm SAP of IEEE 802.15.4-2011,
     * section 6.2.2.4.
     *
     * \param c the callback
     */
    void SetMlmeAssociateConfirmCallback(MlmeAssociateConfirmCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-DISASSOCIATE.confirm SAP of IEEE 802.15.4-2011,
     * section 6.2.3.3
     *
     * \param c the callback
     */
    void SetMlmeDisassociateConfirmCallback(MlmeDisassociateConfirmCallback c);

    /**
     * Set the callback for the indication of an incoming beacon packet.
     * The callback implements MLME-BEACON-NOTIFY.indication SAP of IEEE 802.15.4-2011,
     * section 6.2.4.1.
     *
     * \param c the callback
     */
    void SetMlmeBeaconNotifyIndicationCallback(MlmeBeaconNotifyIndicationCallback c);

    /**
     * Set the callback for the loss of synchronization with a coordinator.
     * The callback implements MLME-BEACON-NOTIFY.indication SAP of IEEE 802.15.4-2011,
     * section 6.2.13.2.
     *
     * \param c the callback
     */
    void SetMlmeSyncLossIndicationCallback(MlmeSyncLossIndicationCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-POLL.confirm SAP of IEEE 802.15.4-2011,
     * section 6.2.14.2
     *
     * \param c the callback
     */
    void SetMlmePollConfirmCallback(MlmePollConfirmCallback c);

    /**
     * Set the callback for the indication of an incoming dsme gts request command.
     * The callback implements MLME-DSME-GTS.indication SAP of IEEE 802.15.4e-2012,
     * Section 6.2.21.1.2.
     *
     * \param c the callback
     */
    void SetMlmeDsmeGtsIndicationCallback(MlmeDsmeGtsIndicationCallback c);

    /**
     * Set the callback for the confirm of an outgoing dsme gts request command.
     * The callback implements MLME-DSME-GTS.confirm SAP of IEEE 802.15.4e-2012,
     * Section 6.2.21.1.4.
     */
    void SetMlmeDsmeGtsConfirmCallback(MlmeDsmeGtsConfirmCallback c);

    /**
     * Set the callback for the indication of an incoming orphan notification command.
     * The callback implements MLME-ORPHAN.indication SAP of IEEE 802.15.4-2011,
     * Section 6.2.7.1.
     * 
     * \param c the callback
     */
    void SetMlmeOrphanIndicationCallback(MlmeOrphanIndicationCallback c);

    /**
     * Set the callback for the indication of an incoming dsme info request command.
     * The callback implements MLME-DSME-INFO.indication SAP of IEEE 802.15.4e-2012,
     * Section 6.2.21.2.2.
     *
     * \param c the callback
     */
    void SetMlmeDsmeInfoIndicationCallback(MlmeDsmeInfoIndicationCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-DSME-INFO.confirm SAP of IEEE 802.15.4e-2012,
     * section 6.2.21.2.3.
     *
     * \param c the callback
     */    
    void SetMlmeDsmeInfoConfirmCallback(MlmeDsmeInfoConfirmCallback c);

    /**
     * Set the callback for the indication of an incoming dsme link status report 
     * request command.
     * The callback implements MLME-DSME-INFO.indication SAP of IEEE 802.15.4e-2012,
     * Section 6.2.21.3.2.
     *
     * \param c the callback
     */    
    void SetMlmeDsmeLinkStatusRptIndicationCallback(MlmeDsmeLinkStatusReportIndicationCallback c);

    /**
     * Set the callback for the confirmation of a data transmission request.
     * The callback implements MLME-DSME-LINKSTATUSRPT.confirm SAP of IEEE 802.15.4e-2012,
     * section 6.2.21.3.3
     *
     * \param c the callback
     */    
    void SetMlmeDsmeLinkStatusRptConfirmCallback(MlmeDsmeLinkStatusRptConfirmCallback c);    

    // interfaces between MAC and PHY

    /**
     * IEEE 802.15.4-2006 section 6.2.1.3
     * PD-DATA.indication
     * Indicates the transfer of an MPDU from PHY to MAC (receiving)
     * \param psduLength number of bytes in the PSDU
     * \param p the packet to be transmitted
     * \param lqi Link quality (LQI) value measured during reception of the PPDU
     */
    void PdDataIndication(uint32_t psduLength, Ptr<Packet> p, uint8_t lqi);

    /**
     * IEEE 802.15.4-2006 section 6.2.1.2
     * Confirm the end of transmission of an MPDU to MAC
     * \param status to report to MAC
     *        PHY PD-DATA.confirm status
     */
    void PdDataConfirm(LrWpanPhyEnumeration status);

    /**
     * IEEE 802.15.4-2006 section 6.2.2.2
     * PLME-CCA.confirm status
     * \param status TRX_OFF, BUSY or IDLE
     */
    void PlmeCcaConfirm(LrWpanPhyEnumeration status);

    /**
     * IEEE 802.15.4-2006 section 6.2.2.4
     * PLME-ED.confirm status and energy level
     * \param status SUCCESS, TRX_OFF or TX_ON
     * \param energyLevel 0x00-0xff ED level for the channel
     */
    void PlmeEdConfirm(LrWpanPhyEnumeration status, uint8_t energyLevel);

    /**
     * IEEE 802.15.4-2006 section 6.2.2.6
     * PLME-GET.confirm
     * Get attributes per definition from Table 23 in section 6.4.2
     * \param status SUCCESS or UNSUPPORTED_ATTRIBUTE
     * \param id the attributed identifier
     * \param attribute the attribute value
     */
    void PlmeGetAttributeConfirm(LrWpanPhyEnumeration status,
                                 LrWpanPibAttributeIdentifier id,
                                 LrWpanPhyPibAttributes* attribute);

    /**
     * IEEE 802.15.4-2006 section 6.2.2.8
     * PLME-SET-TRX-STATE.confirm
     * Set PHY state
     * \param status in RX_ON,TRX_OFF,FORCE_TRX_OFF,TX_ON
     */
    void PlmeSetTRXStateConfirm(LrWpanPhyEnumeration status);

    /**
     * IEEE 802.15.4-2006 section 6.2.2.10
     * PLME-SET.confirm
     * Set attributes per definition from Table 23 in section 6.4.2
     * \param status SUCCESS, UNSUPPORTED_ATTRIBUTE, INVALID_PARAMETER, or READ_ONLY
     * \param id the attributed identifier
     */
    void PlmeSetAttributeConfirm(LrWpanPhyEnumeration status, LrWpanPibAttributeIdentifier id);

    void SetLrWpanMacStateToGTS(uint16_t superframeID, int idx);

    /**
     * CSMA-CA algorithm calls back the MAC after executing channel assessment.
     *
     * \param macState indicate BUSY oder IDLE channel condition
     */
    void SetLrWpanMacState(LrWpanMacState macState);

    /**
     * Get the current association status.
     *
     * \return current association status
     */
    LrWpanAssociationStatus GetAssociationStatus() const;

    /**
     * Set the current association status.
     *
     * \param status new association status
     */
    void SetAssociationStatus(LrWpanAssociationStatus status);

    /**
     * Set the coordinator does allow end device associate with it
     *
     */
    void SetAssociatePermit();

    /**
     * Set the coordinator does not allow end device associate with it
     *
     */
    void SetAssociateNotPermit();

    /**
     * Set the max size of the transmit queue.
     *
     * \param queueSize The transmit queue size.
     */
    void SetTxQMaxSize(uint32_t queueSize);

    /**
     * Set the max size of the indirect transmit queue (Pending Transaction list)
     *
     * \param queueSize The indirect transmit queue size.
     */
    void SetIndTxQMaxSize(uint32_t queueSize);

    // Beacon scheduling 

    void SetBcnSchedulingAllocStatus(uint8_t allocStatus);
    uint8_t GetBcnSchedulingAllocStatus() const;
    // MAC PIB attributes

    /**
     * The time that the device transmitted its last beacon frame.
     * It also indicates the start of the Active Period in the Outgoing superframe.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    Time m_macBeaconTxTime;
    Time m_BeaconStartTxTime;

    /**
     * Indicate the start time of the beacon slot
     * specify the time of beacon transmission in units of microseconds.
     */
    Time m_startOfBcnSlot;

    /**
     * Indicate the start time of the beacon slot of the Parent device.
     * specify the time of beacon transmission in units of microseconds.
     */
    Time m_startOfBcnSlotOfSyncParent;

    bool m_becomeCoord;

    /**
     * Flag to indicate a normal coordinator(Not Pan coordinator) should transmit a beacon
     * currently.
     */
    bool m_sendBcn;

    /**
     * Flag to indicate that if a coordinator realignment command is received while orphan scanning
     * to terminate the scaning.
     */
    bool realignmentRecevied;

    /**
     * The time that the device received its last bit of the beacon frame.
     * It does not indicate the start of the Active Period in the Incoming superframe.
     * Not explicitly listed by the standard but its use is implied.
     * Its purpose is somehow similar to m_macBeaconTxTime
     */
    Time m_macBeaconRxTime;

    Time m_bcnProcessTime;

    /**
     * The maximum time, in multiples of aBaseSuperframeDuration, a device
     * shall wait for a response command frame to be available following a
     * request command frame.
     */
    uint64_t m_macResponseWaitTime;

    /**
     * The maximum wait time for an association response command after the reception
     * of data request command ACK during the association process. Not explicitly
     * listed by the standard but its use is required for a device to react to the lost
     * of the association response (failure of the association: NO_DATA)
     */
    uint64_t m_assocRespCmdWaitTime;

    /**
     * The short address of the coordinator through which the device is
     * associated.
     * 0xFFFF indicates this value is unknown.
     * 0xFFFE indicates the coordinator is only using its extended address.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    Mac16Address m_macCoordShortAddress;

    /**
     * The extended address of the coordinator through which the device
     * is associated.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    Mac64Address m_macCoordExtendedAddress;

    /**
     * Symbol boundary is same as m_macBeaconTxTime.
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    uint64_t m_macSyncSymbolOffset;

    /**
     * Used by a PAN coordinator or coordinator.
     * Defines how often the coordinator transmits its beacon
     * (outgoing superframe). Range 0 - 15 with 15 meaning no beacons are being sent.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    uint8_t m_macBeaconOrder;

    /**
     * Used by a PAN coordinator or coordinator. The length of the active portion
     * of the outgoing superframe, including the beacon frame.
     * 0 - 15 with 15 means the superframe will not be active after the beacon.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    uint8_t m_macSuperframeOrder;

    /**
     * The maximum time (in UNIT periods) that a transaction is stored by a
     * coordinator and indicated in its beacon. This value establish the expiration
     * time of the packets stored in the pending transaction list (indirect transmissions).
     * 1 Unit Period:
     * Beacon-enabled = aBaseSuperframeDuration * 2^BO
     * Non-beacon enabled = aBaseSuperframeDuration
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    uint16_t m_macTransactionPersistenceTime;

    /**
     * The index of a vacant time slot in Beacon bitmap.
     * Used for the MlmeStartRequest of non-Pan coordinator.
     */
    uint16_t m_choosedSDIndexToSendBcn = 0;

    /**
     * The index of the associated Pan in the Pan Descriptor List.
     * To extract the information of BO, MO, SO.
     */
    int m_descIdxOfAssociatedPan;

    /**
     * The total size of the received beacon in symbols.
     * Its value is used to calculate the end CAP time of the incoming superframe.
     */
    uint64_t m_rxBeaconSymbols;

    /**
     * Indication of the Slot where the CAP portion of the OUTGOING Superframe ends.
     */
    uint8_t m_fnlCapSlot;

    /**
     * The beaconOrder value of the INCOMING frame. Used by all devices that have a parent.
     * Specification of how often the parent coordinator transmits its beacon.
     * 0 - 15 with 15 means the parent is not currently transmitting beacons.
     */
    uint8_t m_incomingBeaconOrder;

    /**
     * Used by all devices that have a parent.
     * The length of the active portion of the INCOMING superframe, including the
     * beacon frame.
     * 0 - 15 with 15 meaning the superframe will not be active after the beacon.
     */
    uint8_t m_incomingSuperframeOrder;

    /**
     * Indication of the Slot where the CAP portion of the INCOMING Superframe ends.
     */
    uint8_t m_incomingFnlCapSlot;

    /**
     * Indicates if MAC sublayer is in receive all mode. True mean accept all
     * frames from PHY.
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    bool m_macPromiscuousMode;

    /**
     * Set to true if the mac is currenly intergrated with higher layer such as SixLowPan
     * with differenct time sync method.
     */
    bool m_forDsmeNetDeviceIntegrateWithHigerLayer;

    /**
     * Allow all the data packet frame receive and forward to the higher layer (SixLowPan)
     */
    bool m_acceptAllHilowPkt;

    /**
     * The data packets that are not successfully sent in Cap can be sent during gts.
     */
    bool m_gtsContinuePktSendingFromCap;

    /**
     * 16 bits id of PAN on which this device is operating. 0xffff means not
     * associated.
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    uint16_t m_macPanId;

    /**
     * Temporally stores the value of the current m_macPanId when a MLME-SCAN.request is performed.
     * See IEEE 802.15.4-2011, section 5.1.2.1.2.
     */
    uint16_t m_macPanIdScan;

    /**
     * Sequence number added to transmitted data or MAC command frame, 00-ff.
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    SequenceNumber8 m_macDsn;

    /**
     * Sequence number added to transmitted beacon frame, 00-ff.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    SequenceNumber8 m_macBsn;

    /**
     * The maximum number of retries allowed after a transmission failure.
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    uint8_t m_macMaxFrameRetries;

    /**
     * Indication of whether the MAC sublayer is to enable its receiver during
     * idle periods.
     * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
     */
    bool m_macRxOnWhenIdle;

    /**
     * The minimum time forming a Long InterFrame Spacing (LIFS) period.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    uint32_t m_macLIFSPeriod;

    /**
     * The minimum time forming a Short InterFrame Spacing (SIFS) period.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    uint32_t m_macSIFSPeriod;

    /**
     * Indication of whether a coordinator is currently allowing association.
     * A value of TRUE indicates that the association is permitted.
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    bool m_macAssociationPermit;

    /**
     * Indication of whether a device automatically sends data request command
     * if its address is listed in the beacon frame.
     * TRUE = request command automatically is sent. This command also affects
     * the generation of MLME-BEACON-NOTIFY.indication (6.2.4.1)
     * See IEEE 802.15.4-2011, section 6.4.2, Table 52.
     */
    bool m_macAutoRequest;

    /**
     * The maximum energy level detected during ED scan on the current channel.
     */
    uint8_t m_maxEnergyLevel;

    /**
     * Variable to store the original Channel Num used in the CAP period(CSMA/CA).
     */
    uint8_t m_originalChannelInCAP;

    /**
     * The value of the necessary InterFrame Space after the transmission of a packet.
     */
    uint32_t m_ifs;

    /**
     * Indication of whether the current device is the PAN coordinator
     */
    bool m_panCoor;

    /**
     * Indication of whether the current device is the coordinator
     */
    bool m_coord;

    /**
     * Indication of the Interval used by the coordinator to transmit beacon frames
     * expressed in symbols.
     */
    uint32_t m_beaconInterval;

    /**
     * Indication of the superframe duration in symbols.
     * (e.g. 1 symbol = 4 bits in a 250kbps O-QPSK PHY)
     */
    uint32_t m_superframeDuration;

    /**
     * Indication of the interval a node should receive a superframe
     * expressed in symbols.
     */
    uint32_t m_incomingBeaconInterval;

    /**
     * Indication of the superframe duration in symbols
     * (e.g. 1 symbol = 4 bits in a 250kbps O-QPSK PHY)
     */
    uint32_t m_incomingSuperframeDuration;

    /**
     * Indication of current device capability (FFD or RFD)
     */
    uint8_t m_deviceCapability;

    /**
     * Indication of whether the current device is tracking incoming beacons.
     */
    bool m_beaconTrackingOn;

    /**
     * The number of consecutive loss beacons in a beacon tracking operation.
     */
    uint8_t m_numLostBeacons;

    // MAC PIB attributes, See IEEE 802.15.4e-2012 Section 6.4.2 Table 52
    /**
     * The maximum time (in μs) to wait for the PHY header of 
     * an enhanced acknowledgment frame to arrive followin
     * g a transmitted data frame
     */
    uint16_t m_macEnhAckWaitDuration;

    /**
     * Indicates whether frames without a destination PAN ID 
     * or destination address are to treated as though 
     * they are addressed to the broadcast PANID (0xffff) 
     * and broadcast short address (0xfffff)
     */    
    bool m_macImplicitBroadcast;

    /**
     * The Simple address of the coordinator through which the device is
     * associated.
     * 0xFF indicates that the device does not have a simple address.
     * 0xFE A value of 0xfe indicates that the device has associated but has not been
     * allocated an address
     * See IEEE 802.15.4e-2012, section 6.4.2, Table 52.
     */
    Mac8Address m_macCoordSimpleAddress;

    // General MAC PIB attributes for functional organization, 
    // See IEEE 802.15.4e-2012 Section 6.4.3.2 Table 52a
    
    // the device is capable of functionality specific to DSME
    bool m_macDSMEcapable;

    // the device is capable of unslotted channel hopping
    bool m_macHoppingCapable;

    // the device is using functionality specific to DSME
    bool m_macDSMEenabled;

    // the device is using unslotted channel hopping
    bool m_macHoppingEnabled;

    // MAC PIB attributes for hopping sequence 
    // See IEEE 802.15.4e-2012 Section 6.4.3.4 Table 52f
    // Each hopping sequence has a unique ID.
    uint8_t m_macHoppingSeqID;          

    /**
     * Corresponds to the 5 MSBs (b27, ...,
     * b31) of a row in phyChannelsSupported.
     * Note this may not correspond to the
     * current channelPage in use.
     */
    uint8_t m_macChannelPage;

    // Number of channels supported by the PHY on this channelPage.
    uint16_t m_numOfChannels;

    /**
     * For channel pages 0 to 6, the 27 LSBs (b0, b1, ..., b26) 
     * indicate the status (1 = to be used, 0 = not to be used) 
     * for each of the up to 27 valid channels available
     * to the PHY. For pages 7 and 8, the 27 LSBs indicate 
     * the configuration of the PHY, and the channel list is
     * contained in the extendedBitmap
     */
    uint32_t m_macPhyConfiguration;

    /**
     * For pages 7 and 8, a bitmap of numberOfChannels bits
     * where bk shall indicate the status of channel k for each
     * of the up to numberOfChannels valid channels 
     * supported by that channel page nd phyConfiguration. 
     * Otherwise field is empty.
     */
    std::vector<uint16_t> m_macExtBitmap;   //size varies, has to be allocated in run time

    /**
     * The number of channels in the Hopping Sequence. Does not necessarily equal
     * numberOfChannels.
     */
    uint16_t m_hoppingSeqLen;

    // A macHoppingSequenceLength-element list of channels to be hopped over.
    HoppingSequence m_macHoppingSeqList;

    // Index of the current position in the hopping sequence list.
    uint16_t m_macCurrentHop;

    /**
     * For unslotted channel hopping modes, this field is the channel dwell time
     * , in units of 10 μs. For other modes, the field is empty.
     */
    Time m_hopDwellTime;

    // DSME specific MAC PIB attributes
    // See IEEE 802.15.4e-2012 Section 6.4.3.6 Table 52h and Table 52i
    /**
     * Specifies the channel index of the channel’s DSME link status reported 
     * by the source device.
     */
    uint8_t m_macChannelIndex;

    uint8_t m_macAvgLQI;

    uint8_t macAvgRSSI;

    uint32_t m_macLinkStatusStatisticPeriod;

    /**
     * This flag indicates if the coordinator is currently 
     * using the group acknowledge mechanism for DEMS-GTS frame receptions.
    */
    bool m_macGACKFlag;

    /**
     * Indication of whether the CAP reduction is enabled.
     */
    bool m_macCAPReductionFlag;

    /**
     * Indicates the method of channel diversity:
     * 0x00 = Channel Adaptation
     * 0x01 = Channel Hopping
     * This value is not valid for a nonbeacon-enabled PAN.
     */
    uint8_t m_macChannelDiversityMode;

    /**
     * macMultiSuperframeOrder describes the length of a multi-superframe.
     */
    uint8_t m_macMultisuperframeOrder;

    /**
     *  multi-superframe duration
     */
    uint32_t m_multiSuperframeDuration;

    DsmePANDescriptorIE m_dsmePanDescriptorIE;

    uint8_t m_incomingMultisuperframeOrder;

    uint32_t m_incomingMultisuperframeDuration;

    uint8_t m_incomingChannelDiversityMode;

    bool m_incomingGACKFlag;

    bool m_incomingCAPReductionFlag;

    bool m_incomingDeferredBcnUsed;

    BeaconBitmap m_incomingSDBitmap;

    bool m_isBcnAllocCollision;

    uint16_t m_incSDindex;

    Time m_endCapTime;

    /**
     * Check there is a collision or not.
     * If the value is true, collision occurr, need to be scheduled again.
     */
    bool m_needBcnSchedulingAgain;

    /**
     * Record the time of sending mlmeScanReqest.
     * In order to calculate the scan ~ start period.
     * For beacon scheduling performance indicator.
     */
    Time m_mlmeScanReqTime;
    /**
     * Record the time of sending mlmeStartReqest.
     * In order to calculate the scan ~ start period.
     * For beacon scheduling performance indicator.
     */
    Time m_mlmeStartReqTime;

    /**
     * The number of multi-superframe in a beacon interval
     */
    uint32_t m_numOfMultisuperframes;

    /**
     * The number of superframe in a beacon interval
     */
    uint64_t m_numOfSuperframes;
    /**
     * only present when DSME mode is enabled 
     * Used to determine the first or second CFP portion of the CAP reduction superframe
     */
    bool m_firstCFP;

    bool m_incomingFirstCFP;

    bool m_gtsRetrieve;

    /**
     * The slot allocation bitmap table of the DSME-GTS schedule.
     */
    std::vector<uint16_t> m_macDSMESAB;

    /**
     * The slot allocation bitmap for non CAP reducation device.
     */
    std::vector<uint8_t> m_macDSMESABCapOff;

    /**
     * Data structure to store the current schedule of Gts of this devic
     * Superframe ID (from 0)
     * Direction:  RX : true, TX : false
     * Start Index (from 0) and Gts Slot Length
     */
    std::vector<int> m_gtsSuperframeIDs;
    std::vector<bool> m_gtsDirections;
    std::vector<std::tuple<int, int>> m_gtsStartAndLens;
 
    /**
     * The allocation counter table of the DSME-GTS allocated to the device.
     * key: superframe ID, value: macDSMEACTEntity
     */
    std::map<uint16_t, std::vector<macDSMEACTEntity>> m_macDsmeACT;

    /**
     * Specifies the allocating SD index number for beacon frame.
     * The SD Index field specifies the index of current Superframe Duration (SD) in a beacon interval. 
     * The superframe in which the PAN coordinator sends its beacons serves as the reference point 
     * (SD Index 0)
     */
    uint32_t m_macSDindex;

    /**
     * Indicates the beacon frame allocation information of neighbor nodes.
     * This field is expressed in bitmap format that orderly represents the schedule of
     * beacons, with corresponding bit shall be set to one if a beacon is allocated 
     * in that SD. 
     * Refer to IEEE 802.15.4e-2012 5.2.4.9.3.
     */
    // BeaconBitmap m_macBcnBitmap;
    BeaconBitmap m_macSDBitmap;

    /**
     * ChannelOffset is the offset value of Hopping Sequence.
     */
    uint16_t m_macChannelOfs;

    bool m_macDeferredBcnUsed;

    Mac64Address m_macSyncParentExtAddr;
    Mac16Address m_macSyncParentShortAddr;

    /**
     * Indication of SD index the synchronized parent used.
     */
    uint16_t m_macSyncParentSDIndex;

    // Channel status for each used channel.
    LinkStatusSpecificationField m_macChannelStatus;

    // The number of symbols forming a beacon slot.
    // Default = 60
    uint16_t m_macBcnSlotLen = 60;

    // The number of idle incidents before expiring a DSME-GTS.
    // Default = 7
    uint8_t m_macDSMEGTSExpirationTime = 7;

    /**
     * Specifies the length of ChannelOffsetBitmap in octets.
     */
    uint8_t m_macChannelOfsBitmapLen;

    /**
     * Bit value of ChannelOffsetBitmap sequence represents whether the corresponding
     * channel offset is used. If the corresponding channel offset is used, the
     * bit value shall be set to one. Otherwise, it shall be set to zero.
     * 
     * The number of bits set to one in the ChannelOffsetBitmap is the number of
     * available channels in current channel page
     */
    std::vector<uint16_t> m_macChannelOfsBitmap;

    /**
     * The sequence number added to the transmitted beacon frame 
     * of a PAN coordinator.
     * macPANCoordinatorBSN is an enhanced beacon sequence number of a
     * PAN coordinator. 
     * macEBSN
     */
    SequenceNumber8 m_macPANCoordinatorBSN;

    // Table 52i—Elements of Neighbor Information
    struct macNeighborInformationTable {
        Mac16Address m_shortAddr;
        Mac64Address m_extAddr;
        uint16_t m_sdIndex;
        uint16_t m_channelOfs;
        bool m_trackBcn;
        uint8_t m_bcnLostCount;
    };

    std::vector<macNeighborInformationTable> m_macNeighborInformationTable;

    // Table 52m—EBR-specific MAC PIB attributes
    // See 802.15.4e-2012 Section 6.4.3.10
    bool m_EBRPermitJoining;

    // Contains which EBR filter field bits should be set.
    std::vector<bool> m_macEBRFilters;

    // Link quality level to be transmitted in the EBR.
    uint8_t m_macEBRLinkQuality;

    // Percent filter threshold value to be transmitted in the EBR.
    uint8_t m_macEBRPercentFilter;

    // Contains zero to four attribute IDs. Each ID shall
    // identify a Boolean PIB attribute, refer to IEEE 802.15.4e-2012 Table 6-3.
    std::vector<uint8_t> m_macEBRattributeList;

    /**
     * When TRUE, device responds to beacon requests and enhanced beacon requests 
     * automatically. When FALSE, device passes beacon/enhanced beacon payload up 
     * to higher layer using MLME-BEACONREQUEST.indication.
     */
    bool m_macBeaconAutoRespond;


    // Table 52n—EB-specific MAC PIB attributes
    // See 802.15.4e-2012 Section 6.4.3.11

    /**
     * When TRUE, in a beacon enabled PAN the device
     * should use Enhanced Beacons rather than standard beacons.
     */
    bool m_macUseEnhancedBeacon;

    // Indicates if devices should perform filtering in response to EBR.
    bool m_macFilteringEnabled;

    // Beacon Sequence Number used for Enhanced Beacon Frames (separate from BSN).
    SequenceNumber8 m_macEbsn;

    // Beacon scheduling allocation status
    uint8_t m_macBcnSchedulingAllocStatus;

    /**
     * Beacon scheduling performance parameters - Allocation fail count. 
     * Note : This parameter specify the fail count of each device, calculate by fail count / Total Devices.
     **/ 
    uint32_t m_bcnScehdulingFailCnt;

    /**
     * Beacon scheduling performance parameters - Allocation device count
     * Note : This parameter specify the number of associated lr-wpan devices in the simulation.
     **/ 
    uint32_t m_bcnSchedulingDevCnt;

    /**
     * Beacon scheduling performance parameters - Control packet count
     * Note : This parameters specify the number of Control packet count used by beacon scheduling. 
     * 
     *        There are two types control packet :
     *        1. Dsme Beacon Allocation Notification Command
     *        2. Dsme Beacon Collision Notification Command
     **/ 
    uint32_t m_bcnSchedulingCtrlPktCount;

    // Beacon scheduling allocation average time parameter.
    // Note : This parameter specify the number of the device in PAN.
    //        It calculate the total time of beacon scheduling.
    /**
     * Beacon scheduling performance parameters - Allocation time cost in average
     * Note : This parameters specify the number of the device in PAN.
     *        It calculate the total time of beacon scheduling.
     **/ 
    Time m_bcnSchedulingTime;

    // Enhanced Beacon Scheduling allocation sequence.
    // For the purpose of allocation collision free.
    uint16_t m_allocationSequence;

    /**
     * Indicates the current SuperframeIDx, the value will be added at the beginning of each superframe and reset at the beginning of each multisuperframe.
    */
    uint32_t m_curSuperframeIDx;
    /**
     * Set the current SuperframeIDx, the value will be added at the beginning of each superframe and reset at the beginning of each multisuperframe.
    */
    void SetSuperframeIDx(uint16_t curSuperframeIDx);
    /**
     * Get the current SuperframeIDx, the value will be added at the beginning of each superframe and reset at the beginning of each multisuperframe.
    */
    uint16_t GetSuperframeIDx();

    bool m_isFirstSuperframe = true;
    bool m_isFirstMultiSuperframe = true;

    /**
     * Indicate current Multisuperframe sequence.
    */
    int32_t m_multisuperframeSeq = -1;

    /**
     * Indicate the group ack policy that the decice currently used.
    */
    LrWpanGroupAckPolicy m_groupAckPolicy;
    
    /**
     * The scheduled event to add superframeIDx.
    */
    void SetGroupAckPolicy(LrWpanGroupAckPolicy policy); 

    /**
     * The scheduled event to add superframeIDx.
    */
    void StartSuperframe();

    // Enhanced Beacon Scheduling allocation sequence.
    // For the purpose of allocation collision free.
    void SetAllocationSeq(uint16_t allocSeq);

    // Enhanced Beacon Scheduling allocation sequence.
    // For the purpose of allocation collision free.
    uint16_t GetAllocationSeq();

    //Set the time of sending mlmeScanReqest.
    void SetMlmeScanReqTime(Time scanReqTime);
    //Get the time of sending mlmeScanReqest.
    Time GetMlmeScanReqTime();

    //Set the time of sending mlmeStartReqest.
    void SetMlmeStartReqTime(Time startReqTime);
    //Get the time of sending mlmeStartReqest.
    Time GetMlmeStartReqTime();

    typedef enum {
        EBAutoSA_NONE = 0,
        EBAutoSA_SHORT = 1,
        EBAutoSA_FULL = 2,
    } EBAutoSA;

    /**
     * Indicates if beacons generated by the MAC in
     * response to EB include Source Address field.
     */
    EBAutoSA m_macEBAutoSA;
    
    /**
     * List of additional IEs to be included in enhanced
     * ACKs generated by the device.
     */
    // m_macEAckIElist;

    // DSME
    void SetDsmeModeEnabled();

    void SetDsmeModeDisabled();

    void SetMultisuperframeOrder(uint8_t multisuperfmOrder);

    void SetHoppingSeqLen(uint16_t len);

    void SetHoppingSeq(HoppingSequence seq);

    void SetChannelHoppingEnabled();

    void SetChannelHoppingNotEnabled();

    void SetBcnCollision();

    void SetBcnDoNotCollision();

    bool IsBcnCollision();

    void SetBcnSchedulingTime(Time time);
    Time GetBcnSchedulingTime();

    void SetBcnSchedulingDevCnt(uint32_t devCnt);
    uint32_t GetBcnSchedulingDevCnt();

    void SetBcnSchedulingCtrlPktCnt(uint32_t assocReqCount);
    uint32_t GetBcnSchedulingCtrlPktCnt();
    /**
     * Called by the higher layer to decide the superframe number to send
     * its beacon if it is a coordinator.
     */
    void SetTimeSlotToSendBcn(uint16_t idx);

    /**
     * Track of the index of the pan descriptor that this device 
     * choosed to associate with.
     */
    void SetDescIndexOfAssociatedPan(int idx);

    int GetDescIndexOfAssociatedPan();

    uint16_t GetTimeSlotToSendBcn() const;

    /**
     * Get the macAckWaitDuration attribute value.
     *
     * \return the maximum number symbols to wait for an acknowledgment frame
     */
    uint64_t GetMacAckWaitDuration() const;

    /**
     * Get the macMaxFrameRetries attribute value.
     *
     * \return the maximum number of retries
     */
    uint8_t GetMacMaxFrameRetries() const;

    /**
     * Print the number of elements in the packet transmit queue.
     */
    void PrintTransmitQueueSize();

    /**
     * Set the macMaxFrameRetries attribute value.
     *
     * \param retries the maximum number of retries
     */
    void SetMacMaxFrameRetries(uint8_t retries);

    /**
     * Check if the packet destination is its coordinator
     *
     * \return True if m_txPkt (packet awaiting to be sent) destination is its coordinator
     */
    bool isCoordDest();

    void SetAsCoordinator();

    void SetNotCoordinator();

    bool IsCoord() const;

    bool isCAPReductionOn();

    void SetCAPReduction(bool on);

    void SetDsmeMacIsForIntegratingWithHigerLayer(bool on);

    void SetAcceptAllHilowPkt(bool on);
    
    void SetGtsContinuePktSendingFromCap(bool on);

    /**
     * Check if the packet destination is its coordinator
     *
     *\param mac The coordinator short MAC Address
     */
    void SetAssociatedCoor(Mac16Address mac);

    /**
     * Check if the packet destination is its coordinator
     *
     *\param mac The coordinator extended MAC Address
     */
    void SetAssociatedCoor(Mac64Address mac);

    /**
     * Get the size of the Interframe Space according to MPDU size (m_txPkt).
     *
     * \return the IFS size in symbols
     */
    uint32_t GetIfsSize();

    /**
     * Obtain the number of symbols in the packet which is currently being sent by the MAC layer.
     *
     *\return packet number of symbols
     * */
    uint64_t GetTxPacketSymbols();

    /**
     * Check if the packet to transmit requires acknowledgment
     *
     *\return True if the Tx packet requires acknowledgment
     * */
    bool isTxAckReq();

    /**
     * Print the Pending transaction list.
     * \param os The reference to the output stream used by this print function.
     */
    void PrintPendTxQ(std::ostream& os) const;

    /**
     * Print the Transmit Queue.
     * \param os The reference to the output stream used by this print function.
     */
    void PrintTxQueue(std::ostream& os) const;

    /**
     * TracedCallback signature for sent packets.
     *
     * \param [in] packet The packet.
     * \param [in] retries The number of retries.
     * \param [in] backoffs The number of CSMA backoffs.
     */
    typedef void (*SentTracedCallback)(Ptr<const Packet> packet, uint8_t retries, uint8_t backoffs);

    /**
     * TracedCallback signature for LrWpanMacState change events.
     *
     * \param [in] oldValue The original state value.
     * \param [in] newValue The new state value.
     * \deprecated The LrWpanMacState is now accessible as the
     * TracedValue \c MacStateValue. The \c MacState TracedCallback will
     * be removed in a future release.
     */
    typedef void (*StateTracedCallback)(LrWpanMacState oldState, LrWpanMacState newState);

    uint32_t GetNumOfMultisuperframesInABeaconInterval() const;

    uint64_t GetNumOfSuperframesInABeaconInterval() const;

    void ResizeMacDSMESAB(bool capReduction, uint8_t bcnOrder, uint8_t sfrmOrd);

    void ResizeScheduleGTSsEvent(uint8_t bcnOrder, uint8_t multisfrmOrd, uint8_t sfrmOrd);

    uint8_t GenerateSABSubBlock(uint8_t slotID, uint8_t numSlot);
    uint16_t GenerateSABSubBlockCapOn(uint8_t slotID, uint8_t numSlot);

    void AddDsmeACTEntity(uint16_t superframeID, macDSMEACTEntity entity);

    void SetChannelOffset(uint16_t offset); 

    void AddPanDescriptor(PanDescriptor descriptor);

    void SetNumOfChannelSupported(uint16_t num);

    bool IsIncomingSuperframe();

    void SetRecord(std::map<Address, std::pair<Address, std::vector<int64_t>>> &record);
    void SetRecord(std::map<std::pair<Address, Address>, std::vector<std::pair<int64_t, int64_t>>> &record);

    void ReceiveRecordKeyAndValueIdx(std::pair<Address, Address> recordkey, unsigned int recordValueIdx);

    void SetBecomeCoordAfterAssociation(bool on);

    void CheckBeaconScheduling(MlmeStartRequestParams params);

    // Beacon scheduling entry point
    void BeaconScheduling(LrWpanBeaconSchedulingPolicy schedulingPolicy);

    uint8_t FindVacantBeaconTimeSlot(BeaconBitmap beaconBitmap);

    // Return Beacon scheduling allocation fail count.
    uint32_t GetBcnSchedulingFailCnt();

    /**
     * Self-designed mapping array for Coord <-> SDIdx
     * [Key]   : Coor address
     * [Value] : SDIdx
    */

    std::map<Mac16Address, uint16_t> m_macSDIdxMappingArray;

    /**
     * Convert Extend addr to short addr.
    */
    Mac16Address ConvertExtAddrToShortAddr(Mac64Address ExtAddr);

    /**
     *! Legacy Group Ack 
    */
    uint16_t m_legacyGackBitmap;
    uint8_t  m_legacyGackDevList;
    uint16_t m_legacyGackIdx;
    uint8_t  m_legacyGackDirections;
    uint32_t m_legacyGackIndexCounter;
    
    uint32_t m_currentGTSIdx;
    /**
     * Recording the GTS IDx which transmit before receive a legacy Group Ack bitmap.
    */
    std::vector<uint32_t> m_legacyGackGTSIdxBuffer;

    /**
     * The IEEE 802.15.4e legacy group ack field infos.
    */
    GroupACK m_legacyGroupAck;

    void SendLegacyGroupAck();
    void ResetLegacyGroupAckBitmap();
    void ResetLegacyGroupAckBuffer();

    /**
     *! Enhanced Group Ack 
    */

       /**
     * The enhanced group ack bitmap store at LrWpanMac (device MAC).
    */
    uint64_t m_enhancedGACKBitmap;

    /**
     * Recording the packet sequence which transmit before receive a Enhanced Group Ack bitmap.
    */
    std::vector<uint32_t> m_groupAckPktBuffer;
    
    /**
     * (Input) addr + pkt seq  ---> Hash function --- > (output) hash table key
    */
    uint64_t GenerateHashTableKey(Mac16Address devAddr, uint32_t packetSeq);

    /**
     * Check the hash key is collision or not.
    */
    bool IsHashTableKeyCollision(uint32_t inputHashTableKey);

    uint32_t CheckCollision(uint32_t Key, uint64_t hashedVal);
    uint32_t DoDoubleHash(uint32_t Key, uint64_t hashedVal); // TODO
    uint32_t DoQuadraticProb(uint32_t key, uint32_t count);
    void ResetEnhancedGroupAckBuffer();
    void PrintGroupAckBitmap();
    void ResetEnhancedGroupAckBitmap();
    void SendEnhancedGroupAck();

    /**
     *! DSME GTS Group Ack 
    */

    /**
    * Set the payload length for the DSME-GTS Group Ack packet field.
    */
    void SetDsmeGtsPayloadLength(uint8_t len);

    /**
    * Reset the payloads the DSME-GTS Group Ack structure.
   */
    void ResetDsmeGtsGackPayloads();

    /**
    * Reset the payloads the DSME-GTS Group Ack structure.
   */
    void ResetDsmeGtsGackPayloadLength();

    /**
     * Recording the packet sequence which transmit before receive a DSME-GTS Group Ack bitmap.
    */
    std::vector<uint32_t> m_dsmeGtsGroupAckPktBuffer;

    uint8_t m_dsmeGtsGackPayloadLength;
    std::vector<DsmeGtsGackPayload> m_dsmeGtsGackPayloads;

    void SendDsmeGtsGroupAck();

    /**
     * Reset packet buffer which stores the packet send at GTS.
     **/
    void ResetDsmeGtsGroupAckBuffer();

    /**
     * Check the addr is already exist in the payload field or not.
     **/
    bool CheckAddrExist(Mac16Address addr);


  protected:
    // Inherited from Object.
    void DoInitialize() override;
    void DoDispose() override;

  private:
    /**
     * Helper structure for managing transmission queue elements.
     */
    struct TxQueueElement : public SimpleRefCount<TxQueueElement>
    {
        uint8_t txQMsduHandle; //!< MSDU Handle
        Ptr<Packet> txQPkt;    //!< Queued packet
    };

    /**
     * Helper structure for managing pending transaction list elements (Indirect transmissions).
     */
    struct IndTxQueueElement : public SimpleRefCount<IndTxQueueElement>
    {
        uint8_t seqNum;               //!< The sequence number of  the queued packet
        Mac16Address dstShortAddress; //!< The destination short Mac Address
        Mac64Address dstExtAddress;   //!< The destination extended Mac Address
        Ptr<Packet> txQPkt;           //!< Queued packet.
        Time expireTime; //!< The expiration time of the packet in the indirect transmission queue.
    };

    /**
     * Called to send a single beacon frame.
     */
    void SendOneBeacon();

    /**
     * Called to send a single enhanced beacon frame.
     */
    void SendOneEnhancedBeacon();    

    /**
     * Called to send an associate request command.
     */
    void SendAssocRequestCommand();

    /**
     * Called to send an DSME associate request command.
     */
    void SendDsmeAssocRequestCommand();

    /**
     * Called to send an associate request command with fastA.
     */
    void SendFastAssocRequestCommand();

    /**
     * Called to send an disassociate request command.
     */
    void SendDisassocNotificationCommand();

    void SendDisassocNotificationCommandIndirect(Ptr<Packet> rxDataReqPkt);

    void RemoveReferencesToPAN();

    void CheckDsmeGtsSABFromReplyCmd(DSMESABSpecificationField sAB);

    /**
     * Called when received a dsme gts request command with management type = 1 (allocation)
     * but the DSME GTS Destination address is the not same as the macShortAddress. 
     * the device shall check if the slots marked as one in the command is 
     * conflicting with the readily allocated slots in macDsmeAct
     */
    bool CheckDsmeGtsSABAndDsmeACTConflict(DSMESABSpecificationField sAB);

    /**
     * Called when received a dsme gts request command with management type = 0 (deallocation)
     * to check whether if the DSME-GTSs in the command frame
     *  match the allocated DSME-GTSs in macDSMEACT
     */
    bool CheckDsmeGtsSABFromReqCmdWithDsmeACT(DSMESABSpecificationField sAB);

    /**
     * Called when the coordinator recevice a dsme gts request with deallocation to
     * de-schedule the allocated GTS.
     */
    void UpdateDsmeACTAndDeallocate(DSMESABSpecificationField sAB);

    /**
     * Called when the coordinator receive a dsme information request to find if any
     * allocated GTS match the requesting device.
     */
    bool SearchDsmeACTForAllocatedGTS(Mac16Address addr, uint16_t& superframeID
                                            , uint8_t& slotID);

    /**
     * Used to send a data request command (i.e. Request the coordinator to send the association
     * response)
     */
    void SendDataRequestCommand();

    /**
     * Called to send an associate response command.
     *
     * \param rxDataReqPkt The received data request pkt that instigated the Association response
     * command.
     */
    void SendAssocResponseCommand(Ptr<Packet> rxDataReqPkt);

    /**
     * Called to send an DSME associate response command.
     *
     * \param rxDataReqPkt The received data request pkt that instigated the Dsme Association response
     * command.
     */
    void SendDsmeAssocResponseCommand(Ptr<Packet> rxDataReqPkt);

    /**
     * Called to send an DSME-Beacon allocation notification command.
     */
    void SendDsmeBeaconAllocNotifyCommand();

    /**
     * Called to send an DSME-Beacon collision notification command.
     */

    void SendDsmeBeaconCollisionNotifyCommand(Mac16Address dstAddr, uint16_t collisionSDIndex);

    /**
     * Called to send an DSME-GTS notify command.
     */
    void SendDsmeGtsNotifyCommand(Mac16Address srcAddr, CommandPayloadHeader rxDsmeGtsReplyPayload);

    /**
     * Called to send an Orphan Notification Command
     * See 802.15.4-2011 Section 5.3.6
     */
    void SendOrphanNotificationCmd();

    /**
     * Called after m_assocRespCmdWaitTime timeout while waiting for an association response
     * command.
     */
    void LostAssocRespCommand();

    /**
     * Called to send a beacon request command.
     */
    void SendBeaconRequestCommand();

    /**
     * Called to send a enhanced beacon request command.
     */
    void SendEnhancedBeaconRequestCommand();

    /**
     * Called to send an DSME-INFO response command.
     *
     * \param rxDataReqPkt The received data request pkt that instigated the DSME-INFO response
     * command.
     */
    void SendDsmeInfoResponseCommand(Ptr<Packet> rxDsmeInfoReqPkt, uint16_t& superframeID
                                    , uint8_t& slotID);

    /**
     * Called to end a MLME-START.request after changing the page and channel number.
     */
    void EndStartRequest();

    /**
     * Called at the end of the current channel scan (Active or Passive) for a given duration.
     */
    void EndChannelScan();

    /**
     * Called at the end of the current orphan channel scan for a given duration.
     */
    void EndOrphanScan();

    /**
     * Called at the end of one ED channel scan.
     */
    void EndChannelEnergyScan();

    /**
     * Called at the end of one enhanced active scan.
     */
    void EndEnhancedBeaconScan();

    /**
     * Called to end an MLME-ASSOCIATE.request after changing the page and channel number.
     */
    void EndAssociateRequest();

    /**
     * Called to end an MLME-DISASSOCIATE.request.
     */
    void EndDisassociateRequest();

    /**
     * Called to end an MLME-DSME-GTS.request.
     */
    void EndDsmeGtsRequest();

    /**
     * Called to end an MLME-DSME-INFO.request.
     */
    void EndDsmeInfoRequest();

    /**
     * Called to end an MLME-DSME-LINKSTATUSRPT.request.
     */
    void EndDsmeLinkStatusRptRequest();

    /**
     * Called to begin the Guaranteed Time Slot (GTS) in a 
     * Contention Free Period (CFP).
     */
    void StartGTS(SuperframeType superframeType, uint16_t superframeID, int idx);

    void EndGTS(SuperframeType superframeType);

    // For Dsme net device setting use only 
    void ScheduleGtsSyncToCoord(uint16_t curSDIndex);

    void ScheduleGtsSyncToCoordDuringCfp(uint16_t curSDIndex);

    void ScheduleGts(bool indication);

    void PurgeDsmeACT();

    /**
     * Called to begin the Contention Free Period (CFP) in a
     * beacon-enabled mode.
     *
     * \param superframeType The incoming or outgoing superframe reference
     */
    void StartCFP(SuperframeType superframeType);

    /**
     * Called to begin the Contention Access Period (CAP) in a
     * beacon-enabled mode.
     *
     * \param superframeType The incoming or outgoing superframe reference
     */
    void StartCAP(SuperframeType superframeType);

    /**
     * Called to begin the a Multisuperframe in a
     * DSME beacon-enabled mode.
     * Only Pan Coordinator can call it??
     */
    void StartMultisuperframe(SuperframeType superframeType);

    /**
     * Start the Inactive Period in a beacon-enabled mode.
     *
     * \param superframeType The incoming or outgoing superframe reference
     *
     */
    void StartInactivePeriod(SuperframeType superframeType);

    void StartRemainingPeriod(SuperframeType superframeType);

    /**
     * Called after the end of an INCOMING superframe to start the moment a
     * device waits for a new incoming beacon.
     */
    void AwaitBeacon();

    /**
     * Called if the device is unable to locate a beacon in the time set by MLME-SYNC.request.
     */
    void BeaconSearchTimeout();

    /**
     * Send an acknowledgment packet for the given sequence number.
     *
     * \param seqno the sequence number for the ACK
     */
    void SendAck(uint8_t seqno);

    /**
     * Send an acknowledgment packet for the given sequence number.
     * This function is called when receiving a data request command frame.
     *
     * \param seqno the sequence number for the ACK
     */
    void SendAckAfterDataReq(LrWpanMacHeader receivedMacHdr);

    /**
     * Send an enhanced acknowledgment packet for the given sequence number.
     *
     * \param seqno the sequence number for the ACK
     */
    void SendEnhancedAck(uint8_t seqno);

    /**
     * Add an element to the transmission queue.
     *
     * \param txQElement The element added to the Tx Queue.
     */
    void EnqueueTxQElement(Ptr<TxQueueElement> txQElement);

    /**
     * Remove the tip of the transmission queue, including clean up related to the
     * last packet transmission.
     */
    void RemoveFirstTxQElement();

    /**
     * Change the current MAC state to the given new state.
     *
     * \param newState the new state
     */
    void ChangeMacState(LrWpanMacState newState);

    /**
     * Handle an ACK timeout with a packet retransmission, if there are
     * retransmission left, or a packet drop.
     */
    void AckWaitTimeout();

    /**
     * Handle an ACK timeout with a Dsme gts confirm callback with
     * Status = NO_ACK
     */
    void DsmeGtsAckWaitTimeout();

    /**
     * Handle an ACK timeout with a Dsme info confirm callback with
     * Status = NO_ACK
     */
    void DsmeInfoAckWaitTimeout();

    /**
     * After a successful transmission of a frame (beacon, data) or an ack frame reception,
     * the mac layer wait an Interframe Space (IFS) time and triggers this function
     * to continue with the MAC flow.
     *
     * \param ifsTime IFS time
     */
    void IfsWaitTimeout(Time ifsTime);

    /**
     * Handle an Dsme Gts Response timeout.
     */
    void DsmeGtsRespWaitTimeout();

    /**
     * Handle an Dsme Gts Notify timeout.
     */
    void DsmeGtsNotifyWaitTimeout();

    /**
     * Handle an Dsme Info Reply timeout.
     */
    void DsmeGtsReplyWaitTimeout();

    /**
     * Check for remaining retransmissions for the packet currently being sent.
     * Drop the packet, if there are no retransmissions left.
     *
     * \return true, if the packet should be retransmitted, false otherwise.
     */
    bool PrepareRetransmission();

    /**
     * Adds a packet to the pending transactions list (Indirect transmissions).
     *
     * \param p The packet added to pending transaction list.
     */
    void EnqueueInd(Ptr<Packet> p);

    /**
     * Extracts a packet from pending transactions list (Indirect transmissions).
     * \param dst The short address used an index to obtain an element from the pending
     * transaction list.
     * \param entry The dequeued element from the pending transaction list.
     * \return The status of the dequeue
     */
    bool DequeueInd(Mac16Address dst, Ptr<IndTxQueueElement> entry);

    /**
     * Extracts a packet from pending transactions list (Indirect transmissions).
     * \param dst The extended address used an index to obtain an element from the pending
     * transaction list.
     * \param entry The dequeued element from the pending transaction list.
     * \return The status of the dequeue
     */
    bool DequeueInd(Mac64Address dst, Ptr<IndTxQueueElement> entry);

    /**
     * Search a specific tx element in the pending list when received a data request command.
     */
    bool SearchIndQueueElement(Mac16Address dst, const Ptr<IndTxQueueElement> entry);

    /**
     * Search a specific tx element in the pending list when received a data request command.
     */
    bool SearchIndQueueElement(Mac64Address dst, Ptr<IndTxQueueElement> entry);

    /**
     * Purge expired transactions from the pending transactions list.
     */
    void PurgeInd();

    /**
     * Remove an element from the pending transaction list.
     *
     * \param p The packet to be removed from the pending transaction list.
     */
    void RemovePendTxQElement(Ptr<Packet> p);

    /**
     * Check the transmission queue. If there are packets in the transmission
     * queue and the MAC is idle, pick the first one and initiate a packet
     * transmission.
     */
    void CheckQueue();

    /**
     * Constructs a Superframe specification field from the local information,
     * the superframe Specification field is necessary to create a beacon frame.
     *
     * \returns the Superframe specification field
     */
    SuperframeField GetSuperframeField();

    /**
     * Constructs the Guaranteed Time Slots (GTS) Fields from local information.
     * The GTS Fields are part of the beacon frame.
     *
     * \returns the Guaranteed Time Slots (GTS) Fields
     */
    GtsFields GetGtsFields();

    /**
     * Constructs Pending Address Fields from the local information,
     * the Pending Address Fields are part of the beacon frame.
     *
     * \returns the Pending Address Fields
     */
    PendingAddrFields GetPendingAddrFields();

    /**
     * The trace source is fired at the end of any Interframe Space (IFS).
     */
    TracedCallback<Time> m_macIfsEndTrace;

    /**
     * The trace source fired when packets are considered as successfully sent
     * or the transmission has been given up.
     * Only non-broadcast packets are traced.
     *
     * The data should represent:
     * packet, number of retries, total number of csma backoffs
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>, uint8_t, uint8_t> m_sentPktTrace;

    /**
     * The trace source fired when packets come into the "top" of the device
     * at the L3/L2 transition, when being queued for transmission.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macTxEnqueueTrace;

    /**
     * The trace source fired when packets are dequeued from the
     * L3/l2 transmission queue.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macTxDequeueTrace;

    /**
     * The trace source fired when packets come into the "top" of the device
     * at the L3/L2 transition, when being queued for indirect transmission
     * (pending transaction list).
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macIndTxEnqueueTrace;

    /**
     * The trace source fired when packets are dequeued from the
     * L3/l2 indirect transmission queue (Pending transaction list).
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macIndTxDequeueTrace;

    /**
     * The trace source fired when packets are being sent down to L1.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macTxTrace;

    /**
     * The trace source fired when packets where successfully transmitted, that is
     * an acknowledgment was received, if requested, or the packet was
     * successfully sent by L1, if no ACK was requested.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macTxOkTrace;

    /**
     * The trace source fired when packets are dropped due to missing ACKs or
     * because of transmission failures in L1.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macTxDropTrace;

    /**
     * The trace source fired when packets are dropped due to indirect Tx queue
     * overflows or expiration.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macIndTxDropTrace;

    /**
     * The trace source fired for packets successfully received by the device
     * immediately before being forwarded up to higher layers (at the L2/L3
     * transition).  This is a promiscuous trace.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macPromiscRxTrace;

    /**
     * The trace source fired for packets successfully received by the device
     * immediately before being forwarded up to higher layers (at the L2/L3
     * transition).  This is a non-promiscuous trace.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macRxTrace;

    /**
     * The trace source fired for packets successfully received by the device
     * but dropped before being forwarded up to higher layers (at the L2/L3
     * transition).
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_macRxDropTrace;

    /**
     * A trace source that emulates a non-promiscuous protocol sniffer connected
     * to the device.  Unlike your average everyday sniffer, this trace source
     * will not fire on PACKET_OTHERHOST events.
     *
     * On the transmit size, this trace hook will fire after a packet is dequeued
     * from the device queue for transmission.  In Linux, for example, this would
     * correspond to the point just before a device hard_start_xmit where
     * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
     * ETH_P_ALL handlers.
     *
     * On the receive side, this trace hook will fire when a packet is received,
     * just before the receive callback is executed.  In Linux, for example,
     * this would correspond to the point at which the packet is dispatched to
     * packet sniffers in netif_receive_skb.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_snifferTrace;

    /**
     * A trace source that emulates a promiscuous mode protocol sniffer connected
     * to the device.  This trace source fire on packets destined for any host
     * just like your average everyday packet sniffer.
     *
     * On the transmit size, this trace hook will fire after a packet is dequeued
     * from the device queue for transmission.  In Linux, for example, this would
     * correspond to the point just before a device hard_start_xmit where
     * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
     * ETH_P_ALL handlers.
     *
     * On the receive side, this trace hook will fire when a packet is received,
     * just before the receive callback is executed.  In Linux, for example,
     * this would correspond to the point at which the packet is dispatched to
     * packet sniffers in netif_receive_skb.
     *
     * \see class CallBackTraceSource
     */
    TracedCallback<Ptr<const Packet>> m_promiscSnifferTrace;

    /**
     * A trace source that fires when the LrWpanMac changes states.
     * Parameters are the old mac state and the new mac state.
     *
     * \deprecated This TracedCallback is deprecated and will be
     * removed in a future release,  Instead, use the \c MacStateValue
     * TracedValue.
     */
    TracedCallback<LrWpanMacState, LrWpanMacState> m_macStateLogger;

    /**
     * The PHY associated with this MAC.
     */
    Ptr<LrWpanPhy> m_phy;

    /**
     * The CSMA/CA implementation used by this MAC.
     */
    Ptr<LrWpanCsmaCa> m_csmaCa;

    bool m_incSuperframe;

    std::map<Address, std::pair<Address, std::vector<int64_t>>> *m_record;

    std::map<std::pair<Address, Address>, std::vector<std::pair<int64_t, int64_t>>> *m_record2;

    std::pair<Address, Address> m_recordkey;
    std::deque<std::pair<Address, Address>> m_recordKeys;
    unsigned int m_recordValueIdx;

    /**
     * This callback is used to notify incoming beacon packets to the upper layers.
     * See IEEE 802.15.4-2011, section 6.2.4.1.
     */
    MlmeBeaconNotifyIndicationCallback m_mlmeBeaconNotifyIndicationCallback;

    /**
     * This callback is used to indicate the loss of synchronization with a coordinator.
     * See IEEE 802.15.4-2011, section 6.2.13.2.
     */
    MlmeSyncLossIndicationCallback m_mlmeSyncLossIndicationCallback;

    /**
     * This callback is used to report the result of a scan on a group of channels for the
     * selected channel page.
     * See IEEE 802.15.4-2011, section 6.2.10.2.
     */
    MlmeScanConfirmCallback m_mlmeScanConfirmCallback;

    /**
     * This callback is used to report the status after a device request an association with
     * a coordinator.
     * See IEEE 802.15.4-2011, section 6.2.2.4.
     */
    MlmeAssociateConfirmCallback m_mlmeAssociateConfirmCallback;

    /*
    * This callback is called after a MlmeDisassociateRequest has been called from
    * the higher layer. It returns a status of the outcome of the
    * disassociation request
    * 
    *  See 802.15.4-2011 6.2.3.3
    */
    MlmeDisassociateConfirmCallback m_mlmeDisassociateConfirmCallback;

    /**
     * This callback is used to report the status after a device send data command request to
     * the coordinator to transmit data.
     * See IEEE 802.15.4-2011, section 6.2.14.2.
     */
    MlmePollConfirmCallback m_mlmePollConfirmCallback;

    /**
     * This callback is used to report the start of a new PAN or
     * the begin of a new superframe configuration.
     * See IEEE 802.15.4-2006, section 7.1.14.2.
     */
    MlmeStartConfirmCallback m_mlmeStartConfirmCallback;

    /**
     * This callback is called after a MlmeDsmeInfoRequest has been called from
     * the higher layer. It returns a status of the outcome of the
     * MLME-DSME-INFO request
     * 
     *  See 802.15.4e-2012   Section 6.2.21.2.3
     */
    MlmeDsmeInfoConfirmCallback m_mlmeDsmeInfoConfirmCallback;

    /**
     * This callback is called after a MlmeDsmeLinkStatusRequest has been called from
     * the higher layer. It returns a status of the outcome of the
     * MLME-DSME-LINKSTATUSRPT request
     * 
     *  See 802.15.4e-2012   Section 6.2.21.3.3
     */
    MlmeDsmeLinkStatusRptConfirmCallback m_mlmeDsmeLinkStatusRptConfirmCallback;

    /**
     * This callback is used to notify incoming packets to the upper layers.
     * See IEEE 802.15.4-2006, section 7.1.1.3.
     */
    McpsDataIndicationCallback m_mcpsDataIndicationCallback;

    /**
     * This callback is used to indicate the reception of an association request command.
     * See IEEE 802.15.4-2011, section 6.2.2.2
     */
    MlmeAssociateIndicationCallback m_mlmeAssociateIndicationCallback;

    /**
     * This callback is called by the MLME when the MLME receive a disassociation
     * notification command.
     *
     *  See 802.15.4-2011 6.2.3.2
     */
    MlmeDisassociateIndicationCallback m_mlmeDisassociateIndicationCallback;

    /**
     * This callback is instigated through a response primitive.
     * See IEEE 802.15.4-2011, section 6.2.4.2
     */
    MlmeCommStatusIndicationCallback m_mlmeCommStatusIndicationCallback;

    /**
     * This callback is called after a Mlme has successfully received a DSME-GTS request 
     * command frame and wants to deliver it to the higher layer.
     *  
     *  See 802.15.4e-2012   Section 6.2.21.1.2
     */
    MlmeDsmeGtsIndicationCallback m_mlmeDsmeGtsIndicationCallback;

    /**
     * This callback is called after a Mlme has successfully received a DSME-GTS request 
     * command frame and wants to deliver it to the higher layer.
     *  
     *  See 802.15.4e-2012   Section 6.2.21.1.2
     */
    MlmeDsmeGtsConfirmCallback m_mlmeDsmeGtsConfirmCallback;

    /**
     * This callback is called after a Mlme has successfully received a orphan notification 
     * command frame and wants to deliver it to the higher layer.
     */
    MlmeOrphanIndicationCallback m_mlmeOrphanIndicationCallback;

    /**
     * This callback is called after a Mlme has successfully received a DSME-Infomation request 
     * command frame and wants to deliver it to the higher layer.
     * 
     *  See 802.15.4e-2012   Section 6.2.21.2.2
     */
    MlmeDsmeInfoIndicationCallback m_mlmeDsmeInfoIndicationCallback;

    /**
     * This callback is called after a Mlme has successfully received a 
     * MLME-DSME-LINKSTATUSRPT request command frame 
     * and wants to deliver it to the higher layer.
     * 
     *  See 802.15.4e-2012   Section 6.2.21.3.2
     */
    MlmeDsmeLinkStatusReportIndicationCallback m_mlmeDsmeLinkStatusReportIndicationCallback;

    /**
     * This callback is used to report data transmission request status to the
     * upper layers.
     * See IEEE 802.15.4-2006, section 7.1.1.2.
     */
    McpsDataConfirmCallback m_mcpsDataConfirmCallback;


    MlmeStartRequestCallback m_mlmeStartRequestCallback;

    /**
     * The current state of the MAC layer.
     */
    TracedValue<LrWpanMacState> m_lrWpanMacState;

    /**
     * The current period of the incoming superframe.
     */
    TracedValue<SuperframeStatus> m_incSuperframeStatus;

    /**
     * The current period of the outgoing superframe.
     */
    TracedValue<SuperframeStatus> m_outSuperframeStatus;

    /**
     * The current association status of the MAC layer.
     */
    LrWpanAssociationStatus m_associationStatus;

    /**
     * The packet which is currently being sent by the MAC layer.
     */
    Ptr<Packet> m_txPkt; // XXX need packet buffer instead of single packet

    /**
     * The command request packet received. Briefly stored to proceed with operations
     * that take place after ACK messages.
     */
    Ptr<Packet> m_rxPkt;

    /**
     * The packet which will be sent by the MAC layer in GTS duration.
     */
    Ptr<Packet> m_txPktGts;

    /**
     * The simple address used by this MAC. 
     */
    Mac8Address m_simpleAddress;

    /**
     * The short address used by this MAC. Currently we do not have complete
     * extended address support in the MAC, nor do we have the association
     * primitives, so this address has to be configured manually.
     */
    Mac16Address m_shortAddress;

    /**
     * The extended address used by this MAC. Extended addresses are currently not
     * really supported.
     */
    Mac64Address m_selfExt;

    /**
     * The transmit queue used by the MAC.
     */
    std::deque<Ptr<TxQueueElement>> m_txQueue;

    /**
     * The indirect transmit queue used by the MAC pending messages (The pending transaction
     * list).
     */
    std::deque<Ptr<IndTxQueueElement>> m_indTxQueue;

    /**
     * The maximum size of the transmit queue.
     */
    uint32_t m_maxTxQueueSize;

    /**
     * The maximum size of the indirect transmit queue (The pending transaction list).
     */
    uint32_t m_maxIndTxQueueSize;

    /**
     * The list of PAN descriptors accumulated during channel scans, used to select a PAN to
     * associate.
     */
    std::vector<PanDescriptor> m_panDescriptorList;

    /**
     * The list of energy measurements, one for each channel searched during an ED scan.
     */
    std::vector<uint8_t> m_energyDetectList;

    /**
     * The parameters used during a MLME-SCAN.request. These parameters are stored here while
     * PLME-SET operations (set channel page, set channel number) and multiple ed scans take place.
     */
    MlmeScanRequestParams m_scanParams;
    
    MlmeBeaconRequestParams m_beaconReqParams;

    /**
     * The parameters used during a MLME-START.request. These parameters are stored here while
     * PLME-SET operations (set channel page, set channel number) take place.
     */
    MlmeStartRequestParams m_startParams;

    /**
     * The parameters used during a MLME-ASSOCIATE.request. These parameters are stored here while
     * PLME-SET operations (set channel page, set channel number) take place.
     */
    MlmeAssociateRequestParams m_associateParams;

    /**
     * The parameters used during a MLME-DISASSOCIATE.request. These parameters are stored here while
     * PLME-SET operations (set channel page, set channel number) take place.
     */
    MlmeDisassociateRequestParams m_disassociateParams;

    /**
     * The parameters used during a MLME-DSME-GTS.request. 
     */
    MlmeDsmeGtsRequestParams m_dsmeGtsReqParams;

    /**
     * The parameters used during a MLME-DSME-GTS.reponse. 
     */
    MlmeDsmeGtsResponseParams m_dsmeGtsRespParams;

    McpsDataRequestParams m_mcpsDataRequestParams;

    /**
     * Method to get Channel number C at the given DSME-GTS Slot ID i in SDIndex j
     * See 802.15.4e-2012 Section 5.1.10.2.2 Channel hopping
     */
    uint16_t GetDsmeCurrentHoppingChannel();    

    /**
     * The channel list index used to obtain the current scanned channel.
     */
    uint16_t m_channelScanIndex;

    /**
     * Indicates the pending primitive when PLME.SET operation (page or channel switch) is called
     * from within another MLME primitive (e.g. Association, Scan, Sync, Start).
     */
    PendingPrimitiveStatus m_pendPrimitive;

    /**
     * The number of already used retransmission for the currently transmitted
     * packet.
     */
    uint8_t m_retransmission;

    /**
     * The number of CSMA/CA retries used for sending the current packet.
     */
    uint8_t m_numCsmacaRetry;

    /**
     * Scheduler event for the ACK timeout of the currently transmitted data
     * packet.
     */
    EventId m_ackWaitTimeout;

    /**
     * Scheduler event for a response to a request command frame.
     */
    EventId m_respWaitTimeout;

    /**
     * Scheduler event for the lost of a association response command frame.
     */
    EventId m_assocResCmdWaitTimeout;

    /**
     * Scheduler event for the lost of a acknowlegment of a dsme gts request command frame.
     */
    EventId m_dsmeGtsAckWaitTimeout;

    /**
     * Scheduler event for the lost of a acknowlegment of a dsme info request command frame.
     */
    EventId m_dsmeInfoAckWaitTimeout;

    /**
     * Scheduler event for the lost of a Dsme Gts response command frame.
     */
    EventId m_dsmeGtsRespTimeout;

    /**
     * Scheduler event for the lost of a Dsme Gts response command frame.
     */
    EventId m_dsmeInfoReplyTimeout;

    /**
     * Scheduler event for the lost of a Dsme Gts Notify command frame.
     */
    EventId m_dsmeGtsNotifyTimeout;

    /**
     * Scheduler event for a deferred MAC state change.
     */
    EventId m_setMacState;

    /**
     * Scheduler event for Interframe spacing wait time.
     */
    EventId m_ifsEvent;

    /**
     * Scheduler event for generation of one beacon.
     */
    EventId m_beaconEvent;

    /**
     * Scheduler event for the end of the outgoing superframe CAP.
     **/
    EventId m_capEvent;

    /**
     * Scheduler event for the end of the outgoing superframe CFP.
     */
    EventId m_cfpEvent;

    /**
     * Scheduler event for the end of the incoming superframe CAP.
     **/
    EventId m_incCapEvent;

    /**
     * Scheduler event for the end of the incoming superframe CFP.
     */
    EventId m_incCfpEvent;

    /**
     * Scheduler event for the end of the Dsme gts scheduling.
     */
    EventId m_gtsSchedulingEvent;

    /**
     * Dsme
     * Current scheduler event for the scheduled dsme tx gts slot.
     */
    EventId m_gtsEvent;

    /**
     * Dsme
     * Current scheduler event for the scheduled dsme rx gts slot.
     */
    EventId m_incGtsEvent;

    /**
     * 
     * Index: superframe ID, Value: Slot Allocation per CFP
     */
    std::vector<std::vector<EventId>> m_scheduleGTSsEvent;

    uint16_t m_curGTSSuperframeID;

    int m_curGTSIdx;

    uint16_t m_numOfChannelSupported = 16;

    /**
     * Dsme
     * Scheduler event for the start of the outgoing Multisuperframe.
     */
    EventId m_multisuperframeStartEvent;

    /**
     * Dsme
     * Scheduler event for the end of the outgoing Multisuperframe.
     */
    EventId m_multisuperframeEndEvent;

    /**
     * Dsme
     * Scheduler event for the end of the outgoing Multisuperframe.
     */
    EventId m_startFirstSuperframeEvent;

    /**
     * Dsme
     * Scheduler event for the start of the incoming Multisuperframe.
     */
    EventId m_incMultisuperframeStartEvent;

    /**
     * Scheduler event to track the incoming beacons.
     */
    EventId m_trackingEvent;

    /**
     * Scheduler event for the end of a channel scan.
     */
    EventId m_scanEvent;

    /**
     * Scheduler event for the end of a ED channel scan.
     */
    EventId m_scanEnergyEvent;

    /*
     * Scheduler event for the sending the Dsme-Beacon allocation notification command
     */
    EventId m_sendDsmeBcnAllocNotifiCmd;

}; // class LrWpanMac 

} // namespace ns3

#endif /* LR_WPAN_MAC_H */
