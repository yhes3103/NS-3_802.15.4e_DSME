/*
 * Copyright (c) 2019 Ritsumeikan University, Shiga, Japan.
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
 *  Author: Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

#ifndef LR_WPAN_MAC_PL_HEADERS_H
#define LR_WPAN_MAC_PL_HEADERS_H

#include "lr-wpan-fields.h"

#include <ns3/header.h>
#include <ns3/mac16-address.h>
#include <ns3/mac64-address.h>

#include <memory>

namespace ns3
{

/**
 * 
 * see IEEE 802.15.4e-2012, Section 5.1.1a and Table 52f
 */
class HoppingSequence {
public:
    uint8_t m_len;

    void SetSequenceLength(uint8_t len);
    uint16_t GetChannel(int idx);

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);

private:
    std::vector<uint16_t> m_channelList;
};

/**
 * The DSMESABSpecification field in DSME-GTS request command
 * , see IEEE 802.15.4e-2012, Section 5.3.11.4.7 Figure 59k, 59q, 59r
 * SAP -> Slot Allocation Bitmap
 * In Channel Hopping Mode, if CAP reduction is off, SAB sub-block uint is 7 bits (7 timeslots)
 * if CAP reduction is on, SAB sub-block uint is 15 bits
 */
class DSMESABSpecificationField {
public:
    void setCAPReduction(bool on);
    void setSABSubBlkLen(uint8_t len);
    void setSABSubBlkIdx(uint16_t idx);
    void setSABSubBlk(uint8_t subBlk);
    void setSABSubBlk(uint16_t subBlk);
    void setSABSubBlk(std::vector<uint8_t> subBlk);
    void setSABSubBlk(std::vector<uint16_t> subBlk);

    bool isCAPReduction() const;
    uint8_t GetSABSubBlkLen() const;
    uint16_t GetSABSubBlkIdx() const;
    const std::vector<uint16_t> GetSABSubBlk() const;
    const std::vector<uint8_t> GetSABSubBlkCapOff() const;

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);
    void Print(std::ostream& os) const;

private:
    bool m_capReduction;

    /**
     *! DSME SAB sub-block length , the length of SAB Sub-block, contain the length of the DSME SAB sub-block in units. 
     *! (One unit = 7 slot(No CAP reduction) , 15 slot(For CAP reduction)).
     */
    uint8_t m_dsmeSABSubBlkLen;   

    /**
     *! DSME SAB sub-block index , indicate the start position. 
     *! Indicate the beginning of the DSME SAB Sub-block in the entire SAB (not in unit).
     */                                
    uint16_t m_dsmeSABSubBlkIdx;

    std::vector<uint16_t> m_dsmeSABSubBlk;  //!< DSME SAB sub-block
    std::vector<uint8_t> m_dsmeSABSubBlkCapOff;  //!< DSME SAB sub-block
};

/**
 * The DSME-GTS Management in DSME-GTS request command, reply command
 * , see IEEE 802.15.4e-2012, Section 5.3.11.4.3 Figure 59j, Figure 59m
 * The Management Type field shall be set to one of the nonreserved values listed in Table 7a.
 */
class DSMEGTSManagementField {
public:
    void SetManagementType(uint8_t type);
    void SetRX(bool RX);
    void SetPrioritizedChannelAccess(bool high);
    void SetStatus(uint8_t status);

    uint8_t GetDSMEGTSManagementField() const;
    uint8_t GetManagementType() const;
    uint8_t GetStatus() const;
    bool IsDirectionRX() const;
    bool IsHighPriority() const;

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);

private:
    /* 1 Octet */
    uint8_t m_managementType;         //!< DSME-GTS Management field Management Type (Bit 0-2)
    bool m_direction;                 //!< DSME-GTS Management field Direction (Bit 3)
    bool m_prior_channel_access;      //!< DSME-GTS Management field Prioritized Channel Access (Bit 4)
    uint8_t m_status;                 //!< DSME-GTS Management field Status (Bit 5-7)  
};

/**
 * The Header Element ID namespace, see IEEE 802.15.4e-2012, Table 4a and Table 4b.
 */
enum HeaderElementIDs {
    HEADERIE_UNMANAGED             = 0x00,       //!< Unmanaged (i.e., implementation specific) IEs
    HEADERIE_LE_CSL                = 0x1a,       //!< LE CSL
    HEADERIE_LE_RIT                = 0x1b,       //!< LE RIT
    HEADERIE_DSME_PAN_DESCRIPTOR   = 0x1c,       //!< DSME PAN Descriptor
    HEADERIE_RZ_TIME               = 0x1d,       //!< RZ Time
    HEADERIE_ACK_NACK_TIME_CORR    = 0x1e,       //!< ACK/NACK Time-Correction
    HEADERIE_GACK                  = 0x1f,       //!< Group ACK
    HEADERIE_LOW_LANTENCY_NET_INFO = 0x20,       //!< Low Latency Network Info
    HEADERIE_LIST_TERMINATION_1    = 0x7e,       //!< List Termination 1
    HEADERIE_LIST_TERMINATION_2    = 0x7f,       //!< List Termination 2
    HEADERIE_ENHANCED_GACK         = 0x21,       //!< Reserved bit , extended for E-GACK
    HEADERIE_DSME_GTS_GACK         = 0x22,       //!< Reserved bit , extended for DSME-GTS GACK
    HEADERIE_RESERVED              = 0x23        //!< Reserved
};

/** 
 * Header Information Elements  See IEEE 802.15.4e-2012 Section 5.2.1.7a Figure 36u 
 * and Section 5.2.4.2 Figure 48n
 * 
 * IE descriptor 
 * Length - Element ID -    Type    - Content
 * 7 bits -  8 bits    -    1 bit   -   127 bytes 
 */ 
class HeaderIEDescriptor {
public:
    HeaderIEDescriptor();
    HeaderIEDescriptor(uint8_t len, HeaderElementIDs id, bool type); 
    ~HeaderIEDescriptor();

    void SetLength(uint8_t len);
    void SetHeaderElementID(HeaderElementIDs id);

    const uint8_t GetLength() const;
    const HeaderElementIDs GetHeaderElementID() const;
    const bool GetType() const;

    uint16_t GetDescriptor() const;
    void SetDescriptor(uint16_t descriptor);

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);
    void Print(std::ostream& os) const;

private:
    uint8_t m_length = 0;                     //!< Header Information Elements field Bit 0-6
    HeaderElementIDs m_elementId;         //!< Header Information Elements field Bit 7-14
    bool m_type = 0;                          //!< Header Information Elements field Bit 15
};

/**
 * \ingroup lr-wpan
 * Represent the DSME PAN Descriptor IE which is a kind of Header IE.
 * See IEEE 802.15.4e-2012   Section 5.2.4.9 Figure 48v
 */
class DsmePANDescriptorIE : public Header {
public:
    DsmePANDescriptorIE();
    DsmePANDescriptorIE(uint8_t len, HeaderElementIDs elementID);
    ~DsmePANDescriptorIE();

    void SetHeaderIEDescriptor(uint8_t len, HeaderElementIDs elementID);

    void SetSuperframeField(SuperframeField& superframeField);
    void SetSuperframeField(uint8_t bcnOrder, uint8_t frmOrder, 
                            uint8_t capSlot, bool battLifeExt, bool panCoor, bool assocPermit);

    void SetPendingAddrFields(uint8_t pndAddrSpecField);
    void SetPendingAddrFields(PendingAddrFields& pndAddrSpecField);

    void SetDsmeSuperFrameField(uint16_t dsmeSuperFrm);
    void SetDsmeSuperFrameField(uint8_t multiBcnOrder, bool channelDiversityMode
                                , bool gACKFlag, bool cAPReduction, bool deferredBcnFlag);
    void SetDsmeSuperFrameField(DsmeSuperFrameField& dsmeSuperFrm);                               

    void SetTimeSync(uint64_t bcnTimestamp, uint16_t bcnOfsTimeStamp);
    void SetTimeSync(TimeSync& timeSynField);

    void SetBeaconBitmap(uint16_t sdIndex, uint16_t bitmapLen);
    void SetBeaconBitmap(BeaconBitmap& beaconBitmapField);

    void SetChannelHopping(ChannelHopping& channelHoppingField);
    void SetChannelHopping(uint8_t seqID, uint8_t bcnSeqNum, uint16_t ofs
                            , uint8_t bitmapLen);

    void SetGroupACK(GroupACK& gack);
    void SetGroupACK(uint16_t gack1superFrmID, uint8_t gack1SlotID
                    , uint8_t gack1ChannelID, uint16_t gack2superFrmID
                    , uint16_t gack2SlotID, uint8_t gack2ChannelID);

    const HeaderIEDescriptor GetHeaderIEDescriptor() const;
    const SuperframeField GetSuperframeField() const;
    const PendingAddrFields GetPendingAddrFields() const;
    const DsmeSuperFrameField GetDsmeSuperFrameField() const;
    const TimeSync GetTimeSync() const;
    const BeaconBitmap GetBeaconBitmap() const;
    const ChannelHopping GetChannelHopping() const;
    const GroupACK GetGroupACK() const;

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const;

    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

    void Print(std::ostream &os) const override;

private:
    HeaderIEDescriptor m_descriptor;
    // IE content
    SuperframeField m_superframeField;
    PendingAddrFields m_pndAddrFields;
    DsmeSuperFrameField m_dsmeSuperframeField;
    TimeSync m_timeSynField;
    BeaconBitmap m_beaconBitmapField;
    ChannelHopping m_channelHoppingField;
    GroupACK m_gack;
};

class AckControl 
{
    public:
        AckControl();
        ~AckControl();

        void SetAckCtrl(uint8_t ackCtrl);
        uint8_t GetAckCtrl() const;

        void SetPayloadSize(uint8_t payloadSize);
        void SetBitmapSize(uint8_t bitmapSize);
        uint8_t GetPayloadSize() const;
        uint8_t GetBitmapSize() const;

        Buffer::Iterator Serialize(Buffer::Iterator i) const;
        uint32_t GetSerializedSize() const;
        Buffer::Iterator Deserialize(Buffer::Iterator i);
        // void Print(std::ostream& os) const;

    private:
        uint8_t m_payloadSize;      // Specify the length of the payload of a GACK frame. It is measured in number of octets.  
        
        uint8_t m_bitmapSize;       // Specify the size of bitmap in terms of octets.
        
        uint8_t m_reserved; 
};

/**
 * \ingroup lr-wpan
 * Represent the Group Ack IE of IEEE 802.15.4e.
 * See IEEE 802.15.4e-2012   Section 5.2.4.12 Figure 48cc
 */
class LegacyGroupAckIE : public Header 
{
    public:
        LegacyGroupAckIE();
        ~LegacyGroupAckIE();

        void SetHeaderIEDescriptor(); // TODO
        void SetIELength(uint32_t length); // TODO

        void SetBitmap(uint8_t bitmap, uint8_t bitLocation);
        void ResetBitmap(uint8_t bitmap);
        
        void SetGackBitmapField(uint16_t gackBitmap);
        uint16_t GetGackBitmapField() const;

        void SetGackDevListField(uint8_t gackDevList);
        uint8_t GetGackDevListField() const;

        void SetGackIdxBitmapField(uint16_t gackIdxBitmap);
        uint16_t GetGackIdxBitmapField() const;

        void SetGackIdx(uint16_t gackIdxBitmap, uint16_t startLocation, uint16_t value);
        uint16_t GetGackIdx(uint16_t gackIdxBitmap, uint16_t startLocation);

        void SetGtsDirectionBitmapField(uint8_t gtsDirectionBitmap);
        uint8_t GetGtsDirectionBitmapField() const;

        static TypeId GetTypeId();
        TypeId GetInstanceTypeId() const;

        uint32_t GetSerializedSize() const override;
        void Serialize(Buffer::Iterator start) const override;
        uint32_t Deserialize(Buffer::Iterator start) override;

        void Print(std::ostream &os) const override;

    private:

        HeaderIEDescriptor m_descriptor;
        uint32_t ieLength;  

        AckControl m_ackCtrl;
        uint16_t m_gackBitmap;
        uint8_t m_gackDevList;
        uint16_t m_gackIdx;
        uint8_t m_gtsDirections;
};

/**
 * \ingroup lr-wpan
 * Self Designed Enhanced Group Header IE
 */
class EnhancedGroupAckDescriptorIE : public Header 
{
    public:
        EnhancedGroupAckDescriptorIE();
        ~EnhancedGroupAckDescriptorIE();

        void SetHeaderIEDescriptor();
        void SetIELength(uint32_t length);
        void SetGroupAckBitmap(uint64_t bitmap);
        uint64_t GetGroupAckBitmap() const; 
        uint32_t GetIELength() const;
        static TypeId GetTypeId();
        TypeId GetInstanceTypeId() const;

        uint32_t GetSerializedSize() const override;
        void Serialize(Buffer::Iterator start) const override;
        uint32_t Deserialize(Buffer::Iterator start) override;

        void Print(std::ostream &os) const override;

        void PrintBitmap();

    private:
        HeaderIEDescriptor m_descriptor;

        uint32_t ieLength;  // TODO: if we decide to use dynamic adjust bitmap size, this variable needs to add into IE content

        // IE content
        uint64_t m_u8GroupAckBitmap;
        // uint128_t m_u16GroupAckBitmap;
};

std::ostream& operator << (std::ostream &os, const EnhancedGroupAckDescriptorIE& enhancedGroupAckDescriptorIE);



typedef struct 
{
    Mac16Address nodeAddr;
    uint8_t bitmapLength = 1; // Default 1 bytes
    uint8_t sequenceNumber;
    std::vector<uint8_t> bitmap;
    
} DsmeGtsGackPayload;

class DsmeGtsGroupAckDescriptorIE : public Header 
{
    public:
        DsmeGtsGroupAckDescriptorIE();
        ~DsmeGtsGroupAckDescriptorIE();

        void SetPayloadsNumber(uint8_t payloadNumber);
        uint8_t GetPayloadsNumber() const;

        void SetDsmeGtsGackPayload(std::vector<DsmeGtsGackPayload> payloads);

        void GetDsmeGtsGackPayload(std::vector<DsmeGtsGackPayload> &payloads);
        
        void AdjustBitmapSize(DsmeGtsGackPayload &gtsGackPayload);  // Adjust m_bitmap in DsmeGtsGackPayload according to the m_bitmapLength
        void AdjustPayloadSize(); // Adjust m_payload size according to the m_payloadsNumber;
        
        uint32_t GetPayloadTotalSize() const; // return the total size of the m_payload vector

        void SetHeaderIEDescriptor();
        void SetIELength(uint32_t length);
        uint32_t GetIELength() const;
        static TypeId GetTypeId();
        TypeId GetInstanceTypeId() const;
        uint32_t GetSerializedSize() const override;
        void Serialize(Buffer::Iterator start) const override;
        uint32_t Deserialize(Buffer::Iterator start) override;

        void Print(std::ostream &os) const override;

    private:
        HeaderIEDescriptor m_descriptor;
        uint32_t ieLength;  

        // IE content
        uint8_t m_payloadsNumber;                       //!<  The number of the payloads.

        std::vector<DsmeGtsGackPayload> m_payloads;     //!<  The Payloads in the DSME-GTS bitmap structure.
};

std::ostream& operator << (std::ostream &os, const DsmeGtsGroupAckDescriptorIE& dsmeGtsGroupAckDescriptorIE);


/**
 * \ingroup lr-wpan
 * 
 * Table 6-3 of IEEE 802.15.4e-2015.
 */
typedef enum {
    TRLE_ENALBED = 0,
    TSCH_ENABLED = 1,
    DSME_ENABLED = 2,
    LE_ENABLED = 3,
    HOPPING_ENABLED = 4,
    DA_ENALBED = 5, 
    MPMIE_ENALBED = 6,
    EBR_PIB_ATTRIBUTE_ID_RESERVED = 7
} EBRPIBAttributeID;

/**
 * \ingroup lr-wpan
 * Represent the Header IE Termination .
 * See IEEE 802.15.4e-2012 Section 5.2.4.22
 */
class HeaderIETermination : public Header {
public:
    HeaderIETermination();
    ~HeaderIETermination();

    void SetHeaderIEDescriptor(uint8_t len, HeaderElementIDs elementID);

    const HeaderIEDescriptor GetHeaderIEDescriptor() const;

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const;

    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream &os) const override;

private:
    HeaderIEDescriptor m_descriptor;
};

/** 
 * Payload Information Elements  
 * See IEEE 802.15.4e-2012 Section 5.2.1.7a Figure 36u and Section 5.2.4.3 Figure 48o
 * Payload IE descriptor 
 * Length  - Group ID - Type - Content
 * 10 bits -  3 bits  -  1   - 2048 bytes 
 */ 
enum PayloadIEGroupIDs {
    PAYLOADIE_ESDU                = 0x0,       //!< Encapsulated Service Data Unit (ESDU)
    PAYLOADIE_MLME                = 0x1,       //!< MLME (Nested)
    PAYLOADIE_UNMANAGED           = 0x2,       //!< Unmanaged
    PAYLOADIE_RESERVED            = 0xa,       //!< Reserved
    PAYLOADIE_LIST_TERMINATION  = 0xf       //!< List termination
};

class PayloadIEDescriptor {
public:
    PayloadIEDescriptor();
    PayloadIEDescriptor(uint8_t len, PayloadIEGroupIDs id, bool type);  // For termination
    ~PayloadIEDescriptor();

    void SetLength(uint16_t len);
    void SetPayloadIEGroupID(PayloadIEGroupIDs id);

    const uint16_t GetLength() const;
    const PayloadIEGroupIDs GetPayloadIEGroupID() const;
    const bool GetType() const;

    uint16_t GetDescriptor() const;
    void SetDescriptor(uint16_t descriptor);

    // static TypeId GetTypeId();
    // TypeId GetInstanceTypeId() const;

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);
    void Print(std::ostream& os) const;

private:
    uint16_t m_length = 0;                 //!< Payload Information Elements field Bit 0-10
    PayloadIEGroupIDs m_elementId;         //!< Payload Information Elements field Bit 11-14
    bool m_type = 1;                       //!< Payload Information Elements field Bit 15
};

/** 
 * Nested MLME Information Elements  
 * See IEEE 802.15.4e-2012 Section 5.2.4.5 Figure 48q, Figure 48r, Figure 48s
 * 
 * Nested MLME IE Sub-IE descriptor(Short):
 * Length  - Sub-ID  - Type 
 * 8 bits -  7 bits  -  1  
 * 
 * Nested MLME IE(Short)
 * Length  - Group ID - Type - Length  - Sub-ID  - Type - Content
 * 10 bits -  3 bits  -  1   - 8 bits -  7 bits  -  1   - 0 ... 255 bytes
 */ 
enum MLMEIESubID {
    SUBID_RESERVED = 0x00,
    SUBID_TSCH_SYNC = 0x1a,
    SUBID_TSCH_SLOTFRAME_AND_LINK = 0x1b,
    SUBID_TSCH_TIMESLOT = 0x1c,
    SUBID_HOPPING_TIMING = 0x1d,
    SUBID_EB_FILTER = 0x1e,
    SUBID_MAC_METRICS_1 = 0x1f,
    SUBID_MAC_METRICS_2 = 0x20,
    SUBID_UNMANAGED = 0x40
};

class PayloadSubIEDescriptor {
public:
    PayloadSubIEDescriptor();
    ~PayloadSubIEDescriptor();

    void SetLength(uint8_t len);
    void SetMLMESubID(MLMEIESubID id);
    void SetType(bool type);

    uint8_t GetLength() const;
    MLMEIESubID GetMLMESubID() const;
    bool GetType() const;

    uint16_t GetDescriptor() const;
    void SetDescriptor(uint16_t descriptor);

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);
    void Print(std::ostream& os) const;

private:
    uint8_t m_length;           //<! 8, 11 bits
    MLMEIESubID m_subId;        //<! 4, 7 bits
    bool m_type;                //<! 1 bit
};

/**
 * \ingroup lr-wpan
 * Represent the Enhanced Beacon filter IE contained in EBR.
 * See IEEE 802.15.4e-2012 Section 5.2.4.18
 */
class EBFilterIE : public Header {
public:
    EBFilterIE();
    ~EBFilterIE();

    void SetOutterIEDescriptorLen(uint16_t len);
    void SetSubIEDescriptorLen(uint8_t len);
    
    void SetEBFilterDescriptor(uint8_t descriptor);

    void SetPermitJoiningOn();
    void SetNoPermitJoiningOn();

    void SetIncludeLinkQualityFilter();
    void SetNotIncludeLinkQualityFilter();

    void SetIncludePercentFilter();
    void SetNotIncludePercentFilter();

    void SetNumOfEntriesInPIBIdList(uint8_t num);

    void SetLinkQuality(uint8_t quality);

    void SetPercentFilter(uint8_t percent);

    void AddPIBId(EBRPIBAttributeID id);

    PayloadIEDescriptor GetHeaderIEDescriptor() const;

    bool IsPermitJoiningOn() const;
    bool IsLinkQualityFilterIncluded() const;
    bool IsPercentFilterIncluded() const;

    uint8_t GetNumOfEntriesInPIBIdList() const;
    uint8_t GetLinkQuality() const;
    uint8_t GetPercentFilter() const;

    uint8_t GetEBFilterDescriptor() const;

    std::vector<EBRPIBAttributeID> GetPIBIdList() const;

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const;

    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

    void Print(std::ostream &os) const override;    

private:
    PayloadIEDescriptor m_descriptor;
    PayloadSubIEDescriptor m_subIEDescriptor;

    bool m_permitJoiningOn;                 //<! Bit 0
    bool m_includeLinkQualityFilter;        //<! Bit 1
    bool m_includePercentFilter;            //<! Bit 2
    uint8_t m_numOfEntriesInPIBIdList;      //<! Bit 3-4
    uint8_t m_reserved;                     //<! Bit 5-7 Reserved

    uint8_t m_linkQuality;                          //<! 1 Octet
    uint8_t m_percentFilter;                        //<! 1 Octet
    std::vector<EBRPIBAttributeID> m_PIBIdList;     //<! 0/1/2/3 Octets
};

/**
 * \ingroup lr-wpan
 * Represent the Payload IE Termination .
 * See IEEE 802.15.4e-2012 Section 5.2.4.22
 */
class PayloadIETermination : public Header {
public:
    PayloadIETermination();
    ~PayloadIETermination();

    void SetPayloadIEDescriptor(uint8_t len, PayloadIEGroupIDs elementID);

    const PayloadIEDescriptor GetPayloadIEDescriptor() const;

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const;
    
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream &os) const override;

private:
    PayloadIEDescriptor m_descriptor;
};

/**
 * The Link status Descriptor format in The Link Status Specification field
 * , see IEEE 802.15.4e-2012, Section 5.3.11.11.3 Figure 59w
 */
typedef struct {
    uint8_t channel;       //!< Link status Descriptor field, Channel 1 Octet
    uint8_t avgLQI;        //!< Link status Descriptor field, average received LQI 1 Octet
    uint8_t avgRSSI;       //!< Link status Descriptor field, averge received signal power 1 Octet
                           //!< Link status Descriptor field, Reserved 1 Octet
} LinkStatusDescriptor;

/**
 * The DSME Link Status Specification field in DSME-Link status report command
 * , see IEEE 802.15.4e-2012, Section 5.3.11.11.3 Figure 59v
 */
class LinkStatusSpecificationField {
public:
    void SetLinkStatusDescriptorCnt(uint8_t count);

    uint8_t GetLinkStatusDescriptorCnt() const;
    const std::vector<LinkStatusDescriptor> GetLinkStatusList() const;

    uint32_t GetSerializedSize() const;
    Buffer::Iterator Serialize(Buffer::Iterator i) const;
    Buffer::Iterator Deserialize(Buffer::Iterator i);

private:
    uint8_t m_linkStatusDescriptorCnt;
    std::vector<LinkStatusDescriptor> m_linkStatusList;
};

/**
 * \ingroup lr-wpan
 * Implements the header for the MAC payload beacon frame according to
 * see IEEE 802.15.4-2011 Section 5.2.2.1 Figure 38.
 */
class BeaconPayloadHeader : public Header
{
  public:
    BeaconPayloadHeader();
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;
    /**
     * Set the superframe specification field to the beacon payload header.
     * \param sfrmField The superframe specification field
     */
    void SetSuperframeSpecField(SuperframeField sfrmField);
    /**
     * Set the superframe Guaranteed Time Slot (GTS) fields to the beacon payload header.
     * \param gtsFields The GTS fields.
     */
    void SetGtsFields(GtsFields gtsFields);
    /**
     * Set the superframe Pending Address fields to the beacon payload header.
     * \param pndAddrFields The Pending Address fields.
     */
    void SetPndAddrFields(PendingAddrFields pndAddrFields);
    /**
     * Get the superframe specification field from the beacon payload header.
     * \return The superframe specification field
     */
    SuperframeField GetSuperframeSpecField() const;
    /**
     * Get the Guaranteed Time Slots (GTS) fields from the beacon payload header.
     * \return The GTS fields.
     */
    GtsFields GetGtsFields() const;
    /**
     * Get the pending address fields from the beacon payload header.
     * \return The Pending Address fields.
     */
    PendingAddrFields GetPndAddrFields() const;

  private:
    /**
     * Superframe Specification Field
     */
    SuperframeField m_superframeField;
    /**
     * GTS Fields
     */
    GtsFields m_gtsFields;
    /**
     * Pending Address Fields
     */
    PendingAddrFields m_pndAddrFields;
};

/**
 * \ingroup lr-wpan
 * Implements the header for the MAC payload command frame according to
 * the IEEE 802.15.4e-2012 Section 5.2.2.4 Figure 48 and Section 5.3 Table 5
 */
class CommandPayloadHeader : public Header
{
  public:
    /**
     *  The MAC command frames.
     *  See IEEE 802.15.4e-2012, Section 5.3 Table 5
     */
    enum MacCommand
    {
        ASSOCIATION_REQ = 0x01,      //!< Association request (RFD true: Tx)
        ASSOCIATION_RESP = 0x02,     //!< Association response (RFD true: Rx)
        DISASSOCIATION_NOTIF = 0x03, //!< Disassociation notification (RFD true: TX, Rx)
        DATA_REQ = 0x04,             //!< Data Request (RFD true: Tx)
        PANID_CONFLICT = 0x05,       //!< Pan ID conflict notification (RFD true: Tx)
        ORPHAN_NOTIF = 0x06,         //!< Orphan Notification (RFD true: Tx)
        BEACON_REQ = 0x07,           //!< Beacon Request (RFD true: none )
        COOR_REALIGN = 0x08,         //!< Coordinator Realignment (RFD true: Rx)
        GTS_REQ = 0x09,              //!< GTS Request (RFD true: none)
        // LL_DISCOVER_RESP            = 0x0d,
        // LL_CONFIG_STATUS            = 0x0e,
        // LL_CONFIG_REQ               = 0x0f,
        // LL_CTS_SHARED_GROUP         = 0x10,
        // LL_RTS                      = 0x11,
        // LL_CTS                      = 0x12,
        DSME_ASSOCIATION_REQ        = 0x13,        //!< DSME Association Request (RFD true: Tx)
        DSME_ASSOCIATION_RESP       = 0x14,        //!< DSME Association Response (RFD true: Rx)
        DSME_GTS_REQ                = 0x15,        //!< DSME GTS Request (RFD true: TX, Rx)
        DSME_GTS_REPLY              = 0x16,        //!< DSME GTS Reply (RFD true: TX, Rx)
        DSME_GTS_NOTIFY             = 0x17,        //!< DSME GTS Notify (RFD true: TX, Rx)
        DSME_INFO_REQ               = 0x18,        //!< DSME Information Request (RFD true: TX, Rx)
        DSME_INFO_REPLY             = 0x19,        //!< DSME Information Reply (RFD true: TX, Rx)
        DSME_BEACON_ALLOC_NOTIF     = 0x1a,        //!< DSME Beacon allocation notification (RFD true: none)
        DSME_BEACON_COLLISION_NOTIF = 0x1b,        //!< DSME Beacon collision notification (RFD true: Tx)
        DSME_LINK_STATUS_REPORT     = 0x1c,        //!< DSME Link status report (RFD true: TX, Rx)

        /* AMCA + LE-RIT part are ommited here */

        CMD_RESERVED = 0xff          //!< Reserved
    };

    /**
     *  Association Status Field values.
     *  See IEEE 802.15.4e-2012, Section 5.3.2.3 Table 6
     */
    enum AssocStatus
    {
        SUCCESSFUL = 0x00,                      //!< Association successful
        FULL_CAPACITY = 0x01,                   //!< PAN at capacity
        ACCESS_DENIED = 0x02,                   //!< PAN access denied
        HOPPING_SEQ_OFS_DUPLICATION = 0x03,     //!< Hopping Sequence offset duplication
        RESERVED = 0x04,                        //!< Reserved
        FASTA_SUCCESSFUL = 0x80                 //!< FastA successful
    };

    enum DisassociationReason {
        DISASSC_RESERVED = 0x00,
        DISASSC_COORD_WISH_DEV_LEAVE_PAN = 0x01,
        DISASSC_DEV_LEAVE_PAN = 0x02,
    };

    CommandPayloadHeader();
    /**
     * Constructor
     * \param macCmd the command type of this command header
     */
    CommandPayloadHeader(enum MacCommand macCmd);
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    /**
     * Set the command frame type
     * \param macCmd the command frame type
     */
    void SetCommandFrameType(MacCommand macCmd);
    /**
     * Set the Capability Information Field to the command payload header (Association Request
     * Command).
     * \param cap The capability Information field
     */
    void SetCapabilityField(CapabilityField cap);

    /**
     * Set the Disassociation Reason Field to the command payload header (Disassociation notification 
     * Command). \param reason The disassociation Reason Field
     */
    void SetDisassociationReason(DisassociationReason reason);

    DisassociationReason GetDisassociationReason() const;

    /**
     * Set the Short Address Assigned by the coordinator (Association Response Command).
     * \param shortAddr The short address assigned by the coordinator
     */
    void SetShortAddr(Mac16Address shortAddr);
    
    /**
     * Set status resulting from the association attempt (Association Response Command).
     * \param status The status resulting from the association attempt
     */
    void SetAssociationStatus(AssocStatus status);

    void SetPanId(uint16_t id);

    void SetCoordinatorShortAddress(Mac16Address addr);

    void SetChannelNum(uint16_t num);

    void SetChannelPage(uint8_t page);

        /**
     * Set Hopping Sequence ID (Dsme Association Request Command).
     * 1 .Hopping Sequence ID of zero indicates that a default hopping sequence 
     * shall be used.
     * 2 .HoppingSequenceID of one indicates that a hopping sequence generated 
     * by PAN coordinator shall be used.
     * 3. The other value of the Hopping Sequence ID denotes the sequence 
     * set by a higher layer shall be used.
     * \param seqId The hopping sequence ID
     */
    void SetHoppingSeqID(uint8_t seqId);

    /**
     * Set Channel Offset (Dsme Association Request Command).
     * The Channel Offset field shall be set to the offset value 
     * of the unassociated device that wished to associate with a PAN
     * \param ofs The channel offset
     */
    void SetChannelOfs(uint16_t ofs);

    /**
     * Set Channel Hopping Sequence Length (Dsme Association Response Command).
     * Hopping Sequence Length field shall specify the length of 
     * the Hopping Sequence used in the PAN if the PAN runs 
     * in both beacon-enabled mode and Channel Hopping mode
     * \param len The hopping sequence length
     */
    void SetHoppingSeqLen(uint8_t len);

    /**
     * Set Channel Hopping Sequence (Dsme Association Response Command).
     * Hopping Sequence Length field shall specify the length of 
     * the Hopping Sequence used in the PAN if the PAN runs 
     * in both beacon-enabled mode and Channel Hopping mode
     * \param len The hopping sequence length
     */
    void SetHoppingSequnce(HoppingSequence seq);

    void SetDsmeGtsManagementField(DSMEGTSManagementField field);

    void SetDsmeGtsNumOfSlot(uint8_t num);

    void SetDsmeGtsPreferredSuperframeId(uint16_t superframeId);

    void SetDsmeGtsPreferredSlotId(uint8_t slotId);

    void SetDsmeGtsSABSpec(DSMESABSpecificationField spec);

    void SetDsmeGtsDestAddress(Mac16Address addr);

    void SetAllocationBcnSDIndex(uint16_t idx);

    void SetCollisionBcnSDIndex(uint16_t idx);

    /**
     * Setter for Dsme Info Request Command
     */
    void SetDsmeInfoType(uint8_t type);

    void SetDsmeInfoSABSubBlkLen(uint8_t len);

    void SetDsmeInfoSABSubBlkIdx(uint16_t idx);

    /**
     * Setter for Dsme Info Reply Command
     */
    void SetDsmeInfoTimestamp(uint64_t timestamp);

    void SetDsmeInfoSuperframeID(uint16_t id);

    void SetDsmeInfoSlotID(uint8_t id);

    void SetDsmeInfoPANDescriptor(DsmePANDescriptorIE des);

    void SetEBSAllocationSeq(uint16_t allocSeq);

    uint16_t GetEBSAllocationSeq();
    
    /**
     * Get the Short address assigned by the coordinator (Association Response Command).
     * \return The Mac16Address assigned by the coordinator
     */
    Mac16Address GetShortAddr() const;

    /**
     * Get the status resulting from an association request (Association Response Command).
     * \return The resulting status from an association request
     */
    AssocStatus GetAssociationStatus() const;

    uint16_t GetPanId() const ;

    Mac16Address GetCoordinatorShortAddress() const;

    uint16_t GetChannelNum() const;

    uint8_t GetChannelPage() const;

    uint8_t GetHoppingSeqLen();

    HoppingSequence GetHoppingSequnce() const;

    uint16_t GetChannelOfs();

    uint8_t GetHoppingSeqID();    

    /**
     * Get the command frame type ID
     * \return The command type ID from the command payload header
     */
    MacCommand GetCommandFrameType() const;

    /**
     * Get the Capability Information Field from the command payload header. (Association Request
     * Command)
     * \return The Capability Information Field
     */
    CapabilityField GetCapabilityField() const;

    DSMEGTSManagementField GetDsmeGtsManagementField() const;

    uint8_t GetDsmeGtsNumOfSlot() const;

    uint16_t GetDsmeGtsPreferredSuperframeId() const;

    uint8_t GetDsmeGtsPreferredSlotId() const;

    DSMESABSpecificationField GetDsmeGtsSABSpec() const;

    Mac16Address GetDsmeGtsDestAddress() const;

    uint16_t GetAllocationBcnSDIndex() const;

    uint16_t GetCollisionBcnSDIndex() const;

    /**
     * Getter for Dsme Info Request Command
     */
    uint8_t GetDsmeInfoType() const;

    uint8_t GetDsmeInfoSABSubBlkLen() const;

    uint16_t GetDsmeInfoSABSubBlkIdx() const;

    /**
     * Getter for Dsme Info Reply Command
     */
    uint64_t GetDsmeInfoTimestamp() const;

    uint16_t GetDsmeInfoSuperframeID() const;

    uint8_t GetDsmeInfoSlotID() const;

    DsmePANDescriptorIE GetDsmeInfoPANDescriptor() const;

  private:
    MacCommand m_cmdFrameId; //!< The command Frame Identifier
    CapabilityField
        m_capabilityInfo;      //!< Capability Information Field (Association Request Command)
    Mac16Address m_shortAddr;  //!< Contains the short address assigned by the coordinator
                               //!< (Association Response Command) See IEEE 802.15.4-2011 5.3.2.2.
    AssocStatus m_assocStatus; //!< Association Status (Association Response Command)

    /**
     * Disassociation notification command
     * See 802.15.4-2011 Section 5.3.3 Figure 52 and 5.3.3.2
     *      MHR         - Command Frame Identifier - Disassociation Reason 
     * Variable Octets  -            1             -           1           
     */
    uint8_t m_disassociationReason;

    /**
     * Coordinator realignment command
     * See 802.15.4-2011 Section 5.3.8 Figure 57
     *      MHR         - Command Frame Identifier - Pan Id - Coordinator Short Addr -
     * Variable Octets  -            1             -    2   -           2
     * 
     * Channel Num - Short Addr - Channel Page
     *      1      -      2     -      0/1
     */
    uint16_t m_panId;
    Mac16Address m_coordShortAddr;
    uint8_t m_channelNum;
    // Mac16Address m_shortAddr;
    uint8_t m_channelPage;

    /**
     * DSME-Association request command
     * See Section 5.3.11.2 Figure 59g
     *      MHR         - Command Frame Identifier - Capability Inforamtion - Hopping Sequence ID - Channel Offset
     * Variable Octets  -            1             -            1           -           1         -        2
     */
    // addressing mode field of the Frame Control field shall be set to three
    // CapabilityField m_capabilityInfo;
    uint8_t m_hoppingSeqId;
    uint16_t m_channelOfs;

    /**
     * DSME-Association response command
     * See Section 5.3.11.3 Figure 59h
     *      MHR         - Command Frame Identifier - Short Address - Association Status - Hopping Sequence Length - Hopping Sequence
     * Variable Octets  -            1             -       2        -         1         -            1            -     Variable
     */
    // Mac16Address m_shortAddr;
    // AssocStatus m_assocStatus;
    uint8_t m_hoppingSeqLen;                //!< Hopping Sequence Length (DSME-Association response command)
    HoppingSequence m_hoppingSeq;           //!< Hopping Sequence (DSME-Association response command)

    /**
     * DSME-GTS request command 
     * See Section 5.3.11.4 Figure 59i
     *      MHR         - Command Frame Identifier - DSME-GTS Management - Number of Slots - Preferred Superframe ID - Preferred Slot ID - DSMESABSpecification
     * Variable Octets  -            1             -          1          -         0/1     -          0/2            -        0/1        -       Variable
     */
    DSMEGTSManagementField m_dsmeGTSManagement;
    uint8_t m_numOfSlot;
    uint16_t m_preferredSuperframeID;
    uint8_t m_preferredSlotID;
    DSMESABSpecificationField m_dsmeSABSpec;

    /**
     * DSME-GTS reply command, notify command 
     * See Section 5.3.11.5 Figure 59l, Section 5.3.11.6 Figure 59n
     *      MHR         - Command Frame Identifier - DSME-GTS Management - Destination Address - Channel Offset - DSMESABSpecification
     * Variable Octets  -            1             -          1          -         2           -      0/2       -       Variable
     */
    // DSMEGTSManagementField m_dsmeGTSManagement;
    Mac16Address m_destAddr;
    // uint16_t m_channelOfs;
    // DSMESABSpecificationField m_dsmeSABSpec;

    /**
     * DSME-Information request command
     * See Section 5.3.11.7 Figure 59o
     *      MHR         - Command Frame Identifier - Info type - DSME SAB sub-block length - DSME SAB sub-block index
     * Variable Octets  -            1             -     1     -              1            -             2
     */
    uint8_t m_infoType;
    uint8_t m_dsmeSABSubBlkLen;
    uint16_t m_dsmeSABSubBlkIdx;

    /**
     * DSME-Information reply command
     * See Section 5.3.11.8 Figure 59p, Section 5.2.4.9
     *      MHR         - Command Frame Identifier - Info type - Timestamp - Superframe ID - Slot ID - DSME SAB Specification - DSME PAN Descritpor
     * Variable Octets  -            1             -     1     -     1     -       2       -    1    -        Variable        -       Variable                  
     */
    // uint8_t m_infoType;
    uint64_t m_timestamp;
    uint16_t m_superframeID;
    uint8_t m_slotID;
    // DSMESABSpecificationField m_dsmeSABSpecField;
    DsmePANDescriptorIE m_dsmePANDercriptorIE;

    /**
     * DSME-Beacon allocation notification command
     * See Section 5.3.11.9 Figure 59s
     */
    uint16_t m_allocationBcnSDIndex;
    
    /**
     * DSME-Beacon collision notification command
     * See Section 5.3.11.10 Figure 59t
     */

    uint16_t m_collisionSDIndex;

    /**
     * DSME-Link status report command
     * See Section 5.3.11.11 Figure 59u, 5.3.11.11.3 Figure 59v
     */
    LinkStatusSpecificationField m_dsmeLinkStatusSpecField;

    /**
     * Self-designed enhanced beacon scheduling parameters
     * For the purpose of allocation collision free
     */

    uint16_t m_allocationSequence;
};

/**
 * \ingroup lr-wpan
 * To test if I can make a new packet header class
 */
class TestNewHeader : public Header {
public:
    TestNewHeader();

    void SetData(uint16_t data);
    uint16_t GetData() const;

    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

private:

    uint16_t m_testData;
};


/**
 * \ingroup lr-wpan
 * Represent the Channel hopping IE.
 * See IEEE 802.15.4e-2012   Section 5.2.4.16 Figure 48jj
 */
class ChannelHoppingIE : public Header {

};

} // namespace ns3

#endif /* LR_WPAN_MAC_PL_HEADERS_H */
