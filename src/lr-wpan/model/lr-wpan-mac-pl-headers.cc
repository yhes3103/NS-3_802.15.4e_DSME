/*
 * Copyright (c) 2020 Ritsumeikan University, Shiga, Japan.
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

#include "lr-wpan-mac-pl-headers.h"

#include <ns3/address-utils.h>
#include <ns3/simulator.h>

#include <bitset>

namespace ns3
{

/***********************************************************
 *                Beacon MAC Payload
 ***********************************************************/

BeaconPayloadHeader::BeaconPayloadHeader()
{
}

NS_OBJECT_ENSURE_REGISTERED(BeaconPayloadHeader);

TypeId
BeaconPayloadHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::BeaconPayloadHeader")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<BeaconPayloadHeader>();
    return tid;
}

TypeId
BeaconPayloadHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
BeaconPayloadHeader::GetSerializedSize() const
{
    uint32_t size = 0;
    size += m_superframeField.GetSerializedSize();
    size += m_gtsFields.GetSerializedSize();
    size += m_pndAddrFields.GetSerializedSize();

    return size;
}

void
BeaconPayloadHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    i = m_superframeField.Serialize(i);
    i = m_gtsFields.Serialize(i);
    i = m_pndAddrFields.Serialize(i);
}

uint32_t
BeaconPayloadHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    i = m_superframeField.Deserialize(i);
    i = m_gtsFields.Deserialize(i);
    i = m_pndAddrFields.Deserialize(i);

    return i.GetDistanceFrom(start);
}

void
BeaconPayloadHeader::Print(std::ostream& os) const
{
    os << "| Superframe Spec Field | = " << m_superframeField
       << "| GTS Spec Field | = " << m_gtsFields.GetGtsSpecField()
       << "| Pending Spec Field| =" << m_pndAddrFields.GetPndAddrSpecField();
}

void
BeaconPayloadHeader::SetSuperframeSpecField(SuperframeField sf)
{
    m_superframeField = sf;
}

void
BeaconPayloadHeader::SetGtsFields(GtsFields gtsFields)
{
    m_gtsFields = gtsFields;
}

void
BeaconPayloadHeader::SetPndAddrFields(PendingAddrFields pndAddrFields)
{
    m_pndAddrFields = pndAddrFields;
}

SuperframeField
BeaconPayloadHeader::GetSuperframeSpecField() const
{
    return m_superframeField;
}

GtsFields
BeaconPayloadHeader::GetGtsFields() const
{
    return m_gtsFields;
}

PendingAddrFields
BeaconPayloadHeader::GetPndAddrFields() const
{
    return m_pndAddrFields;
}

/***********************************************************
 *                Command MAC Payload
 ***********************************************************/

CommandPayloadHeader::CommandPayloadHeader()
{
    SetCommandFrameType(CMD_RESERVED);
}

CommandPayloadHeader::CommandPayloadHeader(enum MacCommand macCmd)
{
    SetCommandFrameType(macCmd);
}

NS_OBJECT_ENSURE_REGISTERED(CommandPayloadHeader);

TypeId
CommandPayloadHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::CommandPayloadHeader")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<CommandPayloadHeader>();
    return tid;
}

TypeId
CommandPayloadHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
CommandPayloadHeader::GetSerializedSize() const
{
    uint32_t size = 1;  // Command Frame Identifier

    // TODO: add missing serialize commands size when other commands are added.
    switch (m_cmdFrameId) {
        case ASSOCIATION_REQ:
            size += m_capabilityInfo.GetSerializedSize();
            break;

        case ASSOCIATION_RESP:
            size += 3; // (short address + Association Status)
            break;

        case DISASSOCIATION_NOTIF:
            size += 1; // (Disassociation Reason)
            break;

        case DATA_REQ:
            break;
        case PANID_CONFLICT:
            break;
        case ORPHAN_NOTIF:
            break;
        case BEACON_REQ:
            break;
        case COOR_REALIGN:
            size += 2 + 2 + 1 + 2 + 1; 
            break;

        case GTS_REQ:
            break;

        case DSME_ASSOCIATION_REQ:
            size += m_capabilityInfo.GetSerializedSize() + 1 + 2;      // Cap Info + Hopping Seq ID + Channel Offset
            break;

        case DSME_ASSOCIATION_RESP:
            // short address + Association Status + Hopping Seq Length + hopping seq + allocation sequence
            size += 2 + 1 + 1 + ceil(m_hoppingSeqLen / 8) + 2;             
            break;

        case DSME_GTS_REQ:
            size += m_dsmeGTSManagement.GetSerializedSize();

            if (m_dsmeGTSManagement.GetManagementType() == 0b001) {
                size += 1 + 2 + 1;
            }

            size += m_dsmeSABSpec.GetSerializedSize();

            break;

        case DSME_GTS_REPLY:
            size += 1 + 2 + 2 + m_dsmeSABSpec.GetSerializedSize();
            break;

        case DSME_GTS_NOTIFY:
            size += 1 + 2 + 2 + m_dsmeSABSpec.GetSerializedSize();
            break;

        case DSME_INFO_REQ:
            size += 1 + 1 + 2;
            break;

        case DSME_INFO_REPLY:
            // size += 1 + 6 + 2 + 1 + m_dsmeSABSpec.GetSerializedSize() + m_dsmePANDercriptorIE.GetSerializedSize();
            size += 1 + 8 + 2 + 1;

            if (m_infoType == 0x01) {
                size += m_dsmeSABSpec.GetSerializedSize();
            } else if (m_infoType == 0x02) {
                size += m_dsmePANDercriptorIE.GetSerializedSize();
            }

            break;

        case DSME_BEACON_ALLOC_NOTIF:
            size += 2;
            break;

        case DSME_BEACON_COLLISION_NOTIF:
            size += 2;
            break;

        case DSME_LINK_STATUS_REPORT:
            // size += m_dsmeLinkStatusSpecField.GetSerializedSize();
            break;
            
        case CMD_RESERVED:
            break;
    }

    return size;
}

void CommandPayloadHeader::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;
    i.WriteU8(m_cmdFrameId);

    // TODO: add missing serialize commands when other commands are added.
    switch (m_cmdFrameId) {
        case ASSOCIATION_REQ:
            i = m_capabilityInfo.Serialize(i);
            break;

        case ASSOCIATION_RESP:
            WriteTo(i, m_shortAddr);
            i.WriteU8(m_assocStatus);
            break;

        case DISASSOCIATION_NOTIF:
            i.WriteU8(m_disassociationReason);
            break;

        case DATA_REQ:
            break;
        case PANID_CONFLICT:
            break;
        case ORPHAN_NOTIF:
            break;
        case BEACON_REQ:
            break;
        case COOR_REALIGN:
            i.WriteU16(m_panId);
            WriteTo(i, m_coordShortAddr);
            i.WriteU8(m_channelNum);
            WriteTo(i, m_shortAddr);
            i.WriteU8(m_channelPage);
            break;

        case GTS_REQ:
            break;

        case DSME_ASSOCIATION_REQ:
            i = m_capabilityInfo.Serialize(i);
            i.WriteU8(m_hoppingSeqId);
            i.WriteU16(m_channelOfs);
            break;

        case DSME_ASSOCIATION_RESP:
            WriteTo(i, m_shortAddr);
            i.WriteU8(m_assocStatus);
            i.WriteU8(m_hoppingSeqLen);
            i = m_hoppingSeq.Serialize(i);

            /**
             * Add self-designed Enhanced Beacon Scheduling (EBS) feature here
             * Note :
             * Add a new field in association response command - Association sequence (2 bytes, use uint16_t)
             **/ 
            i.WriteU16(m_allocationSequence);
            break;

        case DSME_GTS_REQ:
            i = m_dsmeGTSManagement.Serialize(i);

            if (m_dsmeGTSManagement.GetManagementType() == 0b001) {
                i.WriteU8(m_numOfSlot);
                i.WriteU16(m_preferredSuperframeID);
                i.WriteU8(m_preferredSlotID);
            }

            i = m_dsmeSABSpec.Serialize(i);

            break;

        case DSME_GTS_REPLY:
            i = m_dsmeGTSManagement.Serialize(i);
            WriteTo(i, m_destAddr);
            i.WriteU16(m_channelOfs);
            i = m_dsmeSABSpec.Serialize(i);

            break;

        case DSME_GTS_NOTIFY:
            i = m_dsmeGTSManagement.Serialize(i);
            WriteTo(i, m_destAddr);
            i.WriteU16(m_channelOfs);
            i = m_dsmeSABSpec.Serialize(i);
            break;

        case DSME_INFO_REQ:
            i.WriteU8(m_infoType);
            i.WriteU8(m_dsmeSABSubBlkLen);
            i.WriteU16(m_dsmeSABSubBlkIdx);
            break;

        case DSME_INFO_REPLY:
            i.WriteU8(m_infoType);
            i.WriteU64(m_timestamp);
            i.WriteU16(m_superframeID);
            i.WriteU8(m_slotID);

            if (m_infoType == 0x01) {
                i = m_dsmeSABSpec.Serialize(i);
            } else if (m_infoType == 0x02) {
                m_dsmePANDercriptorIE.Serialize(i);
            }

            break;

        case DSME_BEACON_ALLOC_NOTIF:
            i.WriteU16(m_allocationBcnSDIndex);
            break;
            
        case DSME_BEACON_COLLISION_NOTIF:
            i.WriteU16(m_collisionSDIndex);
            break;
        case DSME_LINK_STATUS_REPORT:
            // i = m_dsmeLinkStatusSpecField.Serialize(i);
            break;

        case CMD_RESERVED:
            break;
    }
}

uint32_t
CommandPayloadHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_cmdFrameId = static_cast<MacCommand>(i.ReadU8());

    // TODO: add missing deserialize commands when other commands are added.
    switch (m_cmdFrameId) {
        case ASSOCIATION_REQ:
            i = m_capabilityInfo.Deserialize(i);
            break;

        case ASSOCIATION_RESP:
            ReadFrom(i, m_shortAddr);
            m_assocStatus = static_cast<AssocStatus>(i.ReadU8());
            break;

        case DISASSOCIATION_NOTIF:
            m_disassociationReason = i.ReadU8();
            break;

        case DATA_REQ:
            break;
        case PANID_CONFLICT:
            break;
        case ORPHAN_NOTIF:
            break;
        case BEACON_REQ:
            break;
        case COOR_REALIGN:
            m_panId = i.ReadU16();
            ReadFrom(i, m_coordShortAddr);
            m_channelNum = i.ReadU8();
            ReadFrom(i, m_shortAddr);
            m_channelPage = i.ReadU8();
            break;

        case GTS_REQ:
            break;
        case DSME_ASSOCIATION_REQ:
            i = m_capabilityInfo.Deserialize(i);
            m_hoppingSeqId = i.ReadU8();
            m_channelOfs = i.ReadU16();
            break;

        case DSME_ASSOCIATION_RESP:
            ReadFrom(i, m_shortAddr);
            m_assocStatus = static_cast<AssocStatus>(i.ReadU8());
            m_hoppingSeqLen = i.ReadU8();
            i = m_hoppingSeq.Deserialize(i);
            /** 
             * Add self-designed Enhanced Beacon Scheduling (EBS) feature here
             * Note :
             * Add a new field in association response command - Association sequence (2 bytes, use uint16_t)
             **/ 
            m_allocationSequence = i.ReadU16();
            break;

        case DSME_GTS_REQ:
            i = m_dsmeGTSManagement.Deserialize(i);

            if (m_dsmeGTSManagement.GetManagementType() == 0b001) {
                m_numOfSlot = i.ReadU8();
                m_preferredSuperframeID = i.ReadU16();
                m_preferredSlotID = i.ReadU8();
            }
            
            i = m_dsmeSABSpec.Deserialize(i);

            break;

        case DSME_GTS_REPLY:
            i = m_dsmeGTSManagement.Deserialize(i);
            ReadFrom(i, m_destAddr);
            m_channelOfs = i.ReadU16();
            i = m_dsmeSABSpec.Deserialize(i);
            
            break;

        case DSME_GTS_NOTIFY:
            i = m_dsmeGTSManagement.Deserialize(i);
            ReadFrom(i, m_destAddr);
            m_channelOfs = i.ReadU16();
            i = m_dsmeSABSpec.Deserialize(i);
            break;

        case DSME_INFO_REQ:
            m_infoType = i.ReadU8();
            m_dsmeSABSubBlkLen = i.ReadU8();
            m_dsmeSABSubBlkIdx = i.ReadU8();
            break;

        case DSME_INFO_REPLY:
            m_infoType = i.ReadU8();
            m_timestamp = i.ReadU64();
            m_superframeID = i.ReadU16();
            m_slotID = i.ReadU8();
            
            if (m_infoType == 0x01) {
                i = m_dsmeSABSpec.Deserialize(i);
            } else if (m_infoType == 0x02) {
                m_dsmePANDercriptorIE.Deserialize(i);
            }

            break;

        case DSME_BEACON_ALLOC_NOTIF:
            m_allocationBcnSDIndex = i.ReadU16();
            break;

        case DSME_BEACON_COLLISION_NOTIF:
            m_collisionSDIndex = i.ReadU16();
            break;

        case DSME_LINK_STATUS_REPORT:
            // i = m_dsmeLinkStatusSpecField.Deserialize(i);
            break;

        case CMD_RESERVED:
            break;
    }

    return i.GetDistanceFrom(start);
}

void CommandPayloadHeader::Print(std::ostream& os) const {
    os << "| MAC Command Frame ID | = " << std::hex <<(uint32_t)m_cmdFrameId << std::endl;

    switch (m_cmdFrameId) {
        case ASSOCIATION_REQ:
            os  << "| Device Type FFD | = " << m_capabilityInfo.IsDeviceTypeFfd()
                << "| Alternative Power Source available | = " << m_capabilityInfo.IsPowSrcAvailable()
                << "| Receiver on when Idle | = " << m_capabilityInfo.IsReceiverOnWhenIdle()
                << "| Security capable | = " << m_capabilityInfo.IsSecurityCapability()
                << "| Allocate address on | = " << m_capabilityInfo.IsShortAddrAllocOn();
            break;

        case ASSOCIATION_RESP:
            os << "| Assigned Short Address | = " << m_shortAddr
                << "| Status Response | = " << m_assocStatus;
            break;
            
        case DISASSOCIATION_NOTIF:
            os << "| Disassociation Reason | = " << std::hex << m_disassociationReason;
            break;

        case DATA_REQ:
            break;
        case PANID_CONFLICT:
            break;
        case ORPHAN_NOTIF:
            break;
        case BEACON_REQ:
            break;
        case COOR_REALIGN:
            break;
        case GTS_REQ:
            break;
        case DSME_ASSOCIATION_REQ:
            break;

        case DSME_ASSOCIATION_RESP:
            os  << "| Assigned Short Address | = " << m_shortAddr
                << "| Status Response | = " << std::hex << m_assocStatus
                << "| Hopping Seq Length | = " << (uint32_t)m_hoppingSeqLen;

                // DSME-TODO
                // <<  "| Hopping Seq | = " << m_hoppingSeq;
            break;

        case DSME_GTS_REQ:
            // DSME-TODO
            os  << " Manage Type : " << (uint16_t) m_dsmeGTSManagement.GetManagementType() << " | "
                << " Direction : " << m_dsmeGTSManagement.IsDirectionRX() << " | "
                << " Priority : " << (uint16_t) m_dsmeGTSManagement.IsHighPriority() << " | "
                << " Num Of Slots : " << (uint16_t) m_numOfSlot << " | "
                << " Preferred Superframe ID : " << m_preferredSuperframeID << " | "
                << " Preferred Slot ID : " << (uint16_t) m_preferredSlotID << " | \n"
                << " SAB Spec: " 
                << "\n";
            
            m_dsmeSABSpec.Print(os);

            break;

        case DSME_GTS_REPLY:
            // DSME-TODO
            os  << " Manage Type : " << (uint16_t) m_dsmeGTSManagement.GetManagementType() << " | "
                << " Direction : " << m_dsmeGTSManagement.IsDirectionRX() << " | "
                << " Priority : " << (uint16_t) m_dsmeGTSManagement.IsHighPriority() << " | "
                << " Status : " << (uint16_t) m_dsmeGTSManagement.GetStatus() << " | "
                << " Num Of Slots : " << (uint16_t) m_numOfSlot << " | "
                << " Preferred Superframe ID : " << m_preferredSuperframeID << " | "
                << " Preferred Slot ID : " << (uint16_t) m_preferredSlotID << " | \n"
                << " SAB Spec: " 
                << "\n";
            
            m_dsmeSABSpec.Print(os);

            break;

        case DSME_GTS_NOTIFY:
            // DSME-TODO
            break;

        case DSME_INFO_REQ:
            // DSME-TODO
            break;

        case DSME_INFO_REPLY:
            // DSME-TODO
            break;

        case DSME_BEACON_ALLOC_NOTIF:
            // DSME-TODO
            break;

        case DSME_BEACON_COLLISION_NOTIF:
            // DSME-TODO
            break;

        case DSME_LINK_STATUS_REPORT:
            // DSME-TODO
            break;

        case CMD_RESERVED:
            break;
    }

    os << std::endl;
}

void
CommandPayloadHeader::SetCommandFrameType(MacCommand macCommand)
{
    m_cmdFrameId = macCommand;
}

void
CommandPayloadHeader::SetCapabilityField(CapabilityField cap)
{
    NS_ASSERT(m_cmdFrameId == ASSOCIATION_REQ || m_cmdFrameId == DSME_ASSOCIATION_REQ);
    m_capabilityInfo = cap;
}

void CommandPayloadHeader::SetDisassociationReason(CommandPayloadHeader::DisassociationReason reason) {
    NS_ASSERT(m_cmdFrameId == DISASSOCIATION_NOTIF);
    m_disassociationReason = reason;
}

CommandPayloadHeader::DisassociationReason CommandPayloadHeader::GetDisassociationReason() const {
    NS_ASSERT(m_cmdFrameId == DISASSOCIATION_NOTIF);
    return static_cast<DisassociationReason>(m_disassociationReason);
}

CommandPayloadHeader::MacCommand
CommandPayloadHeader::GetCommandFrameType() const
{
    switch (m_cmdFrameId)
    {
    case 0x01:
        return ASSOCIATION_REQ;
        break;
    case 0x02:
        return ASSOCIATION_RESP;
        break;
    case 0x03:
        return DISASSOCIATION_NOTIF;
        break;
    case 0x04:
        return DATA_REQ;
        break;
    case 0x05:
        return PANID_CONFLICT;
        break;
    case 0x06:
        return ORPHAN_NOTIF;
        break;
    case 0x07:
        return BEACON_REQ;
        break;
    case 0x08:
        return COOR_REALIGN;
        break;
    case 0x09:
        return GTS_REQ;
        break;
    case 0x13:
        return DSME_ASSOCIATION_REQ;
        break;
    case 0x14:
        return DSME_ASSOCIATION_RESP;
        break;
    case 0x15:
        return DSME_GTS_REQ;
        break;
    case 0x16:
        return DSME_GTS_REPLY;
        break;
    case 0x17:
        return DSME_GTS_NOTIFY;
        break;
    case 0x18:
        return DSME_INFO_REQ;
        break;
    case 0x19:
        return DSME_INFO_REPLY;
        break;
    case 0x1a:
        return DSME_BEACON_ALLOC_NOTIF;
        break;
    case 0x1b:
        return DSME_BEACON_COLLISION_NOTIF;
        break;
    case 0x1c:
        return DSME_LINK_STATUS_REPORT;
        break;
    default:
        return CMD_RESERVED;
    }
}

void CommandPayloadHeader::SetShortAddr(Mac16Address shortAddr) {
    NS_ASSERT(m_cmdFrameId == ASSOCIATION_RESP || m_cmdFrameId == DSME_ASSOCIATION_RESP
              || m_cmdFrameId == COOR_REALIGN);
    m_shortAddr = shortAddr;
}

void
CommandPayloadHeader::SetAssociationStatus(AssocStatus status)
{
    NS_ASSERT(m_cmdFrameId == ASSOCIATION_RESP || m_cmdFrameId == DSME_ASSOCIATION_RESP);
    m_assocStatus = status;
}

void CommandPayloadHeader::SetPanId(uint16_t id) {
    m_panId = id;
}

void CommandPayloadHeader::SetCoordinatorShortAddress(Mac16Address addr) {
    m_coordShortAddr = addr;
}

void CommandPayloadHeader::SetChannelNum(uint16_t num) {
    m_channelNum = num;
}

void CommandPayloadHeader::SetChannelPage(uint8_t page) {
    m_channelPage = page;
}

void CommandPayloadHeader::SetHoppingSeqID(uint8_t seqId) {
    m_hoppingSeqId = seqId;
}

void CommandPayloadHeader::SetChannelOfs(uint16_t ofs) {
    m_channelOfs = ofs;
}

void CommandPayloadHeader::SetHoppingSeqLen(uint8_t len) {
    m_hoppingSeqLen = len;
}

void CommandPayloadHeader::SetHoppingSequnce(HoppingSequence seq) {
    m_hoppingSeq = seq;
}

void CommandPayloadHeader::SetDsmeGtsManagementField(DSMEGTSManagementField field) {
    m_dsmeGTSManagement = field;
}

void CommandPayloadHeader::SetDsmeGtsNumOfSlot(uint8_t num) {
    m_numOfSlot = num;
}

void CommandPayloadHeader::SetDsmeGtsPreferredSuperframeId(uint16_t superframeId) {
    m_preferredSuperframeID = superframeId;
}

void CommandPayloadHeader::SetDsmeGtsPreferredSlotId(uint8_t slotId) {
    m_preferredSlotID = slotId;
}

void CommandPayloadHeader::SetDsmeGtsSABSpec(DSMESABSpecificationField spec) {
    m_dsmeSABSpec = spec;
}

void CommandPayloadHeader::SetDsmeGtsDestAddress(Mac16Address addr) {
    m_destAddr = addr;
}

void CommandPayloadHeader::SetAllocationBcnSDIndex(uint16_t idx) {
    m_allocationBcnSDIndex = idx;
}

void CommandPayloadHeader::SetCollisionBcnSDIndex(uint16_t idx) {
    m_collisionSDIndex = idx;
}

void CommandPayloadHeader::SetDsmeInfoType(uint8_t type) {
    m_infoType = type;
}

void CommandPayloadHeader::SetDsmeInfoSABSubBlkLen(uint8_t len) {
    m_dsmeSABSubBlkLen = len;
}

void CommandPayloadHeader::SetDsmeInfoSABSubBlkIdx(uint16_t idx) {
    m_dsmeSABSubBlkIdx = idx;
}

void CommandPayloadHeader::SetDsmeInfoTimestamp(uint64_t timestamp) {
    m_timestamp = timestamp;
}

void CommandPayloadHeader::SetDsmeInfoSuperframeID(uint16_t id) {
    m_superframeID = id;
}

void CommandPayloadHeader::SetDsmeInfoSlotID(uint8_t id) {
    m_slotID = id;
}

void CommandPayloadHeader::SetDsmeInfoPANDescriptor(DsmePANDescriptorIE des) {
    m_dsmePANDercriptorIE = des;
}

void CommandPayloadHeader::SetEBSAllocationSeq(uint16_t allocSeq)
{
    m_allocationSequence = allocSeq;
}

uint16_t CommandPayloadHeader::GetEBSAllocationSeq()
{
    return m_allocationSequence;
}


Mac16Address
CommandPayloadHeader::GetShortAddr() const
{
    NS_ASSERT(m_cmdFrameId == ASSOCIATION_RESP || m_cmdFrameId == DSME_ASSOCIATION_RESP 
                || m_cmdFrameId == COOR_REALIGN);
    return m_shortAddr;
}

CommandPayloadHeader::AssocStatus
CommandPayloadHeader::GetAssociationStatus() const
{
    NS_ASSERT(m_cmdFrameId == ASSOCIATION_RESP || m_cmdFrameId == DSME_ASSOCIATION_RESP);
    return m_assocStatus;
}

CapabilityField
CommandPayloadHeader::GetCapabilityField() const
{
    NS_ASSERT(m_cmdFrameId == ASSOCIATION_REQ || m_cmdFrameId == DSME_ASSOCIATION_REQ);
    return m_capabilityInfo;
}

uint16_t CommandPayloadHeader::GetPanId() const {
    return m_panId;
}

Mac16Address CommandPayloadHeader::GetCoordinatorShortAddress() const {
    return m_coordShortAddr;
}

uint16_t CommandPayloadHeader::GetChannelNum() const {
    return m_channelNum;
}

uint8_t CommandPayloadHeader::GetChannelPage() const {
    return m_channelPage;
}

uint8_t CommandPayloadHeader::GetHoppingSeqLen() {
    return m_hoppingSeqLen;
}

HoppingSequence CommandPayloadHeader::GetHoppingSequnce() const {
    return m_hoppingSeq;
}

uint16_t CommandPayloadHeader::GetChannelOfs() {
    return m_channelOfs;
}

uint8_t CommandPayloadHeader::GetHoppingSeqID() {
    return m_hoppingSeqId;
}

DSMEGTSManagementField CommandPayloadHeader::GetDsmeGtsManagementField() const {
    return m_dsmeGTSManagement;
}

uint8_t CommandPayloadHeader::GetDsmeGtsNumOfSlot() const {
    return m_numOfSlot;
}

uint16_t CommandPayloadHeader::GetDsmeGtsPreferredSuperframeId() const {
    return m_preferredSuperframeID;
}

uint8_t CommandPayloadHeader::GetDsmeGtsPreferredSlotId() const {
    return m_preferredSlotID;
}

DSMESABSpecificationField CommandPayloadHeader::GetDsmeGtsSABSpec() const {
    return m_dsmeSABSpec;
}

Mac16Address CommandPayloadHeader::GetDsmeGtsDestAddress() const {
    return m_destAddr;
}

uint16_t CommandPayloadHeader::GetAllocationBcnSDIndex() const {
    return m_allocationBcnSDIndex;
}

uint16_t CommandPayloadHeader::GetCollisionBcnSDIndex() const
{
    return m_collisionSDIndex;
}

uint8_t CommandPayloadHeader::GetDsmeInfoType() const {
    return m_infoType;
}

uint8_t CommandPayloadHeader::GetDsmeInfoSABSubBlkLen() const {
    return m_dsmeSABSubBlkLen;
}

uint16_t CommandPayloadHeader::GetDsmeInfoSABSubBlkIdx() const {
    return m_dsmeSABSubBlkIdx;
}

uint64_t CommandPayloadHeader::GetDsmeInfoTimestamp() const {
    return m_timestamp;
}

uint16_t CommandPayloadHeader::GetDsmeInfoSuperframeID() const {
    return m_superframeID;
}

uint8_t CommandPayloadHeader::GetDsmeInfoSlotID() const {
    return m_slotID;
}



DsmePANDescriptorIE CommandPayloadHeader::GetDsmeInfoPANDescriptor() const {
    return m_dsmePANDercriptorIE;
}

/***********************************************************
 *                    Hopping Sequence
 ***********************************************************/
void HoppingSequence::SetSequenceLength(uint8_t len) {
    m_channelList.resize(len);
}

uint16_t HoppingSequence::GetChannel(int idx) {
    return m_channelList[idx];
}

uint32_t HoppingSequence::GetSerializedSize() const {
    return 2 * m_channelList.size();   // Octets, channel Range * hopping list len
}

Buffer::Iterator HoppingSequence::Serialize(Buffer::Iterator i) const {
    for (unsigned int j = 0; j < m_channelList.size(); j++) {
        i.WriteU16(m_channelList[j]);
    }

    return i;
}

Buffer::Iterator HoppingSequence::Deserialize(Buffer::Iterator i) {
    for (unsigned int j = 0; j < m_channelList.size(); j++) {
        m_channelList[j] = i.ReadU16();
    }

    return i;
}











/***********************************************************
 *                  DSME Management Field
 ***********************************************************/
void DSMEGTSManagementField::SetManagementType(uint8_t type) {
    m_managementType = type;
}

void DSMEGTSManagementField::SetRX(bool RX) {
    m_direction = RX;
}

void DSMEGTSManagementField::SetPrioritizedChannelAccess(bool high) {
    m_prior_channel_access = high;
}

void DSMEGTSManagementField::SetStatus(uint8_t status) {
    m_status = status;
}

uint8_t DSMEGTSManagementField::GetDSMEGTSManagementField() const {
    uint8_t managementField = 0;

    managementField = (m_managementType);                              //!< Bit 0-2
    managementField |= (m_direction << 3) & (0x01 << 3);               //!< Bit 3
    managementField |= (m_prior_channel_access << 4) & (0x01 << 4);    //!< Bit 4
    managementField |= (m_status << 5) & (0x03 << 5);                  //!< Bit 5-7

    return managementField;
}

uint8_t DSMEGTSManagementField::GetManagementType() const {
    return m_managementType;
}

uint8_t DSMEGTSManagementField::GetStatus() const {
    return m_status;
}

bool DSMEGTSManagementField::IsDirectionRX() const {
    return m_direction;
}

bool DSMEGTSManagementField::IsHighPriority() const {
    return m_prior_channel_access;
}

uint32_t DSMEGTSManagementField::GetSerializedSize() const {
    return 1;
}

Buffer::Iterator DSMEGTSManagementField::Serialize(Buffer::Iterator i) const {
    // uint8_t managementField;

    // managementField = 0;
    // managementField = (m_managementType);                              //!< Bit 0-2
    // managementField |= (m_direction << 3) & (0x01 << 3);               //!< Bit 3
    // managementField |= (m_prior_channel_access << 4) & (0x01 << 4);    //!< Bit 4
    //                                                                    //!< Bit 5-7

    i.WriteU8(GetDSMEGTSManagementField());
    
    // std::cout << std::bitset<8>(GetDSMEGTSManagementField()) << std::endl; // debug

    return i;
}

Buffer::Iterator DSMEGTSManagementField::Deserialize(Buffer::Iterator i) {
    uint8_t managementField = i.ReadU8();

    m_managementType = (managementField) & (0x07);                     //!< Bit 0-2
    m_direction = (managementField >> 3) & (0x01);                //!< Bit 3
    m_prior_channel_access = (managementField >> 4) & (0x01);     //!< Bit 4
    m_status = (managementField >> 5) & (0x03);                   //!< Bit 5-7

    // std::cout << std::bitset<8>(GetDSMEGTSManagementField()) << std::endl; // debug

    return i;
}

/***********************************************************
 *              DSME SAB Specification Field
 ***********************************************************/
void DSMESABSpecificationField::setCAPReduction(bool on) {
    m_capReduction = on;
}

void DSMESABSpecificationField::setSABSubBlkLen(uint8_t len) {
    m_dsmeSABSubBlkLen = len;
}

void DSMESABSpecificationField::setSABSubBlkIdx(uint16_t idx) {
    m_dsmeSABSubBlkIdx = idx;
}

void DSMESABSpecificationField::setSABSubBlk(uint8_t subBlk) {
    m_dsmeSABSubBlkCapOff.push_back(subBlk);
}

void DSMESABSpecificationField::setSABSubBlk(uint16_t subBlk) {
    m_dsmeSABSubBlk.push_back(subBlk);
}

void DSMESABSpecificationField::setSABSubBlk(std::vector<uint8_t> subBlk) {
    m_dsmeSABSubBlkCapOff = subBlk;
}

void DSMESABSpecificationField::setSABSubBlk(std::vector<uint16_t> subBlk) {
    m_dsmeSABSubBlk = subBlk;
}

bool DSMESABSpecificationField::isCAPReduction() const {
    return m_capReduction;
}

uint8_t DSMESABSpecificationField::GetSABSubBlkLen() const {
    return m_dsmeSABSubBlkLen;
}

uint16_t DSMESABSpecificationField::GetSABSubBlkIdx() const {
    return m_dsmeSABSubBlkIdx;
}

const std::vector<uint16_t> DSMESABSpecificationField::GetSABSubBlk() const {
    return m_dsmeSABSubBlk;
}

const std::vector<uint8_t> DSMESABSpecificationField::GetSABSubBlkCapOff() const {
    return m_dsmeSABSubBlkCapOff;
}

uint32_t DSMESABSpecificationField::GetSerializedSize() const {
    if (isCAPReduction()) {
        return 4 + m_dsmeSABSubBlkLen * 2;
    } else {
        return 4 + m_dsmeSABSubBlkLen;
    }

    return 4;
}

Buffer::Iterator DSMESABSpecificationField::Serialize(Buffer::Iterator i) const {
    i.WriteU8(m_capReduction);
    i.WriteU8(m_dsmeSABSubBlkLen);
    i.WriteU16(m_dsmeSABSubBlkIdx);

    if (isCAPReduction()) {
        for (unsigned int j = 0; j < GetSABSubBlkLen(); j++) {
            i.WriteU16(m_dsmeSABSubBlk[j]);
        }

    } else {
        for (unsigned int j = 0; j < GetSABSubBlkLen(); j++) {
            i.WriteU8(m_dsmeSABSubBlkCapOff[j]);
        }
    }

    return i;
}

Buffer::Iterator DSMESABSpecificationField::Deserialize(Buffer::Iterator i) {
    setCAPReduction(i.ReadU8());
    setSABSubBlkLen(i.ReadU8());
    setSABSubBlkIdx(i.ReadU16());

    for (int j = 0; j < GetSABSubBlkLen(); j++) {
        if (isCAPReduction()) {
            m_dsmeSABSubBlk.push_back(i.ReadU16());

        } else {
            m_dsmeSABSubBlkCapOff.push_back(i.ReadU8());
        }            
    }

    return i;
}

void DSMESABSpecificationField::Print(std::ostream& os) const {
    os << " CAP Reduction : " << m_capReduction << " | "
       << " SAB Sub-Block Index : " << m_dsmeSABSubBlkIdx << " | "
       << " SAB Sub-Block Length : " << (uint16_t) m_dsmeSABSubBlkLen << " | "
       << " DSME SAB sub block : ";
    
    for (int i = 0; i < GetSABSubBlkLen(); ++i) {
        if (isCAPReduction()) {
            os << std::bitset<16>(m_dsmeSABSubBlk[i])
               << std::endl;
        } else {
            os << std::bitset<8>(m_dsmeSABSubBlkCapOff[i])
               << std::endl;
        }
    }
}

/***********************************************************
 *             Link Status Specification Field
 ***********************************************************/
void LinkStatusSpecificationField::SetLinkStatusDescriptorCnt(uint8_t count) {
    m_linkStatusDescriptorCnt = count;
    m_linkStatusList.resize(count);
}

uint8_t LinkStatusSpecificationField::GetLinkStatusDescriptorCnt() const {
    return m_linkStatusDescriptorCnt;
}

const std::vector<LinkStatusDescriptor> LinkStatusSpecificationField::GetLinkStatusList() const {
    return m_linkStatusList;
}

uint32_t LinkStatusSpecificationField::GetSerializedSize() const {
    return m_linkStatusDescriptorCnt * 3;  // count * LinkStatusDescriptor
}

Buffer::Iterator LinkStatusSpecificationField::Serialize(Buffer::Iterator i) const {
    i.WriteU8(m_linkStatusDescriptorCnt);

    for (int j = 0; j < m_linkStatusDescriptorCnt; ++j) {
        i.WriteU8(m_linkStatusList[j].channel);
        i.WriteU8(m_linkStatusList[j].avgLQI);
        i.WriteU8(m_linkStatusList[j].avgRSSI);
    }

    return i;
}

Buffer::Iterator LinkStatusSpecificationField::Deserialize(Buffer::Iterator i) {
    m_linkStatusDescriptorCnt = i.ReadU8();

    for (int j = 0; j < m_linkStatusDescriptorCnt; ++j) {
        m_linkStatusList[j].channel = i.ReadU8();
        m_linkStatusList[j].avgLQI = i.ReadU8();
        m_linkStatusList[j].avgRSSI = i.ReadU8();
    }

    return i;
}

/***********************************************************
 *                       Descriptor
 ***********************************************************/

HeaderIEDescriptor::HeaderIEDescriptor() {

}

HeaderIEDescriptor::HeaderIEDescriptor(uint8_t len, HeaderElementIDs id, bool type) {    
    m_length = len;
    m_elementId = id;
    m_type = type;

    // Print(std::cout); // debug
}

HeaderIEDescriptor::~HeaderIEDescriptor() {

}

void HeaderIEDescriptor::SetLength(uint8_t len) {
    m_length = len;
}

void HeaderIEDescriptor::SetHeaderElementID(HeaderElementIDs id) {
    m_elementId = id;
}

const uint8_t HeaderIEDescriptor::GetLength() const {
    return m_length;
}

const HeaderElementIDs HeaderIEDescriptor::GetHeaderElementID() const {
    return m_elementId;
}

const bool HeaderIEDescriptor::GetType() const {
    return m_type;
}

uint16_t HeaderIEDescriptor::GetDescriptor() const {
    uint16_t descriptor = 0;

    descriptor = m_length & (0x7f);                                         // Bits 0-6
    descriptor |= (static_cast<uint8_t>(m_elementId) << 7) & (0xff << 7);   // Bits 7-14
    descriptor |= (m_type << 15) & (0x01 << 15);                            // Bit 15

    return descriptor;
}

void HeaderIEDescriptor::SetDescriptor(uint16_t descriptor) {
    m_length = (descriptor) & (0x7f);                   // Bits 0-6
    m_elementId = static_cast<HeaderElementIDs>((descriptor >> 7) & (0xff));           // Bits 7-14
    m_type = (descriptor >> 15) & (0x01);               // Bit 15
}

uint32_t HeaderIEDescriptor::GetSerializedSize() const {
    return 2;
}

Buffer::Iterator HeaderIEDescriptor::Serialize(Buffer::Iterator i) const {
    i.WriteHtolsbU16(GetDescriptor());

    return i;
}

Buffer::Iterator HeaderIEDescriptor::Deserialize(Buffer::Iterator i) {
    uint16_t descriptor = i.ReadLsbtohU16();
    SetDescriptor(descriptor);

    return i;
}

void HeaderIEDescriptor::Print(std::ostream& os) const {
    os  << "| Length Field | = " << (uint32_t)m_length << std::endl
        << "| Element ID Field | = " << std::hex << m_elementId << std::endl
        << "| Type Field| = " << (uint32_t)m_type << std::endl;
}

/***********************************************************
 *                DSME PAN Descriptor IE
 ***********************************************************/
DsmePANDescriptorIE::DsmePANDescriptorIE() {
    m_descriptor.SetHeaderElementID(HEADERIE_DSME_PAN_DESCRIPTOR);
    m_descriptor.SetLength(0);
}

DsmePANDescriptorIE::DsmePANDescriptorIE(uint8_t len, HeaderElementIDs elementID) {
    SetHeaderIEDescriptor(len, elementID);
}

NS_OBJECT_ENSURE_REGISTERED(DsmePANDescriptorIE);

DsmePANDescriptorIE::~DsmePANDescriptorIE() {

}

void DsmePANDescriptorIE::SetHeaderIEDescriptor(uint8_t len, HeaderElementIDs elementID) {
    m_descriptor.SetLength(len);
    m_descriptor.SetHeaderElementID(elementID);
}

void DsmePANDescriptorIE::SetSuperframeField(SuperframeField& superframeField) {
    m_superframeField = superframeField;
}  
  
void DsmePANDescriptorIE::SetSuperframeField(uint8_t bcnOrder, uint8_t frmOrder
                    , uint8_t capSlot, bool battLifeExt, bool panCoor, bool assocPermit) {
    m_superframeField.SetBeaconOrder(bcnOrder);
    m_superframeField.SetSuperframeOrder(frmOrder);
    m_superframeField.SetFinalCapSlot(capSlot);
    m_superframeField.SetBattLifeExt(battLifeExt);
    m_superframeField.SetPanCoor(panCoor);
    m_superframeField.SetAssocPermit(assocPermit);
}    

void DsmePANDescriptorIE::SetPendingAddrFields(uint8_t pndAddrSpecField) {
    m_pndAddrFields.SetPndAddrSpecField(pndAddrSpecField);
}    

void DsmePANDescriptorIE::SetPendingAddrFields(PendingAddrFields& pndAddrSpecField) {
    m_pndAddrFields = pndAddrSpecField;
}

void DsmePANDescriptorIE::SetDsmeSuperFrameField(uint16_t dsmeSuperFrm) {
    m_dsmeSuperframeField.SetDsmeSuperframe(dsmeSuperFrm);
}    

void DsmePANDescriptorIE::SetDsmeSuperFrameField(uint8_t multiBcnOrder, bool channelDiversityMode
                            , bool gACKFlag, bool cAPReduction, bool deferredBcnFlag) {
    m_dsmeSuperframeField.SetMultiSuperframeOrder(multiBcnOrder);
    m_dsmeSuperframeField.SetChannelDiversityMode(channelDiversityMode);
    m_dsmeSuperframeField.SetGACKFlag(gACKFlag);
    m_dsmeSuperframeField.SetCAPReductionFlag(cAPReduction);     
    m_dsmeSuperframeField.SetDeferredBeaconFalg(deferredBcnFlag);                     
}  

void DsmePANDescriptorIE::SetDsmeSuperFrameField(DsmeSuperFrameField& dsmeSuperFrm) {
    m_dsmeSuperframeField = dsmeSuperFrm;
}    

void DsmePANDescriptorIE::SetTimeSync(uint64_t bcnTimestamp, uint16_t bcnOfsTimeStamp) {
    m_timeSynField.SetBeaconTimeStamp(bcnTimestamp);
    m_timeSynField.SetBeaconOffsetTimeStamp(bcnOfsTimeStamp);
}   

void DsmePANDescriptorIE::SetTimeSync(TimeSync& timeSynField) {
    m_timeSynField = timeSynField;
}   

void DsmePANDescriptorIE::SetBeaconBitmap(uint16_t sdIndex, uint16_t bitmapLen) {
    m_beaconBitmapField.SetSDIndex(sdIndex);
    m_beaconBitmapField.SetBitmapLength(bitmapLen);
    m_beaconBitmapField.ResetSDBitmap();
}    

void DsmePANDescriptorIE::SetBeaconBitmap(BeaconBitmap& beaconBitmapField) {
    m_beaconBitmapField = beaconBitmapField;
}    

void DsmePANDescriptorIE::SetChannelHopping(ChannelHopping& channelHoppingField) {
    m_channelHoppingField = channelHoppingField;
}    

void DsmePANDescriptorIE::SetChannelHopping(uint8_t seqID, uint8_t bcnSeqNum, uint16_t ofs
                        , uint8_t bitmapLen) {
    m_channelHoppingField.SetHoppingSequenceID(seqID);            
    m_channelHoppingField.SetPANCoordinatorBSN(bcnSeqNum);            
    m_channelHoppingField.SetChannelOffset(ofs);            
    m_channelHoppingField.SetChannelOffsetBitmapLength(bitmapLen);              
    m_channelHoppingField.ResetChannelOffsetBitmap();
}

void DsmePANDescriptorIE::SetGroupACK(GroupACK& gack) {
    m_gack = gack;
}    

void DsmePANDescriptorIE::SetGroupACK(uint16_t gack1superFrmID, uint8_t gack1SlotID
                , uint8_t gack1ChannelID, uint16_t gack2superFrmID
                , uint16_t gack2SlotID, uint8_t gack2ChannelID) {
    m_gack.SetGACK1SuperframeID(gack1superFrmID);
    m_gack.SetGACK1SlotID(gack1SlotID);
    m_gack.SetGACK1ChannelID(gack1ChannelID);

    m_gack.SetGACK2SuperframeID(gack2superFrmID);
    m_gack.SetGACK2SlotID(gack2SlotID);
    m_gack.SetGACK2ChannelID(gack2ChannelID);
}   

const HeaderIEDescriptor DsmePANDescriptorIE::GetHeaderIEDescriptor() const {
    return m_descriptor;
}

const SuperframeField DsmePANDescriptorIE::GetSuperframeField() const {
    return m_superframeField;
}    

const PendingAddrFields DsmePANDescriptorIE::GetPendingAddrFields() const {
    return m_pndAddrFields;
}

const DsmeSuperFrameField  DsmePANDescriptorIE::GetDsmeSuperFrameField() const {
    return m_dsmeSuperframeField;
}

const TimeSync DsmePANDescriptorIE::GetTimeSync() const {
    return m_timeSynField;
}

const BeaconBitmap DsmePANDescriptorIE::GetBeaconBitmap() const {
    return m_beaconBitmapField;
}

const ChannelHopping DsmePANDescriptorIE::GetChannelHopping() const {
    return m_channelHoppingField;
}

const GroupACK DsmePANDescriptorIE::GetGroupACK() const {
    return m_gack;
}    

TypeId DsmePANDescriptorIE::GetTypeId() {
    static TypeId tid = TypeId("ns3::DsmePANDescriptorIE")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<DsmePANDescriptorIE>();
    return tid;
}

TypeId DsmePANDescriptorIE::GetInstanceTypeId() const {
    return GetTypeId();
}

uint32_t DsmePANDescriptorIE::GetSerializedSize() const {
    uint32_t size = 0;

    size += m_descriptor.GetSerializedSize();
    size += m_superframeField.GetSerializedSize();      // Superframe Field
    size += m_pndAddrFields.GetSerializedSize();        // Pending Address Fields
    size += m_dsmeSuperframeField.GetSerializedSize();
    size += m_timeSynField.GetSerializedSize();
    size += m_beaconBitmapField.GetSerializedSize();
    size += m_channelHoppingField.GetSerializedSize();
    size += m_gack.GetSerializedSize();

    return size;
}

void DsmePANDescriptorIE::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;
    i = m_descriptor.Serialize(i);
    i = m_superframeField.Serialize(i);
    i = m_pndAddrFields.Serialize(i);
    i = m_dsmeSuperframeField.Serialize(i);
    i = m_timeSynField.Serialize(i);
    i = m_beaconBitmapField.Serialize(i);
    i = m_channelHoppingField.Serialize(i);
    i = m_gack.Serialize(i);
}

uint32_t DsmePANDescriptorIE::Deserialize(Buffer::Iterator start) {
    // std::cout << "DsmePANDescriptorIE::Deserialize Start" << std::endl;

    Buffer::Iterator i = start;
    i = m_descriptor.Deserialize(i);
    i = m_superframeField.Deserialize(i);
    i = m_pndAddrFields.Deserialize(i);
    i = m_dsmeSuperframeField.Deserialize(i);
    i = m_timeSynField.Deserialize(i);
    i = m_beaconBitmapField.Deserialize(i);
    i = m_channelHoppingField.Deserialize(i);
    i = m_gack.Deserialize(i);

    // std::cout << "DsmePANDescriptorIE::Deserialize End" << std::endl;

    return i.GetDistanceFrom(start);
}

void DsmePANDescriptorIE::Print(std::ostream &os) const {
    os << "| Superframe Spec Field | = " << m_superframeField << std::endl
       << "| Pending Address Field | = " << m_pndAddrFields << std::endl
       << "| DSEM Superframe Field | = " << m_dsmeSuperframeField << std::endl
       << "| Time Sync Field | = " << m_timeSynField << std::endl 
       << "| Beacon Bitmap Field | = " << m_beaconBitmapField << std::endl
       << "| Channel Hopping Field | = " << m_channelHoppingField << std::endl;
    //    << "| Group ACK Field | = " << m_gack;
}

/***********************************************************
 *                   Header IE Termination
 ***********************************************************/

HeaderIETermination::HeaderIETermination() {
    m_descriptor.SetLength(0);
    m_descriptor.SetHeaderElementID(HEADERIE_LIST_TERMINATION_1);
}

HeaderIETermination::~HeaderIETermination() {

}

NS_OBJECT_ENSURE_REGISTERED(HeaderIETermination);

TypeId HeaderIETermination::GetTypeId() {
    static TypeId tid = TypeId("ns3::HeaderIETermination")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<HeaderIETermination>();
    return tid;
}

TypeId HeaderIETermination::GetInstanceTypeId() const {
    return GetTypeId();
}

uint32_t HeaderIETermination::GetSerializedSize() const {
    return 2;
}

void HeaderIETermination::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;

    i = m_descriptor.Serialize(i);
}

uint32_t HeaderIETermination::Deserialize(Buffer::Iterator start) {
    Buffer::Iterator i = start;

    i = m_descriptor.Deserialize(i);

    return i.GetDistanceFrom(start);
}

void HeaderIETermination::Print(std::ostream& os) const {
    m_descriptor.Print(os);
}

/***********************************************************
 *                 Payload IE Descriptor
 ***********************************************************/
PayloadIEDescriptor::PayloadIEDescriptor() {

}

PayloadIEDescriptor::PayloadIEDescriptor(uint8_t len, PayloadIEGroupIDs id, bool type) {
    m_length = len;
    m_elementId = id;
    m_type = type;
}
 
PayloadIEDescriptor::~PayloadIEDescriptor() {
}

void PayloadIEDescriptor::SetLength(uint16_t len) {
    m_length = len;
}

void PayloadIEDescriptor::SetPayloadIEGroupID(PayloadIEGroupIDs id) {
    m_elementId = id;
}

const uint16_t PayloadIEDescriptor::GetLength() const {
    return m_length;
}

const PayloadIEGroupIDs PayloadIEDescriptor::GetPayloadIEGroupID() const {
    return m_elementId;
}

const bool PayloadIEDescriptor::GetType() const {
    return m_type;
}

uint16_t PayloadIEDescriptor::GetDescriptor() const {
    uint16_t descriptor = 0;

    // descriptor = m_length & (0x7f);                                         // Bits 0-6
    // descriptor |= (static_cast<uint8_t>(m_elementId) << 7) & (0xff << 7);   // Bits 7-14
    // descriptor |= (m_type << 15) & (0x01 << 15);                            // Bit 15

    descriptor = m_length & (0x3ff);                                        // Bits 0-10
    descriptor |= (static_cast<uint8_t>(m_elementId) << 11) & (0xf << 11);   // Bits 11-14
    descriptor |= (m_type << 15) & (0x01 << 15);                            // Bit 15

    return descriptor;
}

void PayloadIEDescriptor::SetDescriptor(uint16_t descriptor) {
    // m_length = (descriptor) & (0x7f);                   // Bits 0-6
    // m_elementId = static_cast<PayloadIEGroupIDs>((descriptor >> 7) & (0xff));           // Bits 7-14
    // m_type = (descriptor >> 15) & (0x01);               // Bit 15

    m_length = (descriptor) & (0x3ff);                   // Bits 0-10
    m_elementId = static_cast<PayloadIEGroupIDs>((descriptor >> 11) & (0xf));           // Bits 11-14
    m_type = (descriptor >> 15) & (0x01);               // Bit 15
}

void PayloadIEDescriptor::Print(std::ostream& os) const {
    os  << "| Length Field | = " << (uint32_t)m_length << std::endl
        << "| Group ID Field | = " << std::hex << m_elementId << std::endl
        << "| Type Field| =" << (uint32_t)m_type << std::endl;
}

Buffer::Iterator PayloadIEDescriptor::Serialize(Buffer::Iterator i) const {
    i.WriteHtolsbU16(GetDescriptor());

    // DSME-TODO

    return i;
}

Buffer::Iterator PayloadIEDescriptor::Deserialize(Buffer::Iterator i) {
    uint16_t descriptor = i.ReadLsbtohU16();
    SetDescriptor(descriptor);

    // DSME-TODO

    return i;
}

uint32_t PayloadIEDescriptor::GetSerializedSize() const {
    // DSME-TODO
    return 2;
}

/***********************************************************
 *                 Payload Sub-IE Descriptor
 ***********************************************************/
PayloadSubIEDescriptor::PayloadSubIEDescriptor() {
    SetType(0);
}

PayloadSubIEDescriptor::~PayloadSubIEDescriptor() {

}

void PayloadSubIEDescriptor::SetLength(uint8_t len) {
    m_length = len;
}

void PayloadSubIEDescriptor::SetMLMESubID(MLMEIESubID id) {
    m_subId = id;
}

void PayloadSubIEDescriptor::SetType(bool type) {
    m_type = type;   
}

uint8_t PayloadSubIEDescriptor::GetLength() const {
    return m_length;
}

MLMEIESubID PayloadSubIEDescriptor::GetMLMESubID() const {
    return m_subId;
}

bool PayloadSubIEDescriptor::GetType() const {
    return m_type;
}

uint16_t PayloadSubIEDescriptor::GetDescriptor() const {
    uint16_t descriptor = 0;

    if (GetType()) {           // long type
        descriptor = m_length & (0x7ff);                                        // Bits 0-10
        descriptor |= (static_cast<uint8_t>(m_subId) << 11) & (0xf << 11);      // Bits 11-14
        descriptor |= (m_type << 15) & (0x01 << 15);                            // Bit 15

    } else {    // short
        descriptor = m_length & (0xff);                                         // Bits 0-7
        descriptor |= (static_cast<uint8_t>(m_subId) << 8) & (0x7f << 8);       // Bits 8-14
        descriptor |= (m_type << 15) & (0x01 << 15);                            // Bit 15
    }

    return descriptor;
}

void PayloadSubIEDescriptor::SetDescriptor(uint16_t descriptor) {
    // DSME-TODO
    // 要怎麼提前知道是 long or short??
    m_length = (descriptor) & (0xff);                                                   // Bits 0-7
    m_subId = static_cast<MLMEIESubID>((descriptor >> 8) & (0x7f));           // Bits 8-14
    m_type = (descriptor >> 15) & (0x01);                                               // Bit 15
}

uint32_t PayloadSubIEDescriptor::GetSerializedSize() const {
    return 2;
}

Buffer::Iterator PayloadSubIEDescriptor::Serialize(Buffer::Iterator i) const {
    i.WriteHtolsbU16(GetDescriptor());

    return i; 
}

Buffer::Iterator PayloadSubIEDescriptor::Deserialize(Buffer::Iterator i) {
    uint16_t descriptor = i.ReadLsbtohU16();
    SetDescriptor(descriptor);

    return i;
}

void PayloadSubIEDescriptor::Print(std::ostream& os) const {
    os  << "| Length Field | = " << (uint32_t)m_length << std::endl
        << "| Sub-ID Field | = " << std::hex << m_subId << std::endl
        << "|  Type Field  | = " << (uint32_t)m_type << std::endl;    
}

/***********************************************************
 *                      EB Filter IE
 ***********************************************************/
EBFilterIE::EBFilterIE() {
    m_descriptor.SetLength(0);
    m_descriptor.SetPayloadIEGroupID(PAYLOADIE_MLME);

    m_subIEDescriptor.SetLength(0);
    m_subIEDescriptor.SetMLMESubID(SUBID_EB_FILTER);
    m_subIEDescriptor.SetType(0);
}

NS_OBJECT_ENSURE_REGISTERED(EBFilterIE);

EBFilterIE::~EBFilterIE() {
}

void EBFilterIE::SetOutterIEDescriptorLen(uint16_t len) {
    m_descriptor.SetLength(len);
}

void EBFilterIE::SetSubIEDescriptorLen(uint8_t len) {
    m_subIEDescriptor.SetLength(len);
}

void EBFilterIE::SetEBFilterDescriptor(uint8_t descriptor) {
    m_permitJoiningOn = (descriptor) & (0x01);                  //<! Bit 0
    m_includeLinkQualityFilter = (descriptor >> 1) & (0x01);    //<! Bit 1
    m_includePercentFilter = (descriptor >> 2) & (0x01);        //<! Bit 2
    m_numOfEntriesInPIBIdList = (descriptor >> 3) & (0x03);     //<! Bit 3-4
    m_reserved = (descriptor >> 5) & (0x07);                    //<! Bit 5-7 Reserved
}

void EBFilterIE::SetPermitJoiningOn() {
    m_permitJoiningOn = true;
}

void EBFilterIE::SetNoPermitJoiningOn() {
    m_permitJoiningOn = false;
}

void EBFilterIE::SetIncludeLinkQualityFilter() {
    m_includeLinkQualityFilter = true;
}

void EBFilterIE::SetNotIncludeLinkQualityFilter() {
    m_includeLinkQualityFilter = false;
}

void EBFilterIE::SetIncludePercentFilter() {
    m_includePercentFilter = true;
}

void EBFilterIE::SetNotIncludePercentFilter() {
    m_includePercentFilter = false;
}

void EBFilterIE::SetNumOfEntriesInPIBIdList(uint8_t num) {
    NS_ASSERT(num < 5);
    NS_ASSERT(num >= 0);

    m_numOfEntriesInPIBIdList = num;
}

void EBFilterIE::AddPIBId(EBRPIBAttributeID id) {
    NS_ASSERT(m_PIBIdList.size() < 4);

    m_PIBIdList.push_back(id);
}

PayloadIEDescriptor EBFilterIE::GetHeaderIEDescriptor() const {
    return m_descriptor;
}

bool EBFilterIE::IsPermitJoiningOn() const {
    return m_permitJoiningOn;
}

bool EBFilterIE::IsLinkQualityFilterIncluded() const {
    return m_includeLinkQualityFilter;
}

bool EBFilterIE::IsPercentFilterIncluded() const {
    return m_includePercentFilter;
}

uint8_t EBFilterIE::GetNumOfEntriesInPIBIdList() const {
    return m_numOfEntriesInPIBIdList;
}

uint8_t EBFilterIE::GetLinkQuality() const {
    return m_linkQuality;
}

uint8_t EBFilterIE::GetPercentFilter() const {
    return m_percentFilter;
}

std::vector<EBRPIBAttributeID> EBFilterIE::GetPIBIdList() const {
    return m_PIBIdList;
}

uint8_t EBFilterIE::GetEBFilterDescriptor() const {
    uint8_t descriptor = 0;
    descriptor = (m_permitJoiningOn) & (0x01);                   //<! Bit 0
    descriptor |= (m_includeLinkQualityFilter << 1) & (0x01);    //<! Bit 1
    descriptor |= (m_includePercentFilter << 2) & (0x01);        //<! Bit 2
    descriptor |= (m_numOfEntriesInPIBIdList << 3) & (0x03);     //<! Bit 3-4
    descriptor |= (m_reserved << 5) & (0x07);                    //<! Bit 5-7 Reserved

    return descriptor;
}

TypeId EBFilterIE::GetTypeId() {
    static TypeId tid = TypeId("ns3::EBFilterIE")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<EBFilterIE>();
    return tid;
}

TypeId EBFilterIE::GetInstanceTypeId() const {
    return GetTypeId();
}

uint32_t EBFilterIE::GetSerializedSize() const {
    uint32_t size = 0;

    size += m_descriptor.GetSerializedSize();
    size += m_subIEDescriptor.GetSerializedSize();
    size += 2;

    if (IsLinkQualityFilterIncluded()) {
        size += 1;
    }

    if (IsPercentFilterIncluded()) {
        size += 1;
    }

    if (GetNumOfEntriesInPIBIdList()) {
        size += GetNumOfEntriesInPIBIdList();
    }

    // std::cout << size << std::endl;

    return size;
}

void EBFilterIE::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;
    i = m_descriptor.Serialize(i);
    i = m_subIEDescriptor.Serialize(i);

    i.WriteHtolsbU16(GetEBFilterDescriptor());

    if (IsLinkQualityFilterIncluded()) {
        i.WriteU8(GetLinkQuality());
    }

    if (IsPercentFilterIncluded()) {
        i.WriteU8(GetPercentFilter());
    }

    if (GetNumOfEntriesInPIBIdList()) {
        for (unsigned int j = 0; j < m_PIBIdList.size(); ++j) {
            i.WriteU8(m_PIBIdList[j]);
        }
    }
}

uint32_t EBFilterIE::Deserialize(Buffer::Iterator start) {
    Buffer::Iterator i = start;
    i = m_descriptor.Deserialize(i);
    i = m_subIEDescriptor.Deserialize(i);

    uint16_t descriptor = i.ReadLsbtohU16();
    SetEBFilterDescriptor(descriptor);

    if (IsLinkQualityFilterIncluded()) {
        m_linkQuality = i.ReadU8();
    }

    if (IsPercentFilterIncluded()) {
        m_percentFilter = i.ReadU8();
    }

    if (GetNumOfEntriesInPIBIdList()) {
        for (unsigned int j = 0; j < m_PIBIdList.size(); ++j) {
            AddPIBId(static_cast<EBRPIBAttributeID>(i.ReadU8()));
        }
    }

    return i.GetDistanceFrom(start);
}

void EBFilterIE::Print(std::ostream &os) const {
    m_descriptor.Print(os);
    m_subIEDescriptor.Print(os);

    os  << "| Permit Joining On | = " << m_permitJoiningOn 
        << " | Include Link Quality Filter | = " << m_includeLinkQualityFilter
        << " | Include Percent Filter  | = " << m_includePercentFilter 
        << " | Num Of Entries In PIB ID List  | = " << m_numOfEntriesInPIBIdList << std::endl;    

    if (IsLinkQualityFilterIncluded()) {
        os  << "| Link Quality Filter | = " << m_linkQuality << std::endl;
    }

    if (IsPercentFilterIncluded()) {
        os  << "| Link Quality Filter | = " << m_percentFilter << std::endl;
    }

    for (unsigned int i = 0; i < m_PIBIdList.size(); ++i) {
        os  << "| EBR PIB Attribute ID | = " << std::hex << m_PIBIdList[i] << std::endl;
    }
}

/***********************************************************
 *                  Payload IE Termination
 ***********************************************************/
PayloadIETermination::PayloadIETermination() {
    m_descriptor.SetLength(0);
    m_descriptor.SetPayloadIEGroupID(PAYLOADIE_LIST_TERMINATION);
}

PayloadIETermination::~PayloadIETermination() {

}

NS_OBJECT_ENSURE_REGISTERED(PayloadIETermination);

TypeId PayloadIETermination::GetTypeId() {
    static TypeId tid = TypeId("ns3::PayloadIETermination")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<PayloadIETermination>();
    return tid;
}

TypeId PayloadIETermination::GetInstanceTypeId() const {
    return GetTypeId();
}

uint32_t PayloadIETermination::GetSerializedSize() const {
    return m_descriptor.GetSerializedSize();
}

void PayloadIETermination::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;

    i = m_descriptor.Serialize(i);
}

uint32_t PayloadIETermination::Deserialize(Buffer::Iterator start) {
    Buffer::Iterator i = start;

    i = m_descriptor.Deserialize(i);

    return i.GetDistanceFrom(start);
}

void PayloadIETermination::Print(std::ostream& os) const {
    m_descriptor.Print(os);
}

/***********************************************************
 *                   Test MAC Payload
 ***********************************************************/

TestNewHeader::TestNewHeader() {

}

NS_OBJECT_ENSURE_REGISTERED(TestNewHeader);

TypeId TestNewHeader::GetTypeId() {
    static TypeId tid = TypeId("ns3::TestNewHeader")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<TestNewHeader>();
    return tid;
}

TypeId TestNewHeader::GetInstanceTypeId() const {
    return GetTypeId();
}

uint32_t TestNewHeader::GetSerializedSize() const {
    return 2;
}

void TestNewHeader::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;

    i.WriteHtolsbU16(m_testData);
}

uint32_t TestNewHeader::Deserialize(Buffer::Iterator start) {
    Buffer::Iterator i = start;

    uint16_t testData = i.ReadLsbtohU16();

    SetData(testData);

    return i.GetDistanceFrom(start);
}

void TestNewHeader::Print(std::ostream& os) const {
    os << "| Test Data Field | = " << m_testData << std::endl;
}

void TestNewHeader::SetData(uint16_t data) {
    m_testData = data;
}

uint16_t TestNewHeader::GetData() const {
    return m_testData;
}



/***********************************************************
 *                   Channel hopping IE
 ***********************************************************/



/***********************************************************
 *                  Enhanced Group Ack IE
 ***********************************************************/

NS_OBJECT_ENSURE_REGISTERED(EnhancedGroupAckDescriptorIE);

EnhancedGroupAckDescriptorIE::EnhancedGroupAckDescriptorIE()
{
    m_u8GroupAckBitmap = 0;
}

EnhancedGroupAckDescriptorIE::~EnhancedGroupAckDescriptorIE()
{

}

void EnhancedGroupAckDescriptorIE::SetHeaderIEDescriptor()
{
    m_descriptor.SetLength(ieLength); // According to the length set before, The length field specifies the number of octets in the IE Content
    m_descriptor.SetHeaderElementID(HEADERIE_ENHANCED_GACK);
}

void EnhancedGroupAckDescriptorIE::SetIELength(uint32_t length)
{
    ieLength = length;
}

uint32_t EnhancedGroupAckDescriptorIE::GetIELength() const
{
    return ieLength;
}

// TypeId EnhancedGroupAckDescriptorIE::GetTypeId() {
//     // static TypeId tid = TypeId("ns3::EnhancedGroupAckDescriptorIE")
//     //                         .SetParent<Header>()
//     //                         .SetGroupName("LrWpan")
//     //                         .AddConstructor<EnhancedGroupAckDescriptorIE>();
//     // return tid;
//     return 0;
// }

// TypeId EnhancedGroupAckDescriptorIE::GetInstanceTypeId() const
// {
//     // return GetTypeId();
// }

uint32_t EnhancedGroupAckDescriptorIE::GetSerializedSize() const
{
    
    uint32_t size = GetIELength(); // According to the size
    return size;
}

void EnhancedGroupAckDescriptorIE::Serialize(Buffer::Iterator start) const {
    Buffer::Iterator i = start;
    i = m_descriptor.Serialize(i);
    // if(GetIELength() == 64) TODO : Dynamic adjust size
    // {

    // }

    i.WriteHtolsbU64(m_u8GroupAckBitmap);

}

uint32_t EnhancedGroupAckDescriptorIE::Deserialize(Buffer::Iterator start) {
    
    Buffer::Iterator i = start;
    i = m_descriptor.Deserialize(i);
    m_u8GroupAckBitmap = i.ReadLsbtohU64();
    // PrintBitmap();
    return i.GetDistanceFrom(start);
}

void EnhancedGroupAckDescriptorIE::PrintBitmap(){
    std::bitset<64> bitmap(m_u8GroupAckBitmap); 
    std::string bitmapStr = bitmap.to_string();
    std::cout << "EnhancedGroupAckDescriptorIE - Bitmap = " << bitmapStr;
}

std::ostream& operator << (std::ostream &os, const EnhancedGroupAckDescriptorIE& enhancedGroupAckDescriptorIE) {

    os << std::bitset<64>(enhancedGroupAckDescriptorIE.GetGroupAckBitmap()); 
    return os;
}

void EnhancedGroupAckDescriptorIE::SetGroupAckBitmap(uint64_t bitmap)
{
    m_u8GroupAckBitmap = bitmap;
}


void EnhancedGroupAckDescriptorIE::Print(std::ostream &os) const {
    os << "| m_u8GroupAckBitmap | = " << m_u8GroupAckBitmap << std::endl;

}

TypeId
EnhancedGroupAckDescriptorIE::GetTypeId()
{
    static TypeId tid = TypeId("ns3::EnhancedGroupAckDescriptorIE")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<EnhancedGroupAckDescriptorIE>();
    return tid;
}

TypeId
EnhancedGroupAckDescriptorIE::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint64_t EnhancedGroupAckDescriptorIE::GetGroupAckBitmap() const
{
    return m_u8GroupAckBitmap;
}


/***********************************************************
 *              Group Ack IE - Ack control
 ***********************************************************/

AckControl::AckControl()
{
    m_payloadSize = 0;
    m_bitmapSize = 0;
    m_reserved = 0;
}

AckControl::~AckControl()
{
    
}

void AckControl::SetAckCtrl(uint8_t ackCtrl)
{
    m_payloadSize = (ackCtrl) & (0x07);         //<! Bit 0-2
    m_bitmapSize = (ackCtrl >> 3) & (0x03);     //<! Bit 3-4
    m_reserved = (ackCtrl >> 5) & (0x03);       //<! Bit 5-7
}

uint8_t AckControl::GetAckCtrl() const
{
    uint8_t ackCtrl = 0;
    ackCtrl = (m_payloadSize) & (0x07);         //<! Bit 0-2
    ackCtrl |= (m_bitmapSize << 3) & (0x03);    //<! Bit 3-4
    ackCtrl |= (m_reserved << 5) & (0x03);      //<! Bit 5-7
    return ackCtrl;
}

void AckControl::SetPayloadSize(uint8_t payloadSize)
{
    m_payloadSize = payloadSize;
}

uint8_t AckControl::GetPayloadSize() const
{
    return m_payloadSize;
}

void AckControl::SetBitmapSize(uint8_t bitmapSize)
{
    m_bitmapSize = bitmapSize;
}

uint8_t AckControl::GetBitmapSize() const
{
    return m_bitmapSize;
}

Buffer::Iterator AckControl::Serialize(Buffer::Iterator i) const {

    i.WriteU8(GetAckCtrl());   
    return i;
}

Buffer::Iterator AckControl::Deserialize(Buffer::Iterator i) {
    
    uint8_t ackCtrl = i.ReadU8();
    SetAckCtrl(ackCtrl);
    return i;
}

uint32_t AckControl::GetSerializedSize() const {
    
    uint32_t size = 1; // Ack control field = 1 byte
    return size;
}

/***********************************************************
 *                  Legacy Group Ack IE
 ***********************************************************/

LegacyGroupAckIE::LegacyGroupAckIE()
{

}

LegacyGroupAckIE::~LegacyGroupAckIE()
{

}

void LegacyGroupAckIE::SetBitmap(uint8_t bitmap, uint8_t bitLocation)
{
    bitmap |= (1 << bitLocation);
}

void LegacyGroupAckIE::ResetBitmap(uint8_t bitmap)
{
    bitmap = 0;
}

uint16_t LegacyGroupAckIE::GetGackBitmapField() const
{
    return m_gackBitmap;
}

void LegacyGroupAckIE::SetGackBitmapField(uint16_t gackBitmap) 
{
    m_gackBitmap = gackBitmap;
}

uint8_t LegacyGroupAckIE::GetGtsDirectionBitmapField() const
{
    return m_gtsDirections;
}

void LegacyGroupAckIE::SetGtsDirectionBitmapField(uint8_t gtsDirectionBitmap) 
{
    m_gtsDirections = gtsDirectionBitmap;
}

void LegacyGroupAckIE::SetGackIdxBitmapField(uint16_t gackIdxBitmap)
{
    m_gackIdx = gackIdxBitmap;
}

void LegacyGroupAckIE::SetGackIdx(uint16_t gackIdxBitmap, uint16_t startLocation, uint16_t value)
{
    // startLocation = 0  , bit 0-3
    // startLocation = 4  , bit 4-7
    // startLocation = 8  , bit 8-11
    // startLocation = 12 , bit 12-15
    
    gackIdxBitmap |= ((value & (0x0F)) << startLocation);

}

uint16_t LegacyGroupAckIE::GetGackIdx(uint16_t gackIdxBitmap, uint16_t startLocation)
{
    // startLocation = 0  , bit 0-3
    // startLocation = 4  , bit 4-7
    // startLocation = 8  , bit 8-11
    // startLocation = 12 , bit 12-15
    uint16_t value = 0;
    value |= ((gackIdxBitmap >> startLocation) & (0x0F));
    return value;
}

uint16_t LegacyGroupAckIE::GetGackIdxBitmapField() const
{
    return m_gackIdx;
}

TypeId
LegacyGroupAckIE::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LegacyGroupAckIE")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LegacyGroupAckIE>();
    return tid;
}

TypeId
LegacyGroupAckIE::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t LegacyGroupAckIE::GetSerializedSize() const
{
    uint32_t size = 0;
    size += 1; // Ack. Control
    size += 2; // GACK Bitmap
    size += 1; // GACK Device List
    size += 2; // GACK Index
    size += 1; // GTS Directions
    return size;
}

void LegacyGroupAckIE::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;

    i = m_ackCtrl.Serialize(i);
    i.WriteU16(m_gackBitmap);
    i.WriteU8(m_gackDevList);
    i.WriteU16(m_gackIdx); 
    i.WriteU8(m_gtsDirections);
}

uint32_t LegacyGroupAckIE::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    i = m_ackCtrl.Deserialize(i); 
    m_gackBitmap = i.ReadU16();
    m_gackDevList = i.ReadU8();
    m_gackIdx = i.ReadU16();
    m_gtsDirections = i.ReadU8();
    return i.GetDistanceFrom(start); 
}

void LegacyGroupAckIE::Print(std::ostream &os) const 
{
    os << "test = " << std::endl;
}

std::ostream& operator << (std::ostream &os, const LegacyGroupAckIE& legacyGroupAckDescriptorIE) {

    os << std::bitset<16>(legacyGroupAckDescriptorIE.GetGackBitmapField()); 
    return os;
}

void LegacyGroupAckIE::SetHeaderIEDescriptor()
{
    m_descriptor.SetLength(ieLength); // According to the length set before, The length field specifies the number of octets in the IE Content
    m_descriptor.SetHeaderElementID(HEADERIE_GACK);
}

void LegacyGroupAckIE::SetIELength(uint32_t length)
{
    ieLength = length;
}

uint8_t LegacyGroupAckIE::GetGackDevListField() const
{
    return m_gackDevList;
}

void LegacyGroupAckIE::SetGackDevListField(uint8_t gackDevList) 
{
    m_gackDevList = gackDevList;
}

/***********************************************************
 *                 DSME-GTS Group Ack IE
 ***********************************************************/

DsmeGtsGroupAckDescriptorIE::DsmeGtsGroupAckDescriptorIE()
{

}

DsmeGtsGroupAckDescriptorIE::~DsmeGtsGroupAckDescriptorIE()
{

}

void DsmeGtsGroupAckDescriptorIE::SetHeaderIEDescriptor()
{
    m_descriptor.SetLength(ieLength); // According to the length set before, The length field specifies the number of octets in the IE Content
    m_descriptor.SetHeaderElementID(HEADERIE_DSME_GTS_GACK);
}

void DsmeGtsGroupAckDescriptorIE::SetIELength(uint32_t length)
{
    ieLength = length;
}

uint32_t DsmeGtsGroupAckDescriptorIE::GetIELength() const
{
    return ieLength;
}

void DsmeGtsGroupAckDescriptorIE::Print(std::ostream &os) const 
{
    os << "test = " << std::endl;
}

std::ostream& operator << (std::ostream &os, const DsmeGtsGroupAckDescriptorIE& dsmeGtsGroupAckDescriptorIE) 
{
    return os;
}

TypeId
DsmeGtsGroupAckDescriptorIE::GetTypeId()
{
    static TypeId tid = TypeId("ns3::DsmeGtsGroupAckDescriptorIE")
                            .SetParent<Header>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<DsmeGtsGroupAckDescriptorIE>();
    return tid;
}

TypeId
DsmeGtsGroupAckDescriptorIE::GetInstanceTypeId() const
{
    return GetTypeId();
}

void DsmeGtsGroupAckDescriptorIE::SetPayloadsNumber(uint8_t payloadNumber)
{
    m_payloadsNumber = payloadNumber;
}

uint8_t DsmeGtsGroupAckDescriptorIE::GetPayloadsNumber() const
{
    return m_payloadsNumber;
}

void DsmeGtsGroupAckDescriptorIE::AdjustPayloadSize()
{
    m_payloads.resize(GetPayloadsNumber());
}

void DsmeGtsGroupAckDescriptorIE::AdjustBitmapSize(DsmeGtsGackPayload &gtsGackPayload)
{
    gtsGackPayload.bitmap.resize(gtsGackPayload.bitmapLength);
}

void DsmeGtsGroupAckDescriptorIE::SetDsmeGtsGackPayload(std::vector<DsmeGtsGackPayload> payloads)
{
    m_payloads.assign(payloads.begin(), payloads.end());
}

void DsmeGtsGroupAckDescriptorIE::GetDsmeGtsGackPayload(std::vector<DsmeGtsGackPayload> &payloads)
{
    payloads.assign(m_payloads.begin(), m_payloads.end());
}

uint32_t DsmeGtsGroupAckDescriptorIE::GetPayloadTotalSize() const
{
    uint32_t totalSize = 0;
    // Calculate the total payload size
    for(int index = 0; index < (int)m_payloads.size(); index++)
    {
        // node addr (2) + bitmap len (1) + seqNum (1) + bitmap (dynamic)
        totalSize += 2 + 1 + 1 + m_payloads[index].bitmap.size(); 
    }
    return totalSize;
}

uint32_t DsmeGtsGroupAckDescriptorIE::GetSerializedSize() const
{
    uint32_t size = 0;
    size += 1; // payloads number = 1 byte
    size += GetPayloadTotalSize(); 

    return size;
}

void DsmeGtsGroupAckDescriptorIE::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    i.WriteU8(m_payloadsNumber);

    for(int index = 0; index < (int)m_payloadsNumber; index++)
    {
        WriteTo(i, m_payloads[index].nodeAddr);
        i.WriteU8(m_payloads[index].bitmapLength);
        i.WriteU8(m_payloads[index].sequenceNumber);

        for(int bmpidx = 0; bmpidx < m_payloads[index].bitmapLength; bmpidx++)
        {
            i.WriteU8(m_payloads[index].bitmap[bmpidx]);
        }
    }
}

uint32_t DsmeGtsGroupAckDescriptorIE::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_payloadsNumber = i.ReadU8();

    for(int index = 0; index < (int)m_payloadsNumber; index++)
    {
        DsmeGtsGackPayload payloadEntity;
        ReadFrom(i, payloadEntity.nodeAddr);
        payloadEntity.bitmapLength = i.ReadU8();
        payloadEntity.sequenceNumber = i.ReadU8();

        for(int bmpidx = 0; bmpidx < payloadEntity.bitmapLength; bmpidx++)
        {
            payloadEntity.bitmap.push_back(i.ReadU8());
        }
        m_payloads.push_back(payloadEntity);
    }

    return i.GetDistanceFrom(start);
}

} // namespace ns3
