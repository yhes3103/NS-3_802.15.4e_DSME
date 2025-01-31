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

#include "lr-wpan-fields.h"

#include <ns3/address-utils.h>
#include <ns3/log.h>

#include <bitset>

namespace ns3
{

SuperframeField::SuperframeField()
{
    SetBeaconOrder(15);
    SetSuperframeOrder(15);
    SetFinalCapSlot(0);
    SetBattLifeExt(false);
    SetPanCoor(false);
    SetAssocPermit(false);
}

void
SuperframeField::SetSuperframe(uint16_t superFrmSpec)
{
    m_sspecBcnOrder = (superFrmSpec) & (0x0F);          // Bits 0-3
    m_sspecSprFrmOrder = (superFrmSpec >> 4) & (0x0F);  // Bits 4-7
    m_sspecFnlCapSlot = (superFrmSpec >> 8) & (0x0F);   // Bits 8-11
    m_sspecBatLifeExt = (superFrmSpec >> 12) & (0x01);  // Bit 12
                                                        // Bit 13 (Reserved)
    m_sspecPanCoor = (superFrmSpec >> 14) & (0x01);     // Bit 14
    m_sspecAssocPermit = (superFrmSpec >> 15) & (0x01); // Bit 15
}

void
SuperframeField::SetBeaconOrder(uint8_t bcnOrder)
{
    if (bcnOrder > 15)
    {
        NS_ABORT_MSG("SuperframeField Beacon Order value must be 15 or less");
    }
    else
    {
        m_sspecBcnOrder = bcnOrder;
    }
}

void
SuperframeField::SetSuperframeOrder(uint8_t frmOrder)
{
    if (frmOrder > 15)
    {
        NS_ABORT_MSG("SuperframeField Frame Order value must be 15 or less");
    }
    else
    {
        m_sspecSprFrmOrder = frmOrder;
    }
}

void
SuperframeField::SetFinalCapSlot(uint8_t capSlot)
{
    if (capSlot > 15)
    {
        NS_ABORT_MSG("The final slot cannot be greater than the slots in a CAP (15)");
    }
    else
    {
        m_sspecFnlCapSlot = capSlot;
    }
}

void
SuperframeField::SetBattLifeExt(bool battLifeExt)
{
    m_sspecBatLifeExt = battLifeExt;
}

void
SuperframeField::SetPanCoor(bool panCoor)
{
    m_sspecPanCoor = panCoor;
}

void
SuperframeField::SetAssocPermit(bool assocPermit)
{
    m_sspecAssocPermit = assocPermit;
}

uint8_t
SuperframeField::GetBeaconOrder() const
{
    return m_sspecBcnOrder;
}

uint8_t
SuperframeField::GetFrameOrder() const
{
    return m_sspecSprFrmOrder;
}

uint8_t
SuperframeField::GetFinalCapSlot() const
{
    return m_sspecFnlCapSlot;
}

bool
SuperframeField::IsBattLifeExt() const
{
    return m_sspecBatLifeExt;
}

bool
SuperframeField::IsPanCoor() const
{
    return m_sspecPanCoor;
}

bool
SuperframeField::IsAssocPermit() const
{
    return m_sspecAssocPermit;
}

uint16_t
SuperframeField::GetSuperframe() const
{
    uint16_t superframe;

    superframe = m_sspecBcnOrder & (0x0F);                   // Bits 0-3
    superframe |= (m_sspecSprFrmOrder << 4) & (0x0F << 4);   // Bits 4-7
    superframe |= (m_sspecFnlCapSlot << 8) & (0x0F << 8);    // Bits 8-11
    superframe |= (m_sspecBatLifeExt << 12) & (0x01 << 12);  // Bit 12
                                                             // Bit 13 (Reserved)
    superframe |= (m_sspecPanCoor << 14) & (0x01 << 14);     // Bit 14
    superframe |= (m_sspecAssocPermit << 15) & (0x01 << 15); // Bit 15

    return superframe;
}

uint32_t
SuperframeField::GetSerializedSize() const
{
    return 2; // 2 Octets (superframeSpec)
}

Buffer::Iterator
SuperframeField::Serialize(Buffer::Iterator i) const
{
    i.WriteHtolsbU16(GetSuperframe());
    return i;
}

Buffer::Iterator
SuperframeField::Deserialize(Buffer::Iterator i)
{
    uint16_t superframe = i.ReadLsbtohU16();
    SetSuperframe(superframe);

    return i;
}

std::ostream&
operator<<(std::ostream& os, const SuperframeField& superframeField)
{
    os << " Beacon Order = " << uint32_t(superframeField.GetBeaconOrder())
       << ", Frame Order = " << uint32_t(superframeField.GetFrameOrder())
       << ", Final CAP slot = " << uint32_t(superframeField.GetFinalCapSlot())
       << ", Battery Life Ext = " << bool(superframeField.IsBattLifeExt())
       << ", PAN Coordinator = " << bool(superframeField.IsPanCoor())
       << ", Association Permit = " << bool(superframeField.IsAssocPermit());
    return os;
}

/***********************************************************
 *         Guaranteed Time Slots (GTS) Fields
 ***********************************************************/

GtsFields::GtsFields()
{
    // GTS Specification Field
    m_gtsSpecDescCount = 0;
    m_gtsSpecPermit = 0;
    // GTS Direction Field
    m_gtsDirMask = 0;
}

uint8_t
GtsFields::GetGtsSpecField() const
{
    uint8_t gtsSpecField;

    gtsSpecField = m_gtsSpecDescCount & (0x07);           // Bits 0-2
                                                          // Bits 3-6 (Reserved)
    gtsSpecField |= (m_gtsSpecPermit << 7) & (0x01 << 7); // Bit 7

    return gtsSpecField;
}

uint8_t
GtsFields::GetGtsDirectionField() const
{
    uint8_t gtsDirectionField;

    gtsDirectionField = m_gtsDirMask & (0x7F); // Bit 0-6
                                               // Bit 7 (Reserved)
    return gtsDirectionField;
}

void
GtsFields::SetGtsSpecField(uint8_t gtsSpec)
{
    m_gtsSpecDescCount = (gtsSpec) & (0x07);   // Bits 0-2
                                               // Bits 3-6 (Reserved)
    m_gtsSpecPermit = (gtsSpec >> 7) & (0x01); // Bit 7
}

void
GtsFields::SetGtsDirectionField(uint8_t gtsDir)
{
    m_gtsDirMask = (gtsDir) & (0x7F); // Bits 0-6
                                      // Bit 7 (Reserved)
}

bool
GtsFields::GetGtsPermit() const
{
    return m_gtsSpecPermit;
}

uint32_t
GtsFields::GetSerializedSize() const
{
    uint32_t size;

    size = 1; // 1 octet  GTS Specification Field
    if (m_gtsSpecDescCount > 0)
    {
        size += 1;                        // 1 octet GTS Direction Field
        size += (m_gtsSpecDescCount * 3); // 3 octets per GTS descriptor
    }

    return size;
}

Buffer::Iterator
GtsFields::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(GetGtsSpecField());

    if (m_gtsSpecDescCount > 0)
    {
        uint8_t gtsDescStartAndLenght;
        i.WriteU8(GetGtsDirectionField());

        for (int j = 0; j < m_gtsSpecDescCount; j++)
        {
            WriteTo(i, m_gtsList[j].m_gtsDescDevShortAddr);

            gtsDescStartAndLenght =
                (m_gtsList[j].m_gtsDescStartSlot & 0x0F) | // GTS descriptor bits 16-19
                (m_gtsList[j].m_gtsDescLength & 0xF0);     // GTS descriptor bits 20-23

            i.WriteU8(gtsDescStartAndLenght);
        }
    }
    return i;
}

Buffer::Iterator
GtsFields::Deserialize(Buffer::Iterator i)
{
    uint8_t gtsSpecField = i.ReadU8();
    SetGtsSpecField(gtsSpecField);

    if (m_gtsSpecDescCount > 0)
    {
        uint8_t gtsDirectionField = i.ReadU8();
        SetGtsDirectionField(gtsDirectionField);

        uint8_t gtsDescStartAndLenght;
        for (int j = 0; j < m_gtsSpecDescCount; j++)
        {
            ReadFrom(i, m_gtsList[j].m_gtsDescDevShortAddr);

            gtsDescStartAndLenght = i.ReadU8();
            m_gtsList[j].m_gtsDescStartSlot = (gtsDescStartAndLenght) & (0x0F);
            m_gtsList[j].m_gtsDescLength = (gtsDescStartAndLenght >> 4) & (0x0F);
        }
    }
    return i;
}

std::ostream&
operator<<(std::ostream& os, const GtsFields& gtsFields)
{
    os << " GTS specification = " << uint32_t(gtsFields.GetGtsSpecField())
       << ", GTS direction = " << uint32_t(gtsFields.GetGtsDirectionField());
    return os;
}

/***********************************************************
 *              Pending Address Fields
 ***********************************************************/

PendingAddrFields::PendingAddrFields()
{
    m_pndAddrSpecNumShortAddr = 0;
    m_pndAddrSpecNumExtAddr = 0;
}

uint8_t
PendingAddrFields::GetNumShortAddr() const
{
    return m_pndAddrSpecNumShortAddr;
}

uint8_t
PendingAddrFields::GetNumExtAddr() const
{
    return m_pndAddrSpecNumExtAddr;
}

uint8_t
PendingAddrFields::GetPndAddrSpecField() const
{
    uint8_t pndAddrSpecField;

    pndAddrSpecField = m_pndAddrSpecNumShortAddr & (0x07);            // Bits 0-2
                                                                      // Bit  3 (Reserved)
    pndAddrSpecField |= (m_pndAddrSpecNumExtAddr << 4) & (0x07 << 4); // Bits 4-6
                                                                      // Bit  7 (Reserved)

    return pndAddrSpecField;
}

void
PendingAddrFields::AddAddress(Mac16Address shortAddr)
{
    uint8_t totalPendAddr = m_pndAddrSpecNumShortAddr + m_pndAddrSpecNumExtAddr;

    if (totalPendAddr == 7)
    {
        return;
    }
    else
    {
        m_shortAddrList[m_pndAddrSpecNumShortAddr] = shortAddr;
        m_pndAddrSpecNumShortAddr++;
    }
}

void
PendingAddrFields::AddAddress(Mac64Address extAddr)
{
    uint8_t totalPendAddr = m_pndAddrSpecNumShortAddr + m_pndAddrSpecNumExtAddr;

    if (totalPendAddr == 7)
    {
        return;
    }
    else
    {
        m_extAddrList[m_pndAddrSpecNumExtAddr] = extAddr;
        m_pndAddrSpecNumExtAddr++;
    }
}

bool
PendingAddrFields::SearchAddress(Mac16Address shortAddr)
{
    for (int j = 0; j <= (m_pndAddrSpecNumShortAddr - 1); j++)
    {
        if (shortAddr == m_shortAddrList[j])
        {
            return true;
        }
    }

    return false;
}

bool
PendingAddrFields::SearchAddress(Mac64Address extAddr)
{
    for (int j = 0; j <= (m_pndAddrSpecNumExtAddr - 1); j++)
    {
        if (extAddr == m_extAddrList[j])
        {
            return true;
        }
    }

    return false;
}

void PendingAddrFields::SetNumOfShortAdrr(uint8_t num) {
    m_pndAddrSpecNumShortAddr = num;
}

void PendingAddrFields::SetNumOfExtAdrr(uint8_t num) {
    m_pndAddrSpecNumExtAddr = num;
}

void
PendingAddrFields::SetPndAddrSpecField(uint8_t pndAddrSpecField)
{
    m_pndAddrSpecNumShortAddr = (pndAddrSpecField) & (0x07);    // Bit 0-2
                                                                // Bit 3
    m_pndAddrSpecNumExtAddr = (pndAddrSpecField >> 4) & (0x07); // Bit 4-6
                                                                // Bit 7
}

uint32_t
PendingAddrFields::GetSerializedSize() const
{
    uint32_t size;

    size = 1;                                      // 1 octet  (Pending Address Specification Field)
    size = size + (m_pndAddrSpecNumShortAddr * 2); // X octets (Short Pending Address List)
    size = size + (m_pndAddrSpecNumExtAddr * 8);   // X octets (Extended Pending Address List)

    return size;
}

Buffer::Iterator
PendingAddrFields::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(GetPndAddrSpecField());

    for (int j = 0; j < m_pndAddrSpecNumShortAddr; j++)
    {
        WriteTo(i, m_shortAddrList[j]);
    }

    for (int k = 0; k < m_pndAddrSpecNumExtAddr; k++)
    {
        WriteTo(i, m_extAddrList[k]);
    }

    return i;
}

Buffer::Iterator
PendingAddrFields::Deserialize(Buffer::Iterator i)
{
    uint8_t pndAddrSpecField = i.ReadU8();

    SetPndAddrSpecField(pndAddrSpecField);

    for (int j = 0; j < m_pndAddrSpecNumShortAddr; j++)
    {
        ReadFrom(i, m_shortAddrList[j]);
    }

    for (int k = 0; k < m_pndAddrSpecNumExtAddr; k++)
    {
        ReadFrom(i, m_extAddrList[k]);
    }

    return i;
}

std::ostream&
operator<<(std::ostream& os, const PendingAddrFields& pendingAddrFields)
{
    os << " Num. Short Addr = " << uint32_t(pendingAddrFields.GetNumShortAddr())
       << ", Num. Ext   Addr = " << uint32_t(pendingAddrFields.GetNumExtAddr());
    return os;
}

/***********************************************************
 *              DSME Superframe Specification Fields
 ***********************************************************/
// DSME-TODO
// Maybe Set according to IEEE 802.15.4e-2012 Section I.4.2 Table I.1?
DsmeSuperFrameField::DsmeSuperFrameField() {
    SetMultiSuperframeOrder(14);
    SetChannelDiversityMode(0);
    SetGACKFlag(0);
    SetCAPReductionFlag(1);
    SetDeferredBeaconFalg(0);
}

void DsmeSuperFrameField::SetDsmeSuperframe(uint16_t dsmeSuperFrm) {
    m_sspecMultiSuperframeOrder = dsmeSuperFrm & (0x0F);          // Bit 0-3
    m_sspecChannelDiversityMode = (dsmeSuperFrm >> 4) & (0x01);   // Bit 4
    m_sspecGACKFlag = (dsmeSuperFrm >> 5) & (0x01);               // Bit 5
    m_sspecCAPReductionFlag = (dsmeSuperFrm >> 6) & (0x01);       // Bit 6
    m_sspecDeferredBcnFlag = (dsmeSuperFrm >> 7) & (0x01);        // Bit 7
}

void DsmeSuperFrameField::SetMultiSuperframeOrder(uint8_t multiBcnOrder) {
    m_sspecMultiSuperframeOrder = multiBcnOrder;
}

void DsmeSuperFrameField::SetChannelDiversityMode(bool channelDiversityMode) {
    m_sspecChannelDiversityMode = channelDiversityMode;
}

void DsmeSuperFrameField::SetGACKFlag(bool gACKFlag) {
    m_sspecGACKFlag = gACKFlag;
}

void DsmeSuperFrameField::SetCAPReductionFlag(bool cAPReduction) {
    m_sspecCAPReductionFlag = cAPReduction;
}

void DsmeSuperFrameField::SetDeferredBeaconFalg(bool deferredBcnFlag) {
    m_sspecDeferredBcnFlag = deferredBcnFlag;
}

uint8_t DsmeSuperFrameField::GetDsmeSuperframe() const {
    uint8_t superframe;

    superframe = m_sspecMultiSuperframeOrder & (0x0F);
    superframe |= (m_sspecChannelDiversityMode << 4) & (0x01 << 4);
    superframe |= (m_sspecGACKFlag << 5) & (0x01 << 5);
    superframe |= (m_sspecCAPReductionFlag << 6) & (0x01 << 6);
    superframe |= (m_sspecDeferredBcnFlag << 7) & (0x01 << 7);

    return superframe;
}

uint8_t DsmeSuperFrameField::GetMultiSuperframeOrder() const {
    return m_sspecMultiSuperframeOrder;
}

bool DsmeSuperFrameField::GetChannelDiversityMode() const {
    return m_sspecChannelDiversityMode;
}

bool DsmeSuperFrameField::GetGACKFlag() const {
    return m_sspecGACKFlag;
}

bool DsmeSuperFrameField::GetCAPReductionFlag() const {
    return m_sspecCAPReductionFlag;
}

bool DsmeSuperFrameField::GetDeferredBeaconFalg() const {
    return m_sspecDeferredBcnFlag;
}

uint32_t DsmeSuperFrameField::GetSerializedSize() const {
    return 1; // 1 Octets
}

Buffer::Iterator DsmeSuperFrameField::Serialize(Buffer::Iterator i) const {
    i.WriteU8(GetDsmeSuperframe());
    return i;
}

Buffer::Iterator DsmeSuperFrameField::Deserialize(Buffer::Iterator i) {
    uint8_t superframe = i.ReadU8();
    SetDsmeSuperframe(superframe);
    return i;
}

std::ostream &operator << (std::ostream &os, const DsmeSuperFrameField& dsmeSperfrmField) {
    os << " DSME Multi Beacon Order = "      << uint32_t(dsmeSperfrmField.GetMultiSuperframeOrder())
     << ", DSME Channel Diversity mode = "   << bool(dsmeSperfrmField.GetChannelDiversityMode())
     << ", DSME GACK = "                     << bool(dsmeSperfrmField.GetGACKFlag())
     << ", DSME CAP Reduction = "            << bool(dsmeSperfrmField.GetCAPReductionFlag())
     << ", DSME Deferred Beacon = "          << bool(dsmeSperfrmField.GetDeferredBeaconFalg());
    return os;
}

/***********************************************************
 *      DSME Time Synchronization Specification Fields
 ***********************************************************/
TimeSync::TimeSync() {
    SetBeaconTimeStamp(0);
    // SetBeaconOffsetTimeStamp(0);
}

void TimeSync::SetBeaconTimeStamp(uint64_t bcnTimestamp) {
    m_sspecBcnTimestamp = bcnTimestamp;
}

void TimeSync::SetBeaconOffsetTimeStamp(uint16_t bcnOfsTimeStamp) {
    m_sspecBcnOffsetTimestamp = bcnOfsTimeStamp;
}

uint64_t TimeSync::GetBeaconTimeStamp() const {
    return m_sspecBcnTimestamp;
}

uint16_t TimeSync::GetBeaconOffsetTimeStamp() const {
    return m_sspecBcnOffsetTimestamp;
}

uint32_t TimeSync::GetSerializedSize() const {
    return 8;   // 8 Octets
}

Buffer::Iterator TimeSync::Serialize(Buffer::Iterator i) const {
    // DSME-TODO
    i.WriteHtolsbU64(GetBeaconTimeStamp());
    // i.WriteHtolsbU16(GetBeaconOffsetTimeStamp());

    return i;
}

Buffer::Iterator TimeSync::Deserialize(Buffer::Iterator i) {
    // DSME-TODO
    m_sspecBcnTimestamp = i.ReadLsbtohU64();
    // m_sspecBcnOffsetTimestamp = i.ReadLsbtohU16();

    return i;
}

std::ostream &
operator << (std::ostream &os, const TimeSync &timeSynSpec) {
    os << " Time Synchronization Beacon Timestamp = "     << uint32_t(timeSynSpec.GetBeaconTimeStamp())
       << ", Time Synchronization Beacon Offset Timestamp = " << uint32_t(timeSynSpec.GetBeaconOffsetTimeStamp());
    return os;
}

/***********************************************************
 *              DSME Beacon Bitmap Fields
 ***********************************************************/
BeaconBitmap::BeaconBitmap() {
    SetSDIndex(0);
    SetBitmapLength(0);            // Set Default BO = 14 , SO = 14
    ResetSDBitmap();
}

BeaconBitmap::BeaconBitmap(uint16_t sdIndex, uint16_t bitmapLen) {
    SetSDIndex(sdIndex);
    SetBitmapLength(bitmapLen);            
    ResetSDBitmap();
}

void BeaconBitmap::SetSDIndex(uint16_t sdIndex) {
    m_sspecSDIndex = sdIndex;
}

void BeaconBitmap::SetBitmapLength(uint16_t bitmapLen) {        
    m_sspecSDBitmapLen = bitmapLen;
}

void BeaconBitmap::SetSDBitmap(uint16_t position) {
    uint16_t idx = position / 16;

    m_sspecSDBitmap[idx] |= 1 << (position % 16);
}

void BeaconBitmap::SetSDBitmap(std::vector<uint16_t>& sdBitmap) {
    m_sspecSDBitmap = std::move(sdBitmap);
}

void BeaconBitmap::ResetSDBitmap() {
    uint16_t len = ceil(m_sspecSDBitmapLen / 16.0);
    
    m_sspecSDBitmap = std::vector<uint16_t>(len, 0);
}

uint16_t BeaconBitmap::GetSDIndex() const {
    return m_sspecSDIndex;
}

uint16_t BeaconBitmap::GetSDBitmapLength() const {
    return m_sspecSDBitmapLen;
}

const std::vector<uint16_t> BeaconBitmap::GetSDBitmap() const {
    return m_sspecSDBitmap;
}

uint32_t BeaconBitmap::GetSerializedSize() const {
    uint32_t size = 0;
    size += 2;                                // SD Index
    size += 2;                                // SD Bitmap Length
    // DSME-TODO
    size += m_sspecSDBitmap.size() * 2;   // SD Bitmap           

    return size;
}

Buffer::Iterator BeaconBitmap::Serialize(Buffer::Iterator i) const {
    i.WriteHtolsbU16(GetSDIndex());
    i.WriteHtolsbU16(GetSDBitmapLength());

    // DSME-TODO
    for (uint16_t j = 0; j < m_sspecSDBitmap.size(); j++) {
        i.WriteHtolsbU16(m_sspecSDBitmap[j]);
    }

    return i;
}

Buffer::Iterator BeaconBitmap::Deserialize(Buffer::Iterator i) {
    m_sspecSDIndex = i.ReadLsbtohU16();
    m_sspecSDBitmapLen = i.ReadLsbtohU16();

    ResetSDBitmap();
    
    // DSME-TODO
    for (uint16_t j = 0; j < m_sspecSDBitmap.size(); j++) {
        m_sspecSDBitmap[j] = i.ReadLsbtohU16();
    }

    return i;
}

void BeaconBitmap::PrintSDBitMap(std::ostream &os) const {
    for (uint16_t i = 0 ; i < m_sspecSDBitmap.size() ; i++) {
        os << " " << std::bitset<16>(m_sspecSDBitmap[i]); 
        os << std::endl;
    } 
}

std::ostream& operator << (std::ostream &os, const BeaconBitmap& beaconBitmap) {
    os << "Beacon Bitmap SD Index = "  << uint32_t(beaconBitmap.GetSDIndex())
        << ", SD Bitmap Length = "      << uint32_t(beaconBitmap.GetSDBitmapLength())
        << ", SD bitmap =";

    beaconBitmap.PrintSDBitMap(os);
        
    return os;
}

BeaconBitmap BeaconBitmap::operator| (const BeaconBitmap& beaconBitmap) const {
    NS_ASSERT(GetSDBitmapLength() == beaconBitmap.GetSDBitmapLength());

    BeaconBitmap result(0, GetSDBitmapLength());

    std::vector<uint16_t> sdBitmap1 = GetSDBitmap();
    std::vector<uint16_t> sdBitmap2 = beaconBitmap.GetSDBitmap();
    std::vector<uint16_t> resultBitmap(ceil(GetSDBitmapLength() / 16.0), 0);

    for (uint16_t i = 0 ; i < sdBitmap1.size() ; i++) {
        resultBitmap[i] = sdBitmap1[i] | sdBitmap2[i];
    }

    result.SetSDBitmap(resultBitmap);

    return result;
}

/***********************************************************
 *      DSME Channel Hopping Specification Fields
 ***********************************************************/
ChannelHopping::ChannelHopping() {
    SetHoppingSequenceID(0);
    SetPANCoordinatorBSN(0);
    SetChannelOffset(0);
    SetChannelOffsetBitmapLength(16);
    ResetChannelOffsetBitmap();
}

void ChannelHopping::SetHoppingSequenceID(uint8_t seqID) {
    m_sspecHoppingSequenceID = seqID;
}

void ChannelHopping::SetPANCoordinatorBSN(uint8_t bcnSeqNum) {
    m_sspecPANCoordBSN = bcnSeqNum;
}

void ChannelHopping::SetChannelOffset(uint16_t ofs) {
    m_sspecChannelOfs = ofs;
}

void ChannelHopping::SetChannelOffsetBitmapLength(uint8_t bitmapLen) {
    m_sspecChannelOfsBitmapLen = bitmapLen;
}

void ChannelHopping::SetChannelOffsetBitmap(std::vector<uint16_t> bitmap) {
    m_sspecChannelOfsBitmap = bitmap;
}

void ChannelHopping::SetChannelOffsetBitmap(uint16_t position) {
    uint16_t idx = position / 16;

    m_sspecChannelOfsBitmap[idx] |= 1 << (position % 16);
}

void ChannelHopping::ResetChannelOffsetBitmap() {
    uint16_t len = ceil(m_sspecChannelOfsBitmapLen / 16.0);
    
    m_sspecChannelOfsBitmap = std::vector<uint16_t>(len, 0);
}

uint8_t ChannelHopping::GetHoppingSequenceID() const {
    return m_sspecHoppingSequenceID;
}

uint8_t ChannelHopping::GetPANCoordinatorBSN() const {
    return m_sspecPANCoordBSN;
}

uint16_t ChannelHopping::GetChannelOffset() const {
    return m_sspecChannelOfs;
}

uint8_t ChannelHopping::GetChannelOffsetBitmapLength() const {
    return m_sspecChannelOfsBitmapLen;
}

const std::vector<uint16_t> ChannelHopping::GetChannelOffsetBitmap() const {
    return m_sspecChannelOfsBitmap;
}

uint32_t ChannelHopping::GetSerializedSize() const {
    uint32_t size = 0;
    size = 1;
    size += 1;
    size += 2;
    size += 1;
    size += m_sspecChannelOfsBitmap.size() * 2;

    return size;
}

Buffer::Iterator ChannelHopping::Serialize(Buffer::Iterator i) const {
    i.WriteU8(GetHoppingSequenceID());
    i.WriteU8(GetPANCoordinatorBSN());
    i.WriteHtolsbU16(GetChannelOffset());
    i.WriteU8(GetChannelOffsetBitmapLength());

    for (uint8_t j = 0; j < m_sspecChannelOfsBitmap.size(); j++) {
        i.WriteHtolsbU16(m_sspecChannelOfsBitmap[j]);
    }

    return i;
}

Buffer::Iterator ChannelHopping::Deserialize(Buffer::Iterator i) {
    m_sspecHoppingSequenceID = i.ReadU8();
    m_sspecPANCoordBSN = i.ReadU8();
    m_sspecChannelOfs = i.ReadLsbtohU16();
    m_sspecChannelOfsBitmapLen = i.ReadU8();

    ResetChannelOffsetBitmap();

    for (uint8_t j = 0; j < m_sspecChannelOfsBitmap.size(); j++) {
        m_sspecChannelOfsBitmap[j] = i.ReadLsbtohU16();
    }

    return i;
}

void ChannelHopping::PrintChannelOffsetBitMap(std::ostream &os) const {
    for (uint16_t i = 0 ; i < m_sspecChannelOfsBitmap.size() ; i++) {
            os << " " << std::bitset<16>(m_sspecChannelOfsBitmap[i]); 
            os << std::endl;
    } 
}

std::ostream &operator << (std::ostream &os, const ChannelHopping& channelHopping) {
    os << "Beacon Bitmap SD Index = "    << uint32_t(channelHopping.GetHoppingSequenceID())
        << ", SD Bitmap Length = "        << uint32_t(channelHopping.GetPANCoordinatorBSN())
        << ", Channel Offset = "          << uint32_t(channelHopping.GetChannelOffset())
        << ", Channel Offset Length = "    << uint32_t(channelHopping.GetChannelOffsetBitmapLength())
        << ", Channel Offset Bitmap = ";

    channelHopping.PrintChannelOffsetBitMap(os);

    return os;
}

/***********************************************************
 *                  DSME Group ACK Fields
 ***********************************************************/
GroupACK::GroupACK() {
    // DSME-TODO
}

void GroupACK::SetGACK1SuperframeID(uint16_t gack1superFrmID) {
    m_sspecGACK1SuperfrmID = gack1superFrmID;
}

void GroupACK::SetGACK1SlotID(uint8_t gack1SlotID) {
    m_sspecGACK1SlotID = gack1SlotID;
}

void GroupACK::SetGACK1ChannelID(uint8_t gack1ChannelID) {
    m_sspecGACK1ChannelID = gack1ChannelID;
}

void GroupACK::SetGACK2SuperframeID(uint16_t gack2superFrmID) {
    m_sspecGACK2SuperfrmID = gack2superFrmID;
}

void GroupACK::SetGACK2SlotID(uint16_t gack2SlotID) {
    m_sspecGACK2SlotID = gack2SlotID;
}

void GroupACK::SetGACK2ChannelID(uint8_t gack2ChannelID) {
    m_sspecGACK2ChannelID = gack2ChannelID;
}

uint16_t GroupACK::GetGACK1SuperframeID() const {
    return m_sspecGACK1SuperfrmID;
}

uint8_t GroupACK::GetGACK1SlotID() const {
    return m_sspecGACK1SlotID;
}

uint8_t GroupACK::GetGACK1ChannelID() const {
    return m_sspecGACK1ChannelID;
}

uint16_t GroupACK::GetGACK2SuperframeID() const {
    return m_sspecGACK2SuperfrmID;
}

uint8_t GroupACK::GetGACK2SlotID() const {
    return m_sspecGACK2SlotID;
}

uint8_t GroupACK::GetGACK2ChannelID() const {
    return m_sspecGACK2ChannelID;
}

uint32_t GroupACK::GetSerializedSize() const {
    // DSME-TODO
    return 8;  // 7 Octets, padding to 8
}

void GroupACK::SetGroupAck(uint64_t groupAck)
{
    m_sspecGACK1SuperfrmID = (groupAck) & (0xFFFF);       // Bit 0-15
    m_sspecGACK1SlotID = (groupAck >> 16) & (0x0F);       // Bit 16-19
    m_sspecGACK1ChannelID = (groupAck >> 20) & (0xFF);    // Bit 20-27
    m_sspecGACK2SuperfrmID = (groupAck >> 28) & (0xFFFF); // Bit 28-43
    m_sspecGACK2SlotID = (groupAck >> 44) & (0x0F);       // Bit 44-47
    m_sspecGACK2ChannelID = (groupAck >> 48) & (0xFF);    // Bit 48-55
}

uint64_t GroupACK::GetGroupAck() const
{
    uint64_t groupAck;
    groupAck = m_sspecGACK1SuperfrmID & (0xFFFF);                 // Bit 0-15
    groupAck |= (m_sspecGACK1SlotID << 16) & (0x0F << 16);        // Bit 16-19
    groupAck |= (m_sspecGACK1ChannelID << 20) & (0xFF << 20);     // Bit 20-27
    groupAck |= ((uint64_t)m_sspecGACK2SuperfrmID << 28) & ((uint64_t)0xFFFF << 28);  // Bit 28-43
    groupAck |= ((uint64_t)m_sspecGACK2SlotID << 44) & ((uint64_t)0x0F << 44);        // Bit 44-47
    groupAck |= ((uint64_t)m_sspecGACK2ChannelID << 48) & ((uint64_t)0xFF << 48);     // Bit 48-55
    return groupAck;
}

Buffer::Iterator GroupACK::Serialize(Buffer::Iterator i) const {
    // DSME-TODO
    i.WriteHtolsbU64(GetGroupAck());
    return i;
}

Buffer::Iterator GroupACK::Deserialize(Buffer::Iterator i) {
    // DSME-TODO
    uint64_t groupAck = i.ReadLsbtohU64();
    SetGroupAck(groupAck);
    return i;
}

std::ostream &operator << (std::ostream &os, const GroupACK& GroupACK) 
{
    os << " [Legacy Group Ack infos] " 
        << ", GACK1ChannelID = "        << uint64_t(GroupACK.GetGACK1ChannelID())
        << ", GACK1SuperframeID = "          << uint64_t(GroupACK.GetGACK1SuperframeID())
        << ", GACK1SlotID = "          << uint32_t(GroupACK.GetGACK1SlotID())
        << ", GACK2ChannelID = "        << uint64_t(GroupACK.GetGACK2ChannelID())
        << ", GACK2SuperframeID = "          << uint32_t(GroupACK.GetGACK2SuperframeID())
        << ", GACK2SlotID = "          << uint32_t(GroupACK.GetGACK2SlotID());
    return os;
}


/***********************************************************
 *              Capability Information Field
 ***********************************************************/

CapabilityField::CapabilityField()
{
    m_deviceType = true;
    m_powerSource = false;
    m_receiverOnWhenIdle = true;
    m_securityCap = false;
    m_allocAddr = true;
}

uint32_t
CapabilityField::GetSerializedSize() const
{
    return 1;
}

Buffer::Iterator
CapabilityField::Serialize(Buffer::Iterator i) const
{
    // uint8_t capability;

    // capability = 0;                                          //!< Bit 0 (reserved)
    // capability = (m_deviceType << 1) & (0x01 << 1);          //!< Bit 1
    // capability |= (m_powerSource << 2) & (0x01 << 2);        //!< Bit 2
    // capability |= (m_receiverOnWhenIdle << 3) & (0x01 << 3); //!< Bit 3
    //                                                          //!< Bit 4-5 (reserved)
    // capability |= (m_securityCap << 6) & (0x01 << 6);        //!< Bit 6
    // capability |= (m_allocAddr << 7) & (0x01 << 7);          //!< Bit 7
    // i.WriteU8(capability);
    // return i;
    uint8_t capability;

    capability = 0;                                          //!< Bit 0 (reserved)
    capability = (m_deviceType << 1) & (0x01 << 1);          //!< Bit 1
    capability |= (m_powerSource << 2) & (0x01 << 2);        //!< Bit 2
    capability |= (m_receiverOnWhenIdle << 3) & (0x01 << 3); //!< Bit 3
    capability |= (m_associationType << 4) & (0x01 << 4);    //!< Bit 4
                                                             //!< Bit 5 (reserved)
    capability |= (m_securityCap << 6) & (0x01 << 6);        //!< Bit 6
    capability |= (m_allocAddr << 7) & (0x01 << 7);          //!< Bit 7

    i.WriteU8(capability);

    return i;
}

Buffer::Iterator
CapabilityField::Deserialize(Buffer::Iterator i)
{
    // uint8_t capability = i.ReadU8();
    // //!< Bit 0 (reserved)
    // m_deviceType = (capability >> 1) & (0x01);         //!< Bit 1
    // m_powerSource = (capability >> 2) & (0x01);        //!< Bit 2
    // m_receiverOnWhenIdle = (capability >> 3) & (0x01); //!< Bit 3
    //                                                    //!< Bit 4-5 (reserved)
    // m_securityCap = (capability >> 6) & (0x01);        //!< Bit 6
    // m_allocAddr = (capability >> 7) & (0x01);          //!< Bit 7

    // return i;
    uint8_t capability = i.ReadU8();
                                                       //!< Bit 0 (reserved)
    m_deviceType = (capability >> 1) & (0x01);         //!< Bit 1
    m_powerSource = (capability >> 2) & (0x01);        //!< Bit 2
    m_receiverOnWhenIdle = (capability >> 3) & (0x01); //!< Bit 3
    m_associationType = (capability >> 4) & (0x01);    //!< Bit 4
                                                       //!< Bit 5 (reserved)
    m_securityCap = (capability >> 6) & (0x01);        //!< Bit 6
    m_allocAddr = (capability >> 7) & (0x01);          //!< Bit 7

    return i;
}

bool
CapabilityField::IsDeviceTypeFfd() const
{
    return m_deviceType;
}

bool
CapabilityField::IsPowSrcAvailable() const
{
    return m_powerSource;
}

bool
CapabilityField::IsReceiverOnWhenIdle() const
{
    return m_receiverOnWhenIdle;
}

bool
CapabilityField::IsSecurityCapability() const
{
    return m_securityCap;
}

bool
CapabilityField::IsShortAddrAllocOn() const
{
    return m_allocAddr;
}

bool CapabilityField::IsFastAOn() const {
    return m_associationType;
}

void
CapabilityField::SetFfdDevice(bool devType)
{
    m_deviceType = devType;
}

void
CapabilityField::SetPowSrcAvailable(bool pow)
{
    m_powerSource = pow;
}

void
CapabilityField::SetRxOnWhenIdle(bool rxIdle)
{
    m_receiverOnWhenIdle = rxIdle;
}

void
CapabilityField::SetSecurityCap(bool sec)
{
    m_securityCap = sec;
}

void
CapabilityField::SetShortAddrAllocOn(bool addrAlloc)
{
    m_allocAddr = addrAlloc;
}

void CapabilityField::SetFastAOn() {
    m_associationType = 1;
}

void CapabilityField::SetFastAOff() {
    m_associationType = 0;
}

/**
 * output stream output operator
 *
 * \param os output stream
 * \param capabilityField the Capability Information Field
 *
 * \returns output stream
 */
std::ostream&
operator<<(std::ostream& os, const CapabilityField& capabilityField)
{
    os << " FFD device capable = " << bool(capabilityField.IsDeviceTypeFfd())
       << ", Alternate Power Current Available  = " << bool(capabilityField.IsPowSrcAvailable())
       << ", Receiver On When Idle  = " << bool(capabilityField.IsReceiverOnWhenIdle())
       << ", Security Capable  = " << bool(capabilityField.IsSecurityCapability())
       << ", Coordinator Allocate Short Address  = " << bool(capabilityField.IsShortAddrAllocOn());
    return os;
}


/***********************************************************
 *          Self Designed Enhanced Group Ack Field
 ***********************************************************/

EnhancedGroupACK::EnhancedGroupACK()
{
    m_groupAckHashTableBitmap = 0;
}

void
EnhancedGroupACK::SetBit(uint64_t bitmap, int bitLocation)
{
    bitmap |= (1 << bitLocation);
}

uint32_t
EnhancedGroupACK::GetSerializedSize() const
{
    return 16;
}

uint64_t
EnhancedGroupACK::GetHashTableBitmap() const
{
    return m_groupAckHashTableBitmap;
}

uint16_t
EnhancedGroupACK::GetHashTableBitmapSize() const
{
    return 64;
}

void
EnhancedGroupACK::ResetHashTableBitmap()
{
    m_groupAckHashTableBitmap = 0;
}

Buffer::Iterator
EnhancedGroupACK::Serialize(Buffer::Iterator i) const
{
    i.WriteHtolsbU64(GetHashTableBitmap());
    return i;
}
Buffer::Iterator
EnhancedGroupACK::Deserialize(Buffer::Iterator i)
{
    m_groupAckHashTableBitmap = i.ReadLsbtohU64();
    return i;
}

std::ostream &operator << (std::ostream &os, const EnhancedGroupACK& enhancedGroupACK) 
{
    os << " Enhanced group ack hash table bitmap = " << uint64_t(enhancedGroupACK.GetHashTableBitmap())
       << " , bitmap size = " << uint16_t(enhancedGroupACK.GetHashTableBitmapSize());
    return os;
}






} // namespace ns3
