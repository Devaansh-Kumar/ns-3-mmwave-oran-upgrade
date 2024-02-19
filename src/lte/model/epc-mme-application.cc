/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2016, University of Padova, Dep. of Information Engineering, SIGNET lab
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *     (support for RRC_CONNECTED->RRC_IDLE state transition + fix for bug 2161
 *      + extension to application & support for real S1AP link)
 */

#include <ns3/fatal-error.h>
#include <ns3/log.h>

#include "epc-s1ap-sap.h"
#include "epc-s11-sap.h"

#include "epc-mme-application.h"

#include "epc-gtpc-header.h"

#include "ns3/log.h"

#include <map>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EpcMmeApplication");

NS_OBJECT_ENSURE_REGISTERED(EpcMmeApplication);

EpcMmeApplication::EpcMmeApplication()
    :m_s11SapSgw (0)
{
    NS_LOG_FUNCTION(this);
    m_s1apSapMme = new MemberEpcS1apSapMme<EpcMmeApplication> (this);
    m_s11SapMme = new MemberEpcS11SapMme<EpcMmeApplication> (this);
}


EpcMmeApplication::~EpcMmeApplication()
{
    NS_LOG_FUNCTION(this);
}

void
EpcMmeApplication::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete m_s1apSapMme;
    delete m_s11SapMme;
}

TypeId
EpcMmeApplication::GetTypeId()
{
    static TypeId tid = TypeId("ns3::EpcMmeApplication")
                            .SetParent<Object>()
                            .SetGroupName("Lte")
                            .AddConstructor<EpcMmeApplication>();
    return tid;
}

EpcS1apSapMme*
EpcMmeApplication::GetS1apSapMme()
{
    return m_s1apSapMme;
}
void 
EpcMmeApplication::SetS1apSapMmeProvider(EpcS1apSapMmeProvider* provider)
{
  m_s1apSapMmeProvider = provider;
}
void 
EpcMmeApplication::SetS11SapSgw (EpcS11SapSgw * s)
{
  m_s11SapSgw = s;
}

EpcS11SapMme* 
EpcMmeApplication::GetS11SapMme ()
{
  return m_s11SapMme;
}

void
EpcMmeApplication::AddSgw(Ipv4Address sgwS11Addr, Ipv4Address mmeS11Addr, Ptr<Socket> mmeS11Socket)
{
    NS_LOG_FUNCTION(this << sgwS11Addr << mmeS11Addr << mmeS11Socket);
    m_sgwS11Addr = sgwS11Addr;
    m_mmeS11Addr = mmeS11Addr;
    m_s11Socket = mmeS11Socket;
    m_s11Socket->SetRecvCallback(MakeCallback(&EpcMmeApplication::RecvFromS11Socket, this));
}

void
EpcMmeApplication::AddEnb(uint16_t gci, Ipv4Address enbS1uAddr, EpcS1apSapEnb* enbS1apSap)
{
    NS_LOG_FUNCTION(this << gci << enbS1uAddr << enbS1apSap);
    Ptr<EnbInfo> enbInfo = Create<EnbInfo>();
    enbInfo->gci = gci;
    enbInfo->s1uAddr = enbS1uAddr;
    enbInfo->s1apSapEnb = enbS1apSap;
    m_enbInfoMap[gci] = enbInfo;
}

void
EpcMmeApplication::AddUe(uint64_t imsi)
{
    NS_LOG_FUNCTION(this << imsi);
    Ptr<UeInfo> ueInfo = Create<UeInfo>();
    ueInfo->imsi = imsi;
    ueInfo->mmeUeS1Id = imsi;
    ueInfo->bearerCounter = 0;
    m_ueInfoMap[imsi] = ueInfo;
}

uint8_t
EpcMmeApplication::AddBearer(uint64_t imsi, Ptr<EpcTft> tft, EpsBearer bearer)
{
    NS_LOG_FUNCTION(this << imsi);
    auto it = m_ueInfoMap.find(imsi);
    NS_ASSERT_MSG(it != m_ueInfoMap.end(), "could not find any UE with IMSI " << imsi);
    NS_ASSERT_MSG(it->second->bearerCounter < 11,
                  "too many bearers already! " << it->second->bearerCounter);
    BearerInfo bearerInfo;
    bearerInfo.bearerId = ++(it->second->bearerCounter);
    bearerInfo.tft = tft;
    bearerInfo.bearer = bearer;
    it->second->bearersToBeActivated.push_back(bearerInfo);
    return bearerInfo.bearerId;
}

// S1-AP SAP MME forwarded methods

void
EpcMmeApplication::DoInitialUeMessage(uint64_t mmeUeS1Id,
                                      uint16_t enbUeS1Id,
                                      uint64_t imsi,
                                      uint16_t gci)
{
    NS_LOG_FUNCTION (this << mmeUeS1Id << enbUeS1Id << imsi << gci);
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoMap.find (imsi);
  NS_ASSERT_MSG (it != m_ueInfoMap.end (), "could not find any UE with IMSI " << imsi);
  it->second->cellId = gci;
  EpcS11SapSgw::CreateSessionRequestMessage msg = EpcS11SapSgw::CreateSessionRequestMessage();
  msg.imsi = imsi;
  msg.uli.gci = gci;
  for (std::list<BearerInfo>::iterator bit = it->second->bearersToBeActivated.begin ();
       bit != it->second->bearersToBeActivated.end ();
       ++bit)
    {
      EpcS11SapSgw::BearerContextToBeCreated bearerContext;
      bearerContext.epsBearerId =  bit->bearerId;
      NS_LOG_INFO("Mme: sending as bearerId " << (uint32_t) bit->bearerId);
      bearerContext.bearerLevelQos = bit->bearer; 
      bearerContext.tft = bit->tft;
      msg.bearerContextsToBeCreated.push_back (bearerContext);
    }
  m_s11SapSgw->CreateSessionRequest (msg);
}

void
EpcMmeApplication::DoInitialContextSetupResponse(
    uint64_t mmeUeS1Id,
    uint16_t enbUeS1Id,
    std::list<EpcS1apSapMme::ErabSetupItem> erabSetupList)
{
    NS_LOG_FUNCTION(this << mmeUeS1Id << enbUeS1Id);
    NS_FATAL_ERROR("unimplemented");
}

void
EpcMmeApplication::DoPathSwitchRequest(
    uint64_t enbUeS1Id,
    uint64_t mmeUeS1Id,
    uint16_t gci,
    std::list<EpcS1apSapMme::ErabSwitchedInDownlinkItem> erabToBeSwitchedInDownlinkList)
{
    NS_LOG_FUNCTION (this << mmeUeS1Id << enbUeS1Id << gci);

  uint64_t imsi = mmeUeS1Id; 
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoMap.find (imsi);
  NS_ASSERT_MSG (it != m_ueInfoMap.end (), "could not find any UE with IMSI " << imsi);
  NS_LOG_INFO ("IMSI " << imsi << " old eNB: " << it->second->cellId << ", new eNB: " << gci);
  it->second->cellId = gci;
  it->second->enbUeS1Id = enbUeS1Id;

  EpcS11SapSgw::ModifyBearerRequestMessage msg;
  msg.teid = imsi; // trick to avoid the need for allocating TEIDs on the S11 interface
  msg.uli.gci = gci;
  // bearer modification is not supported for now
  m_s11SapSgw->ModifyBearerRequest (msg);
}
void 
EpcMmeApplication::DoCreateSessionResponse (EpcS11SapMme::CreateSessionResponseMessage msg)
{
  NS_LOG_FUNCTION (this << msg.teid);
  uint64_t imsi = msg.teid;
  std::list<EpcS1apSapEnb::ErabToBeSetupItem> erabToBeSetupList;
  for (std::list<EpcS11SapMme::BearerContextCreated>::iterator bit = msg.bearerContextsCreated.begin ();
       bit != msg.bearerContextsCreated.end ();
       ++bit)
    {
      EpcS1apSapEnb::ErabToBeSetupItem erab;
      erab.erabId = bit->epsBearerId;
      erab.erabLevelQosParameters = bit->bearerLevelQos;
      erab.transportLayerAddress = bit->sgwFteid.address;
      erab.sgwTeid = bit->sgwFteid.teid;      
      erabToBeSetupList.push_back (erab);
    }
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoMap.find (imsi);
  NS_ASSERT_MSG (it != m_ueInfoMap.end (), "could not find any UE with IMSI " << imsi);
  uint16_t cellId = it->second->cellId;
  uint16_t enbUeS1Id = it->second->enbUeS1Id;
  uint64_t mmeUeS1Id = it->second->mmeUeS1Id;
  std::map<uint16_t, Ptr<EnbInfo> >::iterator jt = m_enbInfoMap.find (cellId);
  NS_ASSERT_MSG (jt != m_enbInfoMap.end (), "could not find any eNB with CellId " << cellId);
  m_s1apSapMmeProvider->SendInitialContextSetupRequest (mmeUeS1Id, enbUeS1Id, erabToBeSetupList, cellId);
}


void 
EpcMmeApplication::DoModifyBearerResponse (EpcS11SapMme::ModifyBearerResponseMessage msg)
{
  NS_LOG_FUNCTION (this << msg.teid);
  NS_ASSERT (msg.cause == EpcS11SapMme::ModifyBearerResponseMessage::REQUEST_ACCEPTED);
  uint64_t imsi = msg.teid;
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoMap.find (imsi);
  NS_ASSERT_MSG (it != m_ueInfoMap.end (), "could not find any UE with IMSI " << imsi);
  uint64_t enbUeS1Id = it->second->enbUeS1Id;
  uint64_t mmeUeS1Id = it->second->mmeUeS1Id;
  uint16_t cgi = it->second->cellId;
  std::list<EpcS1apSapEnb::ErabSwitchedInUplinkItem> erabToBeSwitchedInUplinkList; // unused for now
  std::map<uint16_t, Ptr<EnbInfo> >::iterator jt = m_enbInfoMap.find (it->second->cellId);
  NS_ASSERT_MSG (jt != m_enbInfoMap.end (), "could not find any eNB with CellId " << it->second->cellId);
  m_s1apSapMmeProvider->SendPathSwitchRequestAcknowledge (enbUeS1Id, mmeUeS1Id, cgi, erabToBeSwitchedInUplinkList);
}


void
EpcMmeApplication::DoErabReleaseIndication(
    uint64_t mmeUeS1Id,
    uint16_t enbUeS1Id,
    std::list<EpcS1apSapMme::ErabToBeReleasedIndication> erabToBeReleaseIndication)
{
    NS_LOG_FUNCTION (this << mmeUeS1Id << enbUeS1Id);
  uint64_t imsi = mmeUeS1Id;
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoMap.find (imsi);
  NS_ASSERT_MSG (it != m_ueInfoMap.end (), "could not find any UE with IMSI " << imsi);

  EpcS11SapSgw::DeleteBearerCommandMessage msg;
  // trick to avoid the need for allocating TEIDs on the S11 interface
  msg.teid = imsi;

  for (std::list<EpcS1apSapMme::ErabToBeReleasedIndication>::iterator bit = erabToBeReleaseIndication.begin (); bit != erabToBeReleaseIndication.end (); ++bit)
    {
      EpcS11SapSgw::BearerContextToBeRemoved bearerContext;
      bearerContext.epsBearerId =  bit->erabId;
      msg.bearerContextsToBeRemoved.push_back (bearerContext);
    }
  //Delete Bearer command towards epc-sgw-pgw-application
  m_s11SapSgw->DeleteBearerCommand (msg);
}


void
EpcMmeApplication::DoDeleteBearerRequest (EpcS11SapMme::DeleteBearerRequestMessage msg)
{
  NS_LOG_FUNCTION (this);
  uint64_t imsi = msg.teid;
  std::map<uint64_t, Ptr<UeInfo> >::iterator it = m_ueInfoMap.find (imsi);
  NS_ASSERT_MSG (it != m_ueInfoMap.end (), "could not find any UE with IMSI " << imsi);
  EpcS11SapSgw::DeleteBearerResponseMessage res;

  res.teid = imsi;

  for (std::list<EpcS11SapMme::BearerContextRemoved>::iterator bit = msg.bearerContextsRemoved.begin ();
       bit != msg.bearerContextsRemoved.end ();
       ++bit)
    {
      EpcS11SapSgw::BearerContextRemovedSgwPgw bearerContext;
      bearerContext.epsBearerId = bit->epsBearerId;
      res.bearerContextsRemoved.push_back (bearerContext);

      RemoveBearer (it->second, bearerContext.epsBearerId); //schedules function to erase, context of de-activated bearer
    }
  //schedules Delete Bearer Response towards epc-sgw-pgw-application
  m_s11SapSgw->DeleteBearerResponse (res);
}

void EpcMmeApplication::RemoveBearer (Ptr<UeInfo> ueInfo, uint8_t epsBearerId)
{
  NS_LOG_FUNCTION (this << epsBearerId);
  for (std::list<BearerInfo>::iterator bearerIterator = ueInfo->bearersToBeActivated.begin ();
       bearerIterator != ueInfo->bearersToBeActivated.end ();
       ++bearerIterator)
    {
      if (bearerIterator->bearerId == epsBearerId)
        {
          ueInfo->bearersToBeActivated.erase (bearerIterator);
          ueInfo->bearerCounter = ueInfo->bearerCounter - 1;
          break;
        }
    }
}


void
EpcMmeApplication::RecvFromS11Socket(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_ASSERT(socket == m_s11Socket);
    Ptr<Packet> packet = socket->Recv();
    GtpcHeader header;
    packet->PeekHeader(header);
    uint16_t msgType = header.GetMessageType();

    switch (msgType)
    {
    case GtpcHeader::CreateSessionResponse:
        DoRecvCreateSessionResponse(header, packet);
        break;

    case GtpcHeader::ModifyBearerResponse:
        DoRecvModifyBearerResponse(header, packet);
        break;

    case GtpcHeader::DeleteBearerRequest:
        DoRecvDeleteBearerRequest(header, packet);
        break;

    default:
        NS_FATAL_ERROR("GTP-C message not supported");
        break;
    }
}

void
EpcMmeApplication::DoRecvCreateSessionResponse(GtpcHeader& header, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << header);
    uint64_t imsi = header.GetTeid();
    NS_LOG_DEBUG("TEID/IMSI " << imsi);
    auto it = m_ueInfoMap.find(imsi);
    NS_ASSERT_MSG(it != m_ueInfoMap.end(), "could not find any UE with IMSI " << imsi);
    uint16_t cellId = it->second->cellId;
    uint16_t enbUeS1Id = it->second->enbUeS1Id;
    uint64_t mmeUeS1Id = it->second->mmeUeS1Id;
    NS_LOG_DEBUG("cellId " << cellId << " mmeUeS1Id " << mmeUeS1Id << " enbUeS1Id " << enbUeS1Id);
    auto jt = m_enbInfoMap.find(cellId);
    NS_ASSERT_MSG(jt != m_enbInfoMap.end(), "could not find any eNB with CellId " << cellId);

    GtpcCreateSessionResponseMessage msg;
    packet->RemoveHeader(msg);

    std::list<EpcS1apSapEnb::ErabToBeSetupItem> erabToBeSetupList;
    std::list<GtpcCreateSessionResponseMessage::BearerContextCreated> bearerContexts =
        msg.GetBearerContextsCreated();
    NS_LOG_DEBUG("BearerContextsCreated size = " << bearerContexts.size());
    for (auto& bearerContext : bearerContexts)
    {
        EpcS1apSapEnb::ErabToBeSetupItem erab;
        erab.erabId = bearerContext.epsBearerId;
        erab.erabLevelQosParameters = bearerContext.bearerLevelQos;
        erab.transportLayerAddress = bearerContext.fteid.addr; // SGW S1U address
        erab.sgwTeid = bearerContext.fteid.teid;
        NS_LOG_DEBUG("SGW " << erab.transportLayerAddress << " TEID " << erab.sgwTeid);
        erabToBeSetupList.push_back(erab);
    }

    NS_LOG_DEBUG("Send InitialContextSetupRequest to eNB " << jt->second->s1apSapEnb);
    jt->second->s1apSapEnb->InitialContextSetupRequest(mmeUeS1Id, enbUeS1Id, erabToBeSetupList);
}

void
EpcMmeApplication::DoRecvModifyBearerResponse(GtpcHeader& header, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << header);
    GtpcModifyBearerResponseMessage msg;
    packet->RemoveHeader(msg);
    NS_ASSERT(msg.GetCause() == GtpcModifyBearerResponseMessage::REQUEST_ACCEPTED);

    uint64_t imsi = header.GetTeid();
    NS_LOG_DEBUG("TEID/IMSI " << imsi);
    auto it = m_ueInfoMap.find(imsi);
    NS_ASSERT_MSG(it != m_ueInfoMap.end(), "could not find any UE with IMSI " << imsi);
    uint16_t cellId = it->second->cellId;
    uint16_t enbUeS1Id = it->second->enbUeS1Id;
    uint64_t mmeUeS1Id = it->second->mmeUeS1Id;
    NS_LOG_DEBUG("cellId " << cellId << " mmeUeS1Id " << mmeUeS1Id << " enbUeS1Id " << enbUeS1Id);
    std::list<EpcS1apSapEnb::ErabSwitchedInUplinkItem>
        erabToBeSwitchedInUplinkList; // unused for now
    auto jt = m_enbInfoMap.find(cellId);
    NS_ASSERT_MSG(jt != m_enbInfoMap.end(), "could not find any eNB with CellId " << cellId);

    NS_LOG_DEBUG("Send PathSwitchRequestAcknowledge to eNB " << jt->second->s1apSapEnb);
    jt->second->s1apSapEnb->PathSwitchRequestAcknowledge(enbUeS1Id,
                                                         mmeUeS1Id,
                                                         cellId,
                                                         erabToBeSwitchedInUplinkList);
}

void
EpcMmeApplication::DoRecvDeleteBearerRequest(GtpcHeader& header, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << header);
    uint64_t imsi = header.GetTeid();
    NS_LOG_DEBUG("TEID/IMSI " << imsi);
    auto it = m_ueInfoMap.find(imsi);
    NS_ASSERT_MSG(it != m_ueInfoMap.end(), "could not find any UE with IMSI " << imsi);

    GtpcDeleteBearerRequestMessage msg;
    packet->RemoveHeader(msg);

    GtpcDeleteBearerResponseMessage msgOut;

    std::list<uint8_t> epsBearerIds;
    for (auto& ebid : msg.GetEpsBearerIds())
    {
        epsBearerIds.push_back(ebid);
        /*
         * This condition is added to not remove bearer info at MME
         * when UE gets disconnected since the bearers are only added
         * at beginning of simulation at MME and if it is removed the
         * bearers cannot be activated again unless scheduled for
         * addition of the bearer during simulation
         *
         */
        if (it->second->cellId == 0)
        {
            RemoveBearer(it->second,
                         ebid); // schedules function to erase, context of de-activated bearer
        }
    }
    msgOut.SetEpsBearerIds(epsBearerIds);
    msgOut.SetTeid(imsi);
    msgOut.ComputeMessageLength();

    Ptr<Packet> packetOut = Create<Packet>();
    packetOut->AddHeader(msgOut);
    NS_LOG_DEBUG("Send DeleteBearerResponse to SGW " << m_sgwS11Addr);
    m_s11Socket->SendTo(packetOut, 0, InetSocketAddress(m_sgwS11Addr, m_gtpcUdpPort));
}

} // namespace ns3
