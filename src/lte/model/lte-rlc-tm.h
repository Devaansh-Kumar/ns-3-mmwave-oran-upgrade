/*
 * Copyright (c) 2011,2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 *         Nicola Baldo <nbaldo@cttc.es>
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *          Dual Connectivity functionalities
 */

#ifndef LTE_RLC_TM_H
#define LTE_RLC_TM_H

#include "lte-rlc.h"

#include <ns3/event-id.h>

#include <map>

namespace ns3
{

/**
 * LTE RLC Transparent Mode (TM), see 3GPP TS 36.322
 */
class LteRlcTm : public LteRlc
{
  public:
    LteRlcTm();
    ~LteRlcTm() override;
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     * RLC SAP
     *
     * \param p packet
     */
    void DoTransmitPdcpPdu(Ptr<Packet> p) override;

    /**
     * MAC SAP
     *
     * \param txOpParams the LteMacSapUser::TxOpportunityParameters
     */
    void DoNotifyTxOpportunity(LteMacSapUser::TxOpportunityParameters txOpParams) override;
    /**
     * Notify HARQ deliver failure
     */
    void DoNotifyHarqDeliveryFailure() override;
    void DoReceivePdu(LteMacSapUser::ReceivePduParameters rxPduParams) override;

    void DoSendMcPdcpSdu(EpcX2Sap::UeDataParams params) override;

  private:
    /// Expire RBS timer function
    void ExpireRbsTimer();
    /// Report buffer status
    void DoReportBufferStatus();

  private:
    uint32_t m_maxTxBufferSize; ///< maximum transmit buffer size
    uint32_t m_txBufferSize;    ///< transmit buffer size
    std::vector<TxPdu> m_txBuffer; ///< Transmission buffer

    EventId m_rbsTimer; ///< RBS timer
};

} // namespace ns3

#endif // LTE_RLC_TM_H
