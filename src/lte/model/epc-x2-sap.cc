/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *          Dual Connectivity functionalities
 */

#include "epc-x2-sap.h"

namespace ns3
{

EpcX2Sap::~EpcX2Sap()
{
}

EpcX2Sap::ErabToBeSetupItem::ErabToBeSetupItem()
    : erabLevelQosParameters(EpsBearer(EpsBearer::GBR_CONV_VOICE))
{
}

EpcX2SapProvider::~EpcX2SapProvider()
{
}

EpcX2SapUser::~EpcX2SapUser()
{
}

EpcX2PdcpUser::~EpcX2PdcpUser()
{
}

EpcX2PdcpProvider::~EpcX2PdcpProvider()
{
}

EpcX2RlcUser::~EpcX2RlcUser()
{
}

EpcX2RlcProvider::~EpcX2RlcProvider()
{
}

} // namespace ns3
