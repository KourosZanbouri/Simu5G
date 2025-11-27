//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//


package simu5g.nodes;

import simu5g.corenetwork.statsCollector.NrUeStatsCollector;
import simu5g.nodes.NrUe;
import inet.common.MessageDispatcher;

//
// This module implements a User Equipment (UE) for a 5G LTE network. It extends the ~Ue module
// by implementing a ~NRNicUe as a Network Interface Card (NIC) module. This module can be
// used to model a device that can connect to either a 4G eNodeB or a 5G gNodeB, or both.
//
module NRUeDrb extends NrUe
{
    parameters:
        cellularNic.typename = default("NRNicUeDrb");

}
