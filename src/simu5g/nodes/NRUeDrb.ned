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
    submodules:
        //# message dispatcher for SAP to link layer
        bi: MessageDispatcher {
            parameters:
                @display("p=329.2,506.9;b=219.2,6.9,,,,1");
        }
        tsntt: TSNTT {
            @display("p=333.8,474.6;is=m");
        }
	    connections allowunconnected:

        ipv4.ifIn <-- nl.out++ if hasIpv4;
        ipv4.ifOut --> nl.in++ if hasIpv4;


        cellularNic.upperLayerOut --> bi.in++;
        cellularNic.upperLayerIn <-- bi.out++;

        tsntt.stackOut --> bi.in++;
        tsntt.stackIn <-- bi.out++;
        tsntt.stackOut --> nl.in++;
        tsntt.stackIn <-- nl.out++;

        if numEthInterfaces > 0 {
    eth[0].upperLayerOut --> bi.in++;     // L2 Frame -> TSNTT
    eth[0].upperLayerIn <-- bi.out++;    // L2 Frame <- TSNTT
}
        tsntt.tsnOut --> bi.in++;
        tsntt.tsnIn <-- bi.out++;
        tsntt.tsnOut --> nl.in++;
        tsntt.tsnIn <-- nl.out++;

}
