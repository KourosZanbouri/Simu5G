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



simple TSNTT
{
    parameters:
        @display("i=block/layer");

        // The default value "^.interfaceTable" means "look in my parent module for 'interfaceTable'"
        string interfaceTableModule = default("^.interfaceTable");

        // Configuration Strings for your .ini file
        // Example: "6:9, 5:10" (PCP 6 -> QFI 9)
        string qosMapping = default("");

        // Example: "192.168.1.2:00-01-02-03-04-05"
        string staticMacMapping = default("");

        // Interface name to look for (usually "eth0")
        string tsnInterfaceName = default("eth0");

    gates:
        // To/From the local TSN network (e.g., eth[0])
        input tsnIn @labels(inet::EtherFrame);
        output tsnOut @labels(inet::EtherFrame);

        // To/From the 5G stack
        input stackIn @labels(inet::Packet);
        output stackOut @labels(inet::Packet);
}