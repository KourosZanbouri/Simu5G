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

// The TSNTT module acts as a "Gateway" or "Shim" between Layer 2 (Ethernet) and Layer 3 (IPv4/5G).
simple TSNTT
{
    parameters:
        @display("i=block/layer");

        // QoS Mapping: Translates Ethernet Priority (PCP) to 5G QoS Flow (QFI)
        // Example: "7:80, 0:9" means PCP 7 -> QFI 80, PCP 0 -> QFI 9
        string qosMapping = default("");

        // Downlink NAT: If set, rewrites the Destination IP of incoming 5G packets
        // This allows the Server to target the UE, but the packet to be forwarded to the TSN Device.
        string downlinkDestIp = default("");

        // MAC Resolution: Static mapping of IP -> MAC for the wired link
        // Example: "192.168.1.2:0A-AA-00-00-00-04"
        string staticMacMapping = default("");

        // Interface Name: The name of the Ethernet interface in the UE (e.g., "tsnEth")
        string tsnInterfaceName = default("tsnEth");

        // Binder Module: Used to register the wired IP with the 5G Core
        string binderModule = default("binder");

        // Helper to find the interface table
        string interfaceTableModule = default("^.interfaceTable");

    gates:
        // Connections to the Wired World (Ethernet Interface)
        input tsnIn @labels(inet::EtherFrame);
        output tsnOut @labels(inet::EtherFrame);

        // Connections to the 5G Stack (Network Layer)
        input stackIn @labels(inet::Packet);
        output stackOut @labels(inet::Packet);
}