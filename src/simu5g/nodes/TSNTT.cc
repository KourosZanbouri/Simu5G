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
#include "TSNTT.h"

// --- Essential INET Includes ---
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#include "inet/linklayer/common/VlanTag_m.h"
#include "inet/linklayer/common/UserPriorityTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/common/ProtocolTag_m.h"

// [FIX] Correct paths for Headers
#include "inet/linklayer/ieee8021q/Ieee8021qTagHeader_m.h"
#include "inet/linklayer/ethernet/common/EthernetMacHeader_m.h" // <--- FIXED PATH

namespace simu5g {

Define_Module(TSNTT);

void TSNTT::initialize(int stage)
{
    // Stage 0: Configuration
    if (stage == INITSTAGE_LOCAL) {
        const char* qosMapStr = par("qosMapping").stringValue();
        const char* macMapStr = par("staticMacMapping").stringValue();
        parseQosMapping(qosMapStr);
        parseMacMapping(macMapStr);
        registerSignals();
    }
    // Stage 2: Binding to Interface
    else if (stage == INITSTAGE_NETWORK_LAYER) {
        const char* ifName = par("tsnInterfaceName").stringValue();
        IInterfaceTable *ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), getParentModule());
        NetworkInterface *ie = ift->findInterfaceByName(ifName);

        if (!ie) throw cRuntimeError("TSNTT: Interface '%s' not found.", ifName);

        tsnInterfaceId = ie->getInterfaceId();
        tsnInterfaceMac = ie->getMacAddress();
        EV_INFO << "TSNTT: Bound to " << ifName << " (ID:" << tsnInterfaceId << ")\n";
    }
}

void TSNTT::registerSignals()
{
    for(int i=0; i<8; i++)
        ulPcpInSignal[i] = registerSignal(("ulPcp" + std::to_string(i)).c_str());
    for(int i=0; i<64; i++)
        dlQfiInSignal[i] = registerSignal(("dlQfi" + std::to_string(i)).c_str());
    droppedPacketsSignal = registerSignal("droppedPackets");
}

void TSNTT::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) { delete msg; return; }
    Packet *pkt = check_and_cast<Packet *>(msg);

    if (msg->arrivedOn("tsnIn")) handleUplink(pkt);
    else if (msg->arrivedOn("stackIn")) handleDownlink(pkt);
    else delete msg;
}

// ---------------------------------------------------------
// UPLINK: Wired TSN -> Wireless 5G
// ---------------------------------------------------------
void TSNTT::handleUplink(Packet *pkt)
{
    // [FIX] 1. Strip Ethernet MAC Header
    // We look for EthernetMacHeader to remove the L2 wrapping
    if (pkt->peekAtFront<EthernetMacHeader>()) {
        pkt->popAtFront<EthernetMacHeader>();
    }

    // 2. Handle VLAN Header / PCP
    int pcp = 0;

    // Check if there is a VLAN Tag (802.1Q)
    if (pkt->peekAtFront<Ieee8021qTagEpdHeader>()) {
        auto vlanHeader = pkt->popAtFront<Ieee8021qTagEpdHeader>();
        pcp = vlanHeader->getPcp();
    }
    else {
        // Check Metadata Tags
        pcp = getPcpFromPacket(pkt);
    }

    // 3. Map PCP -> QFI
    emit(ulPcpInSignal[pcp], 1);
    int qfi = 0;
    auto it = pcpToQfiMap_.find(pcp);
    if (it != pcpToQfiMap_.end()) qfi = it->second;

    // 4. Tag for SDAP (5G QoS)
    auto qosReq = pkt->addTagIfAbsent<simu5g::QosReq>();
    qosReq->setQfi(qfi);

    EV_INFO << "TSNTT-UL: Decapsulated. PCP=" << pcp << " -> QFI=" << qfi << ". Forwarding to 5G Stack.\n";
    send(pkt, "stackOut");
}

// ---------------------------------------------------------
// DOWNLINK: Wireless 5G -> Wired TSN
// ---------------------------------------------------------
void TSNTT::handleDownlink(Packet *pkt)
{
    int qfi = getQfiFromPacket(pkt);
    emit(dlQfiInSignal[qfi], 1);

    int pcp = 0;
    auto it = qfiToPcpMap_.find(qfi);
    if (it != qfiToPcpMap_.end()) pcp = it->second;

    const auto& ipHeader = pkt->peekAtFront<Ipv4Header>();
    if (!ipHeader) { emit(droppedPacketsSignal, 1); delete pkt; return; }

    MacAddress destMac = resolveDestMac(ipHeader->getDestAddress());
    if (destMac.isUnspecified()) {
        EV_WARN << "TSNTT-DL: No MAC found for IP " << ipHeader->getDestAddress() << ". Packet Dropped.\n";
        emit(droppedPacketsSignal, 1);
        delete pkt;
        return;
    }

    // Add Requests for EthernetInterface to build the L2 frame
    pkt->addTagIfAbsent<InterfaceReq>()->setInterfaceId(tsnInterfaceId);

    auto macReq = pkt->addTagIfAbsent<MacAddressReq>();
    macReq->setDestAddress(destMac);
    macReq->setSrcAddress(tsnInterfaceMac);

    auto vlanReq = pkt->addTagIfAbsent<VlanReq>();
    vlanReq->setVlanId(1);

    auto upReq = pkt->addTagIfAbsent<UserPriorityReq>();
    upReq->setUserPriority(pcp);

    pkt->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::ipv4);

    EV_INFO << "TSNTT-DL: QFI=" << qfi << " -> PCP=" << pcp << " Dest=" << destMac << ". Forwarding to Wire.\n";
    send(pkt, "tsnOut");
}

// ---------------------------------------------------------
// HELPERS
// ---------------------------------------------------------

int TSNTT::getPcpFromPacket(Packet *pkt) {
    if (auto upInd = pkt->findTag<UserPriorityInd>()) return upInd->getUserPriority();
    return 0;
}

int TSNTT::getQfiFromPacket(Packet *pkt) {
    if (auto qosInd = pkt->findTag<simu5g::QosInd>()) return qosInd->getQfi();
    return 0;
}

MacAddress TSNTT::resolveDestMac(const L3Address& destIp) {
    auto it = ipToMacMap_.find(destIp);
    return (it != ipToMacMap_.end()) ? it->second : MacAddress::UNSPECIFIED_ADDRESS;
}

void TSNTT::parseQosMapping(const char* mapStr) {
    std::string s(mapStr);
    auto tokens = split(s, ',');
    for (auto& token : tokens) {
        auto pair = split(token, ':');
        if (pair.size() == 2) {
            pcpToQfiMap_[std::stoi(pair[0])] = std::stoi(pair[1]);
            qfiToPcpMap_[std::stoi(pair[1])] = std::stoi(pair[0]);
        }
    }
}

void TSNTT::parseMacMapping(const char* mapStr) {
    std::string s(mapStr);
    auto tokens = split(s, ',');
    for (auto& token : tokens) {
        auto pair = split(token, ':');
        if (pair.size() == 2) ipToMacMap_[L3Address(pair[0].c_str())] = MacAddress(pair[1].c_str());
    }
}

std::vector<std::string> TSNTT::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        size_t first = token.find_first_not_of(' ');
        if (std::string::npos == first) continue;
        size_t last = token.find_last_not_of(' ');
        tokens.push_back(token.substr(first, (last - first + 1)));
    }
    return tokens;
}

} // namespace
