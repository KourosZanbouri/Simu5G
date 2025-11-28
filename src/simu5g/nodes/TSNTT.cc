/**
 * @file TSNTT.cc
 * @brief TSN Translator (DS-TT) Implementation
 *
 * This module acts as a "Shim Layer" or Gateway between a Wired Ethernet Interface
 * and the Wireless 5G Protocol Stack. It performs:
 * 1. Protocol Translation (Ethernet <-> IP)
 * 2. QoS Mapping (802.1p PCP <-> 5G QFI)
 * 3. Network Address Translation (NAT) for Downlink
 */

#include "TSNTT.h"

// --- Essential INET Headers ---
// We need these to manipulate packet headers and tags
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#include "inet/linklayer/common/VlanTag_m.h"
#include "inet/linklayer/common/UserPriorityTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/common/ProtocolTag_m.h" // Defines DispatchProtocolInd/Req & PacketProtocolTag
#include "inet/common/Protocol.h"      // Defines Protocol::arp, Protocol::ipv4
#include "inet/linklayer/ieee8021q/Ieee8021qTagHeader_m.h"
#include "inet/linklayer/ethernet/common/EthernetMacHeader_m.h"
#include "inet/linklayer/ethernet/common/EthernetControlFrame_m.h"
#include "inet/linklayer/common/EtherType_m.h" // Defines ETHERTYPE_ARP, ETHERTYPE_IPv4

// --- Simu5G Headers ---
// Required to register with the Binder (Core Network) and apply 5G QoS tags
#include "simu5g/common/binder/Binder.h"
#include "simu5g/common/LteCommon.h"

namespace simu5g {

Define_Module(TSNTT);

/**
 * @brief Initialization Phase
 * Loads parameters and registers the TSN IP with the 5G Core (Binder).
 */
void TSNTT::initialize(int stage)
{
    // --- STAGE 0: Load Configuration ---
    if (stage == INITSTAGE_LOCAL) {
        const char* qosMapStr = par("qosMapping").stringValue();
        const char* macMapStr = par("staticMacMapping").stringValue();
        parseQosMapping(qosMapStr);
        parseMacMapping(macMapStr);
        registerSignals();
    }
    // --- STAGE 2: Bind to Interface & Register with Core ---
    else if (stage == INITSTAGE_NETWORK_LAYER) {
        // Find the physical Ethernet Interface (e.g., "tsnEth") by name
        const char* ifName = par("tsnInterfaceName").stringValue();
        IInterfaceTable *ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), getParentModule());
        NetworkInterface *ie = ift->findInterfaceByName(ifName);

        if (!ie) throw cRuntimeError("TSNTT: Interface '%s' not found.", ifName);

        tsnInterfaceId = ie->getInterfaceId();
        tsnInterfaceMac = ie->getMacAddress();
        EV_INFO << "TSNTT: Bound to " << ifName << " (ID:" << tsnInterfaceId << ")\n";

        // [BINDER REGISTRATION]
        // This is crucial for Downlink Routing. The 5G UPF needs to know which UE
        // is responsible for the TSN Device IP (e.g., 192.168.1.2).
        // We tell the Binder: "Map IP 192.168.1.2 to this UE's MacNodeId".
        const char* destIpStr = par("downlinkDestIp").stringValue();
        if (destIpStr && *destIpStr) {
            Binder* binder = getModuleFromPar<Binder>(par("binderModule"), getSystemModule());
            if (binder) {
                cModule* ueModule = getParentModule();
                if (ueModule && ueModule->hasPar("macNodeId")) {
                    // Extract integer Node ID from parameter
                    MacNodeId nodeId = MacNodeId(ueModule->par("macNodeId").intValue());
                    Ipv4Address extraIp(destIpStr);
                    binder->setMacNodeId(extraIp, nodeId);
                    EV_WARN << "TSNTT: Registered IP " << extraIp << " to NodeId " << nodeId << "\n";
                }
            }
        }
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

/**
 * @brief Main Message Handler
 * Routes traffic and handles initialization handshake.
 */
void TSNTT::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) { delete msg; return; }

    // --- [CRITICAL FIX] FORWARD CONTROL MESSAGES ---
    // The EthernetInterface sends non-packet registration messages at startup.
    // We MUST forward these to the Stack (nl) so IPv4/ARP can bind to the interface.
    // If we block these, we get "Unknown Packet" errors later.
    if (!msg->isPacket()) {
        EV_DETAIL << "TSNTT: Forwarding control message: " << msg->getName() << "\n";

        // Pass through transparently
        if (msg->arrivedOn("tsnIn")) send(msg, "stackOut");
        else if (msg->arrivedOn("stackIn")) send(msg, "tsnOut");
        else delete msg;

        return;
    }

    Packet *pkt = check_and_cast<Packet *>(msg);

    // Route Data Packets
    if (msg->arrivedOn("tsnIn")) handleUplink(pkt);
    else if (msg->arrivedOn("stackIn")) handleDownlink(pkt);
    else delete msg;
}

// ============================================================================
// UPLINK: Wired TSN (Ethernet) -> Wireless 5G (IP/SDAP)
// ============================================================================
void TSNTT::handleUplink(Packet *pkt)
{
    int pcp = 0;
    int etherType = -1;

    // 1. STRIP ETHERNET HEADER
    // The packet arrives as a raw Ethernet Frame [MacHeader][Payload].
    // We must peel off the L2 header to access the IP or ARP payload inside.
    if (auto macHeader = pkt->peekAtFront<EthernetMacHeader>()) {
        etherType = macHeader->getTypeOrLength();
        pkt->popAtFront<EthernetMacHeader>();
    }

    // 2. CLEAN UP OLD TAGS
    // The packet carries "Ethernet" protocol tags from the interface.
    // We must remove them, otherwise the Network Layer (nl) will reject the packet
    // because it expects "IPv4" or "ARP", not "Ethernet".
    if (pkt->findTag<PacketProtocolTag>()) pkt->removeTag<PacketProtocolTag>();
    if (pkt->findTag<DispatchProtocolReq>()) pkt->removeTag<DispatchProtocolReq>();
    if (pkt->findTag<DispatchProtocolInd>()) pkt->removeTag<DispatchProtocolInd>();

    // 3. ADD INTERFACE INDICATION
    // Tells the stack: "This packet arrived on Interface ID X"
    // This allows the stack to reply out the correct port.
    auto interfaceInd = pkt->getTagForUpdate<InterfaceInd>();
    if (!interfaceInd) interfaceInd = pkt->addTag<InterfaceInd>();
    interfaceInd->setInterfaceId(tsnInterfaceId);

    // 4. ADD DISPATCH INDICATION
    // [CRITICAL] This tag tells the MessageDispatcher: "Direction = UP" (Incoming).
    // Without this, the dispatcher assumes it's a request going DOWN and drops it.
    auto dispatchInd = pkt->addTag<DispatchProtocolInd>();

    // 5. ADD CONTENT PROTOCOL LABEL
    auto protocolTag = pkt->addTag<PacketProtocolTag>();

    // --- HANDLE ARP (Address Resolution Protocol) ---
    if (etherType == ETHERTYPE_ARP) {
        // Tag as ARP so the dispatcher sends it to the ARP module
        protocolTag->setProtocol(&Protocol::arp);
        dispatchInd->setProtocol(&Protocol::arp);

        EV_WARN << "TSNTT-UL: Sending ARP Indication UP to Stack.\n";
        send(pkt, "stackOut");
        return;
    }

    // --- HANDLE IPv4 ---
    if (etherType == ETHERTYPE_IPv4 || etherType == ETHERTYPE_8021Q_TAG) {
        // Check for 802.1Q VLAN Header (PCP is inside here)
        if (pkt->peekAtFront<Ieee8021qTagEpdHeader>()) {
            auto vlanHeader = pkt->popAtFront<Ieee8021qTagEpdHeader>();
            pcp = vlanHeader->getPcp(); // Extract Priority
        } else {
            pcp = getPcpFromPacket(pkt); // Fallback to tags
        }
    } else {
        EV_WARN << "TSNTT-UL: Unknown EtherType " << etherType << ". Treating as IPv4.\n";
    }

    // 6. MAP PCP -> QFI
    emit(ulPcpInSignal[pcp], 1);
    int qfi = 0;
    auto it = pcpToQfiMap_.find(pcp);
    if (it != pcpToQfiMap_.end()) qfi = it->second;

    // 7. ADD SIMU5G QoS TAG
    // The SDAP layer looks for this QosReq tag to map the packet to a DRB.
    auto qosReq = pkt->addTagIfAbsent<simu5g::QosReq>();
    qosReq->setQfi(qfi);

    // Final Tagging for IPv4
    protocolTag->setProtocol(&Protocol::ipv4);
    dispatchInd->setProtocol(&Protocol::ipv4);

    EV_WARN << "TSNTT-UL: Sending IPv4 Indication UP to Stack (QFI " << qfi << ").\n";
    send(pkt, "stackOut");
}

// ============================================================================
// DOWNLINK: Wireless 5G (IP/SDAP) -> Wired TSN (Ethernet)
// ============================================================================
void TSNTT::handleDownlink(Packet *pkt)
{
    int qfi = getQfiFromPacket(pkt);
    emit(dlQfiInSignal[qfi], 1);

    int pcp = 0;
    auto it = qfiToPcpMap_.find(qfi);
    if (it != qfiToPcpMap_.end()) pcp = it->second;

    const auto& ipHeader = pkt->peekAtFront<Ipv4Header>();
    if (!ipHeader) {
        // Pass-through non-IPv4 (e.g. ARP Reply coming from Stack)
        // We clean the DispatchInd tag so the EthernetInterface knows it's outgoing.
        if (pkt->findTag<DispatchProtocolInd>()) pkt->removeTag<DispatchProtocolInd>();
        send(pkt, "tsnOut");
        return;
    }

    // 1. NAT LOGIC (Rewrite Destination IP)
    // If configured, we rewrite the packet's destination to the real TSN Device IP.
    if (par("downlinkDestIp").stringValue()[0] != '\0') {
        const char* natIpStr = par("downlinkDestIp").stringValue();
        L3Address natIp(natIpStr);

        // Convert L3Address to Ipv4Address for comparison
        if (ipHeader->getDestAddress() != natIp.toIpv4()) {
            auto mutableIpHeader = pkt->removeAtFront<Ipv4Header>();
            mutableIpHeader->setDestAddress(natIp.toIpv4());
            mutableIpHeader->setCrc(0); // Force CRC recalculation
            pkt->insertAtFront(mutableIpHeader);
            EV_WARN << "TSNTT-DL: NAT Rewrote DestIP to " << natIp << "\n";
        }
    }

    // 2. RESOLVE DESTINATION MAC
    // We act as the gateway, so we must put the destination MAC on the L2 frame.
    MacAddress destMac = resolveDestMac(ipHeader->getDestAddress());
    if (destMac.isUnspecified()) {
        EV_WARN << "TSNTT-DL: No MAC for " << ipHeader->getDestAddress() << ". Dropped.\n";
        emit(droppedPacketsSignal, 1);
        delete pkt;
        return;
    }

    // 3. TAG FOR ETHERNET INTERFACE
    // We use "Request" tags to tell the EthernetInterface how to build the frame.

    // Force output interface
    pkt->addTagIfAbsent<InterfaceReq>()->setInterfaceId(tsnInterfaceId);

    // Set MAC Addresses
    auto macReq = pkt->addTagIfAbsent<MacAddressReq>();
    macReq->setDestAddress(destMac);
    macReq->setSrcAddress(tsnInterfaceMac);

    // Set VLAN ID and Priority (PCP)
    auto vlanReq = pkt->addTagIfAbsent<VlanReq>();
    vlanReq->setVlanId(1);
    auto upReq = pkt->addTagIfAbsent<UserPriorityReq>();
    upReq->setUserPriority(pcp);

    // Ensure Protocol is IPv4 so EthernetInterface uses EtherType 0x0800
    auto protocolTag = pkt->getTagForUpdate<PacketProtocolTag>();
    if (!protocolTag) protocolTag = pkt->addTag<PacketProtocolTag>();
    protocolTag->setProtocol(&Protocol::ipv4);

    EV_INFO << "TSNTT-DL: QFI=" << qfi << " -> PCP=" << pcp << " Dest=" << destMac << "\n";
    send(pkt, "tsnOut");
}

// ---------------------------------------------------------
// HELPERS
// ---------------------------------------------------------

// Extract PCP from UserPriorityTag (Standard)
int TSNTT::getPcpFromPacket(Packet *pkt) {
    if (auto upInd = pkt->findTag<UserPriorityInd>()) return upInd->getUserPriority();
    return 0;
}

// Extract QFI from Simu5G Tag
int TSNTT::getQfiFromPacket(Packet *pkt) {
    if (auto qosInd = pkt->findTag<simu5g::QosInd>()) return qosInd->getQfi();
    return 0;
}

// Look up MAC in static map
MacAddress TSNTT::resolveDestMac(const L3Address& destIp) {
    auto it = ipToMacMap_.find(destIp);
    return (it != ipToMacMap_.end()) ? it->second : MacAddress::UNSPECIFIED_ADDRESS;
}

// Parse "0:9, 7:80" string
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

// Parse "IP:MAC, IP:MAC" string
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
