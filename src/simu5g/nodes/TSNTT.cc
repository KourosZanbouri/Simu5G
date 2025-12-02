#include "TSNTT.h"

// Essential INET Headers
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#include "inet/linklayer/common/VlanTag_m.h"
#include "inet/linklayer/common/UserPriorityTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/Protocol.h"
#include "inet/linklayer/ieee8021q/Ieee8021qTagHeader_m.h"
#include "inet/linklayer/ethernet/common/EthernetMacHeader_m.h"
#include "inet/linklayer/ethernet/common/EthernetControlFrame_m.h"
#include "inet/common/packet/Message_m.h" // Needed for inet::Message casting

// Binder Headers
#include "simu5g/common/binder/Binder.h"
#include "simu5g/common/LteCommon.h"

namespace simu5g {

Define_Module(TSNTT);

void TSNTT::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL) {
        const char* qosMapStr = par("qosMapping").stringValue();
        const char* macMapStr = par("staticMacMapping").stringValue();
        parseQosMapping(qosMapStr);
        parseMacMapping(macMapStr);
        registerSignals();
    }
    else if (stage == INITSTAGE_NETWORK_LAYER) {
        const char* ifName = par("tsnInterfaceName").stringValue();
        IInterfaceTable *ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), getParentModule());
        NetworkInterface *ie = ift->findInterfaceByName(ifName);

        if (!ie) throw cRuntimeError("TSNTT: Interface '%s' not found.", ifName);

        tsnInterfaceId = ie->getInterfaceId();
        tsnInterfaceMac = ie->getMacAddress();
        EV_INFO << "TSNTT: Bound to " << ifName << " (ID:" << tsnInterfaceId << ")\n";

        // Note: We rely on Registration Hijack in handleMessage() for protocol binding.

        // Binder Registration (Downlink Routing)
        const char* destIpStr = par("downlinkDestIp").stringValue();
        if (destIpStr && *destIpStr) {
            Binder* binder = getModuleFromPar<Binder>(par("binderModule"), getSystemModule());
            if (binder) {
                cModule* ueModule = getParentModule();
                if (ueModule && ueModule->hasPar("macNodeId")) {
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

void TSNTT::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) { delete msg; return; }

    // --- [CRITICAL FIX] REGISTRATION HIJACK ---
    if (!msg->isPacket()) {

        if (msg->arrivedOn("tsnIn")) {

            // Cast to inet::Message to check for Tags
            auto controlMsg = dynamic_cast<inet::Message*>(msg);

            // Check specifically for DispatchProtocolReq (Used for registration in your INET version)
            if (controlMsg && controlMsg->findTag<DispatchProtocolReq>()) {
                EV_WARN << "TSNTT: Intercepted Registration. Cloning for ARP/IPv4/EthernetMac.\n";

                // Helper lambda to send registration clone
                auto sendReg = [&](const Protocol& p) {
                    inet::Message *clone = controlMsg->dup();
                    clone->getTagForUpdate<DispatchProtocolReq>()->setProtocol(&p);
                    send(clone, "stackOut");
                };

                // 1. Register IPv4
                sendReg(Protocol::ipv4);

                // 2. Register ARP
                sendReg(Protocol::arp);

                // 3. Register EthernetMac (Fixes Unknown protocol error for ARP reply)
                sendReg(Protocol::ethernetMac);

                // 4. Forward Original
                send(controlMsg, "stackOut");
                return;
            }
            send(msg, "stackOut");
        }
        else if (msg->arrivedOn("stackIn")) {
            send(msg, "tsnOut");
        }
        else {
            delete msg;
        }
        return;
    }

    Packet *pkt = check_and_cast<Packet *>(msg);

    if (msg->arrivedOn("tsnIn")) handleUplink(pkt);
    else if (msg->arrivedOn("stackIn")) handleDownlink(pkt);
    else delete msg;
}

// ---------------------------------------------------------
// UPLINK
// ---------------------------------------------------------
void TSNTT::handleUplink(Packet *pkt)
{
    int pcp = 0;
    int etherType = -1;

    // 1. Strip Ethernet Header
    if (auto macHeader = pkt->peekAtFront<EthernetMacHeader>()) {
        etherType = macHeader->getTypeOrLength();
        pkt->popAtFront<EthernetMacHeader>();
    }

    // 2. Clean Tags
    if (pkt->findTag<PacketProtocolTag>()) pkt->removeTag<PacketProtocolTag>();
    if (pkt->findTag<DispatchProtocolReq>()) pkt->removeTag<DispatchProtocolReq>();
    if (pkt->findTag<DispatchProtocolInd>()) pkt->removeTag<DispatchProtocolInd>();

    auto interfaceInd = pkt->getTagForUpdate<InterfaceInd>();
    if (!interfaceInd) interfaceInd = pkt->addTag<InterfaceInd>();
    interfaceInd->setInterfaceId(tsnInterfaceId);

    // [CRITICAL FIX] The "Magic Match" for Uplink
    // According to your MessageDispatcher code, if ServicePrimitive is missing (-1),
    // it checks if DispatchProtocolReq == PacketProtocolTag.
    // If they match, it treats it as SP_INDICATION (Incoming/Upward).
    auto dispatchReq = pkt->addTag<DispatchProtocolReq>();
    dispatchReq->setServicePrimitive(SP_INDICATION);

    auto protocolTag = pkt->addTag<PacketProtocolTag>();

    // 3. Handle ARP
    if (etherType == ETHERTYPE_ARP) {
        protocolTag->setProtocol(&Protocol::arp);
        dispatchReq->setProtocol(&Protocol::arp);

        EV_WARN << "TSNTT-UL: Forwarding ARP (Using Magic Match).\n";
        send(pkt, "stackOut");
        return;
    }

    // 4. Handle IPv4
    if (etherType == ETHERTYPE_IPv4 || etherType == ETHERTYPE_8021Q_TAG) {
        if (pkt->peekAtFront<Ieee8021qTagEpdHeader>()) {
            auto vlanHeader = pkt->popAtFront<Ieee8021qTagEpdHeader>();
            pcp = vlanHeader->getPcp();
        } else {
            pcp = getPcpFromPacket(pkt);
        }
    } else {
        EV_WARN << "TSNTT-UL: Unknown EtherType. Treating as IPv4.\n";
    }

    emit(ulPcpInSignal[pcp], 1);
    int qfi = 0;
    auto it = pcpToQfiMap_.find(pcp);
    if (it != pcpToQfiMap_.end()) qfi = it->second;

    auto qosReq = pkt->addTagIfAbsent<simu5g::QosReq>();
    qosReq->setQfi(qfi);

    // Magic Match for IPv4
    protocolTag->setProtocol(&Protocol::ipv4);
    dispatchReq->setProtocol(&Protocol::ipv4);

    EV_WARN << "TSNTT-UL: Forwarding IPv4 (QFI " << qfi << ").\n";
    send(pkt, "stackOut");
}

// ---------------------------------------------------------
// DOWNLINK
// ---------------------------------------------------------
void TSNTT::handleDownlink(Packet *pkt)
{
    int qfi = getQfiFromPacket(pkt);
    emit(dlQfiInSignal[qfi], 1);

    int pcp = 0;
    auto it = qfiToPcpMap_.find(qfi);
    if (it != qfiToPcpMap_.end()) pcp = it->second;

    const auto& ipHeader = pkt->peekAtFront<Ipv4Header>();
    if (!ipHeader) {
        // Clean dispatch tags
        if (pkt->findTag<DispatchProtocolReq>()) pkt->removeTag<DispatchProtocolReq>();
        send(pkt, "tsnOut");
        return;
    }

    // NAT
    if (par("downlinkDestIp").stringValue()[0] != '\0') {
        const char* natIpStr = par("downlinkDestIp").stringValue();
        L3Address natIp(natIpStr);

        if (ipHeader->getDestAddress() != natIp.toIpv4()) {
            auto mutableIpHeader = pkt->removeAtFront<Ipv4Header>();
            mutableIpHeader->setDestAddress(natIp.toIpv4());
            mutableIpHeader->setCrc(0);
            pkt->insertAtFront(mutableIpHeader);
            EV_WARN << "TSNTT-DL: NAT Rewrote DestIP to " << natIp << "\n";
        }
    }

    MacAddress destMac = resolveDestMac(ipHeader->getDestAddress());
    if (destMac.isUnspecified()) {
        EV_WARN << "TSNTT-DL: No MAC for " << ipHeader->getDestAddress() << ". Dropped.\n";
        emit(droppedPacketsSignal, 1);
        delete pkt;
        return;
    }

    pkt->addTagIfAbsent<InterfaceReq>()->setInterfaceId(tsnInterfaceId);

    auto macReq = pkt->addTagIfAbsent<MacAddressReq>();
    macReq->setDestAddress(destMac);
    macReq->setSrcAddress(tsnInterfaceMac);

    auto vlanReq = pkt->addTagIfAbsent<VlanReq>();
    vlanReq->setVlanId(1);
    auto upReq = pkt->addTagIfAbsent<UserPriorityReq>();
    upReq->setUserPriority(pcp);

    auto protocolTag = pkt->getTagForUpdate<PacketProtocolTag>();
    if (!protocolTag) protocolTag = pkt->addTag<PacketProtocolTag>();
    protocolTag->setProtocol(&Protocol::ipv4);

    EV_INFO << "TSNTT-DL: QFI=" << qfi << " -> PCP=" << pcp << " Dest=" << destMac << "\n";
    send(pkt, "tsnOut");
}

// --- Helpers ---

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
