#ifndef __SIMU5G_TSNTT_H_
#define __SIMU5G_TSNTT_H_

#include <omnetpp.h>
#include <map>
#include <string>
#include <vector>

// Essential INET Headers
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/linklayer/common/MacAddress.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/linklayer/common/UserPriorityTag_m.h"
#include "inet/common/Protocol.h"

#include "simu5g/common/QosTag_m.h"

namespace simu5g {

using namespace omnetpp;
using namespace inet;

/**
 * @brief TSN Translator (DS-TT) for Simu5G.
 *
 * Acts as a Gateway between a wired TSN interface (Ethernet) and the 5G Stack (IP/SDAP).
 * Functions:
 * 1. Initialization: Registers Wired IP with Binder.
 * 2. Registration: Manually registers ARP, IPv4, and EthernetMac with the Dispatcher.
 * 3. Uplink: Strips Ethernet, Maps PCP->QFI, Tags for IPv4/ARP.
 * 4. Downlink: NATs IP (optional), Maps QFI->PCP, Adds Ethernet Headers.
 */
class TSNTT : public cSimpleModule
{
  protected:
    // --- Configuration Mappings ---
    std::map<int, int> pcpToQfiMap_;      // Uplink: Ethernet Priority -> 5G QFI
    std::map<int, int> qfiToPcpMap_;      // Downlink: 5G QFI -> Ethernet Priority
    std::map<L3Address, MacAddress> ipToMacMap_; // Static ARP for the wired link

    // --- Interface State ---
    int tsnInterfaceId = -1;
    MacAddress tsnInterfaceMac;

    // --- Statistics ---
    simsignal_t ulPcpInSignal[8];
    simsignal_t dlQfiInSignal[64];
    simsignal_t droppedPacketsSignal;

  protected:
    // Lifecycle
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;

    // Core Processing Paths
    void handleUplink(Packet *pkt);   // Wired -> Wireless
    void handleDownlink(Packet *pkt); // Wireless -> Wired

    // [FIX] Helper to register protocols directly with the dispatcher
    void registerProtocolWithDispatcher(const Protocol& protocol, cGate* outGate);

    // Helpers
    int getPcpFromPacket(Packet *pkt);
    int getQfiFromPacket(Packet *pkt);
    MacAddress resolveDestMac(const L3Address& destIp);

    // Parsing
    void parseQosMapping(const char* mapStr);
    void parseMacMapping(const char* mapStr);
    std::vector<std::string> split(const std::string& str, char delimiter);

    void registerSignals();
};

} // namespace simu5g

#endif
