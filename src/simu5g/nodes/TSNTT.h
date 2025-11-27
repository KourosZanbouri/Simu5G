/*
 * TSNTT.h
 *
 *  Created on: Oct 29, 2025
 *      Author: kouros
 */
#ifndef __SIMU5G_TSNTT_H_
#define __SIMU5G_TSNTT_H_

#include <omnetpp.h>
#include <map>
#include <string>
#include <vector>

#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/linklayer/common/MacAddress.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/linklayer/common/UserPriorityTag_m.h"
#include "inet/common/InitStages.h" // [FIX] Required for Init Stages

// Import generated message headers
#include "simu5g/common/QosTag_m.h"

namespace simu5g {

using namespace omnetpp;
using namespace inet;

class TSNTT : public cSimpleModule
{
  protected:
    // --- Configuration State ---
    std::map<int, int> pcpToQfiMap_;
    std::map<int, int> qfiToPcpMap_;
    std::map<L3Address, MacAddress> ipToMacMap_;

    // --- Interface State ---
    int tsnInterfaceId = -1;
    MacAddress tsnInterfaceMac;

    // --- Statistics ---
    simsignal_t ulPcpInSignal[8];
    simsignal_t dlQfiInSignal[64];
    simsignal_t droppedPacketsSignal;

  protected:
    // [FIX] Switch to Multi-Stage Initialization
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    virtual void handleMessage(cMessage *msg) override;

    // --- Core Processing ---
    void handleUplink(Packet *pkt);
    void handleDownlink(Packet *pkt);

    // --- Helpers ---
    int getPcpFromPacket(Packet *pkt);
    int getQfiFromPacket(Packet *pkt);
    MacAddress resolveDestMac(const L3Address& destIp);

    // --- Initialization Parsers ---
    void parseQosMapping(const char* mapStr);
    void parseMacMapping(const char* mapStr);
    std::vector<std::string> split(const std::string& str, char delimiter);
    void registerSignals();
};

} // namespace simu5g

#endif
