/*
 * TSNTT.h
 *
 *  Created on: Oct 29, 2025
 *      Author: kouros
 */

#ifndef SIMU5G_NODES_TSNTT_H_
#define SIMU5G_NODES_TSNTT_H_

#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/linklayer/common/MacAddress.h"
#include "common/binder/Binder.h"

// Import the Simu5G QoS Tag
#include "simu5g/common/QosTag_m.h"

using namespace omnetpp;
using namespace inet;
using namespace simu5g;

class TSNTT : public cSimpleModule
{
  protected:
    // --- Parameters ---
    int localTsnInterfaceId = -1; // The interface ID of eth[0]
    MacAddress localTsnMac;       // The MAC address of our eth[0] port

    // --- System Modules ---
    Binder *binder = nullptr;

    // --- Statistics ---
    simsignal_t ulPcpInSignal[8];   // Per-PCP UL packet count (0-7)
    simsignal_t dlQfiInSignal[64];  // Per-QFI DL packet count (0-63)
    simsignal_t ulPacketDelay;
    simsignal_t dlPacketDelay;
    simsignal_t ulDroppedPackets;
    simsignal_t dlDroppedPackets;


  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

    /**
     * @brief Handles L2 frames from the wired TSN network (tsnIn).
     *
     * Reads PCP, maps to QFI, strips L2 headers, adds QosReq tag,
     * and sends to the 5G stack (stackOut).
     */
    virtual void handlePacketFromTsn(Packet *pkt);

    /**
     * @brief Handles L3 packets from the 5G stack (stackIn).
     *
     * Reads QFI , maps to PCP, finds dest MAC, adds L2 headers,
     * and sends to the wired TSN network (tsnOut).
     */
    virtual void handlePacketFromStack(Packet *pkt);

    /**
     * @brief Reads the QFI from a DL packet (from a QosInd tag).
     */
    virtual int getQfiFromPacket(Packet *pkt);

    /**
     * @brief Reads the PCP from an UL packet (from a VlanTag).
     */
    virtual int getPcpFromPacket(Packet *pkt);

    /**
     * @brief Maps a QFI (int) to a PCP (int) .
     */
    virtual int convertQfiToPcp(int qfi);

    /**
     * @brief Maps a PCP (int) to a QFI (int).
     */
    virtual int convertPcpToQfi(int pcp);

    /**
     * @brief Looks up the destination MAC address for a given IP address.
     */
    virtual MacAddress resolveMacAddress(const L3Address& destIp);

    virtual void registerSignals();
};


#endif /* SIMU5G_NODES_TSNTT_H_ */
