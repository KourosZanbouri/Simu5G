//
//                  Simu5G
//
// Authors: Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
//
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself,
// and cannot be removed from it.
//


package simu5g.nodes;

import simu5g.corenetwork.statsCollector.NrUeStatsCollector;
import simu5g.nodes.LteUe;
import inet.linklayer.contract.IEthernetInterface;
import inet.common.MessageDispatcher;

//
// This module implements a User Equipment (UE) for a 5G LTE network. It extends the ~LteUe module
// by implementing a ~NrNicUe as a Network Interface Card (NIC) module. This module can be
// used to model a device that can connect to either a 4G eNodeB or a 5G gNodeB, or both.
//
module NrUe extends LteUe
{
    parameters:
        cellularNic.typename = default("NrNicUe");

        int nrMasterId @mutable = default(0);
        int nrMacNodeId @mutable = default(0); // TODO: this is not a real parameter
        int nrMacCellId @mutable = default(0); // TODO: this is not a real parameter
        cellularNic.nrMac.collectorModule = hasRniSupport ? "^.^.nrUeCollector" : "";
        cellularNic.nrPhy.hasCollector = hasRniSupport;
        eth.typename = "EthernetInterface";

    gates:
        input nrRadioIn @directIn;     // connection to master
        inout ethg[numEthInterfaces] @labels(EtherFrame-conn) @allowUnconnected;

    submodules:
        //# UeStatsCollector - for MEC
        nrUeCollector: NrUeStatsCollector if hasRniSupport {
            @display("p=127,506;is=s");
        }

        //# message dispatcher for link layer
        bi: MessageDispatcher {
            parameters:
                @display("p=322,528;b=219.2,6.9,,,,1");
        }
        //# TSN Translator
        tsntt: TSNTT {
            @display("p=333.8,474.6;is=m");
        }


    connections allowunconnected:

        cellularNic.nrRadioIn <-- nrRadioIn;

        cellularNic.upperLayerOut --> nl.in++ if tp == 1; //TEMP HACK
        cellularNic.upperLayerIn <-- nl.out++ if tp == 1; // TEMP HACK

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

        for i=0..numEthInterfaces-1 {
            eth[i].phys <--> { @display("m=s"); } <--> ethg[i];
        }

        tsntt.tsnOut --> bi.in++;
        tsntt.tsnIn <-- bi.out++;
        tsntt.tsnOut --> nl.in++;
        tsntt.tsnIn <-- nl.out++;
}
