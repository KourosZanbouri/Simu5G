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

package simu5g.stack.phy.channelmodel;


// NrChannelModel_3GPP38_901
//
// This channel model implements path loss, LOS probability, and shadowing according to
// the following specifications:
//     3GPP TR 38.901, "Study on channel model for frequencies from 0.5 to 100 GHz", v16.1.0, December 2019
//
// TODO: Implement Urban Macro only
//
simple NrChannelModel_3GPP38_901 extends NrChannelModel
{
    parameters:
        @class("NrChannelModel_3GPP38_901");

        bool useBuildingPenetrationLossHighLossModel = default(false);

        // Added By Kouros -  configurable InF parameters
        double d_clutter @unit(m) = default(10m);
        double clutter_density_r = default(0.9);
        double hClutter @unit(m) = default(1.0m);
        double ceilingHeight @unit(m) = default(10m);

        // Rate at which an Unblocked channel becomes Blocked (e.g., 2.0 events/sec)
        double lambda_U_B = default(2.0);
        // Rate at which a Blocked channel becomes Unblocked (e.g., 5.0 events/sec)
        // Note: A higher B_U rate means blockages clear quickly.
        double lambda_B_U = default(5.0);
        // The severe attenuation penalty applied when blocked
        double blockageAttenuation @unit(dB) = default(20dB);

        ////////////////////////////////////////////////////////
}
