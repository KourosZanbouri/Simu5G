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

        bool useBuildingPenetrationHighLossModel = default(false);

        // InF geometry parameters (Added by Kouros Zanbouri)
        // d_clutter: average size/spacing of clutter objects [m]
        double d_clutter @unit(m) = default(10m);

        // clutter_density_r: volume density of clutter (0..1)
        //   0.0 = empty hall, 1.0 = fully blocked (use ~0.3-0.8 for factories)
        double clutter_density_r = default(0.4);

        // hClutter: height of clutter objects above floor [m]
        double hClutter @unit(m) = default(1.0m);

        // ceilingHeight: height from floor to ceiling [m]
        double ceilingHeight @unit(m) = default(10m);
}
