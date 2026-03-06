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

#include "simu5g/stack/phy/channelmodel/NrChannelModel_3GPP38_901.h"

namespace simu5g {

Define_Module(NrChannelModel_3GPP38_901);

void NrChannelModel_3GPP38_901::initialize(int stage)
{
    NrChannelModel::initialize(stage);
    if (inside_building_)
        useBuildingPenetrationHighLossModel_ = par("useBuildingPenetrationHighLossModel").boolValue();

    // ADDED BY KOUROS
    // Initialize configurable InF parameters
    d_clutter_ = par("d_clutter").doubleValue();
    clutter_density_r_ = par("clutter_density_r").doubleValue();
    hClutter_ = par("hClutter").doubleValue();
    ceilingHeight_ = par("ceilingHeight").doubleValue();
    // Override correlation distance for InF scenarios.
    // TR 38.901 Table 7.5-6 Part 3 "Channel model parameters for InF" specifies 10 m for all InF sub-scenarios.
    // The parent class reads this from NED (default ~50 m for outdoor),
    // so we override it here after the parent has initialized.
    if (scenario_ == INDOOR_FACTORY_SL ||
        scenario_ == INDOOR_FACTORY_DL ||
        scenario_ == INDOOR_FACTORY_SH ||
        scenario_ == INDOOR_FACTORY_DH ||
        scenario_ == INDOOR_FACTORY_HH)
    {
        correlationDistance_ = 10.0; // meters, TR 38.901 Table 7.5-6
        EV << "NrChannelModel_3GPP38_901: InF scenario detected, "
           << "overriding correlationDistance to 10.0 m" << endl;
    }    
}



void NrChannelModel_3GPP38_901::computeLosProbability(double d, MacNodeId nodeId)
{
    double p = 0;
    if (!dynamicLos_) {
        losMap_[nodeId] = fixedLos_;
        return;
    }

    switch (scenario_) {
        case URBAN_MICROCELL:
            if (d <= 18.0)
                p = 1.0;
            else
                p = (18.0 / d) + exp(-1 * d / 36.0) * (1.0 - (18.0 / d));
            break;
        case URBAN_MACROCELL:
            if (d <= 18.0)
                p = 1.0;
            else {
                double C = (hUe_ <= 13.0) ? 0 : pow((hUe_ - 13.0) / 10.0, 1.5);
                p = ((18 / d) + exp(-1 * d / 63) * (1 - (18 / d))) * (1 + C * (5.0 / 4.0) * pow(d / 100.0, 3) * exp(-1 * d / 150.0));
            }
            break;
        case RURAL_MACROCELL:
            if (d <= 10)
                p = 1;
            else
                p = exp(-1 * (d - 10.0) / 1000);
            break;
        case INDOOR_HOTSPOT:
            if (d <= 5.0)
                p = 1;
            else if (d <= 49.0)
                p = exp(-1 * (d - 5.0) / 70.8);
            else
                p = exp(-1 * (d - 49.0) / 211.7);
            break;

            // Added by Kouros
        case INDOOR_FACTORY_SL:
        case INDOOR_FACTORY_DL:
        case INDOOR_FACTORY_SH:
        case INDOOR_FACTORY_DH:
        case INDOOR_FACTORY_HH:
        {
            double k_subsce = 0;
            if (scenario_ == INDOOR_FACTORY_HH) {
                p = 1.0;
            } else {
                if (scenario_ == INDOOR_FACTORY_SL || scenario_ == INDOOR_FACTORY_DL)
                {
                    // Formula for low BS height
                    k_subsce = -d_clutter_ / log(1 - clutter_density_r_);
                }
                else // High BS height (SH, DH)
                {
                    // Formula for high BS height
                    k_subsce = -(d_clutter_ * (hNodeB_ - hUe_)) / (log(1 - clutter_density_r_) * (hClutter_ - hUe_));
                }
                if (k_subsce > 0)
                    p = exp(-d / k_subsce);
            }
            break;
        }

        default:
            NrChannelModel::computeLosProbability(d, nodeId);
            return;
    }

    double random = uniform(0.0, 1.0);
    if (random <= p)
        losMap_[nodeId] = true;
    else
        losMap_[nodeId] = false;
}

double NrChannelModel_3GPP38_901::computePenetrationLoss(double threeDimDistance)
{
    double penetrationLoss = 0.0;
    double inside_distance = (inside_distance_ < threeDimDistance) ? inside_distance_ : threeDimDistance;
    double pLoss_in = 0.5 * inside_distance;
    double pLoss_tw = 0.0;
    if (carrierFrequencyGHz_ <= 6.0)
        pLoss_tw = 20.0;
    else {
        double Lglass = 2 + 0.2 * carrierFrequencyGHz_;
        double LiirGlass = 23 + 0.3 * carrierFrequencyGHz_;
        double Lconcrete = 5 + 4 * carrierFrequencyGHz_;

        if (useBuildingPenetrationHighLossModel_)
            pLoss_tw = 5 - 10 * log10(0.7 * pow(10, (-Lglass / 10)) + 0.3 * pow(10, (-Lconcrete / 10))) + normal(0.0, 4.4);
        else
            pLoss_tw = 5 - 10 * log10(0.3 * pow(10, (-LiirGlass / 10)) + 0.7 * pow(10, (-Lconcrete / 10))) + normal(0.0, 6.5);
    }
    penetrationLoss = pLoss_tw + pLoss_in;
    return penetrationLoss;
}

double NrChannelModel_3GPP38_901::computePathLoss(double threeDimDistance, double twoDimDistance, bool los)
{
    // Compute attenuation based on selected scenario and based on LOS or NLOS
    double pathLoss = 0;
    switch (scenario_) {
        case INDOOR_HOTSPOT:
            pathLoss = computeIndoor(threeDimDistance, twoDimDistance, los);
            break;
        case URBAN_MICROCELL:
            pathLoss = computeUrbanMicro(threeDimDistance, twoDimDistance, los);
            break;
        case URBAN_MACROCELL:
            pathLoss = computeUrbanMacro(threeDimDistance, twoDimDistance, los);
            break;
        case RURAL_MACROCELL:
            pathLoss = computeRuralMacro(threeDimDistance, twoDimDistance, los);
            break;
        // Added By Kouros
        case INDOOR_FACTORY_SL:
        case INDOOR_FACTORY_DL:
        case INDOOR_FACTORY_SH:
        case INDOOR_FACTORY_DH:
            pathLoss = computeIndoorFactory(threeDimDistance, los);
            break;
        default:
            return NrChannelModel::computePathLoss(twoDimDistance, 0.0, los);
    }
    return pathLoss;
}

double NrChannelModel_3GPP38_901::computeUrbanMacro(double threeDimDistance, double twoDimDistance, bool los)
{
    if (twoDimDistance < 10)
        twoDimDistance = 10;

    if (threeDimDistance < 10)
        threeDimDistance = 10;

    if (twoDimDistance > 5000 && !tolerateMaxDistViolation_)
        throw cRuntimeError("Error: Urban macrocell path loss model is valid for d<5000m only");

    // Compute penetration loss
    double penetrationLoss = 0.0;
    if (inside_building_)
        penetrationLoss = computePenetrationLoss(threeDimDistance);

    // Compute break-point distance
    double hEnvir = 0.0;
    double G_2d = (twoDimDistance < 18.0) ? 0 : (5.0 / 4.0) * pow(twoDimDistance / 100.0, 3) * exp(twoDimDistance / 150);
    double C = (hUe_ < 13.0) ? 0 : pow(((hUe_ - 13.0) / 10.0), 1.5) * G_2d;
    double prob = 1.0 / (1.0 + C);
    if (uniform(0.0, 1.0) < prob)
        hEnvir = 1.0;
    else {
        double bound = hUe_ - 1.5;
        std::vector<double> hVec;
        for (double h = 12; h < bound; h += 3)
            hVec.push_back(h);
        hVec.push_back(bound);
        hEnvir = hVec[intuniform(0, hVec.size())];
    }
    double hNodeB = hNodeB_ - hEnvir;
    double hUe = hUe_ - hEnvir;
    double dbp = 4 * hNodeB * hUe * (carrierFrequencyHz_  / SPEED_OF_LIGHT);

    // Compute LOS path loss
    double pLoss_los = 0.0;
    if (twoDimDistance < dbp)
        pLoss_los = 28 + 22 * log10(threeDimDistance) + 20 * log10CarrierFrequencyGHz_;
    else
        pLoss_los = 28 + 40 * log10(threeDimDistance) + 20 * log10CarrierFrequencyGHz_ - 9 * log10((dbp * dbp + (hNodeB_ - hUe_) * (hNodeB_ - hUe_)));
    pLoss_los += penetrationLoss;

    double pLoss = 0.0;
    if (los)
        pLoss = pLoss_los;
    else {
        // Compute NLOS path loss
        double pLoss_nlos = 13.54 + 39.08 * log10(threeDimDistance) + 20 * log10CarrierFrequencyGHz_ - 0.6 * (hUe_ - 1.5);
        pLoss_nlos += penetrationLoss;

        pLoss = (pLoss_los > pLoss_nlos) ? pLoss_los : pLoss_nlos;
    }
    return pLoss;
}

double NrChannelModel_3GPP38_901::computeUrbanMicro(double threeDimDistance, double twoDimDistance, bool los)
{
    if (twoDimDistance < 10)
        twoDimDistance = 10;

    if (twoDimDistance > 5000 && !tolerateMaxDistViolation_)
        throw cRuntimeError("Error: Urban microcell path loss model is valid for d<5000m only");

    // Compute penetration loss
    double penetrationLoss = 0.0;
    if (inside_building_)
        penetrationLoss = computePenetrationLoss(threeDimDistance);

    // Compute break-point distance
    double hEnvir = 1.0;
    double hNodeB = hNodeB_ - hEnvir;
    double hUe = hUe_ - hEnvir;
    double dbp = 4 * hNodeB * hUe * (carrierFrequencyHz_  / SPEED_OF_LIGHT);

    // Compute LOS path loss
    double pLoss_los = 0.0;
    if (twoDimDistance < dbp)
        pLoss_los = 32.4 + 21 * log10(threeDimDistance) + 20 * log10CarrierFrequencyGHz_;
    else
        pLoss_los = 32.4 + 40 * log10(threeDimDistance) + 20 * log10CarrierFrequencyGHz_ - 9.5 * log10((dbp * dbp + (hNodeB_ - hUe_) * (hNodeB_ - hUe_)));
    pLoss_los += penetrationLoss;

    double pLoss = 0.0;
    if (los)
        pLoss = pLoss_los;
    else {
        // Compute NLOS path loss
        double pLoss_nlos = 35.3 * log10(threeDimDistance) + 22.4 + 21.3 * log10CarrierFrequencyGHz_ - 0.3 * (hUe_ - 1.5);
        pLoss_nlos += penetrationLoss;

        pLoss = (pLoss_los > pLoss_nlos) ? pLoss_los : pLoss_nlos;
    }
    return pLoss;
}

double NrChannelModel_3GPP38_901::computeRuralMacro(double threeDimDistance, double twoDimDistance, bool los)
{
    if (twoDimDistance < 10)
        twoDimDistance = 10;

    if (los) {
        if (twoDimDistance > 10000 && !tolerateMaxDistViolation_)
            throw cRuntimeError("Error: LOS rural macrocell path loss model is valid for d<10000m only");
    }
    else {
        if (twoDimDistance > 5000 && !tolerateMaxDistViolation_)
            throw cRuntimeError("Error: NLOS rural macrocell path loss model is valid for d<5000m only");
    }
    // Compute penetration loss
    double penetrationLoss = 0.0;
    if (inside_building_) {
        penetrationLoss = computePenetrationLoss(threeDimDistance);
    }

    // Compute break-point distance
    double dbp = 2 * M_PI * hNodeB_ * hUe_ * (carrierFrequencyHz_  / SPEED_OF_LIGHT);

    double h = 5.0; // Average building height
    double A = 0.03 * pow(h, 1.72);
    double B = 0.044 * pow(h, 1.72);
    double min1 = (A < 10) ? A : 10;
    double min2 = (B < 14.77) ? B : 14.77;
    double pLoss_los = 0.0;
    if (twoDimDistance < dbp) {
        pLoss_los = 20 * log10(40 * M_PI * threeDimDistance * (carrierFrequencyHz_ / 3.0)) + min1 * log10(threeDimDistance) - min2 + 0.002 * log10(h) * threeDimDistance;
    }
    else {
        pLoss_los = 20 * log10(40 * M_PI * dbp * (carrierFrequencyHz_ / 3.0)) + min1 * log10(dbp) - min2 + 0.002 * log10(h) * dbp
            + 40 * log10(threeDimDistance / dbp);
    }
    pLoss_los += penetrationLoss;

    double pLoss = 0.0;
    if (los)
        pLoss = pLoss_los;
    else {
        double W = 20.0;  // Average street width
        double pLoss_nlos = 161.04 - 7.1 * log10(W) + 7.5 * log10(h) - (24.37 - 3.7 * pow(h / hNodeB_, 2)) * log10(hNodeB_)
            + (43.42 - 3.1 * log10(hNodeB_)) * (log10(threeDimDistance) - 3.0) + 20 * log10CarrierFrequencyGHz_
            - (3.2 * pow((log10(11.75 * hUe_)), 2) - 4.97);
        pLoss_nlos += penetrationLoss;
        pLoss = (pLoss_los > pLoss_nlos) ? pLoss_los : pLoss_nlos;
    }

    return pLoss;
}

double NrChannelModel_3GPP38_901::computeIndoor(double threeDimDistance, double twoDimDistance, bool los)
{
    if (threeDimDistance < 1)
        threeDimDistance = 1;

    if (threeDimDistance > 150 && !tolerateMaxDistViolation_)
        throw cRuntimeError("Error: Indoor hotspot path loss model is valid for d<150m only");

    // Compute penetration loss
    double penetrationLoss = 0.0;
    if (inside_building_)
        penetrationLoss = computePenetrationLoss(threeDimDistance);

    // Compute LOS path loss
    double pLoss_los = 32.4 + 17.3 * log10(threeDimDistance) + 20 * log10CarrierFrequencyGHz_;
    pLoss_los += penetrationLoss;

    double pLoss = 0.0;
    if (los)
        pLoss = pLoss_los;
    else {
        // Compute NLOS path loss
        double pLoss_nlos = 38.3 * log10(threeDimDistance) + 17.3 + 24.9 * log10CarrierFrequencyGHz_;
        pLoss_nlos += penetrationLoss;

        pLoss = (pLoss_los > pLoss_nlos) ? pLoss_los : pLoss_nlos;
    }
    return pLoss;
}


double NrChannelModel_3GPP38_901::computeIndoorFactory(double threeDimDistance, bool los)
{
    if (threeDimDistance < 1)
        threeDimDistance = 1;

    // InF model is valid for d_3D up to 600m
    if (threeDimDistance > 600.0 && !tolerateMaxDistViolation_)
        throw cRuntimeError(
            "NrChannelModel_3GPP38_901::computeIndoorFactory - "
            "Indoor Factory path loss model valid for d<600m. Received: %g",
            threeDimDistance);

    // LOS path loss - identical for all InF sub-scenarios
    // TR 38.901 Table 7.4.1-1, InF-LOS
    double pLoss_los = 31.84 + 21.5 * log10(threeDimDistance) + 19.0 * log10CarrierFrequencyGHz_;
    
    
    if (los)
        pLoss = pLoss_los;

    // NLOS path loss
    double pLoss_nlos = 0.0;

    switch (scenario_) {
        case INDOOR_FACTORY_SL:
            pLoss_nlos = 33.0 + 25.5 * log10(threeDimDistance) + 20.0 * log10CarrierFrequencyGHz_;
            break;
        case INDOOR_FACTORY_DL:
            pLoss_nlos = 18.6 + 35.7 * log10(threeDimDistance) + 20.0 * log10CarrierFrequencyGHz_;
            break;
        case INDOOR_FACTORY_SH:
            pLoss_nlos = 32.4 + 23.0 * log10(threeDimDistance) + 20.0 * log10CarrierFrequencyGHz_;
            break;
        case INDOOR_FACTORY_DH:
            pLoss_nlos = 33.63 + 21.9 * log10(threeDimDistance) + 20.0 * log10CarrierFrequencyGHz_;
            break;
        default:
            throw cRuntimeError(
                "NrChannelModel_3GPP38_901::computeIndoorFactory - "
                "Unhandled InF NLOS sub-scenario");
            }
    // The clutter density parameter r (clutter_density_r_) is used in
    // computeLosProbability() to determine whether a path is blocked, but
    // once in NLOS the degree of obstruction must also increase the path
    // loss. This term captures the mean excess attenuation caused by the
    // factory clutter field.
    //
    // L_clutter = -17.3 * log10(1 - r)   [dB]
    if (clutter_density_r_ > 0.0 && clutter_density_r_ < 1.0) {
        double L_clutter = -17.3 * log10(1.0 - clutter_density_r_);
        pLoss_nlos += L_clutter;

        EV << "NrChannelModel_3GPP38_901::computeIndoorFactory - "
           << "clutter loss = " << L_clutter << " dB "
           << "(r=" << clutter_density_r_ << ")" << endl;
    }

    // Per TR 38.901: final NLOS PL = max(PL_LOS, PL_NLOS)
    return std::max(pLoss_los, pLoss_nlos);    
    
}


double NrChannelModel_3GPP38_901::getStdDev(bool dist, MacNodeId nodeId)
{
    switch (scenario_) {
        case URBAN_MICROCELL:
            if (losMap_[nodeId])
                return 4.;
            else
                return 7.82;
        case INDOOR_HOTSPOT:
            if (losMap_[nodeId])
                return 3.;
            else
                return 8.03;
        case URBAN_MACROCELL:
            if (losMap_[nodeId])
                return 4.;
            else
                return 6.;
        case RURAL_MACROCELL:
            if (losMap_[nodeId]) {
                if (dist)
                    return 4.;
                else
                    return 6.;
            }
            else
                return 8.;

            // Added By Kouros
        case INDOOR_FACTORY_HH:
        // HH is always LOS (p = 1.0 in computeLosProbability)
        // ETSI TR 138 901 V19.2.0 Table 7.4.1-1 : LOS shadow fading std dev = 4.3 dB
            return 4.3;
        
        case INDOOR_FACTORY_SL:
        case INDOOR_FACTORY_DL:
        case INDOOR_FACTORY_SH:
        case INDOOR_FACTORY_DH:
            if (losMap_[nodeId]) {
                // For all InF scenarios, the LOS shadow fading std is 4.3 dB
                return 4.3;
            } else {
                // For NLOS, the value depends on the specific InF sub-scenario
                switch (scenario_) {
                    case INDOOR_FACTORY_SL:
                        return 5.7;
                    case INDOOR_FACTORY_DL:
                        return 7.2;
                    case INDOOR_FACTORY_SH:
                        return 5.9;
                    case INDOOR_FACTORY_DH:
                        return 4.0; 
                    default:
                        throw cRuntimeError("Unhandled InF NLOS scenario in getStdDev");
                }
            }
            break;
        default:
            throw cRuntimeError("Wrong path-loss scenario value %d", scenario_);
    }
    return 0.0;
}

double NrChannelModel_3GPP38_901::computeShadowing(double sqrDistance, MacNodeId nodeId, double speed, bool cqiDl)
{
    ShadowFadingMap *actualShadowingMap;

    if (cqiDl) // If we are computing a DL CQI we need the Shadowing Map stored on the UE side
        actualShadowingMap = obtainShadowingMap(nodeId);
    else
        actualShadowingMap = &lastComputedSF_;

    if (actualShadowingMap == nullptr)
        throw cRuntimeError("NrChannelModel_3GPP38_901::computeShadowing - actualShadowingMap not found (nullptr)");

    double mean = 0;
    double dbp = 0.0;
    // Get std deviation according to los/nlos and selected scenario

    double stdDev = getStdDev(sqrDistance < dbp, nodeId);
    double time = 0;
    double space = 0;
    double att;

    // If direction is DOWNLINK it means that this module is located in UE stack than
    // the Move object associated with the UE is myMove_ variable
    // If direction is UPLINK it means that this module is located in UE stack than
    // the Move object associated with the UE is move variable

    // If shadowing for current user has never been computed
    if (actualShadowingMap->find(nodeId) == actualShadowingMap->end()) {
        // Get the log normal shadowing with std deviation stdDev
        att = normal(mean, stdDev);

        // Store the shadowing attenuation for this user and the temporal mark
        std::pair<simtime_t, double> tmp(NOW, att);
        (*actualShadowingMap)[nodeId] = tmp;

        // If the shadowing attenuation has been computed at least one time for this user
        // and the distance traveled by the UE is greater than correlation distance
    }
    else if ((NOW - actualShadowingMap->at(nodeId).first).dbl() * speed
             > correlationDistance_)
    {
        // Get the temporal mark of the last computed shadowing attenuation
        time = (NOW - actualShadowingMap->at(nodeId).first).dbl();

        // Compute the traveled distance
        space = time * speed;

        // Compute shadowing with an EAW (Exponential Average Window) (step1)
        double a = exp(-0.5 * (space / correlationDistance_));

        // Get last shadowing attenuation computed
        double old = actualShadowingMap->at(nodeId).second;

        // Compute shadowing with an EAW (Exponential Average Window) (step2)
        att = a * old + sqrt(1 - pow(a, 2)) * normal(mean, stdDev);

        // Store the new computed shadowing
        std::pair<simtime_t, double> tmp(NOW, att);
        (*actualShadowingMap)[nodeId] = tmp;

        // If the distance traveled by the UE is smaller than correlation distance shadowing attenuation remains the same
    }
    else {
        att = actualShadowingMap->at(nodeId).second;
    }
    EV << " NrChannelModel_3GPP38_901::computeShadowing - shadowing att = " << att << endl;

    return att;
}

} //namespace

