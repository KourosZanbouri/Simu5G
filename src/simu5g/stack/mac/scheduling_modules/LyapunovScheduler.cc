/*
 * LyapunovScheduler.cc
 *
 * Author: kouros
 */

#include "simu5g/stack/mac/scheduling_modules/LyapunovScheduler.h"
#include "simu5g/stack/mac/scheduler/LteSchedulerEnb.h"
#include "simu5g/stack/mac/LteMacEnb.h"
#include "simu5g/stack/mac/buffer/LteMacBuffer.h"
#include <cmath> // For pow()

namespace simu5g {

using namespace omnetpp;

// Constructor saves alpha and beta using an initializer list
LyapunovScheduler::LyapunovScheduler(Binder* binder, double lyAlpha, double lyBeta)
    : LteScheduler(binder), lyAlpha_(lyAlpha), lyBeta_(lyBeta)
{
    loadContextIfNeeded();

    // --- NEW: Initialize Coordination Vectors ---
    // We start with a default size (e.g., 100 RBs).
    // This prevents crashes if we access them before the first TTI.
    neighborProtectedRbs_.resize(100, false);
    myCriticalRbs_.resize(100, false);
    // --------------------------------------------
    EV << "LyapunovScheduler created with lyAlpha: " << lyAlpha_ << ", lyBeta: " << lyBeta_ << endl;
}


void LyapunovScheduler::loadContextIfNeeded()
{
    if (!contextLoaded_) {
        qfiContextMgr_ = QfiContextManager::getInstance();  // singleton
        ASSERT(qfiContextMgr_ != nullptr);
        contextLoaded_ = true;
    }
}

void LyapunovScheduler::receiveX2InterferenceBitmap(const std::vector<bool>& rbs) {
    neighborProtectedRbs_ = rbs;
    EV << "LyapunovScheduler: Received Interference Mask from Neighbor." << endl;
}

double LyapunovScheduler::computeQosWeightFromContext(const QfiContext& ctx)
{
    double weight = 1.0;
    const double priorityBase = 2.0;
    weight *= pow(priorityBase, 9 - ctx.priorityLevel);

    if (ctx.delayBudgetMs <= 10) {
        weight *= 10.0;
    } else if (ctx.delayBudgetMs <= 50) {
        weight *= 3.0;
    }

    if (ctx.isGbr) {
        weight *= 2.0;
    }

    EV_INFO << NOW << " LyapunovScheduler [QFI=" << ctx.qfi << ", PrioLvl=" << ctx.priorityLevel << "]"
            << " --> Computed Aggressive Weight: " << weight << endl;

    return weight;
}

const QfiContext* LyapunovScheduler::getQfiContextForCid(MacCid cid)
{
    if (!qfiContextMgr_) return nullptr;
    int qfi = qfiContextMgr_->getQfiForCid(cid);
    if (qfi < 0) {
        EV_WARN << "LyapunovScheduler: No QFI registered for CID " << cid << "\n";
        return nullptr;
    }
    return qfiContextMgr_->getContextByQfi(qfi);
}

void LyapunovScheduler::prepareSchedule()
{
    EV << NOW << " HybridLyapunovScheduler::prepareSchedule" << endl;


    // 1. Reset Critical RB tracking for this TTI
        std::fill(myCriticalRbs_.begin(), myCriticalRbs_.end(), false);
        if (myCriticalRbs_.size() < 100) myCriticalRbs_.resize(100, false);

    grantedBytes_.clear();
    activeConnectionTempSet_ = *activeConnectionSet_;

    auto compare = [](const ScoredCid& a, const ScoredCid& b) { return a.second < b.second; };
    std::priority_queue<ScoredCid, std::vector<ScoredCid>, decltype(compare)> scoreQueue(compare);

    for (const auto& cid : carrierActiveConnectionSet_)
    {
        MacNodeId nodeId = cid.getNodeId();
        if (nodeId == NODEID_NONE) continue;

        // [Backlog Logic]
        double backlog = 0;
        Direction dir = (direction_ == UL) ? UL : DL;
        if (dir == DL) {
            LteMacBuffer* dlBuffer = eNbScheduler_->mac_->getMacBuffer(cid);
            if (dlBuffer != nullptr) {
                backlog = dlBuffer->getQueueOccupancy();
            }
        } else {
            LteMacBuffer* ulBuffer = eNbScheduler_->mac_->getBsrVirtualBuffer(cid);
            if (ulBuffer != nullptr) {
                backlog = ulBuffer->getQueueOccupancy();
            }
        }
        if (backlog == 0) continue;


        const UserTxParams& info = eNbScheduler_->mac_->getAmc()->computeTxParams(nodeId, dir, carrierFrequency_);
        if (info.readCqiVector().empty() || info.readBands().empty()) continue;

        // --- FIX 1: Define 'isCritical' HERE (Before using it) ---
        const QfiContext* ctx = getQfiContextForCid(cid);
        bool isCritical = (ctx && ctx->qfi == 4);
        // ---------------------------------------------------------

        unsigned int availableBlocks = 0, availableBytes = 0;
        for (auto antenna : info.readAntennaSet()) {
            for (auto band : info.readBands()) {

                // --- FIX 2: Use 'isCritical' for Interference Check ---
                if (!isCritical && band < neighborProtectedRbs_.size() && neighborProtectedRbs_[band] == true) {
                    continue; // Blocked by neighbor
                }
                // ------------------------------------------------------

                unsigned int blocks = eNbScheduler_->readAvailableRbs(nodeId, antenna, band);
                availableBlocks += blocks;
                availableBytes += eNbScheduler_->mac_->getAmc()->computeBytesOnNRbs(nodeId, band, blocks, dir, carrierFrequency_);
            }
        }

        double achievableRate = (availableBlocks > 0) ? static_cast<double>(availableBytes) / availableBlocks : 0.0;
        if (achievableRate == 0) continue;

        double qosWeight = ctx ? computeQosWeightFromContext(*ctx) : 1.0;
        double score = pow(backlog, lyAlpha_) * achievableRate * pow(qosWeight, lyBeta_);

        if (isCritical) score *= 1e12;

        score += uniform(getEnvir()->getRNG(0), -scoreEpsilon_, scoreEpsilon_);
        scoreQueue.push({cid, score});
    }

    // --- LOOP 2: ALLOCATION (Grant Resources) ---
    while (!scoreQueue.empty())
    {
        ScoredCid current = scoreQueue.top();
        scoreQueue.pop();

        bool terminate = false, active = true, eligible = true;

        // --- FIX 3: Re-Define 'isCritical' for the current user ---
        const QfiContext* ctx = getQfiContextForCid(current.first);
        bool isCritical = (ctx && ctx->qfi == 4);
        // ----------------------------------------------------------

        unsigned int granted = requestGrant(current.first, UINT32_MAX, terminate, active, eligible);
        grantedBytes_[current.first] += granted;

        // --- FIX 4: Use 'isCritical' to update myCriticalRbs_ ---
        if (isCritical && granted > 0) {
            for(size_t i=0; i<20 && i<myCriticalRbs_.size(); ++i) {
                myCriticalRbs_[i] = true;
            }
        }

        if (terminate) break;
        if (!active) {
            activeConnectionTempSet_.erase(current.first);
            carrierActiveConnectionSet_.erase(current.first);
        }
    }

    // --- FIX 5: Fix the Casting Error ---
    static int ttiCounter = 0;
    if (++ttiCounter % 10 == 0) {

        LteMacEnb* myMac = eNbScheduler_->mac_.get();

        if (myMac) {
            bool hasCritical = false;
            for(bool b : myCriticalRbs_) if(b) { hasCritical=true; break; }

            if(hasCritical) {
                myMac->sendX2LoadInformation(myCriticalRbs_);
            }
        }
    }
}


void LyapunovScheduler::commitSchedule()
{
    *activeConnectionSet_ = activeConnectionTempSet_;
}



} // namespace simu5g
