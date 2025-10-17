
/*
 * LyapunovScheduler.cc
 *
 * Author: kouros
 */

#include "simu5g/stack/mac/scheduling_modules/LyapunovScheduler.h"
#include "simu5g/stack/mac/scheduler/LteSchedulerEnb.h"
#include "simu5g/stack/mac/LteMacEnb.h"
#include "simu5g/stack/mac/buffer/LteMacBuffer.h"

namespace simu5g {

using namespace omnetpp;



// Constructor saves alpha and beta using an initializer list
LyapunovScheduler::LyapunovScheduler(Binder* binder, double lyAlpha, double lyBeta)
    : LteScheduler(binder), lyAlpha_(lyAlpha), lyBeta_(lyBeta)
{
    loadContextIfNeeded();
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

double LyapunovScheduler::computeQosWeightFromContext(const QfiContext& ctx)
{
    // Base weight for all flows
    double weight = 1.0;

    // --- Exponential Priority Scaling ---
    // Use a base value greater than 1. A higher base creates more separation.
    const double priorityBase = 2.0;
    // Lower priorityLevel is better (e.g., 1 is higher priority than 9).
    // This creates an exponential gap: level 1 gets 2^8, level 9 gets 2^0.
    weight *= pow(priorityBase, 9 - ctx.priorityLevel);

    // --- Delay Budget Scaling ---
    // Extremely aggressive bonus for tight delay budgets (URLLC-like)
    if (ctx.delayBudgetMs <= 10) {
        weight *= 10.0;
    } else if (ctx.delayBudgetMs <= 50) {
        weight *= 3.0;
    }

    // --- GBR Bonus ---
    // Provide a significant, constant multiplier for guaranteed bit rate flows.
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
    return qfiContextMgr_ -> getContextByQfi(qfi);
}



struct SchedulingInfo {
    MacCid cid;
    const QfiContext* qfiContext;
    double queueBacklog;
    double achievableRate; // in bytes per resource block
};

void LyapunovScheduler::prepareSchedule()
{
    EV << NOW << " HybridLyapunovScheduler::prepareSchedule" << endl;

    const LteMacBufferMap* virtualBuffers = (direction_ == UL) ? eNbScheduler_->mac_->getBsrVirtualBuffers() : nullptr;
    grantedBytes_.clear();
    activeConnectionTempSet_ = *activeConnectionSet_;

    // --- Unified priority queue for all traffic ---
    auto compare = [](const ScoredCid& a, const ScoredCid& b) { return a.second < b.second; };
    std::priority_queue<ScoredCid, std::vector<ScoredCid>, decltype(compare)> scoreQueue(compare);

    // --- Single Pass Data Gathering and Scoring ---
    for (const auto& cid : carrierActiveConnectionSet_)
    {
        MacNodeId nodeId = MacCidToNodeId(cid);
        if (nodeId == NODEID_NONE || binder_->getOmnetId(nodeId) == 0) continue;

        double backlog = 0;
        Direction dir = (direction_ == UL) ? UL : DL;

        if (dir == DL) {
            backlog = eNbScheduler_->mac_->getDlQueueSize(cid);
        } else { // Uplink
            if (virtualBuffers && virtualBuffers->count(cid) > 0) {
                backlog = virtualBuffers->at(cid)->getQueueOccupancy();
            }
        }
        if (backlog == 0) continue;

        const UserTxParams& info = eNbScheduler_->mac_->getAmc()->computeTxParams(nodeId, dir, carrierFrequency_);
        if (info.readCqiVector().empty() || info.readBands().empty()) continue;

        unsigned int availableBlocks = 0, availableBytes = 0;
        for (auto antenna : info.readAntennaSet()) {
            for (auto band : info.readBands()) {
                unsigned int blocks = eNbScheduler_->readAvailableRbs(nodeId, antenna, band);
                availableBlocks += blocks;
                availableBytes += eNbScheduler_->mac_->getAmc()->computeBytesOnNRbs(nodeId, band, blocks, dir, carrierFrequency_);
            }
        }
        double achievableRate = (availableBlocks > 0) ? static_cast<double>(availableBytes) / availableBlocks : 0.0;
        if (achievableRate == 0) continue;

        const QfiContext* ctx = getQfiContextForCid(cid);
        double qosWeight = ctx ? computeQosWeightFromContext(*ctx) : 1.0;

        // --- Score calculation with tuning exponents ---
        double score = pow(backlog, lyAlpha_) * achievableRate * pow(qosWeight, lyBeta_);

        // --- Correct Strict Priority logic using a massive score bonus ---
        if (ctx && ctx->qfi == 4) { // QFI 4 for URLLC
            score *= 1e12;
        }

        score += uniform(getEnvir()->getRNG(0), -scoreEpsilon_, scoreEpsilon_);

        EV_INFO << NOW << " LyapunovScheduler [CID=" << cid << ", QFI=" << (ctx ? ctx->qfi : -1) << "]"
                << " Backlog(Q^a)=" << pow(backlog, lyAlpha_)
                << " Rate(R)=" << achievableRate
                << " Weight(W^b)=" << pow(qosWeight, lyBeta_)
                << " --> FINAL SCORE=" << score << endl;

        scoreQueue.push({cid, score});
    }

    // --- Unified Granting Loop ---
    while (!scoreQueue.empty())
    {
        ScoredCid current = scoreQueue.top();
        scoreQueue.pop();

        bool terminate = false, active = true, eligible = true;
        unsigned int granted = requestGrant(current.first, UINT32_MAX, terminate, active, eligible);
        grantedBytes_[current.first] += granted;

        if (terminate) break;
        if (!active) {
            activeConnectionTempSet_.erase(current.first);
            carrierActiveConnectionSet_.erase(current.first);
        }
    }
}


void LyapunovScheduler::commitSchedule()
{
    *activeConnectionSet_ = activeConnectionTempSet_;
}

} // namespace simu5g
