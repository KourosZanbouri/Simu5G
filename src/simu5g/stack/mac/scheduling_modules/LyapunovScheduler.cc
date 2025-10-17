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

    grantedBytes_.clear();
    activeConnectionTempSet_ = *activeConnectionSet_;

    auto compare = [](const ScoredCid& a, const ScoredCid& b) { return a.second < b.second; };
    std::priority_queue<ScoredCid, std::vector<ScoredCid>, decltype(compare)> scoreQueue(compare);

    for (const auto& cid : carrierActiveConnectionSet_)
    {
        // CHANGE: MacCidToNodeId() is replaced with cid.getNodeId()
        MacNodeId nodeId = cid.getNodeId();
        // CHANGE: binder_->getOmnetId() check is removed as it's no longer available
        if (nodeId == NODEID_NONE) continue;

        double backlog = 0;
        Direction dir = (direction_ == UL) ? UL : DL;

        // --- START OF UPDATED BACKLOG LOGIC ---
        if (dir == DL) {
            // CHANGE: getDlQueueSize() is replaced with getMacBuffer()
            LteMacBuffer* dlBuffer = eNbScheduler_->mac_->getMacBuffer(cid);
            if (dlBuffer != nullptr) {
                backlog = dlBuffer->getQueueOccupancy();
            }
        } else { // Uplink
            // CHANGE: getBsrVirtualBuffers() is replaced with getBsrVirtualBuffer()
            LteMacBuffer* ulBuffer = eNbScheduler_->mac_->getBsrVirtualBuffer(cid);
            if (ulBuffer != nullptr) {
                backlog = ulBuffer->getQueueOccupancy();
            }
        }
        // --- END OF UPDATED BACKLOG LOGIC ---

        if (backlog == 0) continue;

        const UserTxParams& info = eNbScheduler_->mac_->getAmc()->computeTxParams(nodeId, dir, carrierFrequency_);
        if (info.readCqiVector().empty() || info.readBands().empty() || eNbScheduler_->allocatedCws(nodeId) == info.getLayers().size()) {
            continue;
        }

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

        double score = pow(backlog, lyAlpha_) * achievableRate * pow(qosWeight, lyBeta_);

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
