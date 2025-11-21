/*
 * LyapunovScheduler.h
 *
 * Author: kouros
 */

#ifndef __SIMU5G_LYAPUNOVSCHEDULER_H_
#define __SIMU5G_LYAPUNOVSCHEDULER_H_

#include <omnetpp.h>
#include <vector>
#include <map>
#include <queue>



// 1. Ensure Binder is included for the constructor argument
#include "simu5g/common/binder/Binder.h"

// 2. Include the Base Class
#include "simu5g/stack/mac/scheduler/LteScheduler.h"
#include "simu5g/stack/sdap/common/QfiContextManager.h"

namespace simu5g {

// Forward declaration
class LteSchedulerEnb;

class LyapunovScheduler : public LteScheduler
{
  protected:
    // Manager for QoS Flow Identifier contexts
    QfiContextManager* qfiContextMgr_ = nullptr;
    bool contextLoaded_ = false;

    double lyAlpha_;
    double lyBeta_;

    std::vector<bool> neighborProtectedRbs_;
    std::vector<bool> myCriticalRbs_;

    // Map to store granted bytes in the current TTI for each connection
    std::map<MacCid, unsigned int> grantedBytes_;

    // Temporary set of active connections for the current scheduling period
    ActiveSet activeConnectionTempSet_;

    // Small epsilon value for floating point comparisons and randomization
    const double scoreEpsilon_ = 1e-6;

    typedef std::pair<MacCid, double> ScoredCid;


    // --- Methods ---
    // Initializes the QFI context manager
    void loadContextIfNeeded();

    // Retrieves the QFI context for a given Connection ID (CID)
    const QfiContext* getQfiContextForCid(MacCid cid);

    // Calculates a weight based on the QoS parameters of a flow
    double computeQosWeightFromContext(const QfiContext& ctx);


  public:
    // Constructor - Simplified to remove PF parameters
    LyapunovScheduler(Binder* binder, double lyAlpha, double lyBeta);

    void receiveX2InterferenceBitmap(const std::vector<bool>& rbs);


    // Main scheduling functions
    void prepareSchedule() override;
    void commitSchedule() override;
};

} // namespace simu5g

#endif // __SIMU5G_LYAPUNOVSCHEDULER_H_
