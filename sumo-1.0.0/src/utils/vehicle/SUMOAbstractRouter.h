/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2006-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    SUMOAbstractRouter.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    25.Jan 2006
/// @version $Id$
///
// An abstract router base class
/****************************************************************************/
#ifndef SUMOAbstractRouter_h
#define SUMOAbstractRouter_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <utils/common/SysUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMOAbstractRouter
 * The interface for routing the vehicles over the network.
 */
template<class E, class V>
class SUMOAbstractRouter {
public:
    /// Type of the function that is used to retrieve the edge effort.
    typedef double(* Operation)(const E* const, const V* const, double);

    /// Constructor
    SUMOAbstractRouter(const std::string& type, Operation operation = nullptr, Operation ttOperation = nullptr) :
        myOperation(operation), myTTOperation(ttOperation),
        myBulkMode(false),
        myType(type),
        myQueryVisits(0),
        myNumQueries(0),
        myQueryStartTime(0),
        myQueryTimeSum(0) {
    }

    /// Destructor
    virtual ~SUMOAbstractRouter() {
        if (myNumQueries > 0) {
            WRITE_MESSAGE(myType + " answered " + toString(myNumQueries) + " queries and explored " + toString(double(myQueryVisits) / myNumQueries) +  " edges on average.");
            WRITE_MESSAGE(myType + " spent " + toString(myQueryTimeSum) + "ms answering queries (" + toString(double(myQueryTimeSum) / myNumQueries) +  "ms on average).");
        }
    }

    virtual SUMOAbstractRouter* clone() = 0;

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    virtual bool compute(const E* from, const E* to, const V* const vehicle,
                         SUMOTime msTime, std::vector<const E*>& into) = 0;

    virtual bool isProhibited(const E* const /* edge */, const V* const /* vehicle */) const  {
        return false;
    }

    inline double getTravelTime(const E* const e, const V* const v, const double t, const double effort) const {
        return myTTOperation == nullptr ? effort : (*myTTOperation)(e, v, t);
    }

    inline void updateViaCost(const E* const prev, const E* const e, const V* const v, double& time, double& effort) const {
        for (const std::pair<const E*, const E*>& follower : prev->getViaSuccessors()) {
            if (follower.first == e) {
                const E* viaEdge = follower.second;
                while (viaEdge != nullptr && viaEdge != e) {
                    const double viaEffortDelta = this->getEffort(viaEdge, v, time);
                    time += getTravelTime(viaEdge, v, time, viaEffortDelta);
                    effort += viaEffortDelta;
                    viaEdge = viaEdge->getViaSuccessors().front().first;
                }
                break;
            }
        }
    }


    inline double recomputeCosts(const std::vector<const E*>& edges, const V* const v, SUMOTime msTime) const {
        double effort = 0.;
        double time = STEPS2TIME(msTime);
        const E* prev = nullptr;
        for (const E* const e : edges) {
            if (isProhibited(e, v)) {
                return -1;
            }
            if (prev != nullptr) {
                updateViaCost(prev, e, v, time, effort);
            }
            const double effortDelta = this->getEffort(e, v, time);
            effort += effortDelta;
            time += getTravelTime(e, v, time, effortDelta);
            prev = e;
        }
        return effort;
    }


    inline double getEffort(const E* const e, const V* const v, double t) const {
        return (*myOperation)(e, v, t);
    }

    inline void startQuery() {
        myNumQueries++;
        myQueryStartTime = SysUtils::getCurrentMillis();
    }

    inline void endQuery(int visits) {
        myQueryVisits += visits;
        myQueryTimeSum += (SysUtils::getCurrentMillis() - myQueryStartTime);
    }

    void setBulkMode(const bool mode) {
        myBulkMode = mode;
    }

protected:
    /// @brief The object's operation to perform.
    Operation myOperation;

    /// @brief The object's operation to perform for travel times
    Operation myTTOperation;

    /// @brief whether we are currently operating several route queries in a bulk
    bool myBulkMode;

private:
    /// @brief the type of this router
    const std::string myType;

    /// @brief counters for performance logging
    long long int myQueryVisits;
    long long int myNumQueries;
    /// @brief the time spent querying in milliseconds
    long long int myQueryStartTime;
    long long int myQueryTimeSum;
private:
    /// @brief Invalidated assignment operator
    SUMOAbstractRouter& operator=(const SUMOAbstractRouter& s);
};


template<class E, class V>
class SUMOAbstractRouterPermissions : public SUMOAbstractRouter<E, V> {
public:
    /// Constructor
    SUMOAbstractRouterPermissions(const std::string& type, typename SUMOAbstractRouter<E, V>::Operation operation = nullptr, typename SUMOAbstractRouter<E, V>::Operation ttOperation = nullptr) :
        SUMOAbstractRouter<E, V>(type, operation, ttOperation) {
    }

    /// Destructor
    virtual ~SUMOAbstractRouterPermissions() {
    }

    bool isProhibited(const E* const edge, const V* const vehicle) const {
        if (std::find(myProhibited.begin(), myProhibited.end(), edge) != myProhibited.end()) {
            return true;
        }
        return edge->prohibits(vehicle);
    }

    void prohibit(const std::vector<E*>& toProhibit) {
        myProhibited = toProhibit;
    }

protected:
    std::vector<E*> myProhibited;

};


#endif

/****************************************************************************/

