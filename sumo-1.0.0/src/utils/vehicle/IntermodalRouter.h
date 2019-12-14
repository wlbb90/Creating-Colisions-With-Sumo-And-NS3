/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    IntermodalRouter.h
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 03 March 2014
/// @version $Id$
///
// The IntermodalRouter builds a special network and (delegates to a SUMOAbstractRouter)
/****************************************************************************/
#ifndef IntermodalRouter_h
#define IntermodalRouter_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include "SUMOAbstractRouter.h"
#include "DijkstraRouter.h"
#include "IntermodalNetwork.h"
#include "EffortCalculator.h"
#include "CarEdge.h"
#include "StopEdge.h"
#include "PedestrianRouter.h"

//#define IntermodalRouter_DEBUG_ROUTES


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class IntermodalRouter
 * The router for pedestrians (on a bidirectional network of sidewalks and crossings)
 */
template<class E, class L, class N, class V>
class IntermodalRouter : public SUMOAbstractRouter<E, IntermodalTrip<E, N, V> > {
public:
    typedef IntermodalNetwork<E, L, N, V> Network;

private:
    typedef void(*CreateNetCallback)(IntermodalRouter <E, L, N, V>&);
    typedef IntermodalEdge<E, L, N, V> _IntermodalEdge;
    typedef IntermodalTrip<E, N, V> _IntermodalTrip;
    typedef SUMOAbstractRouterPermissions<IntermodalEdge<E, L, N, V>, IntermodalTrip<E, N, V> > _InternalRouter;
    typedef DijkstraRouter<IntermodalEdge<E, L, N, V>, IntermodalTrip<E, N, V>, SUMOAbstractRouterPermissions<IntermodalEdge<E, L, N, V>, IntermodalTrip<E, N, V> > > _InternalDijkstra;

public:
    struct TripItem {
        TripItem(const std::string& _line = "") :
            line(_line), intended(_line), depart(-1), traveltime(0.), cost(0.) {}
        std::string line;
        std::string destStop;
        std::string intended; // intended public transport vehicle id
        double depart; // intended public transport departure
        std::vector<const E*> edges;
        double traveltime;
        double cost;
    };

    /// Constructor
    IntermodalRouter(CreateNetCallback callback, const int carWalkTransfer, const int routingMode = 0, EffortCalculator* calc = nullptr) :
        SUMOAbstractRouter<E, _IntermodalTrip>("IntermodalRouter"),
        myAmClone(false), myInternalRouter(nullptr), myIntermodalNet(nullptr),
        myCallback(callback), myCarWalkTransfer(carWalkTransfer), myRoutingMode(routingMode),
        myExternalEffort(calc) {
    }

    /// Destructor
    virtual ~IntermodalRouter() {
        delete myInternalRouter;
        if (!myAmClone) {
            delete myIntermodalNet;
        }
    }

    SUMOAbstractRouter<E, _IntermodalTrip>* clone() {
        createNet();
        return new IntermodalRouter<E, L, N, V>(myIntermodalNet);
    }

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    bool compute(const E* from, const E* to, const double departPos, const double arrivalPos,
                 const std::string stopID, const double speed,
                 const V* const vehicle, const SVCPermissions modeSet, const SUMOTime msTime,
                 std::vector<TripItem>& into, const double externalFactor = 0.) {
        createNet();
        _IntermodalTrip trip(from, to, departPos, arrivalPos, speed, msTime, 0, vehicle, modeSet, myExternalEffort, externalFactor);
        std::vector<const _IntermodalEdge*> intoEdges;
        const bool success = myInternalRouter->compute(myIntermodalNet->getDepartEdge(from, trip.departPos),
                             stopID != "" ? myIntermodalNet->getStopEdge(stopID) : myIntermodalNet->getArrivalEdge(to, trip.arrivalPos),
                             &trip, msTime, intoEdges);
        if (success) {
            std::string lastLine = "";
            double time = STEPS2TIME(msTime);
            double effort = 0.;
            const _IntermodalEdge* prev = nullptr;
            for (const _IntermodalEdge* iEdge : intoEdges) {
                if (iEdge->includeInRoute(false)) {
                    if (iEdge->getLine() == "!stop") {
                        into.back().destStop = iEdge->getID();
                        if (lastLine == "!ped") {
                            lastLine = ""; // a stop always starts a new trip item
                        }
                    } else {
                        if (iEdge->getLine() != lastLine) {
                            lastLine = iEdge->getLine();
                            if (lastLine == "!car") {
                                into.push_back(TripItem(vehicle->getID()));
                            } else if (lastLine == "!ped") {
                                into.push_back(TripItem());
                            } else {
                                into.push_back(TripItem(lastLine));
                                into.back().depart = iEdge->getIntended(time, into.back().intended);
                            }
                        }
                        if (into.back().edges.empty() || into.back().edges.back() != iEdge->getEdge()) {
                            into.back().edges.push_back(iEdge->getEdge());
                        }
                    }
                }
                if (prev != nullptr) {
                    myInternalRouter->updateViaCost(prev, iEdge, &trip, time, effort);
                }
                const double edgeEffort = myInternalRouter->getEffort(iEdge, &trip, time);
                effort += edgeEffort;
                const double edgeTime = myInternalRouter->getTravelTime(iEdge, &trip, time, edgeEffort);
                time += edgeTime;
                prev = iEdge;
                if (!into.empty()) {
                    into.back().traveltime += edgeTime;
                    into.back().cost += edgeEffort;
                }
            }
        }
#ifdef IntermodalRouter_DEBUG_ROUTES
        double time = STEPS2TIME(msTime);
        for (const _IntermodalEdge* iEdge : intoEdges) {
            const double edgeEffort = myInternalRouter->getEffort(iEdge, &trip, time);
            time += edgeEffort;
            std::cout << iEdge->getID() << "(" << iEdge->getLine() << "): " << edgeEffort << std::endl;
        }
        std::cout << TIME2STEPS(msTime) << " trip from " << from->getID() << " to " << (to != nullptr ? to->getID() : stopID)
                  << " departPos=" << trip.departPos
                  << " arrivalPos=" << trip.arrivalPos
                  << " edges=" << toString(intoEdges)
//                  << " resultEdges=" << toString(into)
                  << " time=" << time
                  << "\n";
#endif
        return success;
    }

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    bool compute(const E*, const E*, const _IntermodalTrip* const,
                 SUMOTime, std::vector<const E*>&) {
        throw ProcessError("Do not use this method");
    }

    void prohibit(const std::vector<E*>& toProhibit) {
        createNet();
        std::vector<_IntermodalEdge*> toProhibitPE;
        for (typename std::vector<E*>::const_iterator it = toProhibit.begin(); it != toProhibit.end(); ++it) {
            toProhibitPE.push_back(myIntermodalNet->getBothDirections(*it).first);
            toProhibitPE.push_back(myIntermodalNet->getBothDirections(*it).second);
            toProhibitPE.push_back(myIntermodalNet->getCarEdge(*it));
        }
        myInternalRouter->prohibit(toProhibitPE);
    }

    void writeNetwork(OutputDevice& dev) {
        createNet();
        for (_IntermodalEdge* e : myIntermodalNet->getAllEdges()) {
            dev.openTag(SUMO_TAG_EDGE);
            dev.writeAttr(SUMO_ATTR_ID, e->getID());
            dev.writeAttr(SUMO_ATTR_LINE, e->getLine());
            dev.writeAttr(SUMO_ATTR_LENGTH, e->getLength());
            dev.writeAttr("successors", toString(e->getSuccessors(SVC_IGNORING)));
            dev.closeTag();
        }
    }

    void writeWeights(OutputDevice& dev) {
        createNet();
        _IntermodalTrip trip(nullptr, nullptr, 0., 0., DEFAULT_PEDESTRIAN_SPEED, 0, 0, nullptr, SVC_PASSENGER | SVC_BICYCLE | SVC_BUS);
        for (_IntermodalEdge* e : myIntermodalNet->getAllEdges()) {
            dev.openTag(SUMO_TAG_EDGE);
            dev.writeAttr(SUMO_ATTR_ID, e->getID());
            dev.writeAttr("traveltime", e->getTravelTime(&trip, 0.));
            dev.writeAttr("effort", e->getEffort(&trip, 0.));
            dev.closeTag();
        }
    }

    Network* getNetwork() const {
        return myIntermodalNet;
    }

private:
    IntermodalRouter(Network* net):
        SUMOAbstractRouter<E, _IntermodalTrip>("IntermodalRouterClone"), myAmClone(true),
        myInternalRouter(new _InternalDijkstra(net->getAllEdges(), true, &_IntermodalEdge::getTravelTimeStatic)),
        myIntermodalNet(net), myCarWalkTransfer(0), myRoutingMode(0), myExternalEffort(nullptr) {}

    static inline double getEffortAggregated(const _IntermodalEdge* const edge, const _IntermodalTrip* const trip, double time) {
        return edge == nullptr || !edge->hasEffort() ? 0. : edge->getEffort(trip, time);
    }

    static inline double getCombined(const _IntermodalEdge* const edge, const _IntermodalTrip* const trip, double time) {
        return edge->getTravelTime(trip, time) + trip->externalFactor * trip->calc->getEffort(edge->getNumericalID());
    }

    inline void createNet() {
        if (myIntermodalNet == nullptr) {
            myIntermodalNet = new Network(E::getAllEdges(), false, myCarWalkTransfer);
            myIntermodalNet->addCarEdges(E::getAllEdges());
            myCallback(*this);
            switch (myRoutingMode) {
                case 0:
                    myInternalRouter = new _InternalDijkstra(myIntermodalNet->getAllEdges(), true, &_IntermodalEdge::getTravelTimeStatic);
                    break;
                case 1:
                    myInternalRouter = new _InternalDijkstra(myIntermodalNet->getAllEdges(), true, &getEffortAggregated, &_IntermodalEdge::getTravelTimeStatic);
                    break;
                case 2:
                    myInternalRouter = new _InternalDijkstra(myIntermodalNet->getAllEdges(), true, &_IntermodalEdge::getEffortStatic, &_IntermodalEdge::getTravelTimeStatic);
                    break;
                case 3:
                    if (myExternalEffort != nullptr) {
                        std::vector<std::string> edgeIDs;
                        for (const auto e : myIntermodalNet->getAllEdges()) {
                            edgeIDs.push_back(e->getID());
                        }
                        myExternalEffort->init(edgeIDs);
                    }
                    myInternalRouter = new _InternalDijkstra(myIntermodalNet->getAllEdges(), true, &getCombined, &_IntermodalEdge::getTravelTimeStatic, false, myExternalEffort);
                    break;
            }
        }
    }

private:
    const bool myAmClone;
    _InternalRouter* myInternalRouter;
    Network* myIntermodalNet;
    CreateNetCallback myCallback;
    const int myCarWalkTransfer;
    const int myRoutingMode;
    EffortCalculator* const myExternalEffort;


private:
    /// @brief Invalidated assignment operator
    IntermodalRouter& operator=(const IntermodalRouter& s);

};


#endif

/****************************************************************************/
