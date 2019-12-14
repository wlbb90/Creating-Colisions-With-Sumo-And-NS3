/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    IntermodalNetwork.h
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Robert Hilbrich
/// @date    Mon, 03 March 2014
/// @version $Id$
///
// The Edge definition for the Intermodal Router
/****************************************************************************/
#ifndef IntermodalNetwork_h
#define IntermodalNetwork_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/Named.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include "AccessEdge.h"
#include "CarEdge.h"
#include "IntermodalEdge.h"
#include "PedestrianEdge.h"
#include "PublicTransportEdge.h"
#include "StopEdge.h"
#include "SUMOVehicleParameter.h"

//#define IntermodalRouter_DEBUG_NETWORK


// ===========================================================================
// function definitions
// ===========================================================================
template <class E, class L>
inline const L* getSidewalk(const E* edge) {
    if (edge == nullptr) {
        return nullptr;
    }
    // prefer lanes that are exclusive to pedestrians
    const std::vector<L*>& lanes = edge->getLanes();
    for (const L* const lane : lanes) {
        if (lane->getPermissions() == SVC_PEDESTRIAN) {
            return lane;
        }
    }
    for (const L* const lane : lanes) {
        if (lane->allowsVehicleClass(SVC_PEDESTRIAN)) {
            return lane;
        }
    }
    return nullptr;
}


// ===========================================================================
// class definitions
// ===========================================================================
/// @brief the intermodal network storing edges, connections and the mappings to the "real" edges
template<class E, class L, class N, class V>
class IntermodalNetwork {
private:
    typedef IntermodalEdge<E, L, N, V> _IntermodalEdge;
    typedef AccessEdge<E, L, N, V> _AccessEdge;
    typedef PedestrianEdge<E, L, N, V> _PedestrianEdge;
    typedef PublicTransportEdge<E, L, N, V> _PTEdge;
    typedef std::pair<_IntermodalEdge*, _IntermodalEdge*> EdgePair;

public:
    /** @brief where mode changes are possible
    */
    enum ModeChangeOptions {
        /// @brief parking areas
        PARKING_AREAS = 1,
        /// @brief public transport stops and access
        PT_STOPS = 2,
        /// @brief junctions with edges allowing the additional mode
        ALL_JUNCTIONS = 4
    };

    /* @brief build the pedestrian part of the intermodal network (once)
     * @param edges The list of MSEdge or ROEdge to build from
     * @param numericalID the start number for the creation of new edges
     */
    IntermodalNetwork(const std::vector<E*>& edges, const bool pedestrianOnly, const int carWalkTransfer = 0)
        : myNumericalID(0), myCarWalkTransfer(carWalkTransfer) {
#ifdef IntermodalRouter_DEBUG_NETWORK
        std::cout << "initIntermodalNetwork\n";
#endif
        // build the pedestrian edges and the depart / arrival connectors with lookup tables
        bool haveSeenWalkingArea = false;
        for (const E* const edge : edges) {
            if (edge->isTazConnector()) {
                continue;
            }
            const L* lane = getSidewalk<E, L>(edge);
            if (lane != 0) {
                if (edge->isWalkingArea()) {
                    // only a single edge
                    addEdge(new _PedestrianEdge(myNumericalID++, edge, lane, true));
                    myBidiLookup[edge] = std::make_pair(myEdges.back(), myEdges.back());
                    myDepartLookup[edge].push_back(myEdges.back());
                    myArrivalLookup[edge].push_back(myEdges.back());
                    haveSeenWalkingArea = true;
                } else { // regular edge or crossing
                    // forward and backward edges
                    addEdge(new _PedestrianEdge(myNumericalID++, edge, lane, true));
                    addEdge(new _PedestrianEdge(myNumericalID++, edge, lane, false));
                    myBidiLookup[edge] = std::make_pair(myEdges[myNumericalID - 2], myEdges.back());
                }
            }
            if (!edge->isWalkingArea()) {
                // depart and arrival edges (the router can decide the initial direction to take and the direction to arrive from)
                _IntermodalEdge* const departConn = new _IntermodalEdge(edge->getID() + "_depart_connector", myNumericalID++, edge, "!connector");
                _IntermodalEdge* const arrivalConn = new _IntermodalEdge(edge->getID() + "_arrival_connector", myNumericalID++, edge, "!connector");
                addConnectors(departConn, arrivalConn, 0);
            }
        }

        // build the walking connectors if there are no walking areas
        for (const E* const edge : edges) {
            if (edge->isTazConnector()) {
                continue;
            }
            if (haveSeenWalkingArea) {
                // connectivity needs to be ensured only in the real intermodal case, for simple pedestrian routing we don't have connectors if we have walking areas
                if (!pedestrianOnly && getSidewalk<E, L>(edge) == nullptr) {
                    const N* const node = edge->getToJunction();
                    if (myWalkingConnectorLookup.count(node) == 0) {
                        addEdge(new _IntermodalEdge(node->getID() + "_walking_connector", myNumericalID++, nullptr, "!connector"));
                        myWalkingConnectorLookup[node] = myEdges.back();
                    }
                }
            } else {
                for (const N* const node : {
                edge->getFromJunction(), edge->getToJunction()
                }) {
                    if (myWalkingConnectorLookup.count(node) == 0) {
                        addEdge(new _IntermodalEdge(node->getID() + "_walking_connector", myNumericalID++, nullptr, "!connector"));
                        myWalkingConnectorLookup[node] = myEdges.back();
                    }
                }
            }
        }
        // build the connections
        for (const E* const edge : edges) {
            const L* const sidewalk = getSidewalk<E, L>(edge);
            if (sidewalk == nullptr) {
                continue;
            }
            // find all incoming and outgoing lanes for the sidewalk and
            // connect the corresponding IntermodalEdges
            const EdgePair& pair = getBothDirections(edge);
#ifdef IntermodalRouter_DEBUG_NETWORK
            std::cout << "  building connections from " << sidewalk->getID() << "\n";
#endif
            if (haveSeenWalkingArea) {
                const std::vector<std::pair<const L*, const E*> > outgoing = sidewalk->getOutgoingViaLanes();
                // if one of the outgoing lanes is a walking area it must be used.
                // All other connections shall be ignored
                // if it has no outgoing walking area, it probably is a walking area itself
                bool hasWalkingArea = false;
                for (const auto& target : outgoing) {
                    if (target.first->getEdge().isWalkingArea()) {
                        hasWalkingArea = true;
                        break;
                    }
                }
                for (const auto& target : outgoing) {
                    const E* const targetEdge = &(target.first->getEdge());
                    const bool used = (target.first == getSidewalk<E, L>(targetEdge)
                                       && (!hasWalkingArea || targetEdge->isWalkingArea()));
#ifdef IntermodalRouter_DEBUG_NETWORK
                    const L* potTarget = getSidewalk<E, L>(targetEdge);
                    std::cout << "   lane=" << (potTarget == 0 ? "NULL" : potTarget->getID()) << (used ? "(used)" : "") << "\n";
#endif
                    if (used) {
                        const EdgePair& targetPair = getBothDirections(targetEdge);
                        pair.first->addSuccessor(targetPair.first);
                        targetPair.second->addSuccessor(pair.second);
#ifdef IntermodalRouter_DEBUG_NETWORK
                        std::cout << "     " << pair.first->getID() << " -> " << targetPair.first->getID() << "\n";
                        std::cout << "     " << targetPair.second->getID() << " -> " << pair.second->getID() << "\n";
#endif
                    }
                }
            }
            // We may have a network without pedestrian structures or a car-only edge.
            // In the first case we assume that all sidewalks at a junction are interconnected,
            // in the second we connect all car-only edges to all sidewalks.
            _IntermodalEdge* const toNodeConn = myWalkingConnectorLookup[edge->getToJunction()];
            if (toNodeConn != nullptr) {
                if (!haveSeenWalkingArea) {
                    // if we have walking areas we should use them and not the connector
                    pair.first->addSuccessor(toNodeConn);
                }
                toNodeConn->addSuccessor(pair.second);
            }
            _IntermodalEdge* const fromNodeConn = myWalkingConnectorLookup[edge->getFromJunction()];
            if (fromNodeConn != nullptr) {
                if (!haveSeenWalkingArea) {
                    pair.second->addSuccessor(fromNodeConn);
                }
                fromNodeConn->addSuccessor(pair.first);
            }
            if (!edge->isWalkingArea()) {
                // build connections from depart connector
                _IntermodalEdge* startConnector = getDepartConnector(edge);
                startConnector->addSuccessor(pair.first);
                startConnector->addSuccessor(pair.second);
                // build connections to arrival connector
                _IntermodalEdge* endConnector = getArrivalConnector(edge);
                pair.first->addSuccessor(endConnector);
                pair.second->addSuccessor(endConnector);
            }
#ifdef IntermodalRouter_DEBUG_NETWORK
            std::cout << "     " << startConnector->getID() << " -> " << pair.first->getID() << "\n";
            std::cout << "     " << startConnector->getID() << " -> " << pair.second->getID() << "\n";
            std::cout << "     " << pair.first->getID() << " -> " << endConnector->getID() << "\n";
            std::cout << "     " << pair.second->getID() << " -> " << endConnector->getID() << "\n";
#endif
        }
    }

    ~IntermodalNetwork() {
        for (typename std::vector<_IntermodalEdge*>::iterator it = myEdges.begin(); it != myEdges.end(); ++it) {
            delete *it;
        }
    }

    void addEdge(_IntermodalEdge* edge) {
        while ((int)myEdges.size() <= edge->getNumericalID()) {
            myEdges.push_back(0);
        }
        myEdges[edge->getNumericalID()] = edge;
    }

    void addConnectors(_IntermodalEdge* const depConn, _IntermodalEdge* const arrConn, const int index) {
        addEdge(depConn);
        addEdge(arrConn);
        myDepartLookup[depConn->getEdge()].insert(myDepartLookup[depConn->getEdge()].begin() + index, depConn);
        myArrivalLookup[arrConn->getEdge()].insert(myArrivalLookup[arrConn->getEdge()].begin() + index, arrConn);
    }

    const std::vector<_IntermodalEdge*>& getAllEdges() {
        return myEdges;
    }

    /// @brief Returns the pair of forward and backward edge
    const EdgePair& getBothDirections(const E* e) const {
        typename std::map<const E*, EdgePair>::const_iterator it = myBidiLookup.find(e);
        if (it == myBidiLookup.end()) {
            assert(false);
            throw ProcessError("Edge '" + e->getID() + "' not found in intermodal network '");
        }
        return (*it).second;
    }

    /// @brief Returns the departing intermodal edge
    _IntermodalEdge* getDepartEdge(const E* e, const double pos) const {
        typename std::map<const E*, std::vector<_IntermodalEdge*> >::const_iterator it = myDepartLookup.find(e);
        if (it == myDepartLookup.end()) {
            throw ProcessError("Depart edge '" + e->getID() + "' not found in intermodal network.");
        }
        const std::vector<_IntermodalEdge*>& splitList = it->second;
        typename std::vector<_IntermodalEdge*>::const_iterator splitIt = splitList.begin();
        double totalLength = 0.;
        while (splitIt != splitList.end() && totalLength + (*splitIt)->getLength() < pos) {
            totalLength += (*splitIt)->getLength();
            ++splitIt;
        }
        return *splitIt;
    }

    /// @brief Returns the departing intermodal connector at the given split offset
    _IntermodalEdge* getDepartConnector(const E* e, const int splitIndex = 0) const {
        typename std::map<const E*, std::vector<_IntermodalEdge*> >::const_iterator it = myDepartLookup.find(e);
        if (it == myDepartLookup.end()) {
            throw ProcessError("Depart edge '" + e->getID() + "' not found in intermodal network.");
        }
        if (splitIndex >= (int)it->second.size()) {
            throw ProcessError("Split index " + toString(splitIndex) + " invalid for depart edge '" + e->getID() + "' .");
        }
        return it->second[splitIndex];
    }

    /// @brief Returns the arriving intermodal edge
    _IntermodalEdge* getArrivalEdge(const E* e, const double pos) const {
        typename std::map<const E*, std::vector<_IntermodalEdge*> >::const_iterator it = myArrivalLookup.find(e);
        if (it == myArrivalLookup.end()) {
            throw ProcessError("Arrival edge '" + e->getID() + "' not found in intermodal network.");
        }
        const std::vector<_IntermodalEdge*>& splitList = it->second;
        typename std::vector<_IntermodalEdge*>::const_iterator splitIt = splitList.begin();
        double totalLength = 0.;
        while (splitIt != splitList.end() && totalLength + (*splitIt)->getLength() < pos) {
            totalLength += (*splitIt)->getLength();
            ++splitIt;
        }
        return *splitIt;
    }

    /// @brief Returns the arriving intermodal connector at the given split offset
    _IntermodalEdge* getArrivalConnector(const E* e, const int splitIndex = 0) const {
        return myArrivalLookup.find(e)->second[splitIndex];
    }

    /// @brief Returns the outgoing pedestrian edge, which is either a walking area or a walking connector
    _IntermodalEdge* getWalkingConnector(const E* e) const {
        typename std::map<const N*, _IntermodalEdge*>::const_iterator it = myWalkingConnectorLookup.find(e->getToJunction());
        if (it == myWalkingConnectorLookup.end()) {
            const L* const sidewalk = getSidewalk<E, L>(e);
            if (e->isInternal() || sidewalk == 0) {
                return 0;
            }
            for (const auto& target : sidewalk->getOutgoingViaLanes()) {
                if (target.first->getEdge().isWalkingArea()) {
                    return getBothDirections(&target.first->getEdge()).first;
                }
            }
            return 0;
        }
        return it->second;
    }

    void addCarEdges(const std::vector<E*>& edges) {
        for (const E* const edge : edges) {
            if (edge->getFunction() == EDGEFUNC_NORMAL || edge->getFunction() == EDGEFUNC_INTERNAL) {
                myCarLookup[edge] = new CarEdge<E, L, N, V>(myNumericalID++, edge);
                addEdge(myCarLookup[edge]);
            }
        }
        for (const auto& edgePair : myCarLookup) {
            _IntermodalEdge* const carEdge = edgePair.second;
            for (const auto& suc : edgePair.first->getViaSuccessors()) {
                _IntermodalEdge* const sucCarEdge = getCarEdge(suc.first);
                _IntermodalEdge* const sucViaEdge = getCarEdge(suc.second);
                if (sucCarEdge != nullptr) {
                    carEdge->addSuccessor(sucCarEdge, sucViaEdge);
                }
            }
            if ((myCarWalkTransfer & ALL_JUNCTIONS) != 0) {
                _IntermodalEdge* const walkCon = getWalkingConnector(edgePair.first);
                if (walkCon != 0) {
                    carEdge->addSuccessor(walkCon);
                } else {
                    // we are on an edge where pedestrians are forbidden and want to continue on an arbitrary pedestrian edge
                    for (const E* const out : edgePair.first->getToJunction()->getOutgoing()) {
                        if (!out->isInternal() && !out->isTazConnector() && getSidewalk<E, L>(out) != 0) {
                            carEdge->addSuccessor(getBothDirections(out).first);
                        }
                    }
                    for (const E* const in : edgePair.first->getToJunction()->getIncoming()) {
                        if (!in->isInternal() && !in->isTazConnector() && getSidewalk<E, L>(in) != 0) {
                            carEdge->addSuccessor(getBothDirections(in).second);
                        }
                    }
                }
            }
            getDepartConnector(edgePair.first)->addSuccessor(carEdge);
            carEdge->addSuccessor(getArrivalConnector(edgePair.first));
        }
    }

    /// @brief Returns the associated car edge
    _IntermodalEdge* getCarEdge(const E* e) const {
        typename std::map<const E*, _IntermodalEdge*>::const_iterator it = myCarLookup.find(e);
        if (it == myCarLookup.end()) {
            return nullptr;
        }
        return it->second;
    }

    /// @brief Returns the associated stop edge
    _IntermodalEdge* getStopEdge(const std::string& stopId) const {
        auto it = myStopConnections.find(stopId);
        if (it == myStopConnections.end()) {
            return 0;
        }
        return it->second;
    }

    /** @brief Adds access edges for stopping places to the intermodal network
    *
    * This method creates an intermodal stop edge to represent the stopping place
    *  (if not present yet) and determines the edges which need to be splitted (usually the forward
    *  and the backward pedestrian edges and the car edge) and calls splitEdge for the
    *  actual split and the connection of the stop edge with access edges. After that it adds and adapts
    *  the depart and arrival connectors to the new edge(s).
    *
    * @param[in] stopId The id of the stop to add
    * @param[in] stopEdge The edge on which the stop is located
    * @param[in] pos The relative position on the edge where the stop is located
    * @param[in] category The type of stop
    */
    void addAccess(const std::string& stopId, const E* stopEdge, const double pos, const double length, const SumoXMLTag category) {
        assert(stopEdge != 0);
        if (myStopConnections.count(stopId) == 0) {
            myStopConnections[stopId] = new StopEdge<E, L, N, V>(stopId, myNumericalID++, stopEdge);
            addEdge(myStopConnections[stopId]);
        }
        _IntermodalEdge* const stopConn = myStopConnections[stopId];
        const L* lane = getSidewalk<E, L>(stopEdge);
        if (lane != 0) {
            const std::pair<_IntermodalEdge*, _IntermodalEdge*>& pair = getBothDirections(stopEdge);
            double relPos;
            bool needSplit;
            const int splitIndex = findSplitIndex(pair.first, pos, relPos, needSplit);
            _IntermodalEdge* const fwdSplit = needSplit ? new PedestrianEdge<E, L, N, V>(myNumericalID++, stopEdge, lane, true, pos) : nullptr;
            splitEdge(pair.first, splitIndex, fwdSplit, relPos, length, needSplit, stopConn);
            _IntermodalEdge* const backSplit = needSplit ? new PedestrianEdge<E, L, N, V>(myNumericalID++, stopEdge, lane, false, pos) : nullptr;
            splitEdge(pair.second, splitIndex, backSplit, relPos, length, needSplit, stopConn, false);
            _IntermodalEdge* carSplit = nullptr;
            if (myCarLookup.count(stopEdge) > 0) {
                if (needSplit) {
                    carSplit = new CarEdge<E, L, N, V>(myNumericalID++, stopEdge, pos);
                }
                splitEdge(myCarLookup[stopEdge], splitIndex, carSplit, relPos, length, needSplit, stopConn, true, false);
            }
            if (needSplit) {
                if (carSplit != nullptr && ((category == SUMO_TAG_PARKING_AREA && (myCarWalkTransfer & PARKING_AREAS) != 0) || (category == SUMO_TAG_BUS_STOP && (myCarWalkTransfer & PT_STOPS) != 0))) {
                    // adding access from car to walk
                    _IntermodalEdge* const beforeSplit = myAccessSplits[myCarLookup[stopEdge]][splitIndex];
                    for (_IntermodalEdge* conn : {
                                fwdSplit, backSplit
                            }) {
                        _AccessEdge* access = new _AccessEdge(myNumericalID++, beforeSplit, conn, length);
                        addEdge(access);
                        beforeSplit->addSuccessor(access);
                        access->addSuccessor(conn);
                    }
                }

                // fixing depart connections for the forward pedestrian, the backward pedestrian and the car edge
                _IntermodalEdge* const prevDep = getDepartConnector(stopEdge, splitIndex);
                const std::vector<_IntermodalEdge*>& backSplitList = myAccessSplits[pair.second];
                _IntermodalEdge* const backBeforeSplit = backSplitList[backSplitList.size() - 2 - splitIndex];
                _IntermodalEdge* const depConn = new _IntermodalEdge(stopEdge->getID() + "_depart_connector" + toString(pos), myNumericalID++, stopEdge, "!connector");
                depConn->addSuccessor(fwdSplit);
                depConn->addSuccessor(backBeforeSplit);
                depConn->setLength(fwdSplit->getLength());
                prevDep->removeSuccessor(backBeforeSplit);
                prevDep->addSuccessor(backSplit);
                prevDep->setLength(backSplit->getLength());
                if (carSplit != nullptr) {
                    depConn->addSuccessor(carSplit);
                }

                // fixing arrival connections for the forward pedestrian, the backward pedestrian and the car edge
                _IntermodalEdge* const prevArr = getArrivalConnector(stopEdge, splitIndex);
                _IntermodalEdge* const fwdBeforeSplit = myAccessSplits[pair.first][splitIndex];
                _IntermodalEdge* const arrConn = new _IntermodalEdge(stopEdge->getID() + "_arrival_connector" + toString(pos), myNumericalID++, stopEdge, "!connector");
                fwdSplit->addSuccessor(arrConn);
                backBeforeSplit->addSuccessor(arrConn);
                arrConn->setLength(fwdSplit->getLength());
                fwdSplit->removeSuccessor(prevArr);
                fwdBeforeSplit->addSuccessor(prevArr);
                prevArr->setLength(backSplit->getLength());
                if (carSplit != nullptr) {
                    carSplit->addSuccessor(arrConn);
                    carSplit->removeSuccessor(prevArr);
                    myAccessSplits[myCarLookup[stopEdge]][splitIndex]->addSuccessor(prevArr);
                }
                addConnectors(depConn, arrConn, splitIndex + 1);
            }
        }
    }

    void addSchedule(const SUMOVehicleParameter& pars, const std::vector<SUMOVehicleParameter::Stop>* addStops = 0) {
        SUMOTime lastUntil = 0;
        std::vector<SUMOVehicleParameter::Stop> validStops;
        if (addStops != 0) {
            // stops are part of a stand-alone route. until times are offsets from vehicle departure
            for (std::vector<SUMOVehicleParameter::Stop>::const_iterator s = addStops->begin(); s != addStops->end(); ++s) {
                if (myStopConnections.count(s->busstop) > 0) {
                    // compute stop times for the first vehicle
                    SUMOVehicleParameter::Stop stop = *s;
                    stop.until += pars.depart;
                    if (stop.until >= lastUntil) {
                        validStops.push_back(stop);
                        lastUntil = stop.until;
                    } else {
                        WRITE_WARNING("Ignoring unordered stop at '" + stop.busstop + "' at " + time2string(stop.until) + "  for vehicle '" + pars.id + "'.");
                    }
                }
            }
        }
        for (std::vector<SUMOVehicleParameter::Stop>::const_iterator s = pars.stops.begin(); s != pars.stops.end(); ++s) {
            // stops are part of the vehicle until times are absolute times for the first vehicle
            if (myStopConnections.count(s->busstop) > 0 && s->until >= lastUntil) {
                validStops.push_back(*s);
                lastUntil = s->until;
            }
        }
        if (validStops.size() < 2) {
            WRITE_WARNING("Ignoring public transport line '" + pars.line + "' with less than two usable stops.");
            return;
        }

        typename std::vector<_PTEdge*>& lineEdges = myPTLines[pars.line];
        if (lineEdges.empty()) {
            _IntermodalEdge* lastStop = 0;
            SUMOTime lastTime = 0;
            for (std::vector<SUMOVehicleParameter::Stop>::const_iterator s = validStops.begin(); s != validStops.end(); ++s) {
                _IntermodalEdge* currStop = myStopConnections[s->busstop];
                if (lastStop != 0) {
                    _PTEdge* const newEdge = new _PTEdge(s->busstop, myNumericalID++, lastStop, currStop->getEdge(), pars.line);
                    addEdge(newEdge);
                    newEdge->addSchedule(pars.id, lastTime, lastTime + pars.repetitionOffset * (pars.repetitionNumber - 1), pars.repetitionOffset, STEPS2TIME(s->until - lastTime));
                    lastStop->addSuccessor(newEdge);
                    newEdge->addSuccessor(currStop);
                    lineEdges.push_back(newEdge);
                }
                lastTime = s->until;
                lastStop = currStop;
            }
        } else {
            if (validStops.size() != lineEdges.size() + 1) {
                WRITE_WARNING("Number of stops for public transport line '" + pars.line + "' does not match earlier definitions, ignoring schedule.");
                return;
            }
            if (lineEdges.front()->getEntryStop() != myStopConnections[validStops.front().busstop]) {
                WRITE_WARNING("Different stop for '" + pars.line + "' compared to earlier definitions, ignoring schedule.");
                return;
            }
            typename std::vector<_PTEdge*>::const_iterator lineEdge = lineEdges.begin();
            typename std::vector<SUMOVehicleParameter::Stop>::const_iterator s = validStops.begin() + 1;
            for (; s != validStops.end(); ++s, ++lineEdge) {
                if ((*lineEdge)->getSuccessors(SVC_IGNORING)[0] != myStopConnections[s->busstop]) {
                    WRITE_WARNING("Different stop for '" + pars.line + "' compared to earlier definitions, ignoring schedule.");
                    return;
                }
            }
            SUMOTime lastTime = validStops.front().until;
            for (lineEdge = lineEdges.begin(), s = validStops.begin() + 1; lineEdge != lineEdges.end(); ++lineEdge, ++s) {
                (*lineEdge)->addSchedule(pars.id, lastTime, lastTime + pars.repetitionOffset * (pars.repetitionNumber - 1), pars.repetitionOffset, STEPS2TIME(s->until - lastTime));
                lastTime = s->until;
            }
        }
    }


private:
    /** @brief Returns where to insert or use the split edge
    *
    * This method determines whether an edge needs to be split at the given position
    *  (if there is not already a split nearby) and returns the corresponding index in the split list.
    *
    * @param[in] toSplit The first edge in the split list
    * @param[in] pos The relative position on the edge where the stop is located
    * @param[out] relPos The relative position on the splitted edge
    * @param[out] needSplit whether a new split is needed or we reuse an exisiting one
    * @return the index in the split list where the split edge needs to be added or reused
    */
    int findSplitIndex(_IntermodalEdge* const toSplit, const double pos, double& relPos, bool& needSplit) {
        relPos = pos;
        needSplit = true;
        int splitIndex = 0;
        std::vector<_IntermodalEdge*>& splitList = myAccessSplits[toSplit];
        if (!splitList.empty()) {
            for (const _IntermodalEdge* const split : splitList) {
                if (relPos < split->getLength() + POSITION_EPS) {
                    break;
                }
                relPos -= split->getLength();
                splitIndex++;
            }
            assert(splitIndex < (int)splitList.size());
            if (splitIndex + 1 < (int)splitList.size() && fabs(relPos - splitList[splitIndex]->getLength()) < POSITION_EPS) {
                needSplit = false;
            }
        }
        return splitIndex;
    }

    /** @brief Splits an edge (if necessary) and connects it to a stopping edge
    *
    * This method determines whether an edge needs to be split at the given position
    *  (if there is not already a split nearby) and connects the stop edge via new access edges.
    *
    * @param[in] toSplit The first edge in the split list
    * @param[in] afterSplit The edge to add if a split is performed
    * @param[in] pos The relative position on the edge where the stop is located
    * @param[in] stopConn The stop edge to connect to
    * @param[in] forward whether we are aplitting a forward edge (backward edges get different names)
    * @param[in] addExit whether we can just enter the stop or exit as well (cars should not exit yet)
    */
    void splitEdge(_IntermodalEdge* const toSplit, int splitIndex,
                   _IntermodalEdge* afterSplit, const double relPos, const double length, const bool needSplit,
                   _IntermodalEdge* const stopConn, const bool forward = true, const bool addExit = true) {
        std::vector<_IntermodalEdge*>& splitList = myAccessSplits[toSplit];
        if (splitList.empty()) {
            splitList.push_back(toSplit);
        }
        if (!forward) {
            splitIndex = (int)splitList.size() - 1 - splitIndex;
            if (!needSplit) {
                splitIndex--;
            }
        }
        _IntermodalEdge* beforeSplit = splitList[splitIndex];
        if (needSplit) {
            addEdge(afterSplit);
            beforeSplit->transferSuccessors(afterSplit);
            beforeSplit->addSuccessor(afterSplit);
            if (forward) {
                afterSplit->setLength(beforeSplit->getLength() - relPos);
                beforeSplit->setLength(relPos);
            } else {
                afterSplit->setLength(relPos);
                beforeSplit->setLength(beforeSplit->getLength() - relPos);
                // rename backward edges for easier referencing
                const std::string newID = beforeSplit->getID();
                beforeSplit->setID(afterSplit->getID());
                afterSplit->setID(newID);
            }
            splitList.insert(splitList.begin() + splitIndex + 1, afterSplit);
        } else {
            // don't split, use the present split edges
            afterSplit = splitList[splitIndex + 1];
        }
        // add access to / from edge
        _AccessEdge* access = new _AccessEdge(myNumericalID++, beforeSplit, stopConn, length);
        addEdge(access);
        beforeSplit->addSuccessor(access);
        access->addSuccessor(stopConn);
        if (addExit) {
            // pedestrian case only, exit from public to pedestrian
            _AccessEdge* exit = new _AccessEdge(myNumericalID++, stopConn, afterSplit, length);
            addEdge(exit);
            stopConn->addSuccessor(exit);
            exit->addSuccessor(afterSplit);
        }
    }


private:
    /// @brief the edge dictionary
    std::vector<_IntermodalEdge*> myEdges;

    /// @brief retrieve the forward and backward edge for the given input edge E
    std::map<const E*, EdgePair> myBidiLookup;

    /// @brief retrieve the depart edges for the given input edge E
    std::map<const E*, std::vector<_IntermodalEdge*> > myDepartLookup;

    /// @brief retrieve the arrival edges for the given input edge E
    std::map<const E*, std::vector<_IntermodalEdge*> > myArrivalLookup;

    /// @brief the walking connector edge (fake walking area)
    std::map<const N*, _IntermodalEdge*> myWalkingConnectorLookup;

    /// @brief retrieve the car edge for the given input edge E
    std::map<const E*, _IntermodalEdge*> myCarLookup;

    /// @brief retrieve the public transport edges for the given line
    std::map<std::string, std::vector<_PTEdge*> > myPTLines;

    /// @brief retrieve the representing edge for the given stopping place
    std::map<std::string, _IntermodalEdge*> myStopConnections;

    /// @brief retrieve the splitted edges for the given "original"
    std::map<_IntermodalEdge*, std::vector<_IntermodalEdge*> > myAccessSplits;

    int myNumericalID;
    const int myCarWalkTransfer;

private:
    /// @brief Invalidated assignment operator
    IntermodalNetwork& operator=(const IntermodalNetwork& s);

};


#endif

/****************************************************************************/
