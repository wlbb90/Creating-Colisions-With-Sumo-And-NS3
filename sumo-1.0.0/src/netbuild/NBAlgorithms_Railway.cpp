/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2012-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NBAlgorithms_Railway.cpp
/// @author  Jakob Erdmann
/// @author  Melanie Weber
/// @date    29. March 2018
/// @version $Id$
///
// Algorithms for highway on-/off-ramps computation
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <cassert>
#include <utils/options/OptionsCont.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/TplConvert.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/iodevices/OutputDevice_String.h>
#include <utils/vehicle/DijkstraRouter.h>
#include "NBNetBuilder.h"
#include "NBAlgorithms.h"
#include "NBNodeCont.h"
#include "NBEdgeCont.h"
#include "NBNode.h"
#include "NBEdge.h"
#include "NBVehicle.h"
#include "NBAlgorithms_Railway.h"

//#define DEBUG_SEQSTOREVERSE
#define DEBUGNODEID  "gneJ34"
#define DEBUGNODEID2  "28842974"
#define DEBUGEDGEID  "22820560#0"
#define DEBUGCOND(obj) ((obj != 0 && (obj)->getID() == DEBUGNODEID))

#define SHARP_THRESHOLD_SAMEDIR 100
#define SHARP_THRESHOLD 80

// ===========================================================================
// static members
// ===========================================================================

// ---------------------------------------------------------------------------
// Track methods
// ---------------------------------------------------------------------------

void
NBRailwayTopologyAnalyzer::Track::addSuccessor(Track* track) {
    successors.push_back(track);
    viaSuccessors.push_back(std::make_pair(track, nullptr));
    minPermissions &= track->edge->getPermissions();
}

const std::vector<NBRailwayTopologyAnalyzer::Track*>&
NBRailwayTopologyAnalyzer::Track::getSuccessors(SUMOVehicleClass svc) const {
    if ((minPermissions & svc) != 0) {
        return successors;
    } else {
        if (svcSuccessors.count(svc) == 0) {
            std::vector<Track*> succ;
            for (Track* t : successors) {
                if ((t->edge->getPermissions() & svc) != 0) {
                    succ.push_back(t);
                }
            }
            svcSuccessors[svc] = succ;
        }
        return svcSuccessors[svc];
    }
}

const std::vector<std::pair<const NBRailwayTopologyAnalyzer::Track*, const NBRailwayTopologyAnalyzer::Track*> >&
NBRailwayTopologyAnalyzer::Track::getViaSuccessors(SUMOVehicleClass svc) const {
    if ((minPermissions & svc) != 0) {
        return viaSuccessors;
    } else {
        if (svcViaSuccessors.count(svc) == 0) {
            std::vector<std::pair<const Track*, const Track*> >& succ = svcViaSuccessors[svc];
            for (const Track* const t : successors) {
                if ((t->edge->getPermissions() & svc) != 0) {
                    succ.push_back(std::make_pair(t, nullptr));
                }
            }
        }
        return svcViaSuccessors[svc];
    }
}

// ===========================================================================
// method definitions
// ===========================================================================
void
NBRailwayTopologyAnalyzer::analyzeTopology(NBNetBuilder& nb) {
    getBrokenRailNodes(nb, true);
}


void
NBRailwayTopologyAnalyzer::repairTopology(NBNetBuilder& nb) {
    extendBidiEdges(nb);
    reverseEdges(nb);
    addBidiEdgesForBufferStops(nb);
    addBidiEdgesBetweenSwitches(nb);
    addBidiEdgesForStops(nb);
    if (OptionsCont::getOptions().getBool("railway.topology.repair.connect-straight")) {
        addBidiEdgesForStraightConnectivity(nb);
    }
}


void
NBRailwayTopologyAnalyzer::makeAllBidi(NBNetBuilder& nb) {
    int numRailEdges = 0;
    int numBidiEdges = 0;
    int numNotCenterEdges = 0;
    int numAddedBidiEdges = 0;
    for (NBEdge* edge : nb.getEdgeCont().getAllEdges()) {
        if ((edge->getPermissions() & SVC_RAIL_CLASSES) != 0) {
            numRailEdges++;
            // rebuild connections if given from an earlier network
            edge->invalidateConnections(true);
            if (!edge->isBidiRail()) {
                if (edge->getLaneSpreadFunction() == LANESPREAD_CENTER) {
                    NBEdge* e2 = addBidiEdge(nb, edge, false);
                    if (e2 != nullptr) {
                        numAddedBidiEdges++;
                    }
                } else {
                    numNotCenterEdges++;
                }
            } else {
                numBidiEdges++;
            }
        }
    }
    WRITE_MESSAGE("Added " + toString(numAddedBidiEdges) + " bidi-edges to ensure that all tracks are usable in both directions.");
    if (numNotCenterEdges) {
        WRITE_WARNING("Ignore " + toString(numNotCenterEdges) + " edges because they have the wrong spreadType");
    }
}

NBEdge*
NBRailwayTopologyAnalyzer::addBidiEdge(NBNetBuilder& nb, NBEdge* edge, bool update) {
    assert(edge->getLaneSpreadFunction() == LANESPREAD_CENTER);
    assert(!edge->isBidiRail());
    const std::string id2 = (edge->getID()[0] == '-'
                             ? edge->getID().substr(1)
                             : "-" + edge->getID());
    if (nb.getEdgeCont().retrieve(id2) == nullptr) {
        NBEdge* e2 = new NBEdge(id2, edge->getToNode(), edge->getFromNode(),
                                edge, edge->getGeometry().reverse());
        nb.getEdgeCont().insert(e2);
        if (update) {
            updateTurns(edge);
            // reconnected added edges
            e2->getFromNode()->invalidateIncomingConnections();
        }
        return e2;
    } else {
        WRITE_WARNING("Could not add bidi-edge '" + id2 + "'.");
        return nullptr;
    }
}

void
NBRailwayTopologyAnalyzer::getRailEdges(const NBNode* node,
                                        EdgeVector& inEdges, EdgeVector& outEdges) {
    for (NBEdge* e : node->getIncomingEdges()) {
        if ((e->getPermissions() & SVC_RAIL_CLASSES) != 0) {
            inEdges.push_back(e);
        }
    }
    for (NBEdge* e : node->getOutgoingEdges()) {
        if ((e->getPermissions() & SVC_RAIL_CLASSES) != 0) {
            outEdges.push_back(e);
        }
    }
}



std::set<NBNode*>
NBRailwayTopologyAnalyzer::getBrokenRailNodes(NBNetBuilder& nb, bool verbose) {
    std::set<NBNode*> brokenNodes;;
    OutputDevice& device = OutputDevice::getDevice(verbose
                           ? OptionsCont::getOptions().getString("railway.topology.output")
                           : "/dev/null");

    device.writeXMLHeader("railwayTopology", "");
    std::set<NBNode*> railNodes = getRailNodes(nb, verbose);
    std::map<std::pair<int, int>, std::set<NBNode*, ComparatorIdLess> > types;
    std::set<NBEdge*, ComparatorIdLess> bidiEdges;
    std::set<NBEdge*, ComparatorIdLess> bufferStops;
    for (NBNode* node : railNodes) {
        EdgeVector inEdges, outEdges;
        getRailEdges(node, inEdges, outEdges);
        types[std::make_pair((int)inEdges.size(), (int)outEdges.size())].insert(node);
        for (NBEdge* e : outEdges) {
            if (e->isBidiRail() && bidiEdges.count(e->getTurnDestination(true)) == 0) {
                NBEdge* primary = e;
                NBEdge* secondary = e->getTurnDestination(true);
                if (e->getID()[0] == '-') {
                    std::swap(primary, secondary);
                } else if (primary->getID()[0] != '-' && secondary->getID()[0] != '-' && secondary->getID() < primary->getID()) {
                    std::swap(primary, secondary);
                }
                if (bidiEdges.count(secondary) == 0) {
                    // avoid duplicate when both ids start with '-'
                    bidiEdges.insert(primary);
                }
            }
        }
    }

    int numBrokenA = 0;
    int numBrokenB = 0;
    int numBrokenC = 0;
    int numBrokenD = 0;
    int numBufferStops = 0;
    if (verbose && types.size() > 0) {
        WRITE_MESSAGE("Railway nodes by number of incoming,outgoing edges:")
    }
    for (auto it : types) {
        int numBrokenType = 0;
        device.openTag("railNodeType");
        int in = it.first.first;
        int out = it.first.second;
        device.writeAttr("in", in);
        device.writeAttr("out", out);
        for (NBNode* n : it.second) {
            device.openTag(SUMO_TAG_NODE);
            device.writeAttr(SUMO_ATTR_ID, n->getID());
            EdgeVector inRail, outRail;
            getRailEdges(n, inRail, outRail);
            // check if there is a mismatch between angle and edge direction
            // a) edge pair angle supports driving but both are outgoing
            // b) edge pair angle supports driving but both are incoming
            // c) an incoming edge has a sharp angle to all outgoing edges
            // d) an outgoing edge has a sharp angle from all incoming edges

            std::string broken = "";
            if (in < 2 && hasStraightPair(n, outRail, outRail)) {
                broken += "a";
                numBrokenA++;
            }
            if (out < 2 && hasStraightPair(n, inRail, inRail)) {
                broken += "b";
                numBrokenB++;
            }
            if (out > 0) {
                for (NBEdge* e : inRail) {
                    EdgeVector tmp;
                    tmp.push_back(e);
                    if (allSharp(n, tmp, outRail)) {
                        broken += "c";
                        numBrokenC++;
                        break;
                    }
                }
            }
            if (in > 0) {
                for (NBEdge* e : outRail) {
                    EdgeVector tmp;
                    tmp.push_back(e);
                    if (allSharp(n, inRail, tmp)) {
                        broken += "d";
                        numBrokenD++;
                        break;
                    }
                }
            }
            // do not mark bidi nodes as broken
            if (((in == 1 && out == 1) || (in == 2 && out == 2))
                    && allBidi(inRail) && allBidi(outRail)) {
                broken = "";
            }

            if (broken.size() > 0) {
                device.writeAttr("broken", broken);
                brokenNodes.insert(n);
                numBrokenType++;
            }
            if (TplConvert::_2boolSec(n->getParameter("buffer_stop", "false").c_str(), false)) {
                device.writeAttr("buffer_stop", "true");
                numBufferStops++;
            }
            device.closeTag();
        }
        device.closeTag();
        if (verbose) {
            WRITE_MESSAGE("   " + toString(it.first.first) + "," + toString(it.first.second)
                          + " count: " + toString(it.second.size()) + " broken: " + toString(numBrokenType));
        }

    }
    if (verbose) {
        WRITE_MESSAGE("Found " + toString(brokenNodes.size()) + " broken railway nodes "
                      + "(A=" + toString(numBrokenA)
                      + " B=" + toString(numBrokenB)
                      + " C=" + toString(numBrokenC)
                      + " D=" + toString(numBrokenD)
                      + ")");
        WRITE_MESSAGE("Found " + toString(numBufferStops) + " railway nodes marked as buffer_stop");
    }

    for (NBEdge* e : bidiEdges) {
        device.openTag("bidiEdge");
        device.writeAttr(SUMO_ATTR_ID, e->getID());
        device.writeAttr("bidi", e->getTurnDestination(true)->getID());
        device.closeTag();
    }
    if (verbose) {
        WRITE_MESSAGE("Found " + toString(bidiEdges.size()) + " bidirectional rail edges");
    }

    device.close();
    return brokenNodes;
}


std::set<NBNode*>
NBRailwayTopologyAnalyzer::getRailNodes(NBNetBuilder& nb, bool verbose) {
    std::set<NBNode*> railNodes;

    NBEdgeCont& ec = nb.getEdgeCont();
    int numRailEdges = 0;
    for (auto it = ec.begin(); it != ec.end(); it++) {
        if (isRailway(it->second->getPermissions())) {
            numRailEdges++;
            railNodes.insert(it->second->getFromNode());
            railNodes.insert(it->second->getToNode());

        }
    }
    std::set<NBNode*> railSignals;
    for (NBNode* node : railNodes) {
        if (node->getType() == NODETYPE_RAIL_SIGNAL) {
            railSignals.insert(node);
        }
    }
    if (verbose) {
        WRITE_MESSAGE("Found " + toString(numRailEdges) + " railway edges and " + toString(railNodes.size()) + " railway nodes (" + toString(railSignals.size()) + " signals).");
    }
    return railNodes;
}


bool
NBRailwayTopologyAnalyzer::isStraight(const NBNode* node, const NBEdge* e1, const NBEdge* e2) {
    const double relAngle = NBHelpers::normRelAngle(e1->getAngleAtNode(node), e2->getAngleAtNode(node));
    /*
    std::cout << "  isStraight n=" << node->getID()
        << " e1=" << e1->getID()
        << " e2=" << e2->getID()
        << " a1=" << e1->getAngleAtNode(node)
        << " a2=" << e2->getAngleAtNode(node)
        << " rel=" << relAngle
        << "\n";
        */
    if ((e1->getToNode() == node && e2->getFromNode() == node)
            || (e1->getFromNode() == node && e2->getToNode() == node)) {
        // edges go in the same direction
        return fabs(relAngle) < SHARP_THRESHOLD;
    } else {
        // edges go in the opposite direction (both incoming or outgoing)
        return fabs(relAngle) > SHARP_THRESHOLD_SAMEDIR;
    }
}


bool
NBRailwayTopologyAnalyzer::hasStraightPair(const NBNode* node, const EdgeVector& edges,
        const EdgeVector& edges2) {
#ifdef DEBUG_SEQSTOREVERSE
    //if (node->getID() == DEBUGNODEID2) {
    //    std::cout << " edges=" << toString(edges) << " edges2=" << toString(edges2) << "\n";
    //}
#endif
    for (NBEdge* e1 : edges) {
        for (NBEdge* e2 : edges2) {
            //if (e1->getID() == "195411601#2" && e2->getID() == "93584120#3") {
            //    std::cout
            //        << " DEBUG normRelA=" << NBHelpers::normRelAngle(
            //                    e1->getAngleAtNode(node),
            //                    e2->getAngleAtNode(node))
            //        << "\n";
            //}
            if (e1 != e2 && isStraight(node, e1, e2)) {
                return true;
            }
        }
    }
    return false;
}


bool
NBRailwayTopologyAnalyzer::allBroken(const NBNode* node, NBEdge* candOut, const EdgeVector& in, const EdgeVector& out) {
    for (NBEdge* e : in) {
        if (e != candOut && isStraight(node, e, candOut)) {
            if (gDebugFlag1) {
                std::cout << " isStraight e=" << e->getID() << " candOut=" << candOut->getID() << "\n";
            }
            return false;
        }
    }
    for (NBEdge* e : out) {
        if (e != candOut && !isStraight(node, e, candOut)) {
            if (gDebugFlag1) {
                std::cout << " isSharp e=" << e->getID() << " candOut=" << candOut->getID() << "\n";
            }
            return false;
        }
    }
    return true;
}


bool
NBRailwayTopologyAnalyzer::allSharp(const NBNode* node, const EdgeVector& in, const EdgeVector& out, bool countBidiAsSharp) {
    bool allBidi = true;
    for (NBEdge* e1 : in) {
        for (NBEdge* e2 : out) {
            if (e1 != e2 && isStraight(node, e1, e2)) {
                return false;
            }
            if (!e1->isBidiRail(true)) {
                //std::cout << " allSharp node=" << node->getID() << " e1=" << e1->getID() << " is not bidi\n";
                allBidi = false;
            }
        }
    }
    return !allBidi || countBidiAsSharp;
}


bool
NBRailwayTopologyAnalyzer::allBidi(const EdgeVector& edges) {
    for (NBEdge* e : edges) {
        if (!e->isBidiRail()) {
            return false;
        }
    }
    return true;
}


int
NBRailwayTopologyAnalyzer::extendBidiEdges(NBNetBuilder& nb) {
    int added = 0;
    std::set<NBNode*> railNodes = getRailNodes(nb);
    NBEdgeCont& ec = nb.getEdgeCont();
    for (auto it = ec.begin(); it != ec.end(); it++) {
        NBEdge* e = it->second;
        if (e->isBidiRail()) {
            added += extendBidiEdges(nb, e->getFromNode(), e->getTurnDestination(true));
            added += extendBidiEdges(nb, e->getToNode(), e);
        }
    }
    //if (added > 0) {
    //    std::cout << "Addeded " << added << " bidi-edges as extension of existing bidi edges\n";
    //}
    return added;
}


int
NBRailwayTopologyAnalyzer::extendBidiEdges(NBNetBuilder& nb, NBNode* node, NBEdge* bidiIn) {
    assert(bidiIn->getToNode() == node);
    NBEdge* bidiOut = bidiIn->getTurnDestination(true);
    if (bidiOut == nullptr) {
        WRITE_WARNING("Could not find bidi-edge for edge '" + bidiIn->getID() + "'");
        return 0;
    }
    EdgeVector tmpBidiOut;
    tmpBidiOut.push_back(bidiOut);
    EdgeVector tmpBidiIn;
    tmpBidiIn.push_back(bidiIn);
    int added = 0;
    EdgeVector inRail, outRail;
    getRailEdges(node, inRail, outRail);
    for (NBEdge* cand : outRail) {
        //std::cout << " extendBidiEdges n=" << node->getID() << " bidiIn=" << bidiIn->getID() << " cand=" << cand->getID() << " isStraight=" << isStraight(node, bidiIn, cand) <<  " allSharp=" << allSharp(node, inRail, tmpBidiOut, true) << "\n";
        if (!cand->isBidiRail() && isStraight(node, bidiIn, cand)
                && cand->getLaneSpreadFunction() == LANESPREAD_CENTER
                && allSharp(node, inRail, tmpBidiOut, true)) {
            NBEdge* e2 = addBidiEdge(nb, cand);
            if (e2 != nullptr) {
                added += 1 + extendBidiEdges(nb, cand->getToNode(), cand);
            }
        }
    }
    for (NBEdge* cand : inRail) {
        //std::cout << " extendBidiEdges n=" << node->getID() << " bidiOut=" << bidiOut->getID() << " cand=" << cand->getID() << " isStraight=" << isStraight(node, cand, bidiOut) << " allSharp=" << allSharp(node, outRail, tmpBidiIn, true) << "\n";
        if (!cand->isBidiRail() && isStraight(node, cand, bidiOut)
                && cand->getLaneSpreadFunction() == LANESPREAD_CENTER
                && allSharp(node, outRail, tmpBidiIn, true)) {
            NBEdge* e2 = addBidiEdge(nb, cand);
            if (e2 != nullptr) {
                added += 1 + extendBidiEdges(nb, cand->getFromNode(), e2);
            }
        }
    }
    return added;
}


void
NBRailwayTopologyAnalyzer::reverseEdges(NBNetBuilder& nb) {
    std::set<NBNode*> brokenNodes = getBrokenRailNodes(nb);
    // find reversible edge sequences between broken nodes
    std::vector<EdgeVector> seqsToReverse;
    for (NBNode* n : brokenNodes) {
        EdgeVector inRail, outRail;
        getRailEdges(n, inRail, outRail);
        for (NBEdge* start : outRail) {
            EdgeVector tmp;
            tmp.push_back(start);
            // only reverse edges where the node would be unbroken afterwards
            if (!allBroken(n, start, inRail, outRail)
                    || (inRail.size() == 1 && outRail.size() == 1)) {
#ifdef DEBUG_SEQSTOREVERSE
                if (n->getID() == DEBUGNODEID) {
                    std::cout << " abort at start n=" << n->getID() << " (not all broken)\n";
                }
#endif
                continue;
            }
            //std::cout << " get sequences from " << start->getID() << "\n";
            bool forward = true;
            EdgeVector seq;
            while (forward) {
                seq.push_back(start);
                //std::cout << " seq=" << toString(seq) << "\n";
                NBNode* n2 = start->getToNode();
                EdgeVector inRail2, outRail2;
                getRailEdges(n2, inRail2, outRail2);
                if (brokenNodes.count(n2) != 0) {
                    EdgeVector tmp2;
                    tmp2.push_back(start);
                    if (allBroken(n2, start, outRail2, inRail2)) {
                        seqsToReverse.push_back(seq);
                    } else {
#ifdef DEBUG_SEQSTOREVERSE
                        if (n->getID() == DEBUGNODEID) {
                            std::cout << " abort at n2=" << n2->getID() << " (not all broken)\n";
                        }
#endif
                    }
                    forward = false;
                } else {
                    if (outRail2.size() == 0) {
                        // stop at network border
                        forward = false;
#ifdef DEBUG_SEQSTOREVERSE
                        if (n->getID() == DEBUGNODEID) {
                            std::cout << " abort at n2=" << n2->getID() << " (border)\n";
                        }
#endif
                    } else if (outRail2.size() > 1 || inRail2.size() > 1) {
                        // stop at switch
                        forward = false;
#ifdef DEBUG_SEQSTOREVERSE
                        if (n->getID() == DEBUGNODEID) {
                            std::cout << " abort at n2=" << n2->getID() << " (switch)\n";
                        }
#endif
                    } else {
                        start = outRail2.front();
                    }
                }
            }
        }
    }
    // sort by sequence length
    if (seqsToReverse.size() > 0) {
        WRITE_MESSAGE("Found " + toString(seqsToReverse.size()) + " reversible edge sequences between broken rail nodes");
    }
    std::sort(seqsToReverse.begin(), seqsToReverse.end(),
    [](const EdgeVector & a, const EdgeVector & b) {
        return a.size() < b.size();
    });
    int numReversed = 0;
    std::set<NBNode*> affectedEndpoints;
    std::set<std::string> reversedIDs;
    std::map<int, int> seqLengths;
    for (EdgeVector& seq : seqsToReverse) {
        NBNode* seqStart = seq.front()->getFromNode();
        NBNode* seqEnd = seq.back()->getToNode();
        // avoid reversing sequenes on both sides of a broken node
        if (affectedEndpoints.count(seqStart) == 0
                && affectedEndpoints.count(seqEnd) == 0) {
            affectedEndpoints.insert(seqStart);
            affectedEndpoints.insert(seqEnd);
            //WRITE_MESSAGE("  reversed seq=" + toString(seq));
            for (NBEdge* e : seq) {
                e->reinitNodes(e->getToNode(), e->getFromNode());
                e->setGeometry(e->getGeometry().reverse());
                reversedIDs.insert(e->getID());
            }
            seqLengths[(int)seq.size()]++;
            numReversed++;
        }
    }
    if (numReversed > 0) {
        WRITE_MESSAGE("Reversed " + toString(numReversed) + " sequences (count by length: " + joinToString(seqLengths, " ", ":") + ")");
        for (auto& item : nb.getPTStopCont().getStops()) {
            if (reversedIDs.count(item.second->getEdgeId())) {
                item.second->findLaneAndComputeBusStopExtend(nb.getEdgeCont());
            }
        }
    }
}


void
NBRailwayTopologyAnalyzer::addBidiEdgesForBufferStops(NBNetBuilder& nb) {
    std::set<NBNode*> brokenNodes = getBrokenRailNodes(nb);
    std::set<NBNode*> railNodes = getRailNodes(nb);
    // find buffer stops and ensure that thay are connect to the network in both directions
    int numBufferStops = 0;
    int numAddedBidiTotal = 0;
    for (NBNode* node : railNodes) {
        if (TplConvert::_2boolSec(node->getParameter("buffer_stop", "false").c_str(), false)) {
            if (node->getEdges().size() != 1) {
                WRITE_WARNING("Ignoring buffer stop junction '" + node->getID() + "' with " + toString(node->getEdges().size()) + " edges\n");
                continue;
            }
            int numAddedBidi = 0;
            numBufferStops++;
            NBEdge* prev = 0;
            NBEdge* prev2 = 0;
            EdgeVector inRail, outRail;
            getRailEdges(node, inRail, outRail);
            bool addAway = true; // add new edges away from buffer stop
            while (prev == 0 || (inRail.size() + outRail.size()) == 3) {
                NBEdge* e = 0;
                if (prev == 0) {
                    assert(node->getEdges().size() == 1);
                    e = node->getEdges().front();
                    addAway = node == e->getToNode();
                } else {
                    if (addAway) {
                        // XXX if node is broken we need to switch direction
                        assert(inRail.size() == 2);
                        e = inRail.front() == prev2 ? inRail.back() : inRail.front();
                    } else {
                        // XXX if node is broken we need to switch direction
                        assert(outRail.size() == 2);
                        e = outRail.front() == prev2 ? outRail.back() : outRail.front();
                    }
                }
                e->setLaneSpreadFunction(LANESPREAD_CENTER);
                NBNode* e2From = 0;
                NBNode* e2To = 0;
                if (addAway) {
                    e2From = node;
                    e2To = e->getFromNode();
                    node = e2To;
                } else {
                    e2From = e->getToNode();
                    e2To = node;
                    node = e2From;
                }
                NBEdge* e2 = addBidiEdge(nb, e);
                if (e2 == nullptr) {
                    break;
                }
                prev = e;
                prev2 = e2;
                numAddedBidi++;
                numAddedBidiTotal++;
                inRail.clear();
                outRail.clear();
                getRailEdges(node, inRail, outRail);
            }
            //if (numAddedBidi > 0) {
            //    WRITE_MESSAGE(" added " + toString(numAddedBidi) + " edges between buffer stop junction '" + bufferStop->getID() + "' and junction '" + node->getID() + "'");
            //}
        }
    }
    if (numAddedBidiTotal > 0) {
        WRITE_MESSAGE("Added " + toString(numAddedBidiTotal) + " edges to connect " + toString(numBufferStops) + " buffer stops in both directions.");
    }
}

NBEdge*
NBRailwayTopologyAnalyzer::isBidiSwitch(const NBNode* n) {
    EdgeVector inRail, outRail;
    getRailEdges(n, inRail, outRail);
    if (inRail.size() == 2 && outRail.size() == 1 && isStraight(n, inRail.front(), inRail.back())) {
        if (isStraight(n, inRail.front(), outRail.front())) {
            return inRail.front();
        } else if (isStraight(n, inRail.back(), outRail.front())) {
            return inRail.back();
        }
    }
    if (inRail.size() == 1 && outRail.size() == 2 && isStraight(n, outRail.front(), outRail.back())) {
        if (isStraight(n, outRail.front(), inRail.front())) {
            return outRail.front();
        } else if (isStraight(n, outRail.back(), inRail.front())) {
            return outRail.back();
        }
    }
    return nullptr;
}


void
NBRailwayTopologyAnalyzer::addBidiEdgesBetweenSwitches(NBNetBuilder& nb) {
    std::set<NBNode*> brokenNodes = getBrokenRailNodes(nb);
    std::map<int, int> seqLengths;
    int numAdded = 0;
    int numSeqs = 0;
    for (NBNode* n : brokenNodes) {
        NBEdge* edge = isBidiSwitch(n);
        if (edge != nullptr) {
            std::vector<NBNode*> nodeSeq;
            EdgeVector edgeSeq;
            NBNode* prev = n;
            nodeSeq.push_back(prev);
            edgeSeq.push_back(edge);
            bool forward = true;
            //std::cout << "Looking for potential bidi-edge sequence starting at junction '" << n->getID() << "' with edge '" + edge->getID() << "'\n";
            // find a suitable end point for adding bidi edges
            while (forward) {
                NBNode* next = edge->getFromNode() == prev ? edge->getToNode() : edge->getFromNode();
                EdgeVector allRail;
                getRailEdges(next, allRail, allRail);
                if (allRail.size() == 2 && isStraight(next, allRail.front(), allRail.back())) {
                    prev = next;
                    edge = allRail.front() == edge ? allRail.back() : allRail.front();
                    nodeSeq.push_back(prev);
                    edgeSeq.push_back(edge);
                } else {
                    forward = false;
                    EdgeVector inRail2, outRail2;
                    getRailEdges(next, inRail2, outRail2);
                    if (isBidiSwitch(next) == edge) {
                        // suitable switch found as endpoint, add reverse edges
                        //WRITE_MESSAGE("Adding " + toString(edgeSeq.size())
                        //        + " bidi-edges between switches junction '" + n->getID() + "' and junction '" + next->getID() + "'");
                        for (NBEdge* e : edgeSeq) {
                            addBidiEdge(nb, e);
                        }
                        seqLengths[(int)edgeSeq.size()]++;
                        numSeqs++;
                        numAdded += (int)edgeSeq.size();
                    } else {
                        //std::cout << " sequence ended at junction " << next->getID()
                        //    << " in=" << inRail2.size()
                        //    << " out=" << outRail2.size()
                        //    << " bidiSwitch=" << Named::getIDSecure(isBidiSwitch(next))
                        //    << "\n";
                    }

                }
            }

        }
    }
    if (seqLengths.size() > 0) {
        WRITE_MESSAGE("Added " + toString(numAdded) + " bidi-edges between " + toString(numSeqs) + " pairs of railway switches (count by length: " + joinToString(seqLengths, " ", ":") + ")");
    }
}


void
NBRailwayTopologyAnalyzer::addBidiEdgesForStops(NBNetBuilder& nb) {
    // generate bidirectional routing graph
    NBEdgeCont& ec = nb.getEdgeCont();
    std::vector<Track*> tracks;
    for (NBEdge* edge : nb.getEdgeCont().getAllEdges()) {
        tracks.push_back(new Track(edge));
    }
    const int numEdges = (int)tracks.size();
    for (NBEdge* edge : nb.getEdgeCont().getAllEdges()) {
        tracks.push_back(new Track(edge, (int)tracks.size(), edge->getID() + "_reverse"));
    }
    // add special tracks for starting end ending in both directions
    std::map<NBEdge*, std::pair<Track*, Track*> > stopTracks;
    for (NBEdge* edge : nb.getEdgeCont().getAllEdges()) {
        if ((edge->getPermissions() & SVC_RAIL_CLASSES) != 0) {
            Track* start = new Track(edge, (int)tracks.size(), edge->getID() + "_start");
            tracks.push_back(start);
            Track* end = new Track(edge, (int)tracks.size(), edge->getID() + "_end");
            tracks.push_back(end);
            stopTracks[edge] = std::make_pair(start, end);
        }
    }
    // set successors based on angle (connections are not yet built)
    for (NBNode* node : getRailNodes(nb)) {
        EdgeVector railEdges;
        getRailEdges(node, railEdges, railEdges);
        for (NBEdge* e1 : railEdges) {
            for (NBEdge* e2 : railEdges) {
                if (e1 != e2 && isStraight(node, e1, e2)) {
                    int i = e1->getNumericalID();
                    int i2 = e2->getNumericalID();
                    if (e1->getToNode() == node) {
                        if (e2->getFromNode() == node) {
                            // case 1) plain forward connection
                            tracks[i]->addSuccessor(tracks[i2]);
                            // reverse edge (numerical id incremented by numEdges)
                            tracks[i2 + numEdges]->addSuccessor(tracks[i + numEdges]);
                        } else {
                            // case 2) both edges pointing towards each ohter
                            tracks[i]->addSuccessor(tracks[i2 + numEdges]);
                            tracks[i2]->addSuccessor(tracks[i + numEdges]);
                        }
                    } else {
                        if (e2->getFromNode() == node) {
                            // case 3) both edges pointing away from each other
                            tracks[i + numEdges]->addSuccessor(tracks[i2]);
                            tracks[i2 + numEdges]->addSuccessor(tracks[i]);
                        } else {
                            // already handled via case 1)
                        }
                    }

                }
            }
        }
    }
    // define start and end successors
    for (auto& item : stopTracks) {
        const int index = item.first->getNumericalID();
        // start
        item.second.first->addSuccessor(tracks[index]);
        item.second.first->addSuccessor(tracks[index + numEdges]);
        // end
        tracks[index]->addSuccessor(item.second.second);
        tracks[index + numEdges]->addSuccessor(item.second.second);
    }
    // DEBUG
    /*
    for (Track* t : tracks) {
        std::cout << "track " << t->getID() << " e=" << t->edge->getID() << " i=" << t->getNumericalID() << " succ:\n";
        for (Track* s : t->getSuccessors(SVC_IGNORING)) {
            std::cout << "   succ=" << s->getID() << "\n";
        }
    }
    */

    SUMOAbstractRouter<Track, NBVehicle>* router;
    router = new DijkstraRouter<Track, NBVehicle, SUMOAbstractRouter<Track, NBVehicle> >(
        tracks, true, &NBRailwayTopologyAnalyzer::getTravelTimeStatic, nullptr, true);

    int added = 0;
    int numDisconnected = 0;
    std::set<NBEdge*> addBidiStops;
    std::set<NBEdge*> addBidiEdges;
    std::set<std::pair<NBPTStop*, NBPTStop*> > visited;
    for (NBPTLine* line : nb.getPTLineCont().getLines()) {
        std::vector<NBPTStop*> stops = line->getStops();
        if (stops.size() < 2) {
            continue;
        }
        for (auto it = stops.begin(); it + 1 != stops.end(); ++it) {
            std::pair<NBPTStop*, NBPTStop*> trip(*it, *(it + 1));
            if (visited.count(trip) != 0) {
                continue;
            } else {
                visited.insert(trip);
            }
            NBEdge* fromEdge = ec.getByID((*it)->getEdgeId());
            NBEdge* toEdge = ec.getByID((*(it + 1))->getEdgeId());
            if (fromEdge == nullptr || toEdge == nullptr) {
                continue;
            }
            if (stopTracks.count(fromEdge) == 0
                    || stopTracks.count(toEdge) == 0) {
                continue;
            }
            NBVehicle veh(line->getRef(), (SUMOVehicleClass)(fromEdge->getPermissions() & SVC_RAIL_CLASSES));
            std::vector<const Track*> route;
            router->compute(stopTracks[fromEdge].first, stopTracks[toEdge].second, &veh, 0, route);
            //if (fromEdge->getID() == "356053025#1" && toEdge->getID() == "23256161") {
            //    std::cout << "DEBUG: route=" << toString(route) << "\n";
            //}
            if (route.size() > 0) {
                assert(route.size() > 2);
                for (int i = 1; i < (int)route.size() - 1; ++i) {
                    if (route[i]->getNumericalID() >= numEdges) {
                        NBEdge* edge = route[i]->edge;
                        if (addBidiEdges.count(edge) == 0) {
                            if (!edge->isBidiRail(true)) {
                                bool isStop = i == 1 || i == (int)route.size() - 2;
                                if (edge->getLaneSpreadFunction() == LANESPREAD_CENTER) {
                                    addBidiEdges.insert(edge);
                                    if (isStop) {
                                        addBidiStops.insert(edge);
                                    }
                                } else {
                                    if (isStop) {
                                        WRITE_WARNING("Stop on edge " + fromEdge->getID() + " can only be reached in reverse but edge has the wrong spreadType");
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                WRITE_WARNING("No connection found between stops on edge '" + fromEdge->getID() + "' and edge '" + toEdge->getID() + "'");
                numDisconnected++;
            }
        }
    }
    for (NBEdge* edge : addBidiEdges) {
        if (!edge->isBidiRail()) {
            NBEdge* e2 = addBidiEdge(nb, edge);
            //std::cout << " add bidiEdge for stop at edge " << edge->getID() << "\n";
            if (e2 != nullptr) {
                added++;
                added += extendBidiEdges(nb, edge->getToNode(), edge);
                added += extendBidiEdges(nb, edge->getFromNode(), e2);
            }
        }
    }

    if (addBidiEdges.size() > 0 || numDisconnected > 0) {
        WRITE_MESSAGE("Added " + toString(addBidiStops.size()) + " bidi-edges for public transport stops and a total of "
                      + toString(added) + " bidi-edges to ensure connectivity of stops ("
                      + toString(numDisconnected) + " stops remain disconnected)");
    }

    // clean up
    for (Track* t : tracks) {
        delete t;
    }
    delete router;
}


void
NBRailwayTopologyAnalyzer::addBidiEdgesForStraightConnectivity(NBNetBuilder& nb) {
    int added = 0;
    std::set<NBNode*> brokenNodes = getBrokenRailNodes(nb);
    for (const auto& e : nb.getEdgeCont()) {
        if (!isRailway(e.second->getPermissions())) {
            continue;
        }
        NBNode* const from = e.second->getFromNode();
        NBNode* const to = e.second->getToNode();
        if (brokenNodes.count(from) == 0 && brokenNodes.count(to) == 0) {
            continue;
        }
        EdgeVector inRailFrom, outRailFrom, inRailTo, outRailTo;
        getRailEdges(from, inRailFrom, outRailFrom);
        getRailEdges(to, inRailTo, outRailTo);
        bool haveReverse = false;
        for (const NBEdge* cand : outRailTo) {
            if (cand->getToNode() == from) {
                haveReverse = true;
                break;
            }
        }
        if (haveReverse) {
            continue;
        }
        bool haveStraightFrom = false;
        for (const NBEdge* fromStraightCand : outRailFrom) {
            if (fromStraightCand != e.second && isStraight(from, fromStraightCand, e.second)) {
                haveStraightFrom = true;
                break;
            }
        }
        if (!haveStraightFrom) {
            continue;
        }
        bool haveStraightTo = false;
        for (const NBEdge* toStraightCand : inRailTo) {
            if (toStraightCand != e.second && isStraight(to, toStraightCand, e.second)) {
                NBEdge* e2 = addBidiEdge(nb, e.second);
                //std::cout << " add bidiEdge for straight connectivity at edge " << e.second->getID() << " fromBroken=" << brokenNodes.count(from) << " toBroken=" << brokenNodes.count(to) << "\n";
                if (e2 != nullptr) {
                    added++;
                    added += extendBidiEdges(nb, to, e.second);
                    added += extendBidiEdges(nb, from, e2);
                }
                break;
            }
        }
    }
    if (added > 0) {
        WRITE_MESSAGE("Added " + toString(added) + " bidi-edges to ensure connectivity of straight tracks.");
    }
}


void
NBRailwayTopologyAnalyzer::updateTurns(NBEdge* edge) {
    NBTurningDirectionsComputer::computeTurnDirectionsForNode(edge->getFromNode(), false);
    NBTurningDirectionsComputer::computeTurnDirectionsForNode(edge->getToNode(), false);
}


double
NBRailwayTopologyAnalyzer::getTravelTimeStatic(const Track* const track, const NBVehicle* const veh, double time) {
    return NBEdge::getTravelTimeStatic(track->edge, veh, time);
}

/****************************************************************************/

