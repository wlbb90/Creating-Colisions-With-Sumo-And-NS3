/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NBNodeCont.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Yun-Pang Floetteroed
/// @author  Walter Bamberger
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Sascha Krieg
/// @date    Tue, 20 Nov 2001
/// @version $Id$
///
// Container for nodes during the netbuilding process
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <map>
#include <algorithm>
#include <cmath>
#include <utils/options/OptionsCont.h>
#include <utils/geom/Boundary.h>
#include <utils/geom/GeomHelper.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StdDefs.h>
#include <utils/common/ToString.h>
#include <utils/common/TplConvert.h>
#include <utils/common/IDSupplier.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include "NBHelpers.h"
#include "NBAlgorithms.h"
#include "NBDistrict.h"
#include "NBEdgeCont.h"
#include "NBTrafficLightLogicCont.h"
#include "NBOwnTLDef.h"
#include "NBNodeCont.h"
#include "NBPTStopCont.h"
#include "NBPTLineCont.h"
#include "NBParking.h"

//#define DEBUG_JOINJUNCTIONS
#define DEBUGNODEID "1311774270"
//#define DEBUGNODEID "5548037023"
#define DEBUGCOND(obj) ((obj != 0 && (obj)->getID() == DEBUGNODEID))

// ===========================================================================
// method definitions
// ===========================================================================
NBNodeCont::NBNodeCont()
    : myInternalID(1) {
}


NBNodeCont::~NBNodeCont() {
    clear();
}


// ----------- Insertion/removal/retrieval of nodes
bool
NBNodeCont::insert(const std::string& id, const Position& position,
                   NBDistrict* district) {
    NodeCont::iterator i = myNodes.find(id);
    if (i != myNodes.end()) {
        return false;
    }
    NBNode* node = new NBNode(id, position, district);
    myNodes[id] = node;
    const float pos[2] = {(float)position.x(), (float)position.y()};
    myRTree.Insert(pos, pos, node);
    return true;
}


bool
NBNodeCont::insert(NBNode* node) {
    std::string id = node->getID();
    NodeCont::iterator i = myNodes.find(id);
    if (i != myNodes.end()) {
        return false;
    }
    myNodes[id] = node;
    const float pos[2] = {(float)node->getPosition().x(), (float)node->getPosition().y()};
    myRTree.Insert(pos, pos, node);
    return true;
}


NBNode*
NBNodeCont::retrieve(const std::string& id) const {
    NodeCont::const_iterator i = myNodes.find(id);
    if (i == myNodes.end()) {
        return 0;
    }
    return (*i).second;
}


NBNode*
NBNodeCont::retrieve(const Position& position, const double offset) const {
    const double extOffset = offset + POSITION_EPS;
    const float cmin[2] = {(float)(position.x() - extOffset), (float)(position.y() - extOffset)};
    const float cmax[2] = {(float)(position.x() + extOffset), (float)(position.y() + extOffset)};
    std::set<std::string> into;
    Named::StoringVisitor sv(into);
    myRTree.Search(cmin, cmax, sv);
    for (std::set<std::string>::const_iterator i = into.begin(); i != into.end(); i++) {
        NBNode* const node = myNodes.find(*i)->second;
        if (fabs(node->getPosition().x() - position.x()) <= offset
                &&
                fabs(node->getPosition().y() - position.y()) <= offset) {
            return node;
        }
    }
    return 0;
}


bool
NBNodeCont::erase(NBNode* node) {
    if (extract(node)) {
        delete node;
        return true;
    } else {
        return false;
    }
}


bool
NBNodeCont::extract(NBNode* node, bool remember) {
    NodeCont::iterator i = myNodes.find(node->getID());
    if (i == myNodes.end()) {
        return false;
    }
    myNodes.erase(i);
    const float pos[2] = {(float)node->getPosition().x(), (float)node->getPosition().y()};
    myRTree.Remove(pos, pos, node);
    node->removeTrafficLights();
    if (remember) {
        myExtractedNodes.insert(node);
    }
    return true;
}


// ----------- Adapting the input
void
NBNodeCont::removeSelfLoops(NBDistrictCont& dc, NBEdgeCont& ec, NBTrafficLightLogicCont& tc) {
    int no = 0;
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        no += (*i).second->removeSelfLoops(dc, ec, tc);
    }
    if (no != 0) {
        WRITE_WARNING(toString(no) + " self-looping edge(s) removed.");
    }
}


void
NBNodeCont::joinSimilarEdges(NBDistrictCont& dc, NBEdgeCont& ec, NBTrafficLightLogicCont& tlc) {
    // magic values
    const double distanceThreshold = 7.; // don't merge edges further apart
    const double lengthThreshold = 0.10; // don't merge edges with higher relative length-difference

    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        // count the edges to other nodes outgoing from the current node
        std::map<NBNode*, EdgeVector> connectionCount;
        const EdgeVector& outgoing = (*i).second->getOutgoingEdges();
        for (EdgeVector::const_iterator j = outgoing.begin(); j != outgoing.end(); j++) {
            connectionCount[(*j)->getToNode()].push_back(*j);
        }
        // check whether more than a single edge connect another node and join them
        std::map<NBNode*, EdgeVector>::iterator k;
        for (k = connectionCount.begin(); k != connectionCount.end(); k++) {
            // possibly we do not have anything to join...
            if ((*k).second.size() < 2) {
                continue;
            }
            // for the edges that seem to be a single street,
            //  check whether the geometry is similar
            const EdgeVector& ev = (*k).second;
            const NBEdge* const first = ev.front();
            EdgeVector::const_iterator jci; // join candidate iterator
            for (jci = ev.begin() + 1; jci != ev.end(); ++jci) {
                const double relativeLengthDifference = fabs(first->getLoadedLength() - (*jci)->getLoadedLength()) / first->getLoadedLength();
                if ((!first->isNearEnough2BeJoined2(*jci, distanceThreshold)) ||
                        (relativeLengthDifference > lengthThreshold) ||
                        (fabs(first->getSpeed() - (*jci)->getSpeed()) >= 0.01) || // output accuracy
                        (first->getPermissions() != (*jci)->getPermissions())
                   ) {
                    break;
                }
            }
            // @bug If there are 3 edges of which 2 can be joined, no joining will
            //   take place with the current implementation
            if (jci == ev.end()) {
                ec.joinSameNodeConnectingEdges(dc, tlc, ev);
            }
        }
    }
}


void
NBNodeCont::removeIsolatedRoads(NBDistrictCont& dc, NBEdgeCont& ec) {
    // Warn of isolated edges, i.e. a single edge with no connection to another edge
    const std::vector<std::string>& edgeNames = ec.getAllNames();
    for (std::vector<std::string>::const_iterator it = edgeNames.begin(); it != edgeNames.end(); ++it) {
        // Test whether this node starts at a dead end, i.e. it has only one adjacent node
        // to which an edge exists and from which an edge may come.
        NBEdge* e = ec.retrieve(*it);
        if (e == 0) {
            continue;
        }
        NBNode* from = e->getFromNode();
        const EdgeVector& outgoingEdges = from->getOutgoingEdges();
        if (outgoingEdges.size() != 1) {
            // At this node, several edges or no edge start; so, this node is no dead end.
            continue;
        }
        const EdgeVector& incomingEdges = from->getIncomingEdges();
        if (incomingEdges.size() > 1) {
            // At this node, several edges end; so, this node is no dead end.
            continue;
        } else if (incomingEdges.size() == 1) {
            NBNode* fromNodeOfIncomingEdge = incomingEdges[0]->getFromNode();
            NBNode* toNodeOfOutgoingEdge = outgoingEdges[0]->getToNode();
            if (fromNodeOfIncomingEdge != toNodeOfOutgoingEdge) {
                // At this node, an edge ends which is not the inverse direction of
                // the starting node.
                continue;
            }
        }
        // Now we know that the edge e starts a dead end.
        // Next we test if the dead end is isolated, i.e. does not lead to a junction
        bool hasJunction = false;
        EdgeVector road;
        NBEdge* eOld = 0;
        NBNode* to;
        NodeSet adjacentNodes;
        do {
            road.push_back(e);
            eOld = e;
            from = e->getFromNode();
            to = e->getToNode();
            const EdgeVector& outgoingEdgesOfToNode = to->getOutgoingEdges();
            const EdgeVector& incomingEdgesOfToNode = to->getIncomingEdges();
            adjacentNodes.clear();
            for (EdgeVector::const_iterator itOfOutgoings = outgoingEdgesOfToNode.begin(); itOfOutgoings != outgoingEdgesOfToNode.end(); ++itOfOutgoings) {
                if ((*itOfOutgoings)->getToNode() != from        // The back path
                        && (*itOfOutgoings)->getToNode() != to   // A loop / dummy edge
                   ) {
                    e = *itOfOutgoings; // Probably the next edge
                }
                adjacentNodes.insert((*itOfOutgoings)->getToNode());
            }
            for (EdgeVector::const_iterator itOfIncomings = incomingEdgesOfToNode.begin(); itOfIncomings != incomingEdgesOfToNode.end(); ++itOfIncomings) {
                adjacentNodes.insert((*itOfIncomings)->getFromNode());
            }
            adjacentNodes.erase(to);  // Omit loops
            if (adjacentNodes.size() > 2) {
                hasJunction = true;
            }
        } while (!hasJunction && eOld != e);
        if (!hasJunction) {
            std::string warningString = "Removed a road without junctions: ";
            for (EdgeVector::iterator roadIt = road.begin(); roadIt != road.end(); ++roadIt) {
                if (roadIt == road.begin()) {
                    warningString += (*roadIt)->getID();
                } else {
                    warningString += ", " + (*roadIt)->getID();
                }

                NBNode* fromNode = (*roadIt)->getFromNode();
                NBNode* toNode = (*roadIt)->getToNode();
                ec.erase(dc, *roadIt);
                if (fromNode->getIncomingEdges().size() == 0 && fromNode->getOutgoingEdges().size() == 0) {
                    // Node is empty; can be removed
                    erase(fromNode);
                }
                if (toNode->getIncomingEdges().size() == 0 && toNode->getOutgoingEdges().size() == 0) {
                    // Node is empty; can be removed
                    erase(toNode);
                }
            }
            WRITE_WARNING(warningString);
        }
    }
}


void
NBNodeCont::removeComponents(NBDistrictCont& dc, NBEdgeCont& ec, const int numKeep) {
    std::vector<std::set<NBEdge*> > components;
    // need to use ids here to have the same ordering on all platforms
    std::set<std::string> edgesLeft;
    for (std::map<std::string, NBEdge*>::const_iterator edgeIt = ec.begin(); edgeIt != ec.end(); ++edgeIt) {
        edgesLeft.insert(edgeIt->first);
    }
    EdgeVector queue;
    std::set<NBEdge*> toRemove;
    while (!edgesLeft.empty()) {
        queue.push_back(ec.getByID(*edgesLeft.begin()));
        std::set<NBEdge*> component;
        while (!queue.empty()) {
            NBEdge* const e = queue.back();
            queue.pop_back();
            component.insert(e);
            std::vector<EdgeVector> edgeLists;
            edgeLists.push_back(e->getFromNode()->getOutgoingEdges());
            edgeLists.push_back(e->getFromNode()->getIncomingEdges());
            edgeLists.push_back(e->getToNode()->getOutgoingEdges());
            edgeLists.push_back(e->getToNode()->getIncomingEdges());
            for (std::vector<EdgeVector>::const_iterator listIt = edgeLists.begin(); listIt != edgeLists.end(); ++listIt) {
                for (EdgeVector::const_iterator edgeIt = listIt->begin(); edgeIt != listIt->end(); ++edgeIt) {
                    std::set<std::string>::iterator leftIt = edgesLeft.find((*edgeIt)->getID());
                    if (leftIt != edgesLeft.end()) {
                        queue.push_back(*edgeIt);
                        edgesLeft.erase(leftIt);
                    }
                }
            }
        }
        std::vector<std::set<NBEdge*> >::iterator cIt;
        for (cIt = components.begin(); cIt != components.end(); ++cIt) {
            if (cIt->size() < component.size()) {
                break;
            }
        }
        components.insert(cIt, component);
        if ((int)components.size() > numKeep) {
            toRemove.insert(components.back().begin(), components.back().end());
            components.pop_back();
        }
    }
    for (std::set<NBEdge*>::iterator edgeIt = toRemove.begin(); edgeIt != toRemove.end(); ++edgeIt) {
        NBNode* const fromNode = (*edgeIt)->getFromNode();
        NBNode* const toNode = (*edgeIt)->getToNode();
        ec.erase(dc, *edgeIt);
        if (fromNode->getIncomingEdges().size() == 0 && fromNode->getOutgoingEdges().size() == 0) {
            erase(fromNode);
        }
        if (toNode->getIncomingEdges().size() == 0 && toNode->getOutgoingEdges().size() == 0) {
            erase(toNode);
        }
    }
}


int
NBNodeCont::removeUnwishedNodes(NBDistrictCont& dc, NBEdgeCont& ec,
                                NBTrafficLightLogicCont& tlc, NBPTStopCont& sc, NBPTLineCont& lc,
                                NBParkingCont& pc,
                                bool removeGeometryNodes) {
    // load edges that shall not be modified
    std::set<std::string> edges2keep;
    if (removeGeometryNodes) {
        const OptionsCont& oc = OptionsCont::getOptions();
        if (oc.isSet("geometry.remove.keep-edges.input-file")) {
            NBHelpers::loadEdgesFromFile(oc.getString("geometry.remove.keep-edges.input-file"), edges2keep);
        }
        if (oc.isSet("geometry.remove.keep-edges.explicit")) {
            const std::vector<std::string> edges = oc.getStringVector("geometry.remove.keep-edges.explicit");
            edges2keep.insert(edges.begin(), edges.end());
        }
        sc.addEdges2Keep(oc, edges2keep);
        lc.addEdges2Keep(oc, edges2keep);
        pc.addEdges2Keep(oc, edges2keep);
    }
    int no = 0;
    std::vector<NBNode*> toRemove;
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        NBNode* current = (*i).second;
        bool remove = false;
        std::vector<std::pair<NBEdge*, NBEdge*> > toJoin;
        // check for completely empty nodes
        if (current->getOutgoingEdges().size() == 0 && current->getIncomingEdges().size() == 0) {
            // remove if empty
            remove = true;
        }
        // check for nodes which are only geometry nodes
        if (removeGeometryNodes && mySplit.count(current) == 0) {
            if ((current->getOutgoingEdges().size() == 1 && current->getIncomingEdges().size() == 1)
                    ||
                    (current->getOutgoingEdges().size() == 2 && current->getIncomingEdges().size() == 2)) {
                // ok, one in, one out or two in, two out
                //  -> ask the node whether to join
                remove = current->checkIsRemovable();
                // check whether any of the edges must be kept
                for (EdgeVector::const_iterator it_edge = current->getEdges().begin(); it_edge != current->getEdges().end(); ++it_edge) {
                    if (edges2keep.find((*it_edge)->getID()) != edges2keep.end()) {
                        remove = false;
                        break;
                    }
                }
                if (remove) {
                    toJoin = current->getEdgesToJoin();
                }
            }
        }
        // remove the node and join the geometries when wished
        if (!remove) {
            continue;
        }
        for (std::vector<std::pair<NBEdge*, NBEdge*> >::iterator j = toJoin.begin(); j != toJoin.end(); j++) {
            NBEdge* begin = (*j).first;
            NBEdge* continuation = (*j).second;
            begin->append(continuation);
            continuation->getToNode()->replaceIncoming(continuation, begin, 0);
            tlc.replaceRemoved(continuation, -1, begin, -1);
            ec.extract(dc, continuation, true);
        }
        toRemove.push_back(current);
        no++;
    }
    // erase all
    for (std::vector<NBNode*>::iterator j = toRemove.begin(); j != toRemove.end(); ++j) {
        extract(*j, true);
    }
    return no;
}


void
NBNodeCont::avoidOverlap() {
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        (*i).second->avoidOverlap();
    }
}

// ----------- (Helper) methods for joining nodes
void
NBNodeCont::generateNodeClusters(double maxDist, NodeClusters& into) const {
    std::set<NBNode*> visited;
    for (NodeCont::const_iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        std::vector<NodeAndDist> toProc;
        if (visited.find((*i).second) != visited.end()) {
            continue;
        }
        toProc.push_back(std::make_pair((*i).second, 0));
        NodeSet c;
        while (!toProc.empty()) {
            NodeAndDist nodeAndDist = toProc.back();
            NBNode* n = nodeAndDist.first;
            double dist = nodeAndDist.second;
            toProc.pop_back();
            if (visited.find(n) != visited.end()) {
                continue;
            }
            visited.insert(n);
            bool pureRail = true;
            bool railAndPeds = true;
            for (NBEdge* e : n->getEdges()) {
                if ((e->getPermissions() & ~(SVC_RAIL_CLASSES | SVC_PEDESTRIAN)) != 0) {
                    railAndPeds = false;
                    pureRail = false;
                    break;
                }
                if ((e->getPermissions() & ~(SVC_RAIL_CLASSES)) != 0) {
                    pureRail = false;
                }
            }
            if (pureRail) {
                // do not join pure rail nodes
                continue;
            }
            c.insert(n);
#ifdef DEBUG_JOINJUNCTIONS
            if (DEBUGCOND(n)) {
                std::cout << "generateNodeClusters: consider n=" << n->getID() << " edges=" << toString(n->getEdges(), ' ') << " with cluster " << joinNamedToString(c, ' ') << " pureRail=" << pureRail << "\n";
            }
#endif
            for (NBEdge* e : n->getEdges()) {
                NBNode* s = n->hasIncoming(e) ? e->getFromNode() : e->getToNode();
                if (railAndPeds && n->getType() != NODETYPE_RAIL_CROSSING) {
                    bool railAndPeds2 = true;
                    for (NBEdge* e : n->getEdges()) {
                        if ((e->getPermissions() & ~(SVC_RAIL_CLASSES | SVC_PEDESTRIAN)) != 0) {
                            railAndPeds2 = false;
                            break;
                        }
                    }
                    if (railAndPeds2 && s->getType() != NODETYPE_RAIL_CROSSING) {
                        // do not join rail/ped nodes unless at a rail crossing
                        // (neither nodes nor the traffic lights)
                        continue;
                    }
                }

                const double length = e->getLoadedLength();
#ifdef DEBUG_JOINJUNCTIONS
                if (DEBUGCOND(n)) {
                    std::cout << "generateNodeClusters edge=" << e->getID() << " length=" << length << "\n";
                }
#endif
                const bool bothCrossing = n->getType() == NODETYPE_RAIL_CROSSING && s->getType() == NODETYPE_RAIL_CROSSING;
                const bool joinPedCrossings = bothCrossing && e->getPermissions() == SVC_PEDESTRIAN;
                if ( // never join pedestrian stuff (unless at a rail crossing
                    !joinPedCrossings && (
                        e->getPermissions() == SVC_PEDESTRIAN
                        // only join edges for regular passenger traffic or edges that are extremely short
                        || (length > 3 * POSITION_EPS
                            && (e->getPermissions() & (SVC_PASSENGER | SVC_TRAM)) == 0
                            && n->getPosition().distanceTo2D(s->getPosition()) > SUMO_const_laneWidth))) {
                    continue;
                }
                // never join rail_crossings with other node types unless the crossing is only for tram
                if ((n->getType() == NODETYPE_RAIL_CROSSING && s->getType() != NODETYPE_RAIL_CROSSING)
                        || (n->getType() != NODETYPE_RAIL_CROSSING && s->getType() == NODETYPE_RAIL_CROSSING)) {
                    const SVCPermissions railNoTram = (SVC_RAIL_CLASSES & ~SVC_TRAM);
                    bool foundRail = false;
                    NBNode* crossingNode = n->getType() == NODETYPE_RAIL_CROSSING ? n : s;
                    for (NBEdge* e2 : crossingNode->getIncomingEdges()) {
                        if ((e2->getPermissions() & railNoTram) != 0) {
                            foundRail = true;
                            break;
                        }
                    }
                    if (foundRail) {
                        continue;
                    }
                }
                // never join rail_crossings via a rail edge
                if (bothCrossing && (e->getPermissions() & ~SVC_RAIL_CLASSES) == 0) {
                    continue;
                }
                if (visited.find(s) != visited.end()) {
                    continue;
                }
                if (length + dist < maxDist) {
                    if (s->geometryLike()) {
                        toProc.push_back(std::make_pair(s, dist + length));
                    } else {
                        toProc.push_back(std::make_pair(s, 0));
                    }
                }
            }
        }
        if (c.size() < 2) {
            continue;
        }
#ifdef DEBUG_JOINJUNCTIONS
        std::cout << " DEBUG: consider cluster " << joinNamedToString(c, ' ') << "\n";
#endif
        into.push_back(c);
    }
}


void
NBNodeCont::addJoinExclusion(const std::vector<std::string>& ids, bool check) {
    for (std::vector<std::string>::const_iterator it = ids.begin(); it != ids.end(); it++) {
        // error handling has to take place here since joinExclusions could be
        // loaded from multiple files / command line
        if (myJoined.count(*it) > 0) {
            WRITE_WARNING("Ignoring join exclusion for junction '" + *it +  "' since it already occurred in a list of nodes to be joined");
        } else if (check && retrieve(*it) == 0) {
            WRITE_WARNING("Ignoring join exclusion for unknown junction '" + *it + "'");
        } else {
            myJoinExclusions.insert(*it);
        }
    }
}


void
NBNodeCont::addCluster2Join(std::set<std::string> cluster) {
    // error handling has to take place here since joins could be loaded from multiple files
    for (std::set<std::string>::const_iterator it = cluster.begin(); it != cluster.end(); it++) {
        if (myJoinExclusions.count(*it) > 0) {
            WRITE_WARNING("Ignoring join-cluster because junction '" + *it + "' was already excluded from joining");
            return;
        } else if (myJoined.count(*it) > 0) {
            WRITE_WARNING("Ignoring join-cluster because junction '" + *it + "' already occurred in another join-cluster");
            return;
        } else {
            myJoined.insert(*it);
        }
    }
    myClusters2Join.push_back(cluster);
}


int
NBNodeCont::joinLoadedClusters(NBDistrictCont& dc, NBEdgeCont& ec, NBTrafficLightLogicCont& tlc) {
    NodeClusters clusters;
    for (std::vector<std::set<std::string> >::iterator it = myClusters2Join.begin(); it != myClusters2Join.end(); it++) {
        // verify loaded cluster
        NodeSet cluster;
        for (std::set<std::string>::iterator it_id = it->begin(); it_id != it->end(); it_id++) {
            NBNode* node = retrieve(*it_id);
            if (node == 0) {
                WRITE_WARNING("Ignoring unknown junction '" + *it_id + "' while joining");
            } else {
                cluster.insert(node);
            }
        }
        if (cluster.size() > 1) {
            clusters.push_back(cluster);
        }
    }
    joinNodeClusters(clusters, dc, ec, tlc);
    myClusters2Join.clear(); // make save for recompute
    return (int)clusters.size();
}


int
NBNodeCont::joinJunctions(double maxDist, NBDistrictCont& dc, NBEdgeCont& ec, NBTrafficLightLogicCont& tlc, NBPTStopCont& sc) {
#ifdef DEBUG_JOINJUNCTIONS
    std::cout << "joinJunctions...\n";
#endif
    NodeClusters cands;
    NodeClusters clusters;
    generateNodeClusters(maxDist, cands);
    for (NodeClusters::iterator i = cands.begin(); i != cands.end(); ++i) {
        NodeSet cluster = (*i);
        // remove join exclusions
        for (NodeSet::iterator j = cluster.begin(); j != cluster.end();) {
            NodeSet::iterator check = j;
            ++j;
            if (myJoinExclusions.count((*check)->getID()) > 0) {
                cluster.erase(check);
            }
        }
        // remove nodes that can be eliminated by geometry.remove
        pruneClusterFringe(cluster);
        // avoid removal of long edges (must have been added via an alternative path).
        std::set<NBNode*> toRemove;
        for (std::set<NBNode*>::iterator j = cluster.begin(); j != cluster.end(); ++j) {
            NBNode* n = *j;
            for (NBEdge* edge : n->getOutgoingEdges()) {
                if (cluster.count(edge->getToNode()) != 0 && edge->getLoadedLength() > maxDist /*&& (edge->getPermissions() & SVC_PASSENGER) != 0*/) {
#ifdef DEBUG_JOINJUNCTIONS
                    if (DEBUGCOND(n) || DEBUGCOND(edge->getToNode())) {
                        std::cout << "long edge " << edge->getID() << " (" << edge->getLoadedLength() << ", max=" << maxDist << ")\n";
                    }
#endif
                    toRemove.insert(n);
                    toRemove.insert(edge->getToNode());
                }
            }
        }
        for (std::set<NBNode*>::iterator j = toRemove.begin(); j != toRemove.end(); ++j) {
            cluster.erase(*j);
        }
        if (cluster.size() < 2) {
            continue;
        }
        std::string reason;
        bool feasible = feasibleCluster(cluster, ec, sc, reason);
        //if (!feasible) std::cout << "\ntry to reduce cluster " << joinNamedToString(cluster, ',') << "\n";
        if (!feasible) {
            std::string origCluster = joinNamedToString(cluster, ',');
            if (reduceToCircle(cluster, 4, cluster)) {
                pruneClusterFringe(cluster);
                feasible = feasibleCluster(cluster, ec, sc, reason);
                if (feasible) {
                    WRITE_WARNING("Reducing junction cluster " + origCluster + " (" + reason + ")");
                }
            }
        }
        if (!feasible) {
            std::string origCluster = joinNamedToString(cluster, ',');
            if (reduceToCircle(cluster, 2, cluster)) {
                pruneClusterFringe(cluster);
                feasible = feasibleCluster(cluster, ec, sc, reason);
                if (feasible) {
                    WRITE_WARNING("Reducing junction cluster " + origCluster + " (" + reason + ")");
                }
            }
        }
        if (!feasible) {
            WRITE_WARNING("Not joining junctions " + joinNamedToString(cluster, ',') + " (" + reason + ")");
            continue;
        }
        // compute all connected components of this cluster
        // (may be more than 1 if intermediate nodes were removed)
        NodeClusters components;
        for (std::set<NBNode*>::iterator j = cluster.begin(); j != cluster.end(); ++j) {
            // merge all connected components into newComp
            NodeSet newComp;
            NBNode* current = *j;
            //std::cout << "checking connectivity for " << current->getID() << "\n";
            newComp.insert(current);
            for (NodeClusters::iterator it_comp = components.begin(); it_comp != components.end();) {
                NodeClusters::iterator check = it_comp;
                //std::cout << "   connected with " << toString(*check) << "?\n";
                bool connected = false;
                for (std::set<NBNode*>::iterator k = (*check).begin(); k != (*check).end(); ++k) {
                    if (current->getConnectionTo(*k) != 0 || (*k)->getConnectionTo(current) != 0) {
                        //std::cout << "joining with connected component " << toString(*check) << "\n";
                        newComp.insert((*check).begin(), (*check).end());
                        it_comp = components.erase(check);
                        connected = true;
                        break;
                    }
                }
                if (!connected) {
                    it_comp++;
                }
            }
            //std::cout << "adding new component " << toString(newComp) << "\n";
            components.push_back(newComp);
        }
        for (NodeClusters::iterator it_comp = components.begin(); it_comp != components.end(); ++it_comp) {
            if ((*it_comp).size() > 1) {
                //std::cout << "adding cluster " << toString(*it_comp) << "\n";
                clusters.push_back(*it_comp);
            }
        }
    }
    joinNodeClusters(clusters, dc, ec, tlc);
    return (int)clusters.size();
}


void
NBNodeCont::pruneClusterFringe(NodeSet& cluster) const {
#ifdef DEBUG_JOINJUNCTIONS
    if (true) {
        std::cout << "pruning cluster=" << joinNamedToString(cluster, ' ') << "\n";
    }
#endif
    // iteratively remove the fringe
    bool pruneFringe = true;
    // collect nodes that shall be joined due to distance but are not connected
    // to the cluster for passenger traffic
    while (pruneFringe) {
        pruneFringe = false;
        for (std::set<NBNode*>::iterator j = cluster.begin(); j != cluster.end();) {
            std::set<NBNode*>::iterator check = j;
            NBNode* n = *check;
            ++j;

            // compute clusterDist for node (length of shortest edge which connects this node to the cluster)
            double clusterDist = std::numeric_limits<double>::max();
            bool touchingCluster = false;
            for (EdgeVector::const_iterator it_edge = n->getOutgoingEdges().begin(); it_edge != n->getOutgoingEdges().end(); ++it_edge) {
                NBNode* neighbor = (*it_edge)->getToNode();
                if (cluster.count(neighbor) != 0) {
                    clusterDist = MIN2(clusterDist, (*it_edge)->getLoadedLength());
                    touchingCluster |= n->getPosition().distanceTo2D(neighbor->getPosition()) <= SUMO_const_laneWidth;
                }
            }
            for (EdgeVector::const_iterator it_edge = n->getIncomingEdges().begin(); it_edge != n->getIncomingEdges().end(); ++it_edge) {
                NBNode* neighbor = (*it_edge)->getFromNode();
                if (cluster.count(neighbor) != 0) {
                    clusterDist = MIN2(clusterDist, (*it_edge)->getLoadedLength());
                    touchingCluster |= n->getPosition().distanceTo2D(neighbor->getPosition()) <= SUMO_const_laneWidth;
                }
            }
            // remove geometry-like nodes at fringe of the cluster
            // (they have 1 neighbor in the cluster and at most 1 neighbor outside the cluster)
            std::set<NBNode*> outsideNeighbors;
            std::set<NBNode*> clusterNeighbors;
            const double pedestrianFringeThreshold = 0.3;
            for (NBEdge* e : n->getEdges()) {
                NBNode* neighbor = e->getFromNode() == n ? e->getToNode() : e->getFromNode();
                if (cluster.count(neighbor) == 0) {
                    if ((e->getPermissions() & SVC_PASSENGER) != 0 || clusterDist <= pedestrianFringeThreshold || touchingCluster) {
                        outsideNeighbors.insert(neighbor);
                    }
                } else {
                    clusterNeighbors.insert(neighbor);
                }
            }
#ifdef DEBUG_JOINJUNCTIONS
            if (DEBUGCOND(n)) std::cout << "  check n=" << n->getID()
                                            << " clusterDist=" << clusterDist
                                            << " cd<th=" << (clusterDist <= pedestrianFringeThreshold)
                                            << " touching=" << touchingCluster
                                            << " out=" << joinNamedToString(outsideNeighbors, ',')
                                            << " in=" << joinNamedToString(clusterNeighbors, ',')
                                            << "\n";
#endif
            if (outsideNeighbors.size() <= 1
                    && clusterNeighbors.size() == 1
                    && !n->isTLControlled()) {
                cluster.erase(check);
                pruneFringe = true; // other nodes could belong to the fringe now
#ifdef DEBUG_JOINJUNCTIONS
                if (DEBUGCOND(n)) {
                    std::cout << "  pruned n=" << n->getID() << "\n";
                }
#endif
            }
        }
    }
}


bool
NBNodeCont::feasibleCluster(const NodeSet& cluster, const NBEdgeCont& ec, const NBPTStopCont& sc, std::string& reason) const {
    // check for clusters which are to complex and probably won't work very well
    // we count the incoming edges of the final junction
    std::map<std::string, double> finalIncomingAngles;
    std::map<std::string, double> finalOutgoingAngles;
    for (std::set<NBNode*>::const_iterator j = cluster.begin(); j != cluster.end(); ++j) {
        for (EdgeVector::const_iterator it_edge = (*j)->getIncomingEdges().begin(); it_edge != (*j)->getIncomingEdges().end(); ++it_edge) {
            NBEdge* edge = *it_edge;
            if (cluster.count(edge->getFromNode()) == 0 && (edge->getPermissions() & SVC_PASSENGER) != 0) {
                // incoming edge, does not originate in the cluster
                finalIncomingAngles[edge->getID()] = edge->getAngleAtNode(edge->getToNode());
            }
        }
        for (EdgeVector::const_iterator it_edge = (*j)->getOutgoingEdges().begin(); it_edge != (*j)->getOutgoingEdges().end(); ++it_edge) {
            NBEdge* edge = *it_edge;
            if (cluster.count(edge->getToNode()) == 0 && (edge->getPermissions() & SVC_PASSENGER) != 0) {
                // outgoing edge, does not end in the cluster
                finalOutgoingAngles[edge->getID()] = edge->getAngleAtNode(edge->getFromNode());
            }
        }

    }
#ifdef DEBUG_JOINJUNCTIONS
    for (NBNode* n : cluster) {
        if (DEBUGCOND(n)) {
            std::cout << "feasibleCluster c=" << joinNamedToString(cluster, ',')
                      << "\n inAngles=" << joinToString(finalIncomingAngles, ' ', ':')
                      << "\n outAngles=" << joinToString(finalOutgoingAngles, ' ', ':')
                      << "\n";
        }
    }
#endif
    if (finalIncomingAngles.size() > 4) {
        reason = toString(finalIncomingAngles.size()) + " incoming edges";
        return false;
    }
    // check for incoming parallel edges
    const double PARALLEL_INCOMING_THRESHOLD = 10.0;
    bool foundParallel = false;
    for (std::map<std::string, double>::const_iterator j = finalIncomingAngles.begin(); j != finalIncomingAngles.end() && !foundParallel; ++j) {
        std::map<std::string, double>::const_iterator k = j;
        for (++k; k != finalIncomingAngles.end() && !foundParallel; ++k) {
            if (fabs(j->second - k->second) < PARALLEL_INCOMING_THRESHOLD) {
                reason = "parallel incoming " + j->first + "," + k->first;
                return false;
            }
        }
    }
    // check for outgoing parallel edges
    for (std::map<std::string, double>::const_iterator j = finalOutgoingAngles.begin(); j != finalOutgoingAngles.end() && !foundParallel; ++j) {
        std::map<std::string, double>::const_iterator k = j;
        for (++k; k != finalOutgoingAngles.end() && !foundParallel; ++k) {
            if (fabs(j->second - k->second) < PARALLEL_INCOMING_THRESHOLD) {
                reason = "parallel outgoing " + j->first + "," + k->first;
                return false;
            }
        }
    }
    // check for stop edges within the cluster
    if (OptionsCont::getOptions().isSet("ptstop-output")) {
        for (auto it = sc.begin(); it != sc.end(); it++) {
            NBEdge* edge = ec.retrieve(it->second->getEdgeId());
            if (edge != 0 && cluster.count(edge->getFromNode()) != 0 && cluster.count(edge->getToNode()) != 0) {
                reason = "it contains stop '" + it->first + "'";
                return false;
            }
        }
    }
    // prevent removal of long edges unless there is weak circle or a traffic light
    if (cluster.size() > 2) {
        // find the nodes with the biggests physical distance between them
        double maxDist = -1;
        bool hasTLS = false;
        NBEdge* maxEdge = 0;
        for (NBNode* n1 : cluster) {
            hasTLS |= n1->isTLControlled();
            for (NBNode* n2 : cluster) {
                NBEdge* e1 = n1->getConnectionTo(n2);
                NBEdge* e2 = n2->getConnectionTo(n1);
                if (e1 != 0 && e1->getLoadedLength() > maxDist) {
                    maxDist = e1->getLoadedLength();
                    maxEdge = e1;
                }
                if (e2 != 0 && e2->getLoadedLength() > maxDist) {
                    maxDist = e2->getLoadedLength();
                    maxEdge = e2;
                }
            }
        }
#ifdef DEBUG_JOINJUNCTIONS
        for (NBNode* n : cluster) {
            if (DEBUGCOND(n)) {
                std::cout << "feasible hasTLS=" << hasTLS << " maxDist=" << maxDist << " maxEdge=" << maxEdge->getID() << "\n";
            }
        }
#endif
        if (!hasTLS && maxDist > 5) {
            // find a weak circle within cluster that does not use maxEdge
            std::vector<NBNode*> toCheck;
            std::set<NBNode*> visited;
            toCheck.push_back(maxEdge->getToNode());
            bool foundCircle = false;
            while (!toCheck.empty()) {
                NBNode* n = toCheck.back();
                if (n == maxEdge->getFromNode()) {
                    foundCircle = true;
                    break;
                }
                toCheck.pop_back();
                visited.insert(n);
                for (NBEdge* e : n->getEdges()) {
                    if (e != maxEdge) {
                        NBNode* cand = e->getFromNode() == n ? e->getToNode() : e->getFromNode();
                        if (visited.count(cand) == 0 && cluster.count(cand) != 0) {
                            toCheck.push_back(cand);
                        }
                    }
                }
            }
            if (!foundCircle) {
                reason = "not compact (maxEdge=" + maxEdge->getID() + " length=" + toString(maxDist) + ")";
                return false;
            }
        }
    }
    return true;
}


bool
NBNodeCont::reduceToCircle(NodeSet& cluster, int circleSize, NodeSet startNodes, std::vector<NBNode*> cands) const {
    //std::cout << " cs=" << circleSize << " cands=" << toString(cands) << " startNodes=" << toString(startNodes) << "\n";
    assert(circleSize >= 2);
    if ((int)cands.size() == circleSize) {
        if (cands.back()->getConnectionTo(cands.front()) != 0) {
            // cluster found
            cluster.clear();
            cluster.insert(cands.begin(), cands.end());
            return true;
        } else {
            return false;
        }
    }
    if ((int)cluster.size() <= circleSize || startNodes.size() == 0) {
        // no reduction possible
        return false;
    }
    if (cands.size() == 0) {
        // try to find a circle starting from another start node
        NBEdge* e = shortestEdge(cluster, startNodes, cands);
        if (e != 0) {
            cands.push_back(e->getFromNode());
            startNodes.erase(e->getFromNode());
            if (reduceToCircle(cluster, circleSize, startNodes, cands)) {
                return true;
            } else {
                // try another start node
                return reduceToCircle(cluster, circleSize, startNodes);
            }
        }
    } else {
        NodeSet singleStart;
        singleStart.insert(cands.back());
        NBEdge* e = shortestEdge(cluster, singleStart, cands);
        if (e != 0) {
            std::vector<NBNode*> cands2(cands);
            cands2.push_back(e->getToNode());
            if (reduceToCircle(cluster, circleSize, startNodes, cands2)) {
                return true;
            }
        }
    }
    return false;
}


NBEdge*
NBNodeCont::shortestEdge(const NodeSet& cluster, const NodeSet& startNodes, const std::vector<NBNode*>& exclude) const {
    double minDist = std::numeric_limits<double>::max();
    NBEdge* result = 0;
    for (NBNode* n : startNodes) {
        for (NBEdge* e : n->getOutgoingEdges()) {
            NBNode* neigh = e->getToNode();
            if (cluster.count(neigh) != 0 && std::find(exclude.begin(), exclude.end(), neigh) == exclude.end()) {
                const double dist = n->getPosition().distanceTo2D(neigh->getPosition());
                //std::cout << "    e=" << e->getID() << " dist=" << dist << " minD=" << minDist << "\n";
                if (dist < minDist) {
                    minDist = dist;
                    result = e;
                }
            }
        }
    }
    //std::cout << "closestNeighbor startNodes=" << toString(startNodes) << " result=" << Named::getIDSecure(result) << "\n";
    return result;
}

void
NBNodeCont::joinNodeClusters(NodeClusters clusters,
                             NBDistrictCont& dc, NBEdgeCont& ec, NBTrafficLightLogicCont& tlc) {
    for (NodeClusters::iterator i = clusters.begin(); i != clusters.end(); ++i) {
        NodeSet cluster = *i;
        assert(cluster.size() > 1);
        Position pos;
        bool setTL;
        std::string id = "cluster";
        TrafficLightType type;
        SumoXMLNodeType nodeType = NODETYPE_UNKNOWN;
        analyzeCluster(cluster, id, pos, setTL, type, nodeType);
        if (!insert(id, pos)) {
            // should not fail
            WRITE_WARNING("Could not join junctions " + id);
            continue;
        }
        NBNode* newNode = retrieve(id);
        newNode->reinit(pos, nodeType);
        if (setTL) {
            NBTrafficLightDefinition* tlDef = new NBOwnTLDef(id, newNode, 0, type);
            if (!tlc.insert(tlDef)) {
                // actually, nothing should fail here
                delete tlDef;
                throw ProcessError("Could not allocate tls '" + id + "'.");
            }
        }
        // collect edges
        std::set<NBEdge*> allEdges;
        for (NBNode* n : cluster) {
            const EdgeVector& edges = n->getEdges();
            allEdges.insert(edges.begin(), edges.end());
        }

        // remap and remove edges which are completely within the new intersection
        for (std::set<NBEdge*>::iterator j = allEdges.begin(); j != allEdges.end();) {
            NBEdge* e = (*j);
            NBNode* from = e->getFromNode();
            NBNode* to = e->getToNode();
            if (cluster.count(from) > 0 && cluster.count(to) > 0) {
                for (std::set<NBEdge*>::iterator l = allEdges.begin(); l != allEdges.end(); ++l) {
                    if (e != *l) {
                        (*l)->replaceInConnections(e, e->getConnections());
                    }
                }
                ec.extract(dc, e, true);
                allEdges.erase(j++); // erase does not invalidate the other iterators
            } else {
                ++j;
            }
        }

        // remap edges which are incoming / outgoing
        for (std::set<NBEdge*>::iterator j = allEdges.begin(); j != allEdges.end(); ++j) {
            NBEdge* e = (*j);
            std::vector<NBEdge::Connection> conns = e->getConnections();
            const bool outgoing = cluster.count(e->getFromNode()) > 0;
            NBNode* from = outgoing ? newNode : e->getFromNode();
            NBNode* to   = outgoing ? e->getToNode() : newNode;
            e->reinitNodes(from, to);
            // re-add connections which previously existed and may still valid.
            // connections to removed edges will be ignored
            for (std::vector<NBEdge::Connection>::iterator k = conns.begin(); k != conns.end(); ++k) {
                e->addLane2LaneConnection((*k).fromLane, (*k).toEdge, (*k).toLane, NBEdge::L2L_USER, false, (*k).mayDefinitelyPass);
                if ((*k).fromLane >= 0 && (*k).fromLane < e->getNumLanes() && e->getLaneStruct((*k).fromLane).connectionsDone) {
                    // @note (see NIImporter_DlrNavteq::ConnectedLanesHandler)
                    e->declareConnectionsAsLoaded(NBEdge::INIT);
                }
            }
        }
        // remove original nodes
        registerJoinedCluster(cluster);
        for (NodeSet::const_iterator j = cluster.begin(); j != cluster.end(); ++j) {
            erase(*j);
        }
    }
}


void
NBNodeCont::registerJoinedCluster(const NodeSet& cluster) {
    std::set<std::string> ids;
    for (std::set<NBNode*>::const_iterator j = cluster.begin(); j != cluster.end(); j++) {
        ids.insert((*j)->getID());
    }
    myJoinedClusters.push_back(ids);
}


void
NBNodeCont::analyzeCluster(NodeSet cluster, std::string& id, Position& pos,
                           bool& hasTLS, TrafficLightType& type, SumoXMLNodeType& nodeType) {
    id += "_" + joinNamedToString(cluster, '_');
    hasTLS = false;
    bool ambiguousType = false;
    for (std::set<NBNode*>::const_iterator j = cluster.begin(); j != cluster.end(); j++) {
        pos.add((*j)->getPosition());
        // add a traffic light if any of the cluster members was controlled
        if ((*j)->isTLControlled()) {
            if (!hasTLS) {
                // init type
                type = (*(*j)->getControllingTLS().begin())->getType();
            } else if (type != (*(*j)->getControllingTLS().begin())->getType()) {
                ambiguousType = true;
            }
            hasTLS = true;
        }
        SumoXMLNodeType otherType = (*j)->getType();
        if (nodeType == NODETYPE_UNKNOWN) {
            nodeType = otherType;
        } else if (nodeType != otherType) {
            if (hasTLS) {
                nodeType = NODETYPE_TRAFFIC_LIGHT;;
            } else {
                if ((nodeType != NODETYPE_PRIORITY && (nodeType != NODETYPE_NOJUNCTION || otherType != NODETYPE_PRIORITY))
                        || (otherType != NODETYPE_NOJUNCTION && otherType != NODETYPE_UNKNOWN && otherType != NODETYPE_PRIORITY)) {
                    WRITE_WARNING("Ambiguous node type for node cluster '" + id + "' (" + toString(nodeType) + "," + toString(otherType) + ") set to '" + toString(NODETYPE_PRIORITY) + "'");
                }
                nodeType = NODETYPE_PRIORITY;
            }
        }
    }
    pos.mul(1.0 / cluster.size());
    if (ambiguousType) {
        type = SUMOXMLDefinitions::TrafficLightTypes.get(OptionsCont::getOptions().getString("tls.default-type"));
        WRITE_WARNING("Ambiguous traffic light type for node cluster '" + id + "' set to '" + toString(type) + "'");
    }
}


// ----------- (Helper) methods for guessing/computing traffic lights
bool
NBNodeCont::shouldBeTLSControlled(const NodeSet& c) const {
    int noIncoming = 0;
    int noOutgoing = 0;
    bool tooFast = false;
    double f = 0;
    std::set<NBEdge*> seen;
    for (std::set<NBNode*>::const_iterator j = c.begin(); j != c.end(); ++j) {
        const EdgeVector& edges = (*j)->getEdges();
        for (EdgeVector::const_iterator k = edges.begin(); k != edges.end(); ++k) {
            if (c.find((*k)->getFromNode()) != c.end() && c.find((*k)->getToNode()) != c.end()) {
                continue;
            }
            if ((*j)->hasIncoming(*k)) {
                ++noIncoming;
                f += (double)(*k)->getNumLanes() * (*k)->getLaneSpeed(0);
            } else {
                ++noOutgoing;
            }
            if ((*k)->getLaneSpeed(0) * 3.6 > 79) {
                tooFast = true;
            }
        }
    }
    return !tooFast && f >= 150. / 3.6 && c.size() != 0;
}


void
NBNodeCont::guessTLs(OptionsCont& oc, NBTrafficLightLogicCont& tlc) {
    // build list of definitely not tls-controlled junctions
    std::vector<NBNode*> ncontrolled;
    if (oc.isSet("tls.unset")) {
        std::vector<std::string> notTLControlledNodes = oc.getStringVector("tls.unset");
        for (std::vector<std::string>::const_iterator i = notTLControlledNodes.begin(); i != notTLControlledNodes.end(); ++i) {
            NBNode* n = NBNodeCont::retrieve(*i);
            if (n == 0) {
                throw ProcessError(" The junction '" + *i + "' to set as not-controlled is not known.");
            }
            std::set<NBTrafficLightDefinition*> tls = n->getControllingTLS();
            for (std::set<NBTrafficLightDefinition*>::const_iterator j = tls.begin(); j != tls.end(); ++j) {
                (*j)->removeNode(n);
            }
            n->removeTrafficLights();
            ncontrolled.push_back(n);
        }
    }

    TrafficLightType type = SUMOXMLDefinitions::TrafficLightTypes.get(OptionsCont::getOptions().getString("tls.default-type"));
    // loop#1 checking whether the node shall be tls controlled,
    //  because it is assigned to a district
    if (oc.exists("tls.taz-nodes") && oc.getBool("tls.taz-nodes")) {
        for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
            NBNode* cur = (*i).second;
            if (cur->isNearDistrict() && find(ncontrolled.begin(), ncontrolled.end(), cur) == ncontrolled.end()) {
                setAsTLControlled(cur, tlc, type);
            }
        }
    }

    // figure out which nodes mark the locations of TLS signals
    // This assumes nodes are already joined
    if (oc.exists("tls.guess-signals") && oc.getBool("tls.guess-signals")) {
        // prepare candidate edges
        const double signalDist = oc.getFloat("tls.guess-signals.dist");
        for (std::map<std::string, NBNode*>::const_iterator i = myNodes.begin(); i != myNodes.end(); ++i) {
            NBNode* node = (*i).second;
            if (node->isTLControlled() && node->geometryLike()) {
                const EdgeVector& outgoing = node->getOutgoingEdges();
                for (EdgeVector::const_iterator it_o = outgoing.begin(); it_o != outgoing.end(); ++it_o) {
                    (*it_o)->setSignalOffset((*it_o)->getLength());
                }
            }
        }
        // check which nodes should be controlled
        for (std::map<std::string, NBNode*>::const_iterator i = myNodes.begin(); i != myNodes.end(); ++i) {
            NBNode* node = i->second;
            if (find(ncontrolled.begin(), ncontrolled.end(), node) != ncontrolled.end()) {
                continue;
            }
            const EdgeVector& incoming = node->getIncomingEdges();
            const EdgeVector& outgoing = node->getOutgoingEdges();
            if (!node->isTLControlled() && incoming.size() > 1 && !node->geometryLike() && !NBNodeTypeComputer::isRailwayNode(node)) {
                std::vector<NBNode*> signals;
                bool isTLS = true;
                for (EdgeVector::const_iterator it_i = incoming.begin(); it_i != incoming.end(); ++it_i) {
                    const NBEdge* inEdge = *it_i;
                    if (inEdge->getSignalOffset() == NBEdge::UNSPECIFIED_SIGNAL_OFFSET || inEdge->getSignalOffset() > signalDist) {
                        isTLS = false;
                        break;
                    }
                    if (inEdge->getSignalOffset() == inEdge->getLength()) {
                        signals.push_back(inEdge->getFromNode());
                    }
                }
                // outgoing edges may be tagged with pedestrian crossings. These
                // should also be merged into the main TLS
                for (EdgeVector::const_iterator it_i = outgoing.begin(); it_i != outgoing.end(); ++it_i) {
                    const NBEdge* outEdge = *it_i;
                    NBNode* cand = outEdge->getToNode();
                    if (cand->isTLControlled() && cand->geometryLike() && outEdge->getLength() <= signalDist) {
                        signals.push_back(cand);
                    }
                }
                if (isTLS) {
                    for (std::vector<NBNode*>::iterator j = signals.begin(); j != signals.end(); ++j) {
                        std::set<NBTrafficLightDefinition*> tls = (*j)->getControllingTLS();
                        (*j)->reinit((*j)->getPosition(), NODETYPE_PRIORITY);
                        for (std::set<NBTrafficLightDefinition*>::iterator k = tls.begin(); k != tls.end(); ++k) {
                            tlc.removeFully((*j)->getID());
                        }
                    }
                    NBTrafficLightDefinition* tlDef = new NBOwnTLDef("GS_" + node->getID(), node, 0, type);
                    // @todo patch endOffset for all incoming lanes according to the signal positions
                    if (!tlc.insert(tlDef)) {
                        // actually, nothing should fail here
                        WRITE_WARNING("Could not build joined tls '" + node->getID() + "'.");
                        delete tlDef;
                        return;
                    }
                }
            }
        }
    }

    // maybe no tls shall be guessed
    if (!oc.getBool("tls.guess")) {
        return;
    }

    // guess joined tls first, if wished
    if (oc.getBool("tls.join")) {
        // get node clusters
        NodeClusters cands;
        generateNodeClusters(oc.getFloat("tls.join-dist"), cands);
        // check these candidates (clusters) whether they should be controlled by a tls
        for (NodeClusters::iterator i = cands.begin(); i != cands.end();) {
            NodeSet& c = (*i);
            // regard only junctions which are not yet controlled and are not
            //  forbidden to be controlled
            for (NodeSet::iterator j = c.begin(); j != c.end();) {
                if ((*j)->isTLControlled() || find(ncontrolled.begin(), ncontrolled.end(), *j) != ncontrolled.end()) {
                    c.erase(j++);
                } else {
                    ++j;
                }
            }
            // check whether the cluster should be controlled
            if (!shouldBeTLSControlled(c)) {
                i = cands.erase(i);
            } else {
                ++i;
            }
        }
        // cands now only contain sets of junctions that shall be joined into being tls-controlled
        int index = 0;
        for (NodeClusters::iterator i = cands.begin(); i != cands.end(); ++i) {
            std::vector<NBNode*> nodes;
            for (NodeSet::iterator j = (*i).begin(); j != (*i).end(); j++) {
                nodes.push_back(*j);
            }
            std::string id = "joinedG_" + toString(index++);
            NBTrafficLightDefinition* tlDef = new NBOwnTLDef(id, nodes, 0, type);
            if (!tlc.insert(tlDef)) {
                // actually, nothing should fail here
                WRITE_WARNING("Could not build guessed, joined tls");
                delete tlDef;
                return;
            }
        }
    }

    // guess tls
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        NBNode* cur = (*i).second;
        //  do nothing if already is tl-controlled
        if (cur->isTLControlled()) {
            continue;
        }
        // do nothing if in the list of explicit non-controlled junctions
        if (find(ncontrolled.begin(), ncontrolled.end(), cur) != ncontrolled.end()) {
            continue;
        }
        NodeSet c;
        c.insert(cur);
        if (!shouldBeTLSControlled(c) || cur->getIncomingEdges().size() < 3) {
            continue;
        }
        setAsTLControlled((*i).second, tlc, type);
    }
}


void
NBNodeCont::joinTLS(NBTrafficLightLogicCont& tlc, double maxdist) {
    NodeClusters cands;
    generateNodeClusters(maxdist, cands);
    int index = 0;
    for (NodeSet& c : cands) {
        for (NodeSet::iterator j = c.begin(); j != c.end();) {
            if (!(*j)->isTLControlled()) {
                c.erase(j++);
            } else {
                ++j;
            }
        }
        if (c.size() < 2) {
            continue;
        }
        // figure out type of the joined TLS
        Position dummyPos;
        bool dummySetTL;
        std::string id = "joined"; // prefix (see #3871)
        TrafficLightType type;
        SumoXMLNodeType nodeType = NODETYPE_UNKNOWN;
        analyzeCluster(c, id, dummyPos, dummySetTL, type, nodeType);
        for (std::set<NBNode*>::iterator j = c.begin(); j != c.end(); ++j) {
            std::set<NBTrafficLightDefinition*> tls = (*j)->getControllingTLS();
            (*j)->removeTrafficLights();
            for (std::set<NBTrafficLightDefinition*>::iterator k = tls.begin(); k != tls.end(); ++k) {
                tlc.removeFully((*j)->getID());
            }
        }
        id = "joinedS_" + toString(index++);
        std::vector<NBNode*> nodes;
        for (NodeSet::iterator j = c.begin(); j != c.end(); j++) {
            nodes.push_back(*j);
        }
        NBTrafficLightDefinition* tlDef = new NBOwnTLDef(id, nodes, 0, type);
        if (!tlc.insert(tlDef)) {
            // actually, nothing should fail here
            WRITE_WARNING("Could not build a joined tls.");
            delete tlDef;
            return;
        }
    }
}


void
NBNodeCont::setAsTLControlled(NBNode* node, NBTrafficLightLogicCont& tlc,
                              TrafficLightType type, std::string id) {
    if (id == "") {
        id = node->getID();
    }
    NBTrafficLightDefinition* tlDef = new NBOwnTLDef(id, node, 0, type);
    if (!tlc.insert(tlDef)) {
        // actually, nothing should fail here
        WRITE_WARNING("Building a tl-logic for junction '" + id + "' twice is not possible.");
        delete tlDef;
        return;
    }
}


// -----------
void
NBNodeCont::computeLanes2Lanes() {
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        (*i).second->computeLanes2Lanes();
    }
}


// computes the "wheel" of incoming and outgoing edges for every node
void
NBNodeCont::computeLogics(const NBEdgeCont& ec, OptionsCont& oc) {
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        (*i).second->computeLogic(ec, oc);
    }
}


void
NBNodeCont::clear() {
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        delete((*i).second);
    }
    myNodes.clear();
    for (std::set<NBNode*>::iterator i = myExtractedNodes.begin(); i != myExtractedNodes.end(); i++) {
        delete(*i);
    }
    myExtractedNodes.clear();
}


std::string
NBNodeCont::getFreeID() {
    int counter = 0;
    std::string freeID = "SUMOGenerated" + toString<int>(counter);
    // While there is a node with id equal to freeID
    while (retrieve(freeID) != 0) {
        // update counter and generate a new freeID
        counter++;
        freeID = "SUMOGenerated" + toString<int>(counter);
    }
    return freeID;
}


void
NBNodeCont::computeNodeShapes(double mismatchThreshold) {
    for (NodeCont::iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        (*i).second->computeNodeShape(mismatchThreshold);
    }
}


void
NBNodeCont::printBuiltNodesStatistics() const {
    int numUnregulatedJunctions = 0;
    int numDeadEndJunctions = 0;
    int numPriorityJunctions = 0;
    int numRightBeforeLeftJunctions = 0;
    int numAllWayStopJunctions = 0;
    int numZipperJunctions = 0;
    int numRailSignals = 0;
    for (NodeCont::const_iterator i = myNodes.begin(); i != myNodes.end(); i++) {
        switch ((*i).second->getType()) {
            case NODETYPE_NOJUNCTION:
            case NODETYPE_TRAFFIC_LIGHT_NOJUNCTION:
                ++numUnregulatedJunctions;
                break;
            case NODETYPE_DEAD_END:
                ++numDeadEndJunctions;
                break;
            case NODETYPE_PRIORITY:
            case NODETYPE_PRIORITY_STOP:
            case NODETYPE_TRAFFIC_LIGHT:
            case NODETYPE_TRAFFIC_LIGHT_RIGHT_ON_RED:
            case NODETYPE_RAIL_CROSSING:
                ++numPriorityJunctions;
                break;
            case NODETYPE_RIGHT_BEFORE_LEFT:
                ++numRightBeforeLeftJunctions;
                break;
            case NODETYPE_ALLWAY_STOP:
                ++numAllWayStopJunctions;
                break;
            case NODETYPE_ZIPPER:
                ++numZipperJunctions;
                break;
            case NODETYPE_DISTRICT:
                ++numRightBeforeLeftJunctions;
                break;
            case NODETYPE_UNKNOWN:
                break;
            case NODETYPE_RAIL_SIGNAL:
                ++numRailSignals;
                break;
            default:
                break;
        }
    }
    WRITE_MESSAGE(" Node type statistics:");
    WRITE_MESSAGE("  Unregulated junctions       : " + toString(numUnregulatedJunctions));
    if (numDeadEndJunctions > 0) {
        WRITE_MESSAGE("  Dead-end junctions          : " + toString(numDeadEndJunctions));
    }
    WRITE_MESSAGE("  Priority junctions          : " + toString(numPriorityJunctions));
    WRITE_MESSAGE("  Right-before-left junctions : " + toString(numRightBeforeLeftJunctions));
    if (numAllWayStopJunctions > 0) {
        WRITE_MESSAGE("  All-way stop junctions      : " + toString(numAllWayStopJunctions));
    }
    if (numZipperJunctions > 0) {
        WRITE_MESSAGE("  Zipper-merge junctions      : " + toString(numZipperJunctions));
    }
    if (numRailSignals > 0) {
        WRITE_MESSAGE("  Rail signal junctions      : " + toString(numRailSignals));
    }
}


std::vector<std::string>
NBNodeCont::getAllNames() const {
    std::vector<std::string> ret;
    for (NodeCont::const_iterator i = myNodes.begin(); i != myNodes.end(); ++i) {
        ret.push_back((*i).first);
    }
    return ret;
}


void
NBNodeCont::rename(NBNode* node, const std::string& newID) {
    if (myNodes.count(newID) != 0) {
        throw ProcessError("Attempt to rename node using existing id '" + newID + "'");
    }
    myNodes.erase(node->getID());
    node->setID(newID);
    myNodes[newID] = node;
}


void
NBNodeCont::discardTrafficLights(NBTrafficLightLogicCont& tlc, bool geometryLike, bool guessSignals) {
    for (NodeCont::const_iterator i = myNodes.begin(); i != myNodes.end(); ++i) {
        NBNode* node = i->second;
        if (node->isTLControlled() && (!geometryLike || node->geometryLike())) {
            // make a copy of tldefs
            const std::set<NBTrafficLightDefinition*> tldefs = node->getControllingTLS();
            if (guessSignals && node->isTLControlled() && node->geometryLike()) {
                // record signal location
                const EdgeVector& outgoing = node->getOutgoingEdges();
                for (EdgeVector::const_iterator it_o = outgoing.begin(); it_o != outgoing.end(); ++it_o) {
                    (*it_o)->setSignalOffset((*it_o)->getLength());
                }
            }
            for (std::set<NBTrafficLightDefinition*>::const_iterator it = tldefs.begin(); it != tldefs.end(); ++it) {
                NBTrafficLightDefinition* tlDef = *it;
                node->removeTrafficLight(tlDef);
                tlc.extract(tlDef);
            }
            SumoXMLNodeType newType = NBNodeTypeComputer::isRailwayNode(node) ? NODETYPE_RAIL_SIGNAL : NODETYPE_UNKNOWN;
            node->reinit(node->getPosition(), newType);
        }
    }
}


void
NBNodeCont::discardRailSignals() {
    for (auto& item : myNodes) {
        NBNode* node = item.second;
        if (node->getType() == NODETYPE_RAIL_SIGNAL) {
            node->reinit(node->getPosition(), NODETYPE_PRIORITY);
        }
    }
}


int
NBNodeCont::remapIDs(bool numericaIDs, bool reservedIDs, const std::string& prefix) {
    std::vector<std::string> avoid = getAllNames();
    std::set<std::string> reserve;
    if (reservedIDs) {
        NBHelpers::loadPrefixedIDsFomFile(OptionsCont::getOptions().getString("reserved-ids"), "node:", reserve);
        avoid.insert(avoid.end(), reserve.begin(), reserve.end());
    }
    IDSupplier idSupplier("", avoid);
    NodeSet toChange;
    for (NodeCont::iterator it = myNodes.begin(); it != myNodes.end(); it++) {
        if (numericaIDs) {
            try {
                TplConvert::_str2long(it->first);
            } catch (NumberFormatException&) {
                toChange.insert(it->second);
            }
        }
        if (reservedIDs && reserve.count(it->first) > 0) {
            toChange.insert(it->second);
        }
    }
    const bool origNames = OptionsCont::getOptions().getBool("output.original-names");
    for (NBNode* node : toChange) {
        myNodes.erase(node->getID());
        if (origNames) {
            node->setParameter(SUMO_PARAM_ORIGID, node->getID());
        }
        node->setID(idSupplier.getNext());
        myNodes[node->getID()] = node;
    }
    if (prefix.empty()) {
        return (int)toChange.size();
    } else {
        int renamed = 0;
        // make a copy because we will modify the map
        auto oldNodes = myNodes;
        for (auto item : oldNodes) {
            if (!StringUtils::startsWith(item.first, prefix)) {
                rename(item.second, prefix + item.first);
                renamed++;
            }
        }
        return renamed;
    }
}

/****************************************************************************/

