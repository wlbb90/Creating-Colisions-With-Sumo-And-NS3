/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NIXMLEdgesHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @author  Laura Bieker
/// @author  Leonhard Luecken
/// @date    Tue, 20 Nov 2001
/// @version $Id$
///
// Importer for network edges stored in XML
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <iostream>
#include <map>
#include <cmath>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/sax/AttributeList.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>
#include <utils/xml/SUMOSAXHandler.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBTypeCont.h>
#include <netbuild/NBNetBuilder.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/TplConvert.h>
#include <utils/common/StringTokenizer.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/common/ToString.h>
#include <utils/options/OptionsCont.h>
#include <utils/geom/GeoConvHelper.h>
#include "NIXMLNodesHandler.h"
#include "NIXMLEdgesHandler.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIXMLEdgesHandler::NIXMLEdgesHandler(NBNodeCont& nc,
                                     NBEdgeCont& ec,
                                     NBTypeCont& tc,
                                     NBDistrictCont& dc,
                                     NBTrafficLightLogicCont& tlc,
                                     OptionsCont& options) :
    SUMOSAXHandler("xml-edges - file"),
    myOptions(options),
    myNodeCont(nc),
    myEdgeCont(ec),
    myTypeCont(tc),
    myDistrictCont(dc),
    myTLLogicCont(tlc),
    myCurrentEdge(0),
    myCurrentLaneIndex(-1),
    myHaveReportedAboutOverwriting(false),
    myHaveReportedAboutTypeOverride(false),
    myHaveWarnedAboutDeprecatedLaneId(false),
    myKeepEdgeShape(!options.getBool("plain.extend-edge-shape")) {
}


NIXMLEdgesHandler::~NIXMLEdgesHandler() {}


void
NIXMLEdgesHandler::myStartElement(int element,
                                  const SUMOSAXAttributes& attrs) {
    switch (element) {
        case SUMO_TAG_EDGE:
            addEdge(attrs);
            break;
        case SUMO_TAG_LANE:
            addLane(attrs);
            break;
        case SUMO_TAG_NEIGH:
            myCurrentEdge->getLaneStruct((int)myCurrentEdge->getNumLanes() - 1).oppositeID = attrs.getString(SUMO_ATTR_LANE);
            break;
        case SUMO_TAG_SPLIT:
            addSplit(attrs);
            break;
        case SUMO_TAG_DELETE:
            deleteEdge(attrs);
            break;
        case SUMO_TAG_ROUNDABOUT:
            addRoundabout(attrs);
            break;
        case SUMO_TAG_PARAM:
            if (myLastParameterised.size() != 0 && myCurrentEdge != 0) {
                bool ok = true;
                const std::string key = attrs.get<std::string>(SUMO_ATTR_KEY, 0, ok);
                // circumventing empty string test
                const std::string val = attrs.hasAttribute(SUMO_ATTR_VALUE) ? attrs.getString(SUMO_ATTR_VALUE) : "";
                myLastParameterised.back()->setParameter(key, val);
            }
            break;
        case SUMO_TAG_STOPOFFSET: {
            bool ok = true;
            std::map<SVCPermissions, double> stopOffsets = parseStopOffsets(attrs, ok);
            assert(stopOffsets.size() == 1);
            if (!ok) {
                std::stringstream ss;
                ss << "(Error encountered at lane " << myCurrentLaneIndex << " of edge '" << myCurrentID << "' while parsing stopOffsets.)";
                WRITE_ERROR(ss.str());
            } else {
                if (myCurrentEdge->getStopOffsets(myCurrentLaneIndex).size() != 0) {
                    std::stringstream ss;
                    ss << "Duplicate definition of stopOffset for ";
                    if (myCurrentLaneIndex != -1) {
                        ss << "lane " << myCurrentLaneIndex << " on ";
                    }
                    ss << "edge " << myCurrentEdge->getID() << ". Ignoring duplicate specification.";
                    WRITE_WARNING(ss.str());
                    return;
                } else if (stopOffsets.begin()->second > myCurrentEdge->getLength() || stopOffsets.begin()->second < 0) {
                    std::stringstream ss;
                    ss << "Ignoring invalid stopOffset for ";
                    if (myCurrentLaneIndex != -1) {
                        ss << "lane " << myCurrentLaneIndex << " on ";
                    }
                    ss << "edge " << myCurrentEdge->getID();
                    if (stopOffsets.begin()->second > myCurrentEdge->getLength()) {
                        ss << " (offset larger than the edge length).";
                    } else {
                        ss << " (negative offset).";
                    }
                    WRITE_WARNING(ss.str());
                } else {
                    myCurrentEdge->setStopOffsets(myCurrentLaneIndex, stopOffsets);
                }
            }
        }
        break;
        default:
            break;
    }
}


void
NIXMLEdgesHandler::addEdge(const SUMOSAXAttributes& attrs) {
    myIsUpdate = false;
    bool ok = true;
    // initialise the edge
    myCurrentEdge = 0;
    mySplits.clear();
    // get the id, report an error if not given or empty...
    myCurrentID = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    myCurrentEdge = myEdgeCont.retrieve(myCurrentID);
    // check deprecated (unused) attributes
    // use default values, first
    myCurrentPriority = myTypeCont.getPriority("");
    myCurrentLaneNo = myTypeCont.getNumLanes("");
    myCurrentEndOffset = NBEdge::UNSPECIFIED_OFFSET;
    if (myCurrentEdge != 0) {
        // update existing edge. only update lane-specific settings when explicitly requested
        myIsUpdate = true;
        myCurrentSpeed = NBEdge::UNSPECIFIED_SPEED;
        myPermissions = SVC_UNSPECIFIED;
        myCurrentWidth = NBEdge::UNSPECIFIED_WIDTH;
    } else {
        // this is a completely new edge. get the type specific defaults
        myCurrentSpeed = myTypeCont.getSpeed("");
        myPermissions = myTypeCont.getPermissions("");
        myCurrentWidth = myTypeCont.getWidth("");
    }
    myCurrentType = "";
    myShape = PositionVector();
    myLanesSpread = LANESPREAD_RIGHT;
    myLength = NBEdge::UNSPECIFIED_LOADED_LENGTH;
    myCurrentStreetName = "";
    myReinitKeepEdgeShape = false;
    mySidewalkWidth = NBEdge::UNSPECIFIED_WIDTH;
    myBikeLaneWidth = NBEdge::UNSPECIFIED_WIDTH;
    // check whether a type's values shall be used
    if (attrs.hasAttribute(SUMO_ATTR_TYPE)) {
        myCurrentType = attrs.get<std::string>(SUMO_ATTR_TYPE, myCurrentID.c_str(), ok);
        if (!ok) {
            return;
        }
        if (!myTypeCont.knows(myCurrentType) && !myOptions.getBool("ignore-errors.edge-type")) {
            WRITE_ERROR("Type '" + myCurrentType + "' used by edge '" + myCurrentID + "' was not defined (ignore with option --ignore-errors.edge-type).");
            return;
        }
        myCurrentSpeed = myTypeCont.getSpeed(myCurrentType);
        myCurrentPriority = myTypeCont.getPriority(myCurrentType);
        myCurrentLaneNo = myTypeCont.getNumLanes(myCurrentType);
        myPermissions = myTypeCont.getPermissions(myCurrentType);
        myCurrentWidth = myTypeCont.getWidth(myCurrentType);
        mySidewalkWidth = myTypeCont.getSidewalkWidth(myCurrentType);
        myBikeLaneWidth = myTypeCont.getBikeLaneWidth(myCurrentType);
    }
    // use values from the edge to overwrite if existing, then
    if (myIsUpdate) {
        if (!myHaveReportedAboutOverwriting) {
            WRITE_MESSAGE("Duplicate edge id occurred ('" + myCurrentID + "'); assuming overwriting is wished.");
            myHaveReportedAboutOverwriting = true;
        }
        if (attrs.hasAttribute(SUMO_ATTR_TYPE) && myCurrentType != myCurrentEdge->getTypeID()) {
            if (!myHaveReportedAboutTypeOverride) {
                WRITE_MESSAGE("Edge '" + myCurrentID + "' changed it's type; assuming type override is wished.");
                myHaveReportedAboutTypeOverride = true;
            }
        }
        if (attrs.getOpt<bool>(SUMO_ATTR_REMOVE, myCurrentID.c_str(), ok, false)) {
            myEdgeCont.erase(myDistrictCont, myCurrentEdge);
            myCurrentEdge = 0;
            return;
        }
        myCurrentPriority = myCurrentEdge->getPriority();
        myCurrentLaneNo = myCurrentEdge->getNumLanes();
        if (!myCurrentEdge->hasDefaultGeometry()) {
            myShape = myCurrentEdge->getGeometry();
            myReinitKeepEdgeShape = true;
        }
        myLanesSpread = myCurrentEdge->getLaneSpreadFunction();
        if (myCurrentEdge->hasLoadedLength()) {
            myLength = myCurrentEdge->getLoadedLength();
        }
        myCurrentStreetName = myCurrentEdge->getStreetName();
    }
    // speed, priority and the number of lanes have now default values;
    // try to read the real values from the file
    if (attrs.hasAttribute(SUMO_ATTR_SPEED)) {
        myCurrentSpeed = attrs.get<double>(SUMO_ATTR_SPEED, myCurrentID.c_str(), ok);
    }
    if (myOptions.getBool("speed-in-kmh") && myCurrentSpeed != NBEdge::UNSPECIFIED_SPEED) {
        myCurrentSpeed = myCurrentSpeed / (double) 3.6;
    }
    // try to get the number of lanes
    if (attrs.hasAttribute(SUMO_ATTR_NUMLANES)) {
        myCurrentLaneNo = attrs.get<int>(SUMO_ATTR_NUMLANES, myCurrentID.c_str(), ok);
    }
    // try to get the priority
    if (attrs.hasAttribute(SUMO_ATTR_PRIORITY)) {
        myCurrentPriority = attrs.get<int>(SUMO_ATTR_PRIORITY, myCurrentID.c_str(), ok);
    }
    // try to get the width
    if (attrs.hasAttribute(SUMO_ATTR_WIDTH)) {
        myCurrentWidth = attrs.get<double>(SUMO_ATTR_WIDTH, myCurrentID.c_str(), ok);
    }
    // try to get the offset of the stop line from the intersection
    if (attrs.hasAttribute(SUMO_ATTR_ENDOFFSET)) {
        myCurrentEndOffset = attrs.get<double>(SUMO_ATTR_ENDOFFSET, myCurrentID.c_str(), ok);
    }
    // try to get the street name
    if (attrs.hasAttribute(SUMO_ATTR_NAME)) {
        myCurrentStreetName = attrs.get<std::string>(SUMO_ATTR_NAME, myCurrentID.c_str(), ok);
        if (myCurrentStreetName != "" && myOptions.isDefault("output.street-names")) {
            myOptions.set("output.street-names", "true");
        }
    }

    // try to get the allowed/disallowed classes
    if (attrs.hasAttribute(SUMO_ATTR_ALLOW) || attrs.hasAttribute(SUMO_ATTR_DISALLOW)) {
        std::string allowS = attrs.hasAttribute(SUMO_ATTR_ALLOW) ? attrs.getStringSecure(SUMO_ATTR_ALLOW, "") : "";
        std::string disallowS = attrs.hasAttribute(SUMO_ATTR_DISALLOW) ? attrs.getStringSecure(SUMO_ATTR_DISALLOW, "") : "";
        // XXX matter of interpretation: should updated permissions replace or extend previously set permissions?
        myPermissions = parseVehicleClasses(allowS, disallowS);
    }
    // try to set the nodes
    if (!setNodes(attrs)) {
        // return if this failed
        return;
    }
    // try to get the shape
    myShape = tryGetShape(attrs);
    // try to get the spread type
    myLanesSpread = tryGetLaneSpread(attrs);
    // try to get the length
    myLength = attrs.getOpt<double>(SUMO_ATTR_LENGTH, myCurrentID.c_str(), ok, myLength);
    // try to get the sidewalkWidth
    mySidewalkWidth = attrs.getOpt<double>(SUMO_ATTR_SIDEWALKWIDTH, myCurrentID.c_str(), ok, mySidewalkWidth);
    // try to get the bikeLaneWidth
    myBikeLaneWidth = attrs.getOpt<double>(SUMO_ATTR_BIKELANEWIDTH, myCurrentID.c_str(), ok, myBikeLaneWidth);
    // insert the parsed edge into the edges map
    if (!ok) {
        return;
    }
    // check whether a previously defined edge shall be overwritten
    if (myCurrentEdge != 0) {
        myCurrentEdge->reinit(myFromNode, myToNode, myCurrentType, myCurrentSpeed,
                              myCurrentLaneNo, myCurrentPriority, myShape,
                              myCurrentWidth, myCurrentEndOffset,
                              myCurrentStreetName, myLanesSpread,
                              myReinitKeepEdgeShape);
    } else {
        // the edge must be allocated in dependence to whether a shape is given
        if (myShape.size() == 0) {
            myCurrentEdge = new NBEdge(myCurrentID, myFromNode, myToNode, myCurrentType, myCurrentSpeed,
                                       myCurrentLaneNo, myCurrentPriority, myCurrentWidth, myCurrentEndOffset,
                                       myCurrentStreetName, myLanesSpread);
        } else {
            myCurrentEdge = new NBEdge(myCurrentID, myFromNode, myToNode, myCurrentType, myCurrentSpeed,
                                       myCurrentLaneNo, myCurrentPriority, myCurrentWidth, myCurrentEndOffset,
                                       myShape, myCurrentStreetName, "", myLanesSpread,
                                       myKeepEdgeShape);
        }
    }
    myCurrentEdge->setLoadedLength(myLength);
    if (myPermissions != SVC_UNSPECIFIED) {
        myCurrentEdge->setPermissions(myPermissions);
    }
    myLastParameterised.push_back(myCurrentEdge);
}


void
NIXMLEdgesHandler::addLane(const SUMOSAXAttributes& attrs) {
    if (myCurrentEdge == 0) {
        if (!OptionsCont::getOptions().isInStringVector("remove-edges.explicit", myCurrentID)) {
            WRITE_ERROR("Additional lane information could not be set - the edge with id '" + myCurrentID + "' is not known.");
        }
        return;
    }
    bool ok = true;
    int lane;
    if (attrs.hasAttribute(SUMO_ATTR_ID)) {
        lane = attrs.get<int>(SUMO_ATTR_ID, myCurrentID.c_str(), ok);
        if (!myHaveWarnedAboutDeprecatedLaneId) {
            myHaveWarnedAboutDeprecatedLaneId = true;
            WRITE_WARNING("'" + toString(SUMO_ATTR_ID) + "' is deprecated, please use '" + toString(SUMO_ATTR_INDEX) + "' instead.");
        }
    } else {
        lane = attrs.get<int>(SUMO_ATTR_INDEX, myCurrentID.c_str(), ok);
    }
    if (!ok) {
        return;
    }
    // check whether this lane exists
    if (lane >= myCurrentEdge->getNumLanes()) {
        WRITE_ERROR("Lane index is larger than number of lanes (edge '" + myCurrentID + "').");
        return;
    }
    myCurrentLaneIndex = lane;
    // set information about allowed / disallowed vehicle classes (if specified)
    if (attrs.hasAttribute(SUMO_ATTR_ALLOW) || attrs.hasAttribute(SUMO_ATTR_DISALLOW)) {
        const std::string allowed = attrs.getOpt<std::string>(SUMO_ATTR_ALLOW, 0, ok, "");
        const std::string disallowed = attrs.getOpt<std::string>(SUMO_ATTR_DISALLOW, 0, ok, "");
        myCurrentEdge->setPermissions(parseVehicleClasses(allowed, disallowed), lane);
    }
    if (attrs.hasAttribute(SUMO_ATTR_PREFER)) {
        const std::string preferred  = attrs.get<std::string>(SUMO_ATTR_PREFER, 0, ok);
        myCurrentEdge->setPreferredVehicleClass(parseVehicleClasses(preferred), lane);
    }
    // try to get the width
    if (attrs.hasAttribute(SUMO_ATTR_WIDTH)) {
        myCurrentEdge->setLaneWidth(lane, attrs.get<double>(SUMO_ATTR_WIDTH, myCurrentID.c_str(), ok));
    }
    // try to get the end-offset (lane shortened due to pedestrian crossing etc..)
    if (attrs.hasAttribute(SUMO_ATTR_ENDOFFSET)) {
        myCurrentEdge->setEndOffset(lane, attrs.get<double>(SUMO_ATTR_ENDOFFSET, myCurrentID.c_str(), ok));
    }
    // try to get lane specific speed (should not occur for german networks)
    if (attrs.hasAttribute(SUMO_ATTR_SPEED)) {
        myCurrentEdge->setSpeed(lane, attrs.get<double>(SUMO_ATTR_SPEED, myCurrentID.c_str(), ok));
    }
    // check whether this is an acceleration lane
    if (attrs.hasAttribute(SUMO_ATTR_ACCELERATION)) {
        myCurrentEdge->setAcceleration(lane, attrs.get<bool>(SUMO_ATTR_ACCELERATION, myCurrentID.c_str(), ok));
    }

    // check whether this is an acceleration lane
    if (attrs.hasAttribute(SUMO_ATTR_SHAPE)) {
        PositionVector shape = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, myCurrentID.c_str(), ok);
        if (!NBNetBuilder::transformCoordinates(shape)) {
            const std::string laneID = myCurrentID + "_" + toString(lane);
            WRITE_ERROR("Unable to project coordinates for lane '" + laneID + "'.");
        }
        myCurrentEdge->setLaneShape(lane, shape);
    }
    myLastParameterised.push_back(&myCurrentEdge->getLaneStruct(lane));
}


void NIXMLEdgesHandler::addSplit(const SUMOSAXAttributes& attrs) {
    if (myCurrentEdge == 0) {
        if (!OptionsCont::getOptions().isInStringVector("remove-edges.explicit", myCurrentID)) {
            WRITE_WARNING("Ignoring 'split' because it cannot be assigned to an edge");
        }
        return;
    }
    bool ok = true;
    NBEdgeCont::Split e;
    e.pos = attrs.get<double>(SUMO_ATTR_POSITION, 0, ok);
    if (ok) {
        if (fabs(e.pos) > myCurrentEdge->getGeometry().length()) {
            WRITE_ERROR("Edge '" + myCurrentID + "' has a split at invalid position " + toString(e.pos) + ".");
            return;
        }
        std::vector<NBEdgeCont::Split>::iterator i = find_if(mySplits.begin(), mySplits.end(), split_by_pos_finder(e.pos));
        if (i != mySplits.end()) {
            WRITE_ERROR("Edge '" + myCurrentID + "' has already a split at position " + toString(e.pos) + ".");
            return;
        }
        e.nameID = myCurrentID + "." + toString((int)e.pos);
        if (e.pos < 0) {
            e.pos += myCurrentEdge->getGeometry().length();
        }
        std::vector<std::string> lanes;
        SUMOSAXAttributes::parseStringVector(attrs.getOpt<std::string>(SUMO_ATTR_LANES, 0, ok, ""), lanes);
        for (std::vector<std::string>::iterator i = lanes.begin(); i != lanes.end(); ++i) {
            try {
                int lane = TplConvert::_2int((*i).c_str());
                e.lanes.push_back(lane);
            } catch (NumberFormatException&) {
                WRITE_ERROR("Error on parsing a split (edge '" + myCurrentID + "').");
            } catch (EmptyData&) {
                WRITE_ERROR("Error on parsing a split (edge '" + myCurrentID + "').");
            }
        }
        if (e.lanes.empty()) {
            for (int l = 0; l < myCurrentEdge->getNumLanes(); ++l) {
                e.lanes.push_back(l);
            }
        }
        e.speed = attrs.getOpt(SUMO_ATTR_SPEED, 0, ok, myCurrentEdge->getSpeed());
        if (attrs.hasAttribute(SUMO_ATTR_SPEED) && myOptions.getBool("speed-in-kmh")) {
            e.speed /= (double) 3.6;
        }
        e.idBefore = attrs.getOpt(SUMO_ATTR_ID_BEFORE, 0, ok, std::string(""));
        e.idAfter = attrs.getOpt(SUMO_ATTR_ID_AFTER, 0, ok, std::string(""));
        if (!ok) {
            return;
        }
        const std::string nodeID = attrs.getOpt(SUMO_ATTR_ID, 0, ok, e.nameID);
        if (nodeID == myCurrentEdge->getFromNode()->getID() || nodeID == myCurrentEdge->getToNode()->getID()) {
            WRITE_ERROR("Invalid split node id for edge '" + myCurrentEdge->getID() + "' (from- and to-node are forbidden)");
            return;
        }
        e.node = myNodeCont.retrieve(nodeID);
        if (e.node == 0) {
            e.node = new NBNode(nodeID, myCurrentEdge->getGeometry().positionAtOffset(e.pos));
        }
        NIXMLNodesHandler::processNodeType(attrs, e.node, e.node->getID(), e.node->getPosition(), false,
                                           myNodeCont, myTLLogicCont);
        mySplits.push_back(e);
    }
}


bool
NIXMLEdgesHandler::setNodes(const SUMOSAXAttributes& attrs) {
    // the names and the coordinates of the beginning and the end node
    // may be found, try
    bool ok = true;
    std::string begNodeID = myIsUpdate ? myCurrentEdge->getFromNode()->getID() : "";
    std::string endNodeID = myIsUpdate ? myCurrentEdge->getToNode()->getID() : "";
    std::string oldBegID = begNodeID;
    std::string oldEndID = endNodeID;
    if (attrs.hasAttribute(SUMO_ATTR_FROM)) {
        begNodeID = attrs.get<std::string>(SUMO_ATTR_FROM, 0, ok);
    } else if (!myIsUpdate) {
        WRITE_ERROR("The from-node is not given for edge '" + myCurrentID + "'.");
        ok = false;
    }
    if (attrs.hasAttribute(SUMO_ATTR_TO)) {
        endNodeID = attrs.get<std::string>(SUMO_ATTR_TO, 0, ok);
    } else if (!myIsUpdate) {
        WRITE_ERROR("The to-node is not given for edge '" + myCurrentID + "'.");
        ok = false;
    }
    if (!ok) {
        return false;
    }
    myFromNode = myNodeCont.retrieve(begNodeID);
    myToNode = myNodeCont.retrieve(endNodeID);
    if (myFromNode == 0) {
        WRITE_ERROR("Edge's '" + myCurrentID + "' from-node '" + begNodeID + "' is not known.");
    }
    if (myToNode == 0) {
        WRITE_ERROR("Edge's '" + myCurrentID + "' to-node '" + endNodeID + "' is not known.");
    }
    if (myFromNode != 0 && myToNode != 0) {
        if (myIsUpdate && (myFromNode->getID() != oldBegID || myToNode->getID() != oldEndID)) {
            myShape = PositionVector();
        }
    }
    return myFromNode != 0 && myToNode != 0;
}


PositionVector
NIXMLEdgesHandler::tryGetShape(const SUMOSAXAttributes& attrs) {
    if (!attrs.hasAttribute(SUMO_ATTR_SHAPE)) {
        return myShape;
    }
    // try to build shape
    bool ok = true;
    if (!attrs.hasAttribute(SUMO_ATTR_SHAPE)) {
        myReinitKeepEdgeShape = false;
        return PositionVector();
    }
    PositionVector shape = attrs.getOpt<PositionVector>(SUMO_ATTR_SHAPE, 0, ok, PositionVector());
    if (!NBNetBuilder::transformCoordinates(shape)) {
        WRITE_ERROR("Unable to project coordinates for edge '" + myCurrentID + "'.");
    }
    myReinitKeepEdgeShape = myKeepEdgeShape;
    return shape;
}


LaneSpreadFunction
NIXMLEdgesHandler::tryGetLaneSpread(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    LaneSpreadFunction result = myLanesSpread;
    std::string lsfS = toString(result);
    lsfS = attrs.getOpt<std::string>(SUMO_ATTR_SPREADTYPE, myCurrentID.c_str(), ok, lsfS);
    if (SUMOXMLDefinitions::LaneSpreadFunctions.hasString(lsfS)) {
        result = SUMOXMLDefinitions::LaneSpreadFunctions.get(lsfS);
    } else {
        WRITE_WARNING("Ignoring unknown spreadType '" + lsfS + "' for edge '" + myCurrentID + "'.");
    }
    return result;
}


void
NIXMLEdgesHandler::deleteEdge(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    myCurrentID = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    NBEdge* edge = myEdgeCont.retrieve(myCurrentID);
    if (edge == 0) {
        WRITE_WARNING("Ignoring tag '" + toString(SUMO_TAG_DELETE) + "' for unknown edge '" +
                      myCurrentID + "'");
        return;
    }
    const int lane = attrs.getOpt<int>(SUMO_ATTR_INDEX, myCurrentID.c_str(), ok, -1);
    if (lane < 0) {
        myEdgeCont.extract(myDistrictCont, edge, true);
    } else {
        edge->deleteLane(lane, false, true);
    }
}


void
NIXMLEdgesHandler::myEndElement(int element) {
    if (element == SUMO_TAG_EDGE && myCurrentEdge != 0) {
        myLastParameterised.pop_back();
        // add bike lane, wait until lanes are loaded to avoid building if it already exists
        if (myBikeLaneWidth != NBEdge::UNSPECIFIED_WIDTH) {
            myCurrentEdge->addBikeLane(myBikeLaneWidth);
        }
        // add sidewalk, wait until lanes are loaded to avoid building if it already exists
        if (mySidewalkWidth != NBEdge::UNSPECIFIED_WIDTH) {
            myCurrentEdge->addSidewalk(mySidewalkWidth);
        }
        // apply default stopOffsets of edge to all lanes without specified stopOffset.
        std::map<SVCPermissions, double> stopOffsets = myCurrentEdge->getStopOffsets(-1);
        if (stopOffsets.size() != 0) {
            for (int i = 0; i < (int)myCurrentEdge->getLanes().size(); i++) {
                myCurrentEdge->setStopOffsets(i, stopOffsets, false);
            }
        }
        if (!myIsUpdate) {
            try {
                if (!myEdgeCont.insert(myCurrentEdge)) {
                    WRITE_ERROR("Duplicate edge occurred. ID='" + myCurrentID + "'");
                    delete myCurrentEdge;
                }
            } catch (InvalidArgument& e) {
                WRITE_ERROR(e.what());
                throw;
            } catch (...) {
                WRITE_ERROR("An important information is missing in edge '" + myCurrentID + "'.");
            }
        }
        myEdgeCont.processSplits(myCurrentEdge, mySplits, myNodeCont, myDistrictCont, myTLLogicCont);
        myCurrentEdge = 0;
    } else if (element == SUMO_TAG_LANE) {
        myLastParameterised.pop_back();
        myCurrentLaneIndex = -1;
    }
}


void
NIXMLEdgesHandler::addRoundabout(const SUMOSAXAttributes& attrs) {
    if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
        std::vector<std::string> edgeIDs = attrs.getStringVector(SUMO_ATTR_EDGES);
        EdgeSet roundabout;
        for (std::vector<std::string>::iterator it = edgeIDs.begin(); it != edgeIDs.end(); ++it) {
            NBEdge* edge = myEdgeCont.retrieve(*it);
            if (edge == 0) {
                if (!myEdgeCont.wasIgnored(*it)) {
                    WRITE_ERROR("Unknown edge '" + (*it) + "' in roundabout");
                }
            } else {
                roundabout.insert(edge);
            }
        }
        myEdgeCont.addRoundabout(roundabout);
    } else {
        WRITE_ERROR("Empty edges in roundabout.");
    }
}


/****************************************************************************/

