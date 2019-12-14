/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    TraCIServerAPI_GUI.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    07.05.2009
/// @version $Id$
///
// APIs for getting/setting GUI values via TraCI
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <fx.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/windows/GUIGlChildWindow.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIPerspectiveChanger.h>
#include <utils/foxtools/MFXImageHelper.h>
#include <microsim/MSVehicleControl.h>
#include <traci-server/TraCIConstants.h>
#include <guisim/GUINet.h>
#include <guisim/GUIVehicle.h>
#include <guisim/GUIBaseVehicle.h>
#include "GUIEvent_Screenshot.h"
#include "TraCIServerAPI_GUI.h"

// ===========================================================================
// method definitions
// ===========================================================================
bool
TraCIServerAPI_GUI::processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                               tcpip::Storage& outputStorage) {
    // variable & id
    int variable = inputStorage.readUnsignedByte();
    std::string id = inputStorage.readString();
    // check variable
    if (variable != ID_LIST && variable != VAR_VIEW_ZOOM && variable != VAR_VIEW_OFFSET
            && variable != VAR_VIEW_SCHEMA && variable != VAR_VIEW_BOUNDARY && variable != VAR_HAS_VIEW) {
        return server.writeErrorStatusCmd(CMD_GET_GUI_VARIABLE, "Get GUI Variable: unsupported variable " + toHex(variable, 2) + " specified", outputStorage);
    }
    // begin response building
    tcpip::Storage tempMsg;
    //  response-code, variableID, objectID
    tempMsg.writeUnsignedByte(RESPONSE_GET_GUI_VARIABLE);
    tempMsg.writeUnsignedByte(variable);
    tempMsg.writeString(id);
    // process request
    if (variable == ID_LIST) {
        std::vector<std::string> ids = GUIMainWindow::getInstance()->getViewIDs();
        tempMsg.writeUnsignedByte(TYPE_STRINGLIST);
        tempMsg.writeStringList(ids);
    } else {
        GUISUMOAbstractView* v = getNamedView(id);
        if (v == nullptr && variable != VAR_HAS_VIEW) {
            return server.writeErrorStatusCmd(CMD_GET_GUI_VARIABLE, "View '" + id + "' is not known", outputStorage);
        }
        switch (variable) {
            case VAR_VIEW_ZOOM:
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(v->getChanger().getZoom());
                break;
            case VAR_VIEW_OFFSET:
                tempMsg.writeUnsignedByte(POSITION_2D);
                tempMsg.writeDouble(v->getChanger().getXPos());
                tempMsg.writeDouble(v->getChanger().getYPos());
                break;
            case VAR_VIEW_SCHEMA:
                tempMsg.writeUnsignedByte(TYPE_STRING);
                tempMsg.writeString(v->getVisualisationSettings()->name);
                break;
            case VAR_VIEW_BOUNDARY: {
                tempMsg.writeUnsignedByte(TYPE_POLYGON);
                Boundary b = v->getVisibleBoundary();
                tempMsg.writeByte(2);
                tempMsg.writeDouble(b.xmin());
                tempMsg.writeDouble(b.ymin());
                tempMsg.writeDouble(b.xmax());
                tempMsg.writeDouble(b.ymax());
                break;
            }
            case VAR_HAS_VIEW:
                tempMsg.writeUnsignedByte(TYPE_INTEGER);
                tempMsg.writeInt(v != nullptr ? 1 : 0);
                break;
            default:
                break;
        }
    }
    server.writeStatusCmd(CMD_GET_GUI_VARIABLE, RTYPE_OK, "", outputStorage);
    server.writeResponseWithLength(outputStorage, tempMsg);
    return true;
}


bool
TraCIServerAPI_GUI::processSet(TraCIServer& server, tcpip::Storage& inputStorage,
                               tcpip::Storage& outputStorage) {
    std::string warning = ""; // additional description for response
    // variable
    int variable = inputStorage.readUnsignedByte();
    if (variable != VAR_VIEW_ZOOM && variable != VAR_VIEW_OFFSET && variable != VAR_VIEW_SCHEMA && variable != VAR_VIEW_BOUNDARY
            && variable != VAR_SCREENSHOT && variable != VAR_TRACK_VEHICLE
       ) {
        return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Change GUI State: unsupported variable " + toHex(variable, 2) + " specified", outputStorage);
    }
    // id
    std::string id = inputStorage.readString();
    GUISUMOAbstractView* v = getNamedView(id);
    if (v == 0) {
        return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "View '" + id + "' is not known", outputStorage);
    }
    // process
    switch (variable) {
        case VAR_VIEW_ZOOM: {
            Position off, p;
            double zoom = 1;
            if (!server.readTypeCheckingDouble(inputStorage, zoom)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The zoom must be given as a double.", outputStorage);
            }
            off.set(v->getChanger().getXPos(), v->getChanger().getYPos(), v->getChanger().zoom2ZPos(zoom));
            p.set(off.x(), off.y(), 0);
            v->setViewportFromToRot(off, p, v->getChanger().getRotation());
        }
        break;
        case VAR_VIEW_OFFSET: {
            libsumo::TraCIPosition tp;
            if (!server.readTypeCheckingPosition2D(inputStorage, tp)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The view port must be given as a position.", outputStorage);
            }

            Position off, p;
            off.set(tp.x, tp.y, v->getChanger().getZPos());
            p.set(tp.x, tp.y, 0);
            v->setViewportFromToRot(off, p, v->getChanger().getRotation());
        }
        break;
        case VAR_VIEW_SCHEMA: {
            std::string schema;
            if (!server.readTypeCheckingString(inputStorage, schema)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The scheme must be specified by a string.", outputStorage);
            }
            if (!v->setColorScheme(schema)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The scheme is not known.", outputStorage);
            }
        }
        break;
        case VAR_VIEW_BOUNDARY: {
            PositionVector p;
            if (!server.readTypeCheckingPolygon(inputStorage, p)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The boundary must be specified by a bounding box.", outputStorage);
            }
            v->centerTo(Boundary(p[0].x(), p[0].y(), p[1].x(), p[1].y()));
            break;
        }
        case VAR_SCREENSHOT: {
            if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Screenshot requires a compound object.", outputStorage);
            }
            int parameterCount = inputStorage.readInt();
            if (parameterCount != 3) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Screenshot requires three values as parameter.", outputStorage);
            }
            std::string filename;
            if (!server.readTypeCheckingString(inputStorage, filename)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The first variable must be a file name.", outputStorage);
            }
            int width = 0, height = 0;
            if (!server.readTypeCheckingInt(inputStorage, width)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The second variable must be the width given as int.", outputStorage);
            }
            if (!server.readTypeCheckingInt(inputStorage, height)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The third variable must be the height given as int.", outputStorage);
            }
            // take screenshot after the current step is finished (showing the same state as sumo-gui and netstate-output)
            v->addSnapshot(MSNet::getInstance()->getCurrentTimeStep(), filename, width, height);
        }
        break;
        case VAR_TRACK_VEHICLE: {
            std::string id;
            if (!server.readTypeCheckingString(inputStorage, id)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Tracking requires a string vehicle ID.", outputStorage);
            }
            if (id == "") {
                v->stopTrack();
            } else {
                SUMOVehicle* veh = MSNet::getInstance()->getVehicleControl().getVehicle(id);
                if (veh == 0) {
                    return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Could not find vehicle '" + id + "'.", outputStorage);
                }
                if (v->getTrackedID() != static_cast<GUIVehicle*>(veh)->getGlID()) {
                    v->startTrack(static_cast<GUIVehicle*>(veh)->getGlID());
                }
            }
        }
        default:
            break;
    }
    server.writeStatusCmd(CMD_SET_GUI_VARIABLE, RTYPE_OK, warning, outputStorage);
    return true;
}


GUISUMOAbstractView*
TraCIServerAPI_GUI::getNamedView(const std::string& id) {
    GUIMainWindow* const mw = GUIMainWindow::getInstance();
    if (mw == 0) {
        return 0;
    }
    GUIGlChildWindow* const c = static_cast<GUIGlChildWindow*>(mw->getViewByID(id));
    if (c == 0) {
        return 0;
    }
    return c->getView();
}


/****************************************************************************/
