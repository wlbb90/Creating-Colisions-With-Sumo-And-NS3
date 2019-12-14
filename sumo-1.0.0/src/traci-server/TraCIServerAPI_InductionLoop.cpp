/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2009-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    TraCIServerAPI_InductionLoop.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    07.05.2009
/// @version $Id$
///
// APIs for getting/setting induction loop values via TraCI
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <microsim/MSNet.h>
#include <microsim/output/MSDetectorControl.h>
#include <libsumo/InductionLoop.h>
#include "TraCIConstants.h"
#include "TraCIServerAPI_InductionLoop.h"


// ===========================================================================
// method definitions
// ===========================================================================
bool
TraCIServerAPI_InductionLoop::processGet(TraCIServer& server, tcpip::Storage& inputStorage,
        tcpip::Storage& outputStorage) {
    const int variable = inputStorage.readUnsignedByte();
    const std::string id = inputStorage.readString();
    server.initWrapper(RESPONSE_GET_INDUCTIONLOOP_VARIABLE, variable, id);
    try {
        if (!libsumo::InductionLoop::handleVariable(id, variable, &server)) {
            switch (variable) {
                case LAST_STEP_VEHICLE_DATA: {
                    std::vector<libsumo::TraCIVehicleData> vd = libsumo::InductionLoop::getVehicleData(id);
                    server.getWrapperStorage().writeUnsignedByte(TYPE_COMPOUND);
                    tcpip::Storage tempContent;
                    int cnt = 0;
                    tempContent.writeUnsignedByte(TYPE_INTEGER);
                    tempContent.writeInt((int)vd.size());
                    ++cnt;
                    for (const libsumo::TraCIVehicleData& svd : vd) {
                        tempContent.writeUnsignedByte(TYPE_STRING);
                        tempContent.writeString(svd.id);
                        ++cnt;
                        tempContent.writeUnsignedByte(TYPE_DOUBLE);
                        tempContent.writeDouble(svd.length);
                        ++cnt;
                        tempContent.writeUnsignedByte(TYPE_DOUBLE);
                        tempContent.writeDouble(svd.entryTime);
                        ++cnt;
                        tempContent.writeUnsignedByte(TYPE_DOUBLE);
                        tempContent.writeDouble(svd.leaveTime);
                        ++cnt;
                        tempContent.writeUnsignedByte(TYPE_STRING);
                        tempContent.writeString(svd.typeID);
                        ++cnt;
                    }
                    server.getWrapperStorage().writeInt((int)cnt);
                    server.getWrapperStorage().writeStorage(tempContent);
                    break;
                }
                default:
                    return server.writeErrorStatusCmd(CMD_GET_INDUCTIONLOOP_VARIABLE,
                                                      "Get Induction Loop Variable: unsupported variable " + toHex(variable, 2)
                                                      + " specified", outputStorage);
            }
        }
    } catch (libsumo::TraCIException& e) {
        return server.writeErrorStatusCmd(CMD_GET_INDUCTIONLOOP_VARIABLE, e.what(), outputStorage);
    }
    server.writeStatusCmd(CMD_GET_INDUCTIONLOOP_VARIABLE, RTYPE_OK, "", outputStorage);
    server.writeResponseWithLength(outputStorage, server.getWrapperStorage());
    return true;
}


/****************************************************************************/

