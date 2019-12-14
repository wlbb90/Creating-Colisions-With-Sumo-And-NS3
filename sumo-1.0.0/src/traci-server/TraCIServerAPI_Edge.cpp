/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    TraCIServerAPI_Edge.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Jerome Haerri
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @author  Mario Krumnow
/// @author  Gregor Laemmel
/// @date    Sept 2002
/// @version $Id$
///
// APIs for getting/setting edge values via TraCI
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/StdDefs.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/pedestrians/MSPerson.h>
#include "TraCIConstants.h"
#include "TraCIServerAPI_Edge.h"
#include <microsim/MSEdgeWeightsStorage.h>
#include <utils/emissions/HelpersHarmonoise.h>
#include <libsumo/Edge.h>


// ===========================================================================
// method definitions
// ===========================================================================
bool
TraCIServerAPI_Edge::processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                                tcpip::Storage& outputStorage) {
    const int variable = inputStorage.readUnsignedByte();
    const std::string id = inputStorage.readString();
    server.initWrapper(RESPONSE_GET_EDGE_VARIABLE, variable, id);
    try {
        if (!libsumo::Edge::handleVariable(id, variable, &server)) {
            switch (variable) {
                case VAR_EDGE_TRAVELTIME: {
                    double time = 0.;
                    if (!server.readTypeCheckingDouble(inputStorage, time)) {
                        return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE,
                                                          "The message must contain the time definition.", outputStorage);
                    }
                    server.getWrapperStorage().writeUnsignedByte(TYPE_DOUBLE);
                    server.getWrapperStorage().writeDouble(libsumo::Edge::getAdaptedTraveltime(id, time));
                    break;
                }
                case VAR_EDGE_EFFORT: {
                    double time = 0.;
                    if (!server.readTypeCheckingDouble(inputStorage, time)) {
                        return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE,
                                                          "The message must contain the time definition.", outputStorage);
                    }
                    server.getWrapperStorage().writeUnsignedByte(TYPE_DOUBLE);
                    server.getWrapperStorage().writeDouble(libsumo::Edge::getEffort(id, time));
                    break;
                }
                case VAR_PARAMETER: {
                    std::string paramName;
                    if (!server.readTypeCheckingString(inputStorage, paramName)) {
                        return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE,
                                                          "Retrieval of a parameter requires its name.",
                                                          outputStorage);
                    }
                    server.getWrapperStorage().writeUnsignedByte(TYPE_STRING);
                    server.getWrapperStorage().writeString(libsumo::Edge::getParameter(id, paramName));
                    break;
                }
                default:
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE,
                                                      "Get Edge Variable: unsupported variable " + toHex(variable, 2)
                                                      + " specified", outputStorage);
            }
        }
    } catch (libsumo::TraCIException& e) {
        return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, e.what(), outputStorage);
    }
    server.writeStatusCmd(CMD_GET_EDGE_VARIABLE, RTYPE_OK, "", outputStorage);
    server.writeResponseWithLength(outputStorage, server.getWrapperStorage());
    return true;
}


bool
TraCIServerAPI_Edge::processSet(TraCIServer& server, tcpip::Storage& inputStorage,
                                tcpip::Storage& outputStorage) {
    std::string warning; // additional description for response
    // variable
    int variable = inputStorage.readUnsignedByte();
    if (variable != VAR_EDGE_TRAVELTIME && variable != VAR_EDGE_EFFORT && variable != VAR_MAXSPEED
            && variable != VAR_PARAMETER) {
        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                          "Change Edge State: unsupported variable " + toHex(variable, 2)
                                          + " specified", outputStorage);
    }
    // id
    std::string id = inputStorage.readString();
    try {
        // process
        switch (variable) {
            case LANE_ALLOWED: {
                // read and set allowed vehicle classes
                std::vector<std::string> classes;
                if (!server.readTypeCheckingStringList(inputStorage, classes)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "Allowed vehicle classes must be given as a list of strings.",
                                                      outputStorage);
                }
                libsumo::Edge::setAllowedVehicleClasses(id, classes);
                break;
            }
            case LANE_DISALLOWED: {
                // read and set disallowed vehicle classes
                std::vector<std::string> classes;
                if (!server.readTypeCheckingStringList(inputStorage, classes)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "Not allowed vehicle classes must be given as a list of strings.",
                                                      outputStorage);
                }
                libsumo::Edge::setDisallowedVehicleClasses(id, classes);
                break;
            }
            case VAR_EDGE_TRAVELTIME: {
                // read and set travel time
                if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "Setting travel time requires a compound object.", outputStorage);
                }
                const int parameterCount = inputStorage.readInt();
                if (parameterCount == 3) {
                    // bound by time
                    double begTime = 0., endTime = 0., value = 0.;
                    if (!server.readTypeCheckingDouble(inputStorage, begTime)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The first variable must be the begin time given as int.",
                                                          outputStorage);
                    }
                    if (!server.readTypeCheckingDouble(inputStorage, endTime)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The second variable must be the end time given as int.",
                                                          outputStorage);
                    }
                    if (!server.readTypeCheckingDouble(inputStorage, value)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The third variable must be the value given as double",
                                                          outputStorage);
                    }
                    libsumo::Edge::adaptTraveltime(id, value, begTime, endTime);
                } else if (parameterCount == 1) {
                    // unbound
                    double value = 0;
                    if (!server.readTypeCheckingDouble(inputStorage, value)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The variable must be the value given as double", outputStorage);
                    }
                    libsumo::Edge::adaptTraveltime(id, value, 0., std::numeric_limits<double>::max());
                } else {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "Setting travel time requires either begin time, end time, and value, or only value as parameter.",
                                                      outputStorage);
                }
                break;
            }
            case VAR_EDGE_EFFORT: {
                // read and set effort
                if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "Setting effort requires a compound object.",
                                                      outputStorage);
                }
                const int parameterCount = inputStorage.readInt();
                if (parameterCount == 3) {
                    // bound by time
                    double begTime = 0., endTime = 0., value = 0.;
                    if (!server.readTypeCheckingDouble(inputStorage, begTime)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The first variable must be the begin time given as int.",
                                                          outputStorage);
                    }
                    if (!server.readTypeCheckingDouble(inputStorage, endTime)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The second variable must be the end time given as int.",
                                                          outputStorage);
                    }
                    if (!server.readTypeCheckingDouble(inputStorage, value)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The third variable must be the value given as double",
                                                          outputStorage);
                    }
                    libsumo::Edge::setEffort(id, value, begTime, endTime);
                } else if (parameterCount == 1) {
                    // unbound
                    double value = 0.;
                    if (!server.readTypeCheckingDouble(inputStorage, value)) {
                        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                          "The variable must be the value given as double", outputStorage);
                    }
                    libsumo::Edge::setEffort(id, value, 0., std::numeric_limits<double>::max());
                } else {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "Setting effort requires either begin time, end time, and value, or only value as parameter.",
                                                      outputStorage);
                }
                break;
            }
            case VAR_MAXSPEED: {
                // read and set max. speed
                double value = 0.;
                if (!server.readTypeCheckingDouble(inputStorage, value)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The speed must be given as a double.",
                                                      outputStorage);
                }
                libsumo::Edge::setMaxSpeed(id, value);
                break;
            }
            case VAR_PARAMETER: {
                if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "A compound object is needed for setting a parameter.",
                                                      outputStorage);
                }
                //readt itemNo
                inputStorage.readInt();
                std::string name;
                if (!server.readTypeCheckingString(inputStorage, name)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "The name of the parameter must be given as a string.",
                                                      outputStorage);
                }
                std::string value;
                if (!server.readTypeCheckingString(inputStorage, value)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE,
                                                      "The value of the parameter must be given as a string.",
                                                      outputStorage);
                }
                libsumo::Edge::setParameter(id, name, value);
                break;
            }
            default:
                break;
        }
    } catch (libsumo::TraCIException& e) {
        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, e.what(), outputStorage);
    }
    server.writeStatusCmd(CMD_SET_EDGE_VARIABLE, RTYPE_OK, warning, outputStorage);
    return true;
}


/****************************************************************************/
