/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2012-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    POI.h
/// @author  Daniel Krajzewicz
/// @author  Mario Krumnow
/// @author  Michael Behrisch
/// @author  Robert Hilbrich
/// @date    30.05.2012
/// @version $Id$
///
// C++ TraCI client API implementation
/****************************************************************************/
#ifndef POI_h
#define POI_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <vector>
#include <libsumo/TraCIDefs.h>


// ===========================================================================
// class declarations
// ===========================================================================
class NamedRTree;
class PointOfInterest;
class PositionVector;
namespace libsumo {
class VariableWrapper;
}


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class POI
 * @brief C++ TraCI client API implementation
 */
namespace libsumo {
class POI {
public:
    static std::vector<std::string> getIDList();
    static int getIDCount();
    static std::string getType(const std::string& poiID);
    static TraCIPosition getPosition(const std::string& poiID, const bool includeZ = false);
    static TraCIColor getColor(const std::string& poiID);
    static std::string getParameter(const std::string& poiID, const std::string& param);

    static void setType(const std::string& poiID, const std::string& setType);
    static void setColor(const std::string& poiID, const TraCIColor& c);
    static void setPosition(const std::string& poiID, double x, double y);
    static bool add(const std::string& poiID, double x, double y, const TraCIColor& color, const std::string& poiType = "", int layer = 0);
    static bool remove(const std::string& poiID, int layer = 0);

    static void setParameter(const std::string& poiID, const std::string& param, const std::string& value);

    LIBSUMO_SUBSCRIPTION_API

    /** @brief Returns a tree filled with PoI instances
     *  @return The rtree of PoIs
     */
    static NamedRTree* getTree();

    /** @brief Saves the shape of the requested object in the given container
    *  @param id The id of the poi to retrieve
    *  @param shape The container to fill
    */
    static void storeShape(const std::string& id, PositionVector& shape);

    static std::shared_ptr<VariableWrapper> makeWrapper();

    static bool handleVariable(const std::string& objID, const int variable, VariableWrapper* wrapper);

private:
    static PointOfInterest* getPoI(const std::string& id);

private:
    static SubscriptionResults mySubscriptionResults;
    static ContextSubscriptionResults myContextSubscriptionResults;

    /// @brief invalidated standard constructor
    POI() = delete;
};


}


#endif

/****************************************************************************/
