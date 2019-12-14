/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NIVissimVehicleClass.cpp
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>


#include "NIVissimVehicleClass.h"

NIVissimVehicleClass::NIVissimVehicleClass(int vwish)
    : myVWish(vwish) {}


NIVissimVehicleClass::~NIVissimVehicleClass() {}


int
NIVissimVehicleClass::getSpeed() const {
    return myVWish;
}



/****************************************************************************/

