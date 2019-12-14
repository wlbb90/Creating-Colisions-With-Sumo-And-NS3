/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Reisezeitmessungsdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id$
///
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <iostream>
#include <utils/common/TplConvert.h>
#include "../NIImporter_Vissim.h"
#include "../tempstructs/NIVissimClosures.h"
#include "NIVissimSingleTypeParser_Reisezeitmessungsdefinition.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Reisezeitmessungsdefinition::NIVissimSingleTypeParser_Reisezeitmessungsdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Reisezeitmessungsdefinition::~NIVissimSingleTypeParser_Reisezeitmessungsdefinition() {}


bool
NIVissimSingleTypeParser_Reisezeitmessungsdefinition::parse(std::istream& from) {
    std::string tag = myRead(from);
    if (tag == "aggregationsintervall") {
        while (tag != "DATAEND") {
            tag = readEndSecure(from);
        }
        return true;
    }

    while (tag != "strecke") {
        tag = myRead(from);
    }
    tag = myRead(from);
    while (tag != "strecke") {
        tag = myRead(from);
    }
    while (tag != "DATAEND") {
        tag = readEndSecure(from);
    }
    return true;
}



/****************************************************************************/
