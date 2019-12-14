/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Fahrtverlaufdateien.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 30 Apr 2003
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
#include "../tempstructs/NIVissimSource.h"
#include "NIVissimSingleTypeParser_Fahrtverlaufdateien.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Fahrtverlaufdateien::NIVissimSingleTypeParser_Fahrtverlaufdateien(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Fahrtverlaufdateien::~NIVissimSingleTypeParser_Fahrtverlaufdateien() {}


bool
NIVissimSingleTypeParser_Fahrtverlaufdateien::parse(std::istream&) {
    return true;
}



/****************************************************************************/

