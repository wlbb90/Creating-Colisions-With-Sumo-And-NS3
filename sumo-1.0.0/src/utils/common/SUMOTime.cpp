/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    SUMOTime.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Fri, 29.04.2005
/// @version $Id$
///
// Variables, methods, and tools for internal time representation
/****************************************************************************/
// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <sstream>
#include <iostream>
#include "SUMOTime.h"
#include "TplConvert.h"
#include "StringTokenizer.h"
#include "StdDefs.h"


// ===========================================================================
// type definitions
// ===========================================================================
SUMOTime DELTA_T = 1000;


// ===========================================================================
// method definitions
// ===========================================================================
SUMOTime
string2time(const std::string& r) {
    if (r.find(":") == std::string::npos) {
        double time;
        std::istringstream buf(r);
        buf >> time;
        if (buf.fail() || time > STEPS2TIME(SUMOTime_MAX)) {
            throw ProcessError("Input string '" + r + "' is not a valid number or exceeds the time value range.");
        }
        return TIME2STEPS(time);
    } else {
        // try to parse jj:hh:mm:ss.s
        std::vector<std::string> hrt = StringTokenizer(r, ":").getVector();
        if (hrt.size() == 3) {
            //std::cout << "parsed '" << r << "' as " << (3600 * string2time(hrt[0]) + 60 * string2time(hrt[1]) + string2time(hrt[2])) << "\n";
            return 3600 * string2time(hrt[0]) + 60 * string2time(hrt[1]) + string2time(hrt[2]);
        } else if (hrt.size() == 4) {
            //std::cout << "parsed '" << r << "' as " << (24 * 3600 * string2time(hrt[0]) + 3600 * string2time(hrt[1]) + 60 * string2time(hrt[2]) + string2time(hrt[3])) << "\n";
            return 24 * 3600 * string2time(hrt[0]) + 3600 * string2time(hrt[1]) + 60 * string2time(hrt[2]) + string2time(hrt[3]);
        }
        throw ProcessError("Input string '" + r + "' is not a valid time format (jj:HH:MM:SS.S).");
    }
}


std::string
time2string(SUMOTime t) {
    std::ostringstream oss;
    oss.setf(oss.fixed);
    oss.precision(gPrecision);
    if (gHumanReadableTime) {
        // 123456 -> "00:00:12.34"
        double s = STEPS2TIME(t);
        if (s > 3600 * 24) {
            // days
            oss << (int)(s / (3600 * 24)) << ":";
            s = fmod(s, 3600 * 24);
        }
        // hours, pad with zero
        if (s / 3600 < 10) {
            oss << "0";
        }
        oss << (int)(s / 3600) << ":";
        // minutes, pad with zero
        s = fmod(s, 3600);
        if (s / 60 < 10) {
            oss << "0";
        }
        oss << (int)(s / 60) << ":";
        s = fmod(s, 60);
        if (s < 10) {
            oss << "0";
        }
        if (fmod(s, 1) == 0) {
            oss << (int)s;
        } else {
            oss << s;
        }

    } else {
        // 123456 -> "12.34"
        oss << STEPS2TIME(t);
    }
    return oss.str();
}


/****************************************************************************/

