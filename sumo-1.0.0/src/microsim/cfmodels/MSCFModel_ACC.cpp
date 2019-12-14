/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSCFModel_ACC.cpp
/// @author  Kallirroi Porfyri
/// @date    Feb 2018
/// @version $Id$
///
// ACC car-following model based on [1], [2].
// [1] Milanes, V., and S. E. Shladover. Handling Cut-In Vehicles in Strings
//    of Cooperative Adaptive Cruise Control Vehicles. Journal of Intelligent
//     Transportation Systems, Vol. 20, No. 2, 2015, pp. 178-191.
// [2] Xiao, L., M. Wang and B. van Arem. Realistic Car-Following Models for
//    Microscopic Simulation of Adaptive and Cooperative Adaptive Cruise
//     Control Vehicles. Transportation Research Record: Journal of the
//     Transportation Research Board, No. 2623, 2017. (DOI: 10.3141/2623-01).
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <stdio.h>
#include <iostream>

#include "MSCFModel_ACC.h"
#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <math.h>
#include <microsim/MSNet.h>

// ===========================================================================
// debug flags
// ===========================================================================
#define DEBUG_ACC
#define DEBUG_COND (veh->isSelected())


// ===========================================================================
// defaults
// ===========================================================================
#define DEFAULT_SC_GAIN -0.4
#define DEFAULT_GCC_GAIN_SPEED 0.8
#define DEFAULT_GCC_GAIN_SPACE 0.04
#define DEFAULT_GC_GAIN_SPEED 0.07
#define DEFAULT_GC_GAIN_SPACE 0.23
#define DEFAULT_CA_GAIN_SPACE 0.8
#define DEFAULT_CA_GAIN_SPEED 0.23

/// @todo: add attributes for myCollisionAvoidanceGainSpeed and myCollisionAvoidanceGainSpace

// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_ACC::MSCFModel_ACC(const MSVehicleType* vtype) :
    MSCFModel(vtype),
    mySpeedControlGain(vtype->getParameter().getCFParam(SUMO_ATTR_SC_GAIN, DEFAULT_SC_GAIN)),
    myGapClosingControlGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_GCC_GAIN_SPEED, DEFAULT_GCC_GAIN_SPEED)),
    myGapClosingControlGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_GCC_GAIN_SPACE, DEFAULT_GCC_GAIN_SPACE)),
    myGapControlGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_GC_GAIN_SPEED, DEFAULT_GC_GAIN_SPEED)),
    myGapControlGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_GC_GAIN_SPACE, DEFAULT_GC_GAIN_SPACE)),
    myCollisionAvoidanceGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_CA_GAIN_SPEED, DEFAULT_CA_GAIN_SPEED)),
    myCollisionAvoidanceGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_CA_GAIN_SPACE, DEFAULT_CA_GAIN_SPACE)) {
}

MSCFModel_ACC::~MSCFModel_ACC() {}


double
MSCFModel_ACC::finalizeSpeed(MSVehicle* const veh, double vPos) const {
    const double oldV = veh->getSpeed(); // save old v for optional acceleration computation
    const double vSafe = MIN2(vPos, veh->processNextStop(vPos)); // process stops
    // we need the acceleration for emission computation;
    //  in this case, we neglect dawdling, nonetheless, using
    //  vSafe does not incorporate speed reduction due to interaction
    //  on lane changing
//   const double vMin = getSpeedAfterMaxDecel(oldV);
    const double vMin = minNextSpeed(oldV); // maybe minNextSpeedEmergency() ? Why is a custom finalizeSpeed function needed anyway?
    const double vMax = MAX2(vMin, MIN3(veh->getLane()->getVehicleMaxSpeed(veh), maxNextSpeed(oldV, veh), vSafe));
#ifdef _DEBUG
    //if (vMin > vMax) {
    //    WRITE_WARNING("Maximum speed of vehicle '" + veh->getID() + "' is lower than the minimum speed (min: " + toString(vMin) + ", max: " + toString(vMax) + ").");
    //}
#endif
    return veh->getLaneChangeModel().patchSpeed(vMin, MAX2(vMin, vMax), vMax, *this);

}


double
MSCFModel_ACC::followSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double /* predMaxDecel */, const MSVehicle* const /* pred */) const {
    const double desSpeed = MIN2(veh->getLane()->getSpeedLimit(), veh->getMaxSpeed());
    return _v(veh, gap2pred, speed, predSpeed, desSpeed, true);
}


double
MSCFModel_ACC::stopSpeed(const MSVehicle* const veh, const double speed, double gap) const {
    // NOTE: This allows return of smaller values than minNextSpeed().
    // Only relevant for the ballistic update: We give the argument headway=TS, to assure that
    // the stopping position is approached with a uniform deceleration also for tau!=TS.
    return MIN2(maximumSafeStopSpeed(gap, speed, false, veh->getActionStepLengthSecs()), maxNextSpeed(speed, veh));
}


/// @todo update interactionGap logic
double
MSCFModel_ACC::interactionGap(const MSVehicle* const /* veh */, double /* vL */) const {
    /*maximum radar range is ACC is enabled*/
    return 250;
}

double MSCFModel_ACC::accelSpeedControl(double vErr) const {
    // Speed control law
    return mySpeedControlGain * vErr;
}

double MSCFModel_ACC::accelGapControl(const MSVehicle* const veh, const double gap2pred, const double speed, const double predSpeed, double vErr) const {

#ifdef DEBUG_ACC
    if DEBUG_COND {
    std::cout << "        applying gapControl" << std::endl;
}
#endif

// Gap control law
double gclAccel = 0.0;
double desSpacing = myHeadwayTime * speed;
// The argument gap2pred does not consider minGap ->  substract minGap!!
// XXX: It does! (Leo)
double gap = gap2pred - veh->getVehicleType().getMinGap();
    double spacingErr = gap - desSpacing;
    double deltaVel = predSpeed - speed;


    if (fabs(spacingErr) < 0.2 && fabs(vErr) < 0.1) {
        // gap mode
        gclAccel = myGapControlGainSpeed * deltaVel + myGapControlGainSpace * spacingErr;
    } else if (spacingErr < 0)  {
        // collision avoidance mode
        gclAccel = myCollisionAvoidanceGainSpeed * deltaVel + myCollisionAvoidanceGainSpace * spacingErr;
    } else {
        // gap closing mode
        gclAccel = myGapClosingControlGainSpeed * deltaVel + myGapClosingControlGainSpace * spacingErr;
    }

    return gclAccel;
}


double
MSCFModel_ACC::_v(const MSVehicle* const veh, const double gap2pred, const double speed,
                  const double predSpeed, const double desSpeed, const bool /* respectMinGap */) const {

    double accelACC = 0;
    double gapLimit_SC = 120; // lower gap limit in meters to enable speed control law
    double gapLimit_GC = 100; // upper gap limit in meters to enable gap control law

#ifdef DEBUG_ACC
    if DEBUG_COND {
    std::cout << SIMTIME << " MSCFModel_ACC::_v() for veh '" << veh->getID() << "'\n"
        << "        gap=" << gap2pred << " speed="  << speed << " predSpeed=" << predSpeed
        << " desSpeed=" << desSpeed << std::endl;
    }
#endif


    /* Velocity error */
    double vErr = speed - desSpeed;
    int setControlMode = 0;
    ACCVehicleVariables* vars = (ACCVehicleVariables*)veh->getCarFollowVariables();
    if (vars->lastUpdateTime != MSNet::getInstance()->getCurrentTimeStep()) {
        vars->lastUpdateTime = MSNet::getInstance()->getCurrentTimeStep();
        setControlMode = 1;
    }
    if (gap2pred > gapLimit_SC) {

#ifdef DEBUG_ACC
        if DEBUG_COND {
        std::cout << "        applying speedControl" << std::endl;
    }
#endif
    // Find acceleration - Speed control law
    accelACC = accelSpeedControl(vErr);
        // Set cl to vehicle parameters
        if (setControlMode) {
            vars->ACC_ControlMode = 0;
        }
    } else if (gap2pred < gapLimit_GC) {
        // Find acceleration - Gap control law
        accelACC = accelGapControl(veh, gap2pred, speed, predSpeed, vErr);
        // Set cl to vehicle parameters
        if (setControlMode) {
            vars->ACC_ControlMode = 1;
        }
    } else {
        // Follow previous applied law
        int cm = vars->ACC_ControlMode;
        if (!cm) {

#ifdef DEBUG_ACC
            if DEBUG_COND {
            std::cout << "        applying speedControl" << std::endl;
        }
#endif
        accelACC = accelSpeedControl(vErr);
        } else {
            accelACC = accelGapControl(veh, gap2pred, speed, predSpeed, vErr);
        }

    }

    double newSpeed = speed + ACCEL2SPEED(accelACC);

#ifdef DEBUG_ACC
    if DEBUG_COND {
    std::cout << "        result: accel=" << accelACC << " newSpeed="  << newSpeed << std::endl;
}
#endif

return MAX2(0., newSpeed);
}


MSCFModel*
MSCFModel_ACC::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_ACC(vtype);
}
