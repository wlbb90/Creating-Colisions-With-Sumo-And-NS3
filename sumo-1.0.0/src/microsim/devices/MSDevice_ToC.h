/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSDevice_ToC.h
/// @author  Leonhard Luecken
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    01.04.2018
/// @version $Id$
///
// The ToC Device controls the transition of control between automated and manual driving.
//
/****************************************************************************/
#ifndef MSDevice_ToC_h
#define MSDevice_ToC_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "MSDevice.h"
#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;
class MSVehicle;
class Command_ToCTrigger;
class Command_ToCProcess;
class RGBColor;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_ToC
 *
 * @brief The ToC Device controls transition of control between automated and manual driving.
 * @todo: Provide logging facilities
 * @todo: allow manual and automated type to refer to vTypeDistributions
 *
 * @see MSDevice
 */
class MSDevice_ToC : public MSDevice {
public:
    /** @brief Inserts MSDevice_ToC-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a ToC-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into);

private:

    /// @brief Enum describing the different regimes for the device, @see myState
    ///        Possible transitions:
    ///        AUTOMATED -> PREPARING_TOC
    ///        PREPARING_TOC -> PERFORMING_MRM
    ///        PREPARING_TOC -> MANUAL
    ///        PERFORMING_MRM -> MANUAL
    ///        MANUAL -> AUTOMATED
    enum ToCState {
        UNDEFINED = 0,
        MANUAL = 1,
        AUTOMATED = 2,
        PREPARING_TOC = 3, // this applies only to the transition AUTOMATED -> MANUAL !
        MRM = 4,
        RECOVERING = 5
    };

    /// @name Helpers for parameter parsing
    /// @{
    static std::string getManualType(const SUMOVehicle& v, const OptionsCont& oc);
    static std::string getAutomatedType(const SUMOVehicle& v, const OptionsCont& oc);
    static double getResponseTime(const SUMOVehicle& v, const OptionsCont& oc);
    static double getRecoveryRate(const SUMOVehicle& v, const OptionsCont& oc);
    static double getInitialAwareness(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMRMDecel(const SUMOVehicle& v, const OptionsCont& oc);
    static bool useColorScheme(const SUMOVehicle& v, const OptionsCont& oc);

    static ToCState _2ToCState(const std::string&);
    static std::string _2string(ToCState state);
    /// @}


public:
    /// @brief Destructor.
    ~MSDevice_ToC();

    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "toc";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);


    /// @brief Ensure existence of DriverState for equipped vehicles
    SUMOTime ensureDriverStateExistence(SUMOTime);

    /// @brief Trigger execution of an MRM
    SUMOTime triggerMRM(SUMOTime t);

    /// @brief Trigger execution of a ToC X-->AUTOMATED ("upwards")
    SUMOTime triggerUpwardToC(SUMOTime t);

    /// @brief Trigger execution of a ToC X-->MANUAL ("downwards")
    SUMOTime triggerDownwardToC(SUMOTime t);

    /// @brief Continue the ToC preparation for one time step
    SUMOTime ToCPreparationStep(SUMOTime t);

    /// @brief Continue the MRM for one time step
    SUMOTime MRMExecutionStep(SUMOTime t);

    /// @brief Continue the awareness recovery for one time step
    SUMOTime awarenessRecoveryStep(SUMOTime t);

private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_ToC(SUMOVehicle& holder, const std::string& id,
                 std::string manualType, std::string automatedType,
                 SUMOTime responseTime, double recoveryRate, double initialAwareness,
                 double mrmDecel, bool useColorScheme);

    /** @brief Initialize vehicle colors for different states
     *  @note  For MANUAL and AUTOMATED, the color of the given types are used,
     *         and for the other states hardcoded colors are given.
     */
    void initColorScheme();

    /// @brief Set the awareness to the given value
    void setAwareness(double value);

    /// @brief Set the ToC device's state
    void setState(ToCState state);

    // @brief Sets the device holder's color corresponding to the current state
    void setVehicleColor();

    /// @brief Request a ToC.
    ///        If the device is in AUTOMATED or MRM state, a driver response time is sampled
    ///        and the ToC is scheduled. If the response is larger than timeTillMRM,
    ///        an MRM is scheduled as well.
    ///        If the device is in MANUAL or UNDEFINED state, it switches to AUTOMATED.
    ///        The request is ignored if the state is already PREPARING_TOC.
    void requestToC(SUMOTime timeTillMRM);

    /// @brief Request an MRM to be initiated immediately. No downward ToC will be scheduled.
    /// @note  The initiated MRM process will run forever until a new ToC is requested.
    void requestMRM();

    /// @brief Switch the device holder's vehicle type
    void switchHolderType(const std::string& targetTypeID);

    /// @brief Break MRM Process or remove MRM-Trigger command from the event-queue.
    void descheduleMRM();
    /// @brief Remove scheduled ToC-Trigger command from the event-queue.
    void descheduleToC();
    /// @brief Remove ongoing ToC-Preparation process from the event-queue.
    void descheduleToCPreparation();
    /// @brief Remove ongoing awareness recovery process from the event-queue.
    void descheduleRecovery();



private:
    /// @name private state members of the ToC device
    /// @{

    /// @brief vehicle type ID for manual driving
    std::string myManualTypeID;
    /// @brief vehicle type ID for automated driving
    std::string myAutomatedTypeID;

    // @brief Average response time needed by the driver to take back control
    SUMOTime myResponseTime;
    // @brief Recovery rate for the driver's awareness after a ToC
    double myRecoveryRate;
    // @brief Average awareness the driver has initially after a ToC
    double myInitialAwareness;

    /// @brief Deceleration rate applied during MRM
    double myMRMDecel;

    // @brief Current awareness-level of the driver in [0,1]
    double myCurrentAwareness;

    /// @brief Coloring scheme, @see initColorScheme()
    std::map<ToCState, RGBColor> myColorScheme;

    /// @brief Whether a coloring scheme shall by applied to indicate the different toc stages, @see initColorScheme()
    bool myUseColorScheme;

    /// @brief Current state of the device
    ToCState myState;

    /// @}

    /// @brief The holder vehicle casted to MSVehicle*
    MSVehicle* myHolderMS;

    /// @name Commands sent to the EventControl (used for cleanup)
    /// @note Must be removed in destructor.
    /// @{
    WrappingCommand<MSDevice_ToC>* myTriggerMRMCommand;
    WrappingCommand<MSDevice_ToC>* myTriggerToCCommand;
    WrappingCommand<MSDevice_ToC>* myRecoverAwarenessCommand;
    WrappingCommand<MSDevice_ToC>* myExecuteMRMCommand;
    WrappingCommand<MSDevice_ToC>* myPrepareToCCommand;
    /// @}


private:
    /// @brief Invalidated copy constructor.
    MSDevice_ToC(const MSDevice_ToC&);

    /// @brief Invalidated assignment operator.
    MSDevice_ToC& operator=(const MSDevice_ToC&);


};


#endif

/****************************************************************************/

