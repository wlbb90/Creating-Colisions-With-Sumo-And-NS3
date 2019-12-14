/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEAdditionalHandler.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id$
///
// Builds trigger objects for netedit
/****************************************************************************/
#ifndef GNEAdditionalHandler_h
#define GNEAdditionalHandler_h

// ===========================================================================
// included modules
// ===========================================================================

#include <config.h>

#include <string>
#include <vector>
#include <utils/common/MsgHandler.h>
#include <utils/geom/Position.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/SUMOSAXHandler.h>


// ===========================================================================
// class declarations
// ===========================================================================

class GNENet;
class GNEViewNet;
class GNEJunction;
class GNEUndoList;
class GNEEdge;
class GNELane;
class GNEAdditional;

// ===========================================================================
// class definitions
// ===========================================================================

/// @class GNEAdditionalHandler
/// @brief Builds trigger objects for GNENet (busStops, chargingStations, detectors, etc..)
class GNEAdditionalHandler : public SUMOSAXHandler {
public:
    /// @brief Constructor
    GNEAdditionalHandler(const std::string& file, GNEViewNet* viewNet, bool undoAdditionals = true, GNEAdditional* additionalParent = nullptr);

    /// @brief Destructor
    ~GNEAdditionalHandler();

    /// @name inherited from GenericSAXHandler
    /// @{
    /**@brief Called on the opening of a tag;
     * @param[in] element ID of the currently opened element
     * @param[in] attrs Attributes within the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myStartElement
     */
    void myStartElement(int element, const SUMOSAXAttributes& attrs);

    /** @brief Called when a closing tag occurs
     * @param[in] element ID of the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myEndElement
     */
    void myEndElement(int element);

    /// @}

    /// @name parsing methods
    ///
    /// These methods parse the attributes for each of the described trigger
    /// and call the according methods to build the trigger
    /// @{
    /**@brief Builds a vaporization
     * @param[in] attrs SAX-attributes which define the vaporizer
     * @param[in] tag of the additional
     * @note recheck throwing the exception
     */
    void parseAndBuildVaporizer(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Variable Speed Signal (lane speed trigger)
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     * @see buildLaneSpeedTrigger
     */
    void parseAndBuildVariableSpeedSign(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Variable Speed Signal Step
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    * @see buildLaneSpeedTrigger
    */
    void parseAndBuildVariableSpeedSignStep(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a rerouter
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildRerouter(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Rerouter Interval
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    */
    void parseAndBuildRerouterInterval(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Closing Lane reroute
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    */
    void parseAndBuildRerouterClosingLaneReroute(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Closing Reroute
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    */
    void parseAndBuildRerouterClosingReroute(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Destiny Prob Reroute
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    */
    void parseAndBuildRerouterDestProbReroute(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a parkingAreaReroute
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    */
    void parseAndBuildRerouterParkingAreaReroute(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Route Prob Reroute
    * @param[in] attrs SAX-attributes which define the trigger
    * @param[in] tag of the additional
    */
    void parseAndBuildRerouterRouteProbReroute(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a bus stop
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildBusStop(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses values and adds access to the current bus stop
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildAccess(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a container stop
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildContainerStop(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a charging station
     * @param[in] attrs SAXattributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildChargingStation(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a parking area
     * @param[in] attrs SAXattributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildParkingArea(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a parking space
     * @param[in] attrs SAXattributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildParkingSpace(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a mesoscopic or microscopic calibrator
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildCalibrator(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a induction loop detector (E1)
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildDetectorE1(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a lane area detector (E2)
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildDetectorE2(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a multi entry exit detector (E3)
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildDetectorE3(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Entry detector
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildDetectorEntry(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Exit detector
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildDetectorExit(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds a Instant induction loop detector (E1Instant)
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildDetectorE1Instant(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses his values and builds routeProbe
     * @param[in] attrs SAX-attributes which define the trigger
     * @param[in] tag of the additional
     */
    void parseAndBuildRouteProbe(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses route values of Calibrators
     * @param[in] attrs SAX-attributes which define the routes
     * @param[in] tag of the additional
     */
    void parseAndBuildCalibratorRoute(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses vehicle type values of Calibrators
     * @param[in] attrs SAX-attributes which define the vehicle types
     * @param[in] tag of the additional
     */
    void parseAndBuildCalibratorVehicleType(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parses flow values of Calibrators
     * @param[in] attrs SAX-attributes which define the flows
     * @param[in] tag of the additional
     */
    void parseAndBuildCalibratorFlow(const SUMOSAXAttributes& attrs, const SumoXMLTag& tag);

    /**@brief Parse generic parameter and insert it in the last created additional
     * @param[in] attrs SAX-attributes which define the generic parameter
     */
    void parseGenericParameter(const SUMOSAXAttributes& attrs);

    /// @}

    /// @name building methods
    ///
    /// Called with parsed values, these methods build the trigger.
    /// @{
    /**@brief Build additionals
     * @param[in] viewNet pointer to viewNet in wich additional will be created
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] tag tag of the additiona lto create
     * @param[in] values map with the attributes and values of the additional to create
     * @return true if was sucesfully created, false in other case
     */
    static GNEAdditional* buildAdditional(GNEViewNet* viewNet, bool allowUndoRedo, SumoXMLTag tag, std::map<SumoXMLAttr, std::string> values);

    /**@brief Builds a bus stop
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the bus stop
     * @param[in] lane The lane the bus stop is placed on
     * @param[in] startPos Begin position of the bus stop on the lane
     * @param[in] endPos End position of the bus stop on the lane
     * @param[in] name Name of busStop
     * @param[in] lines Names of the bus lines that halt on this bus stop
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the bus stop can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildBusStop(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, const std::string& startPos, const std::string& endPos, const std::string& name, const std::vector<std::string>& lines, bool friendlyPosition, bool blockMovement);

    /**@brief Builds an Access
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] busStop GNEAdditional of this Access belongs
     * @param[in] lane The lane the Access is placed on
     * @param[in] pos position of the Access on the lane
     * @param[in[ length length of the Access
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the detector can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildAccess(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* busStop, GNELane* lane, const std::string& pos, const std::string& length, bool friendlyPos, bool blockMovement);

    /**@brief Builds a container stop
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the container stop
     * @param[in] lane The lane the container stop is placed on
     * @param[in] startPos Begin position of the container stop on the lane
     * @param[in] endPos End position of the container stop on the lane
     * @param[in] name Name of container stop
     * @param[in] lines Names of the bus lines that halt on this container stop
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the container stop can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildContainerStop(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, const std::string& startPos, const std::string& endPos, const std::string& name,
            const std::vector<std::string>& lines, bool friendlyPosition, bool blockMovement);

    /**@brief Builds a charging Station
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the charging Station
     * @param[in] lane The lane the charging Station is placed on
     * @param[in] startPos Begin position of the charging Station on the lane
     * @param[in] endPos End position of the charging Station on the lane
     * @param[in] name Name of charging station
     * @param[in] chargingPower power charged in every timeStep
     * @param[in] efficiency efficiency of the charge
     * @param[in] chargeInTransit enable or disable charge in transit
     * @param[in] chargeDelay delay in the charge
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the charging Station can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildChargingStation(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, const std::string& startPos, const std::string& endPos, const std::string& name,
            double chargingPower, double efficiency, bool chargeInTransit, double chargeDelay, bool friendlyPosition, bool blockMovement);

    /**@brief Builds a Parking Area
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the Parking >Area
     * @param[in] lane The lane the Parking Area is placed on
     * @param[in] startPos Begin position of the Parking Area on the lane
     * @param[in] endPos End position of the Parking Area on the lane
     * @param[in] name Name of Parking Area
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] roadSideCapacity road side capacity of ParkingArea
     * @param[in] width ParkingArea's length
     * @param[in] length ParkingArea's length
     * @param[in] angle ParkingArea's angle
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the charging Station can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildParkingArea(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, const std::string& startPos, const std::string& endPos, const std::string& name,
                                           bool friendlyPosition, int roadSideCapacity, double width, const std::string& length, double angle, bool blockMovement);

    /**@brief Builds a Parking Space
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] parkingAreaParent Pointer to Parking Area Parent
     * @param[in] x ParkingSpace's X position
     * @param[in] y ParkingSpace's Y position
     * @param[in] z ParkingSpace's Z position
     * @param[in] width ParkingArea's width
     * @param[in] length ParkingArea's length
     * @param[in] angle ParkingArea's angle
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the charging Station can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildParkingSpace(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* parkingAreaParent, double x, double y, double z, double width, double length, double angle, bool blockMovement);

    /**@brief Builds a induction loop detector (E1)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the detector
     * @param[in] lane The lane the detector is placed on
     * @param[in] pos position of the detector on the lane
     * @param[in] freq the aggregation period the values the detector collects shall be summed up.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] name E2 detector name
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the detector can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildDetectorE1(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, double pos, double freq, const std::string& filename,
                                          const std::string& vehicleTypes, const std::string& name, bool friendlyPos, bool blockMovement);

    /**@brief Builds a lane Area Detector (E2)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the detector
     * @param[in] lane The lane the detector is placed on
     * @param[in] pos position of the detector on the lane
     * @param[in[ length length of the detector
     * @param[in] freq the aggregation period the values the detector collects shall be summed up.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] name E2 detector name
     * @param[in] timeThreshold The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting
     * @param[in] speedThreshold The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting
     * @param[in] jamThreshold The minimum distance to the next standing vehicle in order to make this vehicle count as a participant to the jam
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the detector can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildDetectorE2(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, double pos, double length, double freq, const std::string& filename,
                                          const std::string& vehicleTypes, const std::string& name, const double timeThreshold, double speedThreshold, double jamThreshold, bool friendlyPos, bool blockMovement);

    /**@brief Builds a multi entry exit detector (E3)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the detector
     * @param[in] pos position of the detector in the map
     * @param[in] freq the aggregation period the values the detector collects shall be summed up.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] name E2 detector name
     * @param[in] timeThreshold The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting
     * @param[in] speedThreshold The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the detector can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildDetectorE3(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, Position pos, double freq, const std::string& filename, const std::string& vehicleTypes, const std::string& name,  const double timeThreshold, double speedThreshold, bool blockMovement);

    /**@brief Builds a entry detector (E3)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] E3Parent pointer to E3 detector parent
     * @param[in] lane The lane in which the entry detector is placed on
     * @param[in] pos position of the entry detector on the lane
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the entry detector can not be added to the net (invalid parent or lane)
     */
    static GNEAdditional* buildDetectorEntry(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* E3Parent, GNELane* lane, double pos, bool friendlyPos, bool blockMovement);

    /**@brief Builds a exit detector (E3)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] E3Parent pointer to E3 detector parent
     * @param[in] lane The lane in which the exit detector is placed on
     * @param[in] pos position of the exit detector on the lane
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the exit detector can not be added to the net (invalid parent or lane
     */
    static GNEAdditional* buildDetectorExit(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* E3Parent, GNELane* lane, double pos, bool friendlyPos, bool blockMovement);

    /**@brief Builds a Instant Induction Loop Detector (E1Instant)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the detector
     * @param[in] lane The lane the detector is placed on
     * @param[in] pos position of the detector on the lane
     * @param[in] filename The path to the output file.
     * @param[in] name E2 detector name
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the detector can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildDetectorE1Instant(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, double pos, const std::string& filename, const std::string& vehicleTypes, const std::string& name, bool friendlyPos, bool blockMovement);

    /**@brief builds a microscopic calibrator over a lane
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the calibrator
     * @param[in] lane The lane the calibrator is placed at
     * @param[in] pos The position on the edge the calibrator lies at
     * @param[in] name Calibrator name
     * @param[in] outfile te file in which write results
     * @return true if was sucesfully created, false in other case
     * @todo Is the position correct/needed
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the entry detector can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildCalibrator(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNELane* lane, double pos, const std::string& name, const std::string& outfile, double freq);

    /**@brief builds a microscopic calibrator over an edge
    * @param[in] viewNet viewNet in which element will be inserted
    * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
    * @param[in] id The id of the calibrator
    * @param[in] edge The edge the calibrator is placed at
    * @param[in] pos The position on the edge the calibrator lies at
    * @param[in] name Calibrator name
    * @param[in] outfile te file in which write results
    * @return true if was sucesfully created, false in other case
    * @todo Is the position correct/needed
    * @return true if was sucesfully created, false in other case
    * @exception InvalidArgument If the entry detector can not be added to the net (is duplicate)
    */
    static GNEAdditional* buildCalibrator(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNEEdge* edge, double pos, const std::string& name, const std::string& outfile, double freq);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* buildCalibratorRoute(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& routeID, const std::vector<GNEEdge*>& edges, const RGBColor& color);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* buildVehicleType(GNEViewNet* viewNet, bool allowUndoRedo, std::string vehicleTypeID,
                                           double accel, double decel, double sigma, double tau, double length, double minGap, double maxSpeed,
                                           double speedFactor, double speedDev, const RGBColor& color, SUMOVehicleClass vClass, const std::string& emissionClass,
                                           SUMOVehicleShape shape, double width, const std::string& filename, double impatience, const std::string& laneChangeModel,
                                           const std::string& carFollowModel, int personCapacity, int containerCapacity, double boardingDuration,
                                           double loadingDuration, const std::string& latAlignment, double minGapLat, double maxSpeedLat);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* buildCalibratorFlow(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* calibratorParent, GNEAdditional* route, GNEAdditional* vtype,
            const std::string& vehsPerHour, const std::string& speed, const RGBColor& color, const std::string& departLane, const std::string& departPos,
            const std::string& departSpeed, const std::string& arrivalLane, const std::string& arrivalPos, const std::string& arrivalSpeed,
            const std::string& line, int personNumber, int containerNumber, bool reroute, const std::string& departPosLat,
            const std::string& arrivalPosLat, double begin, double end);

    /**@brief builds a rerouter
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the rerouter
     * @param[in] pos position of the rerouter in the map
     * @param[in] edges The edges the rerouter is placed at
     * @param[in] prob The probability the rerouter reoutes vehicles with
     * @param[in] name Calibrator name
     * @param[in] file The file to read the reroute definitions from
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     */
    static GNEAdditional* buildRerouter(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, Position pos, const std::vector<GNEEdge*>& edges, double prob, const std::string& name, const std::string& file, bool off, double timeThreshold, const std::string& vTypes, bool blockMovement);

    /**@brief builds a rerouter interval
    * @param[in] viewNet viewNet in which element will be inserted
    * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
    * @param[in] rerouterParent rerouter in which interval is placed
    * @param[in] begin begin of interval
    * @param[in] end end of interval
    * @return true if was sucesfully created, false in other case
    */
    static GNEAdditional* buildRerouterInterval(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* rerouterParent, double begin, double end);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* buildClosingLaneReroute(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* rerouterIntervalParent, GNELane* closedLane, SVCPermissions permissions);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* buildClosingReroute(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* rerouterIntervalParent, GNEEdge* closedEdge, SVCPermissions permissions);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* builDestProbReroute(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* rerouterIntervalParent, GNEEdge* newEdgeDestination, double probability);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* builParkingAreaReroute(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* rerouterIntervalParent, GNEAdditional* newParkignArea, double probability, bool visible);

    /**
    DOCUMENTAR
    */
    static GNEAdditional* buildRouteProbReroute(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* rerouterIntervalParent, const std::string& newRouteId, double probability);

    /**@brief builds a Route probe
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the routeprobe
     * @param[in] edge The edges the routeprobe is placed at
     * @param[in] freq the aggregation period the values the routeprobe collects shall be summed up.
     * @param[in] name Calibrator name
     * @param[in] file The file to read the routeprobe definitions from
     * @param[in] begin The time at which to start generating output
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the Route Probe can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildRouteProbe(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, GNEEdge* edge, const std::string& freq, const std::string& name, const std::string& file, double begin);

    /**@brief Builds a VariableSpeedSign (lane speed trigger)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] id The id of the lane speed trigger
     * @param[in] destLanes List of lanes affected by this speed trigger
     * @param[in] name Calibrator name
     * @param[in] blockMovemet enable or disable block movement
     * @return true if was sucesfully created, false in other case
     * @exception InvalidArgument If the VariableSpeedSign can not be added to the net (is duplicate)
     */
    static GNEAdditional* buildVariableSpeedSign(GNEViewNet* viewNet, bool allowUndoRedo, const std::string& id, Position pos, const std::vector<GNELane*>& destLanes, const std::string& name, bool blockMovement);

    /**@brief Builds a VariableSpeedSign Step
    * @param[in] viewNet viewNet in which element will be inserted
    * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
    * @param[in] VSSParent Variable Speed Sign parent
    * @param[in] time step's time
    * @param[in] speed step's speed
    * @return true if was sucesfully created, false in other case
    * @exception InvalidArgument If the Variable Speed Sign Step can not be added to the net (is duplicate)
    */
    static GNEAdditional* buildVariableSpeedSignStep(GNEViewNet* viewNet, bool allowUndoRedo, GNEAdditional* VSSParent, double time, double speed);

    /**@brief Builds a vaporizer (lane speed trigger)
     * @param[in] viewNet viewNet in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] edge edge in which this vaporizer is placed
     * @param[in] startTime time in which this vaporizer start
     * @param[in] end time in which this vaporizer ends
     * @param[in] name Vaporizer name
     * @return true if was sucesfully created, false in other case
     * @exception ProcessError If the XML definition file is errornous
     */
    static GNEAdditional* buildVaporizer(GNEViewNet* viewNet, bool allowUndoRedo, GNEEdge* edge, double startTime, double end, const std::string& name);

    /**@brief Helper method to obtain the filename
     * @param[in] attrs The attributes to obtain the file name from
     * @param[in] base The base path (the path the loaded additional file lies in)
     * @return The (expanded) path to the named file
     * @todo Recheck usage of the helper class
     */
    std::string getFileName(const SUMOSAXAttributes& attrs, const std::string& base, const bool allowEmpty = false);

    /**@brief extracts the position, checks whether it shall be mirrored and checks whether it is within the lane.
     * @param[in] pos position of additional over lane
     * @param[in] lane The lane the position shall be valid for
     * @param[in] friendlyPos flag to indicate if friendlyPos is enabled
     * @param[in] additionalID ID of additional
     * @return The position on the lane
     */
    double getPosition(double pos, GNELane& lane, bool friendlyPos, const std::string& additionalID);

    /**@brief check if the position of an stoppingPlace over a lane is valid
    * @param[in] startPos Start position of stoppingPlace
    * @param[in] endPos End position of stoppingPlace
    * @param[in] laneLength Length of the lane
    * @param[in] minLength Min length of the stoppingPlace
    * @param[in] friendlyPos Attribute of stoppingPlace
    * @return true if the stoppingPlace position is valid, false in otherweise
    */
    static bool fixStoppinPlacePosition(std::string& startPos, std::string& endPos, const double laneLength, const double minLength, const bool friendlyPos);

    /**@brief check if the position of a detector over a lane is valid
    * @param[in] pos pos position of detector
    * @param[in] laneLength Length of the lane
    * @param[in] friendlyPos Attribute of detector
    * @return true if the detector position is valid, false in otherweise
    */
    static bool checkAndFixDetectorPositionPosition(double& pos, const double laneLength, const bool friendlyPos);

    /**@brief check if the position of a detector over a lane is valid
    * @param[in] startPos Start position of detector
    * @param[in] length length of detector
    * @param[in] laneLength Length of the lane
    * @param[in] friendlyPos Attribute of detector
    * @return true if the detector position is valid, false in otherweise
    */
    static bool fixE2DetectorPositionPosition(double& pos, double& length, const double laneLength, const bool friendlyPos);

    /// @brief check if a GNEAccess can be created in a certain Edge
    static bool accessCanBeCreated(GNEAdditional* busStopParent, GNEEdge& edge);

    /// @brief check if an overlapping is produced in rerouter if a interval with certain begin and end is inserted
    static bool checkOverlappingRerouterIntervals(GNEAdditional* rerouter, double newBegin, double newEnd);

private:
    /// @brief Stack used to save the last inserted element
    struct HierarchyInsertedElements {

        /// @brief insert new element (called only in function myStartElement)
        void insertElement(SumoXMLTag tag);

        /// @brief commit element insertion (used to save ID of last correct inserted element)
        void commitElementInsertion(const std::string& id);

        /// @brief pop last inserted element (used only in function myEndElement)
        void popElement();

        /// @brief retrieve additional parent correspond to current status of myInsertedElements
        GNEAdditional* retrieveAdditionalParent(GNEViewNet* viewNet, SumoXMLTag expectedTag) const;

    private:
        /// @brief vector used as stack
        std::vector<std::pair<SumoXMLTag, std::string> > myInsertedElements;
    };

    /// @brief pointer to View's Net
    GNEViewNet* myViewNet;

    /// @brief flag to check if created additionals must be undo and redo
    bool myUndoAdditionals;

    /// @brief pointer to parent additional (used for loading additional childs placed in a different XML)
    GNEAdditional* myAdditionalParent;

    /// @brief pointer to last inserted additional (used for generic parameters)
    GNEAdditional* myLastInsertedAdditional;

    /// @brief HierarchyInsertedElements used for insert childs
    HierarchyInsertedElements myParentElements;
};


#endif
