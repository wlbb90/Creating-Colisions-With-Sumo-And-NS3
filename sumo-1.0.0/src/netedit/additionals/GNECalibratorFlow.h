/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNECalibratorFlow.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id$
///
// Flow used by GNECalibrators
/****************************************************************************/
#ifndef GNECalibratorFlow_h
#define GNECalibratorFlow_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/UtilExceptions.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/RGBColor.h>

#include "GNEAdditional.h"

// ===========================================================================
// class declaration
// ===========================================================================

class GNECalibrator;
class GNECalibratorDialog;
class GNECalibratorVehicleType;
class GNECalibratorRoute;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNECalibratorFlow
 * flow flow used by GNECalibrators
 */
class GNECalibratorFlow : public GNEAdditional {

public:
    /// @brief default constructor (used only in GNECalibratorDialog)
    GNECalibratorFlow(GNEAdditional* calibratorParent);

    /// @brief parameter constructor
    GNECalibratorFlow(GNEAdditional* calibratorParent, GNEAdditional* vehicleType, GNEAdditional* route, const std::string& vehsPerHour, const std::string& speed,
                      const RGBColor& color, const std::string& departLane, const std::string& departPos, const std::string& departSpeed, const std::string& arrivalLane,
                      const std::string& arrivalPos, const std::string& arrivalSpeed, const std::string& line, int personNumber, int containerNumber, bool reroute,
                      const std::string& departPosLat, const std::string& arrivalPosLat, double begin, double end);

    /// @brief destructor
    ~GNECalibratorFlow();

    /// @name Functions related with geometry of element
    /// @{
    /**@brief change the position of the element geometry without saving in undoList
     * @param[in] newPosition new position of geometry
     * @note should't be called in drawGL(...) functions to avoid smoothness issues
     */
    void moveGeometry(const Position& oldPos, const Position& offset);

    /**@brief commit geometry changes in the attributes of an element after use of moveGeometry(...)
     * @param[in] oldPos the old position of additional
     * @param[in] undoList The undoList on which to register changes
     */
    void commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList);

    /// @brief update pre-computed geometry information
    void updateGeometry(bool updateGrid);

    /// @brief Returns position of additional in view
    Position getPositionInView() const;
    /// @}

    /// @name inherited from GUIGlObject
    /// @{
    /**@brief Returns the name of the parent object
     * @return This object's parent id
     */
    std::string getParentName() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    /// @}

    /// @brief inherited from GNEAttributeCarrier
    /// @{
    /* @brief method for getting the Attribute of an XML key
    * @param[in] key The attribute key
    * @return string with the value associated to key
    */
    std::string getAttribute(SumoXMLAttr key) const;

    /* @brief method for setting the attribute and letting the object perform additional changes
    * @param[in] key The attribute key
    * @param[in] value The new value
    * @param[in] undoList The undoList on which to register changes
    * @param[in] net optionally the GNENet to inform about gui updates
    */
    void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList);

    /* @brief method for setting the attribute and letting the object perform additional changes
    * @param[in] key The attribute key
    * @param[in] value The new value
    * @param[in] undoList The undoList on which to register changes
    */
    bool isValid(SumoXMLAttr key, const std::string& value);

    /// @brief get PopPup ID (Used in AC Hierarchy)
    std::string getPopUpID() const;

    /// @brief get Hierarchy Name (Used in AC Hierarchy)
    std::string getHierarchyName() const;
    /// @}

protected:
    /// @brief type of flow
    GNEAdditional* myVehicleType;

    /// @brief route in which this flow is used
    GNEAdditional* myRoute;

    /// @brief flows per hour (String instead float because can be empty)
    std::string myVehsPerHour;

    /// @brief flow speed (String instead float because can be empty)
    std::string mySpeed;

    /// @brief color of flow
    RGBColor myColor;

    /// @brief depart lane
    std::string myDepartLane;

    /// @brief depart position
    std::string myDepartPos;

    /// @brief depart speed
    std::string myDepartSpeed;

    /// @brief arrival lane
    std::string myArrivalLane;

    /// @brief arrival pos
    std::string myArrivalPos;

    /// @brief arrival speed
    std::string myArrivalSpeed;

    /// @brief line of bus/container stop
    std::string myLine;

    /// @brief number of person
    int myPersonNumber;

    /// @brief number of container
    int myContainerNumber;

    /// @brief reroute
    bool myReroute;

    /// @brief departPosLat
    std::string myDepartPosLat;

    //// @brief arrivalPosLat
    std::string myArrivalPosLat;

    /// @name specific of flows
    /// @{
    /// @brief time step begin
    double myBegin;

    /// @brief time step end
    double myEnd;
    /// @}

private:
    /// @brief method for setting the attribute and nothing else
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief Invalidated copy constructor.
    GNECalibratorFlow(const GNECalibratorFlow&) = delete;

    /// @brief Invalidated assignment operator
    GNECalibratorFlow& operator=(const GNECalibratorFlow&) = delete;
};

#endif
/****************************************************************************/
