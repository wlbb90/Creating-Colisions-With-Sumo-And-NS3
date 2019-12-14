/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEParkingSpace.h
/// @author  Pablo Alvarez Lopez
/// @date    Feb 2018
/// @version $Id$
///
// A class for visualizing ParkingSpace geometry (adapted from GUILaneWrapper)
/****************************************************************************/
#ifndef GNEParkingSpace_h
#define GNEParkingSpace_h

// ===========================================================================
// included modules
// ===========================================================================

#include <config.h>

#include <string>
#include <vector>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/settings/GUIPropertySchemeStorage.h>
#include <utils/geom/PositionVector.h>
#include <netedit/GNEAttributeCarrier.h>
#include "GNEAdditional.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNEParkingArea;

// ===========================================================================
// class definitions
// ===========================================================================

/**
 * @class GNEParkingSpace
 * @brief vehicle space used by GNEParkingAreas
 */
class GNEParkingSpace : public GNEAdditional {

public:
    /**@brief Constructor
     * @param[in] viewNet pointer to GNEViewNet of this additional element belongs
     * @param[in] parkingAreaParent pointer to Parking Area parent
     * @param[in] x ParkingSpace's X position
     * @param[in] y ParkingSpace's Y position
     * @param[in] z ParkingSpace's Z position
     * @param[in] width ParkingArea's width
     * @param[in] length ParkingArea's length
     * @param[in] angle ParkingArea's angle
     * @param[in] block movement enable or disable additional movement
     */
    GNEParkingSpace(GNEViewNet* viewNet, GNEAdditional* parkingAreaParent, double x, double y, double z, double width, double length, double angle, bool blockMovement);

    /// @brief Destructor
    ~GNEParkingSpace();

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
    /// @brief Returns the name of the parent object
    /// @return This object's parent id
    std::string getParentName() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    /// @}

    /// @name inherited from GNEAttributeCarrier
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
     */
    void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList);

    /* @brief method for checking if the key and their correspond attribute are valids
     * @param[in] key The attribute key
     * @param[in] value The value asociated to key key
     * @return true if the value is valid, false in other case
     */
    bool isValid(SumoXMLAttr key, const std::string& value);

    /// @brief get PopPup ID (Used in AC Hierarchy)
    std::string getPopUpID() const;

    /// @brief get Hierarchy Name (Used in AC Hierarchy)
    std::string getHierarchyName() const;
    /// @}

protected:
    /// @brief x position of Parking Space
    double myX;

    /// @brief y position of Parking Space
    double myY;

    /// @brief z position of Parking Space
    double myZ;

    /// @brief width of Parking Space
    double myWidth;

    /// @brief Lenght of Parking Space
    double myLength;

    /// @brief Angle of Parking Space
    double myAngle;

private:
    /// @brief set attribute after validation
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief Invalidated copy constructor.
    GNEParkingSpace(const GNEParkingSpace&) = delete;

    /// @brief Invalidated assignment operator.
    GNEParkingSpace& operator=(const GNEParkingSpace&) = delete;
};


#endif
