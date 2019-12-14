/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEAccess.h
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2018
/// @version $Id$
///
//
/****************************************************************************/
#ifndef GNEAccess_h
#define GNEAccess_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "GNEAdditional.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNEBusStop;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEAccess
 * class for busStop acces
 */
class GNEAccess : public GNEAdditional {

public:
    /**@brief Constructor
     * @param[in] id The storage of gl-ids to get the one for this lane representation from
     * @param[in] busStop GNEBusStop of this Access belongs
     * @param[in] lane GNELane of this Access belongs
     * @param[in] viewNet pointer to GNEViewNet of this additional element belongs
     * @param[in] pos position of the Access on the lane
     * @param[in] length The length of the Access in meters.
     * @param[in] friendlyPos enable or disable friendly positions
     * @param[in] block movement enable or disable additional movement
     */
    GNEAccess(GNEAdditional* busStop, GNELane* lane, GNEViewNet* viewNet, const std::string& pos, const std::string& length, bool friendlyPos, bool blockMovement);

    /// @brief Destructor
    ~GNEAccess();

    /// @brief check if Position of Access is fixed
    bool isAccessPositionFixed() const;

    /// @brief get edge in which this Access is placed
    GNEEdge& getEdge() const;

    /// @name Functions related with geometry of element
    /// @{
    /**@brief change the position of the element geometry without saving in undoList
    * @param[in] oldPos position before start movement
    * @param[in] offset movement offset regardings to oldPos
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
    Position getPositionInView() const ;
    /// @}

    /// @name inherited from GUIGlObject
    /// @{
    /// @brief Returns the name (ID) of the parent object
    std::string getParentName() const;

    /// @{
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
    /// @brief lane in which this Access is placed
    GNELane* myLane;

    /// @brief position over lane
    std::string myPositionOverLane;

    /// @brief Acces lenght
    std::string myLength;

    /// @brief flag to check if friendly position is enabled
    bool myFriendlyPosition;

private:
    /// @brief set attribute after validation
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief Invalidated copy constructor.
    GNEAccess(const GNEAccess&) = delete;

    /// @brief Invalidated assignment operator.
    GNEAccess& operator=(const GNEAccess&) = delete;
};

#endif
/****************************************************************************/
