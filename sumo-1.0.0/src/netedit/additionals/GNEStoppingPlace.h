/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEStoppingPlace.h
/// @author  Pablo Alvarez Lopez
/// @date    Dec 2015
/// @version $Id$
///
// A abstract class to define common parameters of lane area in which vehicles can halt (GNE version)
/****************************************************************************/
#ifndef GNEStoppingPlace_h
#define GNEStoppingPlace_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "GNEAdditional.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEStoppingPlace
 * @briefA abstract class to define common parameters and functions of stopping places
 */
class GNEStoppingPlace : public GNEAdditional {

public:
    /**@brief Constructor.
     * @param[in] id Gl-id of the stopping place (Must be unique)
     * @param[in] viewNet pointer to GNEViewNet of this additional element belongs
     * @param[in] type GUIGlObjectType of stoppingPlace
     * @param[in] tag Type of xml tag that define the StoppingPlace (SUMO_TAG_BUS_STOP, SUMO_TAG_CHARGING_STATION, etc...)
     * @param[in] lane Lane of this StoppingPlace belongs
     * @param[in] startPos Start position of the StoppingPlace
     * @param[in] endPos End position of the StoppingPlace
     * @param[in] nam Name of stoppingPlace
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] block movement enable or disable additional movement
     */
    GNEStoppingPlace(const std::string& id, GNEViewNet* viewNet, GUIGlObjectType type, SumoXMLTag tag, GNELane* lane, const std::string& startPos, const std::string& endPos, const std::string& name, bool friendlyPosition, bool blockMovement);

    /// @brief Destructor
    ~GNEStoppingPlace();

    /// @brief get Lane
    GNELane* getLane() const;

    /// @brief get start Position
    double getStartPosition() const;

    /// @brief get end Position
    double getEndPosition() const;

    /// @brief check if Position of stoppingPlace are fixed
    bool areStoppingPlacesPositionsFixed() const;

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
    virtual void updateGeometry(bool updateGrid) = 0;

    /// @brief Returns position of additional in view
    Position getPositionInView() const;
    /// @}

    /// @name inherited from GNEAdditional
    /// @{
    /// @brief Returns the name of the parent object
    /// @return This object's parent id
    std::string getParentName() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    virtual void drawGL(const GUIVisualizationSettings& s) const = 0;
    /// @}

    /// @name inherited from GNEAttributeCarrier
    /// @{
    /* @brief method for getting the Attribute of an XML key
     * @param[in] key The attribute key
     * @return string with the value associated to key
     */
    virtual std::string getAttribute(SumoXMLAttr key) const = 0;

    /* @brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     */
    virtual void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) = 0;

    /* @brief method for checking if the key and their conrrespond attribute are valids
     * @param[in] key The attribute key
     * @param[in] value The value asociated to key key
     * @return true if the value is valid, false in other case
     */
    virtual bool isValid(SumoXMLAttr key, const std::string& value) = 0;

    /// @brief get PopPup ID (Used in AC Hierarchy)
    std::string getPopUpID() const;

    /// @brief get Hierarchy Name (Used in AC Hierarchy)
    std::string getHierarchyName() const;
    /// @}

protected:
    /// @brief The lane in which this lane is placed
    GNELane* myLane;

    /// @brief The relative start position this stopping place is located at (optional, if empty takes 0)
    std::string  myStartPosition;

    /// @brief The  position this stopping place is located at (optional, if empty takes the lane lenght)
    std::string myEndPosition;

    /// @brief Flag for friendly position
    bool myFriendlyPosition;

    /// @brief The position of the sign
    Position mySignPos;

    /// @brief set geometry common to all stopping places
    void setStoppingPlaceGeometry(double movingToSide);

    /// @brief circle width resolution for all stopping places
    static const double myCircleWidth;

    /// @brief squared circle width resolution for all stopping places
    static const double myCircleWidthSquared;

    /// @brief inner circle width resolution for all stopping places
    static const double myCircleInWidth;

    /// @brief text inner circle width resolution for all stopping places
    static const double myCircleInText;

private:
    /// @brief set attribute after validation
    virtual void setAttribute(SumoXMLAttr key, const std::string& value) = 0;

    /// @brief Invalidate set new position in the view
    void setPosition(const Position& pos) = delete;
};


#endif
