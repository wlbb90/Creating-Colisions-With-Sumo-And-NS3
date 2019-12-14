/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEJunction.h
/// @author  Jakob Erdmann
/// @date    Feb 2011
/// @version $Id$
///
// A class for visualizing and editing junctions in netedit (adapted from
// GUIJunctionWrapper)
/****************************************************************************/
#ifndef GNEJunction_h
#define GNEJunction_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "GNENetElement.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNENet;
class GNEEdge;
class GNECrossing;
class NBTrafficLightDefinition;
class GNEConnection;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEJunction
 *
 * In the case the represented junction's shape is empty, the boundary
 *  is computed using the junction's position to which an offset of 1m to each
 *  side is added.
 */
class GNEJunction : public GNENetElement {

    /// @brief Declare friend class
    friend class GNEChange_TLS;
    friend class GNEChange_Crossing;

public:
    /// @brief constant values for drawing buubles
    static const double BUBBLE_RADIUS;

    /**@brief Constructor
     * @param[in] nbn The represented node
     * @param[in] net The net to inform about gui updates
     * @param[in] loaded Whether the junction was loaded from a file
     */
    GNEJunction(NBNode& nbn, GNENet* net, bool loaded = false);

    /// @brief Destructor
    ~GNEJunction();

    /// @name inherited from GUIGlObject
    /// @{
    /**@brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent);

    /**@brief Returns the boundary to which the view shall be centered in order to show the object
     *
     * @return The boundary the object is within
     * @see GUIGlObject::getCenteringBoundary
     */
    Boundary getCenteringBoundary() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    /// @}

    /// @brief Returns the boundary of the junction
    Boundary getBoundary() const;

    /// @brief Return net build node
    NBNode* getNBNode() const;

    /// @brief Return current position
    Position getPositionInView() const;

    /// @brief return GNEJunction neighbours
    std::vector<GNEJunction*> getJunctionNeighbours() const;

    /// @brief add incoming GNEEdge
    void addIncomingGNEEdge(GNEEdge* edge);

    /// @brief add outgoing GNEEdge
    void addOutgoingGNEEdge(GNEEdge* edge);

    /// @brief remove incoming GNEEdge
    void removeIncomingGNEEdge(GNEEdge* edge);

    /// @brief remove outgoing GNEEdge
    void removeOutgoingGNEEdge(GNEEdge* edge);

    /// @brief Returns all GNEEdges vinculated with this Junction
    const std::vector<GNEEdge*>& getGNEEdges() const;

    /// @brief Returns incoming GNEEdges
    const std::vector<GNEEdge*>& getGNEIncomingEdges() const;

    /// @brief Returns incoming GNEEdges
    const std::vector<GNEEdge*>& getGNEOutgoingEdges() const;

    /// @brief Returns GNECrossings
    const std::vector<GNECrossing*>& getGNECrossings() const;

    /// @brief Returns all GNEConnections vinculated with this junction
    std::vector<GNEConnection*> getGNEConnections() const;

    /// @brief marks as first junction in createEdge-mode
    void markAsCreateEdgeSource();

    /// @brief removes mark as first junction in createEdge-mode
    void unMarkAsCreateEdgeSource();

    /// @brief notify the junction of being selected in tls-mode. (used to control drawing)
    void selectTLS(bool selected);

    /// @brief Update the boundary of the junction
    void updateGeometry(bool updateGrid);

    /// @name functions related with geometry movement
    /// @{

    /// @brief begin movement (used when user click over edge to start a movement, to avoid problems with problems with GL Tree)
    void startGeometryMoving(bool extendToNeighbors = true);

    /// @brief begin movement (used when user click over edge to start a movement, to avoid problems with problems with GL Tree)
    void endGeometryMoving(bool extendToNeighbors = true);

    /**@brief change the position of the element geometry without saving in undoList
    * @param[in] oldPos old position before start moving
    */
    void moveGeometry(const Position& oldPos, const Position& offset);

    /// @brief registers completed movement with the undoList
    void commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList);

    /**@brief update shapes of all elements associated to the junction
     * @note this include the adyacent nodes connected by edges
     * @note if this function is called during 'Move' mode, connections will not be updated to improve efficiency
     */
    void updateShapesAndGeometries(bool updateGrid);
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
    /// @}

    /// @name Function related with Generic Parameters
    /// @{

    /// @brief add generic parameter
    bool addGenericParameter(const std::string& key, const std::string& value);

    /// @brief remove generic parameter
    bool removeGenericParameter(const std::string& key);

    /// @brief update generic parameter
    bool updateGenericParameter(const std::string& oldKey, const std::string& newKey);

    /// @brief update value generic parameter
    bool updateGenericParameterValue(const std::string& key, const std::string& newValue);

    /// @brief return generic parameters in string format
    std::string getGenericParametersStr() const;

    /// @brief return generic parameters as vector of pairs format
    std::vector<std::pair<std::string, std::string> > getGenericParameters() const;

    /// @brief set generic parameters in string format
    void setGenericParametersStr(const std::string& value);

    /// @}

    /// @brief set responsibility for deleting internal strctures
    void setResponsible(bool newVal);

    /* @brief notify junction that one of its edges has changed its shape, and
     * therefore the junction shape is no longer valid */
    void invalidateShape();

    /* @brief update validity of this junctions logic
     * if the logic is invalidated, existing connections are removed via undo-list
     * so that the previous state can be restored
     * also calls invalidateTLS
     * @param[in] valid The new validity of the junction
     * @note: this should always be called with an active command group
     */
    void setLogicValid(bool valid, GNEUndoList* undoList, const std::string& status = FEATURE_GUESSED);

    /// @brief remove all connections from the given edge
    void removeConnectionsFrom(GNEEdge* edge, GNEUndoList* undoList, bool updateTLS, int lane = -1);

    /// @brief remove all connections to the given edge
    void removeConnectionsTo(GNEEdge* edge, GNEUndoList* undoList, bool updateTLS, int lane = -1);

    /// @brief prevent re-guessing connections at this junction
    void markAsModified(GNEUndoList* undoList);

    /* @brief invalidates loaded or edited TLS
     * @param[in] deletedConnection If a valid connection is given a replacement def with this connection removed
     *   but all other information intact will be computed instead of guessing a new tlDef
     * @note: this should always be called with an active command group
     */
    void invalidateTLS(GNEUndoList* undoList,
                       const NBConnection& deletedConnection = NBConnection::InvalidConnection,
                       const NBConnection& addedConnection = NBConnection::InvalidConnection);

    /// @brief replace one edge by another in all tls connections
    void replaceIncomingConnections(GNEEdge* which, GNEEdge* by, GNEUndoList* undoList);

    /// @brief removes the given edge from all pedestrian crossings
    void removeEdgeFromCrossings(GNEEdge* edge, GNEUndoList* undoList);

    /// @brief whether this junction has a valid logic
    bool isLogicValid();

    /// @brief get GNECrossing if exist, and if not create it if create is enabled
    GNECrossing* retrieveGNECrossing(NBNode::Crossing* crossing, bool createIfNoExist = true);

    /// @brief mark connections as deprecated
    void markConnectionsDeprecated(bool includingNeighbours);

private:
    /// @brief A reference to the represented junction
    NBNode& myNBNode;

    /// @brief junction boundary
    Boundary myJunctionBoundary;

    /// @brief vector with the GNEEdges vinculated with this junction
    std::vector<GNEEdge*> myGNEEdges;

    /// @brief vector with the incomings GNEEdges vinculated with this junction
    std::vector<GNEEdge*> myGNEIncomingEdges;

    /// @brief vector with the outgoings GNEEdges vinculated with this junction
    std::vector<GNEEdge*> myGNEOutgoingEdges;

    /// @brief The maximum size (in either x-, or y-dimension) for determining whether to draw or not
    double myMaxSize;

    /// @brief whether this junction is the first junction for a newly creatededge
    /// @see GNEApplicationWindow::createEdgeSource)
    bool myAmCreateEdgeSource;

    /// @brief modification status of the junction logic (all connections across this junction)
    std::string myLogicStatus;

    /// @brief whether we are responsible for deleting myNBNode
    bool myAmResponsible;

    /// @brief whether this junctions logic is valid
    bool myHasValidLogic;

    /// @brief whether this junction is selected in tls-mode
    bool myAmTLSSelected;

    /// @brief the built crossing objects
    std::vector<GNECrossing*> myGNECrossings;

    /// @brief method for setting the attribute and nothing else (used in GNEChange_Attribute)
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief method for check if mouse is over objects
    void mouseOverObject(const GUIVisualizationSettings& s) const;

    /**@brief reposition the node at pos and informs the edges
    * @param[in] pos The new position
    * @note: those operations are not added to the undoList.
    */
    void moveJunctionGeometry(const Position& pos, bool updateGrid);

    /// @brief sets junction color depending on circumstances
    void setColor(const GUIVisualizationSettings& s, bool bubble) const;

    /// @brief determines color value
    double getColorValue(const GUIVisualizationSettings& s, bool bubble) const;

    /// @brief adds a traffic light
    void addTrafficLight(NBTrafficLightDefinition* tlDef, bool forceInsert);

    /// @brief removes a traffic light
    void removeTrafficLight(NBTrafficLightDefinition* tlDef);

    /// @brief rebuilds crossing objects for this junction
    void rebuildGNECrossings(bool rebuildNBNodeCrossings = true);

    /// @brief remove the given connections from all traffic light definitions of this junction
    void removeTLSConnections(std::vector<NBConnection>& connections, GNEUndoList* undoList);

    /// @brief Invalidated copy constructor.
    GNEJunction(const GNEJunction&) = delete;

    /// @brief Invalidated assignment operator.
    GNEJunction& operator=(const GNEJunction&) = delete;
};


#endif

/****************************************************************************/
