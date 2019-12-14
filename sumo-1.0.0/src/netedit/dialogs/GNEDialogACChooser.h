/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEDialogACChooser.h
/// @author  Pablo Alvarez Lopez
/// @date    Apr 2018
/// @version $Id$
///
// Class for the window that allows to choose a street, junction or vehicle
/****************************************************************************/

#ifndef GNEDialogACChooser_h
#define GNEDialogACChooser_h


// ===========================================================================
// included modules
// ===========================================================================

#include <config.h>

#include <string>
#include <vector>
#include <set>
#include <utils/gui/windows/GUIDialog_GLObjChooser.h>


// ===========================================================================
// class declarations
// ===========================================================================

class GNEAttributeCarrier;
class GNEViewParent;

// ===========================================================================
// class definition
// ===========================================================================
/**
 * @class GNEDialogACChooser
 * Instances of this class are windows that display the list of instances
 * from a given artifact like vehicles, edges or junctions and allow
 * one of their items
 */
class GNEDialogACChooser : public GUIDialog_GLObjChooser {

public:
    /** @brief Constructor
     * @param[in] viewParent GNEViewParent of Netedit
     * @param[in] icon The icon to use
     * @param[in] title The title to use
     * @param[in] ACs list of choosen ACs
     */
    GNEDialogACChooser(GNEViewParent* viewParent, FXIcon* icon, const std::string& title, const std::vector<GNEAttributeCarrier*>& ACs);

    /// @brief Destructor
    ~GNEDialogACChooser();

protected:
    /// FOX needs this
    GNEDialogACChooser() {}

    void toggleSelection(int listIndex);

private:
    /// brief get glID for every AC
    std::vector<GUIGlID> getGLIds(const std::vector<GNEAttributeCarrier*>& ACs);

    /// @brief list of displayed ACs
    std::vector<GNEAttributeCarrier*> myACs;
    GNEViewParent* myViewParent;
};


#endif

/****************************************************************************/

