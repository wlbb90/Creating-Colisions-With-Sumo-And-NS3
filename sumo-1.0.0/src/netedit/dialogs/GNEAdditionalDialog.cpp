/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEAdditionalDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    April 2016
/// @version $Id$
///
// A abstract class for editing additional elements
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <iostream>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <netedit/additionals/GNEAdditional.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEUndoList.h>

#include "GNEAdditionalDialog.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEAdditionalDialog) GNEAdditionalDialogMap[] = {
    FXMAPFUNC(SEL_KEYPRESS,     0,                                      GNEAdditionalDialog::onKeyPress),
    FXMAPFUNC(SEL_KEYRELEASE,   0,                                      GNEAdditionalDialog::onKeyRelease),
    FXMAPFUNC(SEL_CLOSE,        0,                                      GNEAdditionalDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,      MID_GNE_ADDITIONALDIALOG_BUTTONACCEPT,  GNEAdditionalDialog::onCmdAccept),
    FXMAPFUNC(SEL_COMMAND,      MID_GNE_ADDITIONALDIALOG_BUTTONCANCEL,  GNEAdditionalDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,      MID_GNE_ADDITIONALDIALOG_BUTTONRESET,   GNEAdditionalDialog::onCmdReset),
};

// Object abstract implementation
FXIMPLEMENT_ABSTRACT(GNEAdditionalDialog, FXTopWindow, GNEAdditionalDialogMap, ARRAYNUMBER(GNEAdditionalDialogMap))

// ===========================================================================
// member method definitions
// ===========================================================================

GNEAdditionalDialog::GNEAdditionalDialog(GNEAdditional* editedAdditional, bool updatingElement, int width, int height) :
    FXTopWindow(editedAdditional->getViewNet(), ("Edit '" + editedAdditional->getID() + "' data").c_str(), editedAdditional->getIcon(), editedAdditional->getIcon(), GUIDesignDialogBoxExplicit, 0, 0, width, height, 0, 0, 0, 0, 4, 4),
    myEditedAdditional(editedAdditional),
    myUpdatingElement(updatingElement),
    myChangesDescription("change " + toString(editedAdditional->getTag()) + " values"),
    myNumberOfChanges(0) {
    // create main frame
    FXVerticalFrame* mainFrame = new FXVerticalFrame(this, GUIDesignAuxiliarFrame);
    // Create frame for contents
    myContentFrame = new FXVerticalFrame(mainFrame, GUIDesignContentsFrame);
    // create buttons centered
    FXHorizontalFrame* buttonsFrame = new FXHorizontalFrame(mainFrame, GUIDesignHorizontalFrame);
    new FXHorizontalFrame(buttonsFrame, GUIDesignAuxiliarHorizontalFrame);
    myAcceptButton = new FXButton(buttonsFrame, "accept\t\tclose accepting changes",  GUIIconSubSys::getIcon(ICON_ACCEPT), this, MID_GNE_ADDITIONALDIALOG_BUTTONACCEPT, GUIDesignButtonAccept);
    myCancelButton = new FXButton(buttonsFrame, "cancel\t\tclose discarding changes", GUIIconSubSys::getIcon(ICON_CANCEL), this, MID_GNE_ADDITIONALDIALOG_BUTTONCANCEL, GUIDesignButtonCancel);
    myResetButton = new FXButton(buttonsFrame,  "reset\t\treset to previous values",  GUIIconSubSys::getIcon(ICON_RESET),  this, MID_GNE_ADDITIONALDIALOG_BUTTONRESET,  GUIDesignButtonReset);
    new FXHorizontalFrame(buttonsFrame, GUIDesignAuxiliarHorizontalFrame);
}


GNEAdditionalDialog::~GNEAdditionalDialog() {
    // return focus to GNEViewNet to avoid minimization
    getParent()->setFocus();
}


FXint
GNEAdditionalDialog::openAsModalDialog(FXuint placement) {
    // create Dialog
    create();
    // show in the given position
    show(placement);
    // refresh APP
    getApp()->refresh();
    // open as modal dialog (will block all windows until stop() or stopModal() is called)
    return getApp()->runModalFor(this);
}


GNEAdditional*
GNEAdditionalDialog::getEditedAdditional() const {
    return myEditedAdditional;
}


long
GNEAdditionalDialog::onKeyPress(FXObject* sender, FXSelector sel, void* ptr) {
    return FXTopWindow::onKeyPress(sender, sel, ptr);
}


long
GNEAdditionalDialog::onKeyRelease(FXObject* sender, FXSelector sel, void* ptr) {
    return FXTopWindow::onKeyRelease(sender, sel, ptr);
}


void
GNEAdditionalDialog::changeAdditionalDialogHeader(const std::string& newHeader) {
    // change FXDialogBox title
    setTitle(newHeader.c_str());
}


void
GNEAdditionalDialog::initChanges() {
    // init commandGroup
    myEditedAdditional->getViewNet()->getUndoList()->p_begin(myChangesDescription);
    // save number of command group changes
    myNumberOfChanges = myEditedAdditional->getViewNet()->getUndoList()->currentCommandGroupSize();
}


void
GNEAdditionalDialog::acceptChanges() {
    // commit changes or abort last command group depending of number of changes did
    if (myNumberOfChanges < myEditedAdditional->getViewNet()->getUndoList()->currentCommandGroupSize()) {
        myEditedAdditional->getViewNet()->getUndoList()->p_end();
    } else {
        myEditedAdditional->getViewNet()->getUndoList()->p_abortLastCommandGroup();
    }
}


void
GNEAdditionalDialog::cancelChanges() {
    myEditedAdditional->getViewNet()->getUndoList()->p_abortLastCommandGroup();
}


void
GNEAdditionalDialog::resetChanges() {
    // abort last command group an start editing again
    myEditedAdditional->getViewNet()->getUndoList()->p_abortLastCommandGroup();
    myEditedAdditional->getViewNet()->getUndoList()->p_begin(myChangesDescription);
}

/****************************************************************************/
