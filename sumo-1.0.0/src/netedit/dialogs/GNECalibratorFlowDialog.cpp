/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNECalibratorFlowDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    March 2017
/// @version $Id$
///
// Dialog for edit calibrator flows
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <iostream>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/common/MsgHandler.h>
#include <netedit/changes/GNEChange_Additional.h>
#include <netedit/additionals/GNECalibrator.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>
#include <netedit/additionals/GNECalibratorFlow.h>
#include <netedit/additionals/GNECalibratorRoute.h>
#include <netedit/additionals/GNECalibratorVehicleType.h>
#include <netedit/additionals/GNECalibrator.h>
#include <netedit/GNEUndoList.h>

#include "GNECalibratorFlowDialog.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNECalibratorFlowDialog) GNECalibratorFlowDialogMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONACCEPT,   GNECalibratorFlowDialog::onCmdAccept),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONCANCEL,   GNECalibratorFlowDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONRESET,    GNECalibratorFlowDialog::onCmdReset),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CALIBRATORDIALOG_SET_VARIABLE,   GNECalibratorFlowDialog::onCmdSetVariable),
};

// Object implementation
FXIMPLEMENT(GNECalibratorFlowDialog, GNEAdditionalDialog, GNECalibratorFlowDialogMap, ARRAYNUMBER(GNECalibratorFlowDialogMap))

// ===========================================================================
// member method definitions
// ===========================================================================

GNECalibratorFlowDialog::GNECalibratorFlowDialog(GNEAdditional* editedCalibratorFlow, bool updatingElement) :
    GNEAdditionalDialog(editedCalibratorFlow, updatingElement, 600, 280),
    myCalibratorFlowValid(false),
    myInvalidAttr(SUMO_ATTR_VEHSPERHOUR) {
    // change default header
    std::string typeOfOperation = updatingElement ? "Edit " + toString(myEditedAdditional->getTag()) + " of " : "Create " + toString(myEditedAdditional->getTag()) + " for ";
    changeAdditionalDialogHeader(typeOfOperation + toString(myEditedAdditional->getFirstAdditionalParent()->getTag()) + " '" + myEditedAdditional->getFirstAdditionalParent()->getID() + "'");

    // Create auxiliar frames for tables
    FXHorizontalFrame* columns = new FXHorizontalFrame(myContentFrame, GUIDesignUniformHorizontalFrame);
    FXVerticalFrame* columnLeftLabel = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnLeftValue = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnRightLabel = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnRightValue = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);

    // 1 create combobox for type
    new FXLabel(columnLeftLabel, toString(SUMO_TAG_VTYPE).c_str(), 0, GUIDesignLabelThick);
    myComboBoxVehicleType = new FXComboBox(columnLeftValue, GUIDesignComboBoxNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignComboBox);
    // 2 create combobox for route
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_ROUTE).c_str(), 0, GUIDesignLabelThick);
    myComboBoxRoute = new FXComboBox(columnLeftValue, GUIDesignComboBoxNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignComboBox);
    // 3 create textfield for vehs per hour
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_VEHSPERHOUR).c_str(), 0, GUIDesignLabelThick);
    myTextFieldVehsPerHour = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextFieldReal);
    myTextFieldVehsPerHour->setTextColor(FXRGB(255, 0, 0));
    // 4 create textfield for vehs per hour
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_SPEED).c_str(), 0, GUIDesignLabelThick);
    myTextFieldSpeed = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextFieldReal);
    myTextFieldSpeed->setTextColor(FXRGB(255, 0, 0));
    // 5 create textfield for color
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_COLOR).c_str(), 0, GUIDesignLabelThick);
    myTextFieldColor = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 6 create textfield for lane
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_DEPARTLANE).c_str(), 0, GUIDesignLabelThick);
    myTextFieldDepartLane = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 7 create textfield for pos
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_DEPARTPOS).c_str(), 0, GUIDesignLabelThick);
    myTextFieldDepartPos = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 8 create textfield for speed
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_DEPARTSPEED).c_str(), 0, GUIDesignLabelThick);
    myTextFieldDepartSpeed = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 9 create textfield for lane
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_ARRIVALLANE).c_str(), 0, GUIDesignLabelThick);
    myTextFieldArrivalLane = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 10 create textfield for arrival pos
    new FXLabel(columnLeftLabel, toString(SUMO_ATTR_ARRIVALPOS).c_str(), 0, GUIDesignLabelThick);
    myTextFieldArrivalPos = new FXTextField(columnLeftValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 11 create textfield for arrival speed
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_ARRIVALSPEED).c_str(), 0, GUIDesignLabelThick);
    myTextFieldArrivalSpeed = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 12 create textfield for arrival line
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_LINE).c_str(), 0, GUIDesignLabelThick);
    myTextFieldLine = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 13 create textfield for person number
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_PERSON_NUMBER).c_str(), 0, GUIDesignLabelThick);
    myTextFieldPersonNumber = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextFieldInt);
    // 14 create textfield for container number
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_CONTAINER_NUMBER).c_str(), 0, GUIDesignLabelThick);
    myTextFieldContainerNumber = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextFieldInt);
    // 15 create textfield for reroute
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_REROUTE).c_str(), 0, GUIDesignLabelThick);
    myRerouteCheckButton = new FXCheckButton(columnRightValue, "false", this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignCheckButtonAttribute);
    // 16 create textfield for depart pos lat
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_DEPARTPOS_LAT).c_str(), 0, GUIDesignLabelThick);
    myTextFieldDepartPosLat = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 17 create textfield for arrival pos lat
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_ARRIVALPOS_LAT).c_str(), 0, GUIDesignLabelThick);
    myTextFieldArrivalPosLat = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);
    // 18 create textfield for begin
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_BEGIN).c_str(), 0, GUIDesignLabelThick);
    myTextFieldBegin = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextFieldReal);
    // 19 create textfield for end
    new FXLabel(columnRightLabel, toString(SUMO_ATTR_END).c_str(), 0, GUIDesignLabelThick);
    myTextFieldEnd = new FXTextField(columnRightValue, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextFieldReal);

    // fill comboBox of VTypes
    for (auto i : myEditedAdditional->getViewNet()->getNet()->getAdditionalByType(SUMO_TAG_VTYPE)) {
        myComboBoxVehicleType->appendItem(i.first.c_str());
    }
    myComboBoxVehicleType->setNumVisible((int)myComboBoxVehicleType->getNumItems());

    // fill comboBox of Routes
    for (auto i : myEditedAdditional->getViewNet()->getNet()->getAdditionalByType(SUMO_TAG_ROUTE)) {
        myComboBoxRoute->appendItem(i.first.c_str());
    }
    myComboBoxRoute->setNumVisible((int)myComboBoxRoute->getNumItems());

    // update tables
    updateCalibratorFlowValues();

    // start a undo list for editing local to this additional
    initChanges();

    // add element if we aren't updating an existent element
    if (myUpdatingElement == false) {
        myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myEditedAdditional, true), true);
    }

    // open as modal dialog
    openAsModalDialog();
}


GNECalibratorFlowDialog::~GNECalibratorFlowDialog() {}


long
GNECalibratorFlowDialog::onCmdAccept(FXObject*, FXSelector, void*) {
    std::string operation1 = myUpdatingElement ? ("updating") : ("creating");
    std::string operation2 = myUpdatingElement ? ("updated") : ("created");
    std::string parentTagString = toString(myEditedAdditional->getFirstAdditionalParent()->getTag());
    std::string tagString = toString(myEditedAdditional->getTag());
    if (myCalibratorFlowValid == false) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning dialog box
        FXMessageBox::warning(getApp(), MBOX_OK,
                              ("Error " + operation1 + " " + parentTagString + "'s " + tagString).c_str(), "%s",
                              (parentTagString + "'s " + tagString + " cannot be " + operation2 +
                               " because parameter " + toString(myInvalidAttr) +
                               " is invalid.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if (!myEditedAdditional->getFirstAdditionalParent()->checkAdditionalChildsOverlapping()) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning dialog box
        FXMessageBox::warning(getApp(), MBOX_OK,
                              ("Error " + operation1 + " " + parentTagString + "'s " + tagString).c_str(), "%s",
                              (parentTagString + "'s " + tagString + " cannot be " + operation2 +
                               " because there is overlapping with another " + tagString + ".").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else {
        // accept changes before closing dialog
        acceptChanges();
        // stop dialgo sucesfully
        getApp()->stopModal(this, TRUE);
        return 1;
    }
}


long
GNECalibratorFlowDialog::onCmdCancel(FXObject*, FXSelector, void*) {
    // cancel changes
    cancelChanges();
    // Stop Modal
    getApp()->stopModal(this, FALSE);
    return 1;
}


long
GNECalibratorFlowDialog::onCmdReset(FXObject*, FXSelector, void*) {
    // reset changes
    resetChanges();
    // update tables
    updateCalibratorFlowValues();
    return 1;
}


long
GNECalibratorFlowDialog::onCmdSetVariable(FXObject*, FXSelector, void*) {
    // At start we assumed, that all values are valid
    myCalibratorFlowValid = true;
    myInvalidAttr = SUMO_ATTR_NOTHING;
    // get pointer to undo list (Only for code legilibity)
    GNEUndoList* undoList = myEditedAdditional->getViewNet()->getUndoList();
    // set color of myComboBoxVehicleType, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_TYPE, myComboBoxVehicleType->getText().text())) {
        myComboBoxVehicleType->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_TYPE, myComboBoxVehicleType->getText().text(), undoList);
    } else {
        myComboBoxVehicleType->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_TYPE;
    }
    // set color of myComboBoxRoute, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_ROUTE, myComboBoxRoute->getText().text())) {
        myComboBoxRoute->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_ROUTE, myComboBoxRoute->getText().text(), undoList);
    } else {
        myComboBoxRoute->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_ROUTE;
    }
    // set color of myTextFieldVehsPerHour, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_VEHSPERHOUR, myTextFieldVehsPerHour->getText().text())) {
        myTextFieldVehsPerHour->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_VEHSPERHOUR, myTextFieldVehsPerHour->getText().text(), undoList);
    } else {
        myTextFieldVehsPerHour->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_VEHSPERHOUR;
    }
    // set color of myTextFieldSpeed, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_SPEED, myTextFieldSpeed->getText().text())) {
        myTextFieldSpeed->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_SPEED, myTextFieldSpeed->getText().text(), undoList);
        // Check VehsPerHour again
        if (myEditedAdditional->isValid(SUMO_ATTR_VEHSPERHOUR, myTextFieldVehsPerHour->getText().text())) {
            myTextFieldVehsPerHour->setTextColor(FXRGB(0, 0, 0));
            myEditedAdditional->setAttribute(SUMO_ATTR_VEHSPERHOUR, myTextFieldVehsPerHour->getText().text(), undoList);
            if (myInvalidAttr == SUMO_ATTR_VEHSPERHOUR) {
                myCalibratorFlowValid = true;
                myInvalidAttr = SUMO_ATTR_NOTHING;
            }
        }
    } else {
        myTextFieldSpeed->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_SPEED;
    }
    // set color of myTextFieldColor, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_COLOR, myTextFieldColor->getText().text())) {
        myTextFieldColor->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_COLOR, myTextFieldColor->getText().text(), undoList);
    } else {
        myTextFieldColor->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_COLOR;
    }
    // set color of myTextFieldDepartLane, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_DEPARTLANE, myTextFieldDepartLane->getText().text())) {
        myTextFieldDepartLane->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_DEPARTLANE, myTextFieldDepartLane->getText().text(), undoList);
    } else {
        myTextFieldDepartLane->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_DEPARTLANE;
    }
    // set color of myTextFieldDepartPos, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_DEPARTPOS, myTextFieldDepartPos->getText().text())) {
        myTextFieldDepartPos->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_DEPARTPOS, myTextFieldDepartPos->getText().text(), undoList);
    } else {
        myTextFieldDepartPos->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_DEPARTPOS;
    }
    // set color of setDepartSpeed, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_DEPARTSPEED, myTextFieldDepartSpeed->getText().text())) {
        myTextFieldDepartSpeed->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_DEPARTSPEED, myTextFieldDepartSpeed->getText().text(), undoList);
    } else {
        myTextFieldDepartSpeed->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_DEPARTSPEED;
    }
    // set color of myTextFieldArrivalLane, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_ARRIVALLANE, myTextFieldArrivalLane->getText().text())) {
        myTextFieldArrivalLane->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_ARRIVALLANE, myTextFieldArrivalLane->getText().text(), undoList);
    } else {
        myTextFieldArrivalLane->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_ARRIVALLANE;
    }
    // set color of myTextFieldArrivalPos, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_ARRIVALPOS, myTextFieldArrivalPos->getText().text())) {
        myTextFieldArrivalPos->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_ARRIVALPOS, myTextFieldArrivalPos->getText().text(), undoList);
    } else {
        myTextFieldArrivalPos->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_ARRIVALPOS;
    }
    // set color of setArrivalSpeed, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_ARRIVALSPEED, myTextFieldArrivalSpeed->getText().text())) {
        myTextFieldArrivalSpeed->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_ARRIVALSPEED, myTextFieldArrivalSpeed->getText().text(), undoList);
    } else {
        myTextFieldArrivalSpeed->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_ARRIVALSPEED;
    }
    // set color of myTextFieldLine, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_LINE, myTextFieldLine->getText().text())) {
        myTextFieldLine->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_LINE, myTextFieldLine->getText().text(), undoList);
    } else {
        myTextFieldLine->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_LINE;
    }
    // set color of myTextFieldPersonNumber, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_PERSON_NUMBER, myTextFieldPersonNumber->getText().text())) {
        myTextFieldPersonNumber->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_PERSON_NUMBER, myTextFieldPersonNumber->getText().text(), undoList);
    } else {
        myTextFieldPersonNumber->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_PERSON_NUMBER;
    }
    // set color of myTextFieldContainerNumber, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_CONTAINER_NUMBER, myTextFieldContainerNumber->getText().text())) {
        myTextFieldContainerNumber->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_CONTAINER_NUMBER, myTextFieldContainerNumber->getText().text(), undoList);
    } else {
        myTextFieldContainerNumber->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_CONTAINER_NUMBER;
    }
    // set reroute
    if (myRerouteCheckButton->getCheck()) {
        myEditedAdditional->setAttribute(SUMO_ATTR_REROUTE, "true", undoList);
        myRerouteCheckButton->setText("true");
    } else {
        myEditedAdditional->setAttribute(SUMO_ATTR_REROUTE, "false", undoList);
        myRerouteCheckButton->setText("false");
    }
    // set color of myTextFieldDepartPosLat, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_DEPARTPOS_LAT, myTextFieldDepartPosLat->getText().text())) {
        myTextFieldDepartPosLat->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_DEPARTPOS_LAT, myTextFieldDepartPosLat->getText().text(), undoList);
    } else {
        myTextFieldDepartPosLat->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_DEPARTPOS_LAT;
    }
    // set color of myTextFieldArrivalPosLat, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_ARRIVALPOS_LAT, myTextFieldArrivalPosLat->getText().text())) {
        myTextFieldArrivalPosLat->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_ARRIVALPOS_LAT, myTextFieldArrivalPosLat->getText().text(), undoList);
    } else {
        myTextFieldArrivalPosLat->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_ARRIVALPOS_LAT;
    }
    // set color of myTextFieldBegin, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_BEGIN, myTextFieldBegin->getText().text())) {
        myTextFieldBegin->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_BEGIN, myTextFieldBegin->getText().text(), undoList);
    } else {
        myTextFieldBegin->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_END;
    }
    // set color of myTextFieldEnd, depending if current value is valid or not
    if (myEditedAdditional->isValid(SUMO_ATTR_END, myTextFieldEnd->getText().text())) {
        myTextFieldEnd->setTextColor(FXRGB(0, 0, 0));
        myEditedAdditional->setAttribute(SUMO_ATTR_END, myTextFieldEnd->getText().text(), undoList);
    } else {
        myTextFieldEnd->setTextColor(FXRGB(255, 0, 0));
        myCalibratorFlowValid = false;
        myInvalidAttr = SUMO_ATTR_BEGIN;
    }
    return 1;
}


void
GNECalibratorFlowDialog::updateCalibratorFlowValues() {
    // update fields
    myComboBoxVehicleType->setText(myEditedAdditional->getAttribute(SUMO_ATTR_TYPE).c_str());
    myComboBoxRoute->setText(myEditedAdditional->getAttribute(SUMO_ATTR_ROUTE).c_str());
    myTextFieldVehsPerHour->setText(myEditedAdditional->getAttribute(SUMO_ATTR_VEHSPERHOUR).c_str());
    myTextFieldSpeed->setText(myEditedAdditional->getAttribute(SUMO_ATTR_SPEED).c_str());
    myTextFieldColor->setText(myEditedAdditional->getAttribute(SUMO_ATTR_COLOR).c_str());
    myTextFieldDepartLane->setText(myEditedAdditional->getAttribute(SUMO_ATTR_DEPARTLANE).c_str());
    myTextFieldDepartPos->setText(myEditedAdditional->getAttribute(SUMO_ATTR_DEPARTPOS).c_str());
    myTextFieldDepartSpeed->setText(myEditedAdditional->getAttribute(SUMO_ATTR_DEPARTSPEED).c_str());
    myTextFieldArrivalLane->setText(myEditedAdditional->getAttribute(SUMO_ATTR_ARRIVALLANE).c_str());
    myTextFieldArrivalPos->setText(myEditedAdditional->getAttribute(SUMO_ATTR_ARRIVALPOS).c_str());
    myTextFieldArrivalSpeed->setText(myEditedAdditional->getAttribute(SUMO_ATTR_ARRIVALSPEED).c_str());
    myTextFieldLine->setText(myEditedAdditional->getAttribute(SUMO_ATTR_LINE).c_str());
    myTextFieldPersonNumber->setText(myEditedAdditional->getAttribute(SUMO_ATTR_PERSON_NUMBER).c_str());
    myTextFieldContainerNumber->setText(myEditedAdditional->getAttribute(SUMO_ATTR_CONTAINER_NUMBER).c_str());
    myRerouteCheckButton->setCheck(GNEAttributeCarrier::parse<bool>(myEditedAdditional->getAttribute(SUMO_ATTR_REROUTE).c_str()));
    myTextFieldDepartPosLat->setText(myEditedAdditional->getAttribute(SUMO_ATTR_DEPARTPOS_LAT).c_str());
    myTextFieldArrivalPosLat->setText(myEditedAdditional->getAttribute(SUMO_ATTR_ARRIVALPOS_LAT).c_str());
    myTextFieldBegin->setText(myEditedAdditional->getAttribute(SUMO_ATTR_BEGIN).c_str());
    myTextFieldEnd->setText(myEditedAdditional->getAttribute(SUMO_ATTR_END).c_str());
}


/****************************************************************************/
