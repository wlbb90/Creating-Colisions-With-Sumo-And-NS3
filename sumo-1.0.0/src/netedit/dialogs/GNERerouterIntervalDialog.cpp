/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNERerouterIntervalDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    eb 2017
/// @version $Id$
///
// Dialog for edit rerouter intervals
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
#include <netedit/additionals/GNERerouter.h>
#include <netedit/additionals/GNERerouterInterval.h>
#include <netedit/additionals/GNEClosingLaneReroute.h>
#include <netedit/additionals/GNEClosingReroute.h>
#include <netedit/additionals/GNEDestProbReroute.h>
#include <netedit/additionals/GNERouteProbReroute.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>

#include "GNERerouterIntervalDialog.h"
#include "GNERerouterDialog.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNERerouterIntervalDialog) GNERerouterIntervalDialogMap[] = {
    // called when user click over buttons
    FXMAPFUNC(SEL_COMMAND,          MID_GNE_REROUTEDIALOG_ADD_CLOSINGLANEREROUTE,   GNERerouterIntervalDialog::onCmdAddClosingLaneReroute),
    FXMAPFUNC(SEL_COMMAND,          MID_GNE_REROUTEDIALOG_ADD_CLOSINGREROUTE,       GNERerouterIntervalDialog::onCmdAddClosingReroute),
    FXMAPFUNC(SEL_COMMAND,          MID_GNE_REROUTEDIALOG_ADD_DESTPROBREROUTE,      GNERerouterIntervalDialog::onCmdAddDestProbReroute),
    FXMAPFUNC(SEL_COMMAND,          MID_GNE_REROUTEDIALOG_ADD_ROUTEPROBREROUTE,     GNERerouterIntervalDialog::onCmdAddRouteProbReroute),
    FXMAPFUNC(SEL_COMMAND,          MID_GNE_REROUTEDIALOG_ADD_PARKINGAREAREROUTE,   GNERerouterIntervalDialog::onCmdAddParkingAreaReroute),

    // clicked table (Double and triple clicks allow to remove element more fast)
    FXMAPFUNC(SEL_CLICKED,          MID_GNE_REROUTEDIALOG_TABLE_CLOSINGLANEREROUTE, GNERerouterIntervalDialog::onCmdClickedClosingLaneReroute),
    FXMAPFUNC(SEL_DOUBLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_CLOSINGLANEREROUTE, GNERerouterIntervalDialog::onCmdClickedClosingLaneReroute),
    FXMAPFUNC(SEL_TRIPLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_CLOSINGLANEREROUTE, GNERerouterIntervalDialog::onCmdClickedClosingLaneReroute),
    FXMAPFUNC(SEL_CLICKED,          MID_GNE_REROUTEDIALOG_TABLE_CLOSINGREROUTE,     GNERerouterIntervalDialog::onCmdClickedClosingReroute),
    FXMAPFUNC(SEL_DOUBLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_CLOSINGREROUTE,     GNERerouterIntervalDialog::onCmdClickedClosingReroute),
    FXMAPFUNC(SEL_TRIPLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_CLOSINGREROUTE,     GNERerouterIntervalDialog::onCmdClickedClosingReroute),
    FXMAPFUNC(SEL_CLICKED,          MID_GNE_REROUTEDIALOG_TABLE_DESTPROBREROUTE,    GNERerouterIntervalDialog::onCmdClickedDestProbReroute),
    FXMAPFUNC(SEL_DOUBLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_DESTPROBREROUTE,    GNERerouterIntervalDialog::onCmdClickedDestProbReroute),
    FXMAPFUNC(SEL_TRIPLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_DESTPROBREROUTE,    GNERerouterIntervalDialog::onCmdClickedDestProbReroute),
    FXMAPFUNC(SEL_CLICKED,          MID_GNE_REROUTEDIALOG_TABLE_ROUTEPROBREROUTE,   GNERerouterIntervalDialog::onCmdClickedRouteProbReroute),
    FXMAPFUNC(SEL_DOUBLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_ROUTEPROBREROUTE,   GNERerouterIntervalDialog::onCmdClickedRouteProbReroute),
    FXMAPFUNC(SEL_TRIPLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_ROUTEPROBREROUTE,   GNERerouterIntervalDialog::onCmdClickedRouteProbReroute),
    FXMAPFUNC(SEL_CLICKED,          MID_GNE_REROUTEDIALOG_TABLE_PARKINGAREAREROUTE, GNERerouterIntervalDialog::onCmdClickedParkingAreaReroute),
    FXMAPFUNC(SEL_DOUBLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_PARKINGAREAREROUTE, GNERerouterIntervalDialog::onCmdClickedParkingAreaReroute),
    FXMAPFUNC(SEL_TRIPLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_PARKINGAREAREROUTE, GNERerouterIntervalDialog::onCmdClickedParkingAreaReroute),

    // use "update" instead of "command" to avoid problems mit icons
    FXMAPFUNC(SEL_UPDATE,           MID_GNE_REROUTEDIALOG_TABLE_CLOSINGLANEREROUTE, GNERerouterIntervalDialog::onCmdEditClosingLaneReroute),
    FXMAPFUNC(SEL_UPDATE,           MID_GNE_REROUTEDIALOG_TABLE_CLOSINGREROUTE,     GNERerouterIntervalDialog::onCmdEditClosingReroute),
    FXMAPFUNC(SEL_UPDATE,           MID_GNE_REROUTEDIALOG_TABLE_DESTPROBREROUTE,    GNERerouterIntervalDialog::onCmdEditDestProbReroute),
    FXMAPFUNC(SEL_UPDATE,           MID_GNE_REROUTEDIALOG_TABLE_ROUTEPROBREROUTE,   GNERerouterIntervalDialog::onCmdEditRouteProbReroute),
    FXMAPFUNC(SEL_UPDATE,           MID_GNE_REROUTEDIALOG_TABLE_PARKINGAREAREROUTE, GNERerouterIntervalDialog::onCmdEditParkingAreaReroute),
    FXMAPFUNC(SEL_UPDATE,           MID_GNE_REROUTEDIALOG_EDIT_INTERVAL,            GNERerouterIntervalDialog::onCmdChangeBeginEnd),
};

// Object implementation
FXIMPLEMENT(GNERerouterIntervalDialog, GNEAdditionalDialog, GNERerouterIntervalDialogMap, ARRAYNUMBER(GNERerouterIntervalDialogMap))

// ===========================================================================
// member method definitions
// ===========================================================================

GNERerouterIntervalDialog::GNERerouterIntervalDialog(GNEAdditional* rerouterInterval, bool updatingElement) :
    GNEAdditionalDialog(rerouterInterval, updatingElement, 960, 480),
    myBeginEndValid(true),
    myClosingLaneReroutesValid(true),
    myClosingReroutesValid(true),
    myDestProbReroutesValid(true),
    myParkingAreaReroutesValid(true),
    myRouteProbReroutesValid(true) {
    // fill closing Reroutes
    for (auto i : myEditedAdditional->getAdditionalChilds()) {
        if (i->getTag() == SUMO_TAG_CLOSING_REROUTE) {
            myClosingReroutesEdited.push_back(i);
        }
    }
    // fill closing Lane Reroutes
    for (auto i : myEditedAdditional->getAdditionalChilds()) {
        if (i->getTag() == SUMO_TAG_CLOSING_LANE_REROUTE) {
            myClosingLaneReroutesEdited.push_back(i);
        }
    }
    // fill Dest Prob Reroutes
    for (auto i : myEditedAdditional->getAdditionalChilds()) {
        if (i->getTag() == SUMO_TAG_DEST_PROB_REROUTE) {
            myDestProbReroutesEdited.push_back(i);
        }
    }
    // fill Route Prob Reroutes
    for (auto i : myEditedAdditional->getAdditionalChilds()) {
        if (i->getTag() == SUMO_TAG_ROUTE_PROB_REROUTE) {
            myRouteProbReroutesEdited.push_back(i);
        }
    }
    // fill Parking Area reroutes
    for (auto i : myEditedAdditional->getAdditionalChilds()) {
        if (i->getTag() == SUMO_TAG_PARKING_ZONE_REROUTE) {
            myParkingAreaRerouteEdited.push_back(i);
        }
    }
    // change default header
    std::string typeOfOperation = myUpdatingElement ? "Edit " + toString(myEditedAdditional->getTag()) + " of " : "Create " + toString(myEditedAdditional->getTag()) + " for ";
    changeAdditionalDialogHeader(typeOfOperation + toString(myEditedAdditional->getFirstAdditionalParent()->getTag()) + " '" + myEditedAdditional->getFirstAdditionalParent()->getID() + "'");

    // Create auxiliar frames for tables
    FXHorizontalFrame* columns = new FXHorizontalFrame(myContentFrame, GUIDesignUniformHorizontalFrame);
    FXVerticalFrame* columnLeft = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnRight = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnRight2 = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);

    // create horizontal frame for begin and end label
    FXHorizontalFrame* beginEndElementsLeft = new FXHorizontalFrame(columnLeft, GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(beginEndElementsLeft, (toString(SUMO_ATTR_BEGIN) + " and " + toString(SUMO_ATTR_END) + " of " + toString(myEditedAdditional->getTag())).c_str(), 0, GUIDesignLabelLeftThick);
    myCheckLabel = new FXLabel(beginEndElementsLeft, "", GUIIconSubSys::getIcon(ICON_CORRECT), GUIDesignLabelIcon32x32Thicked);

    // create horizontal frame for begin and end text fields
    FXHorizontalFrame* beginEndElementsRight = new FXHorizontalFrame(columnRight, GUIDesignAuxiliarHorizontalFrame);
    myBeginTextField = new FXTextField(beginEndElementsRight, GUIDesignTextFieldNCol, this, MID_GNE_REROUTEDIALOG_EDIT_INTERVAL, GUIDesignTextFieldReal);
    myBeginTextField->setText(toString(myEditedAdditional->getAttribute(SUMO_ATTR_BEGIN)).c_str());
    myEndTextField = new FXTextField(beginEndElementsRight, GUIDesignTextFieldNCol, this, MID_GNE_REROUTEDIALOG_EDIT_INTERVAL, GUIDesignTextFieldReal);
    myEndTextField->setText(toString(myEditedAdditional->getAttribute(SUMO_ATTR_END)).c_str());

    // Create labels and tables
    FXHorizontalFrame* buttonAndLabelClosingLaneReroute = new FXHorizontalFrame(columnLeft, GUIDesignAuxiliarHorizontalFrame);
    myAddClosingLaneReroutes = new FXButton(buttonAndLabelClosingLaneReroute, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_REROUTEDIALOG_ADD_CLOSINGLANEREROUTE, GUIDesignButtonIcon);
    new FXLabel(buttonAndLabelClosingLaneReroute, ("Add new " + toString(SUMO_TAG_CLOSING_LANE_REROUTE) + "s").c_str(), 0, GUIDesignLabelThick);
    myClosingLaneRerouteTable = new FXTable(columnLeft, this, MID_GNE_REROUTEDIALOG_TABLE_CLOSINGLANEREROUTE, GUIDesignTableAdditionals);
    myClosingLaneRerouteTable->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myClosingLaneRerouteTable->setSelTextColor(FXRGBA(0, 0, 0, 255));

    FXHorizontalFrame* buttonAndLabelClosinReroute = new FXHorizontalFrame(columnLeft, GUIDesignAuxiliarHorizontalFrame);
    myAddClosingReroutes = new FXButton(buttonAndLabelClosinReroute, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_REROUTEDIALOG_ADD_CLOSINGREROUTE, GUIDesignButtonIcon);
    new FXLabel(buttonAndLabelClosinReroute, ("Add new " + toString(SUMO_TAG_CLOSING_REROUTE) + "s").c_str(), 0, GUIDesignLabelThick);
    myClosingRerouteTable = new FXTable(columnLeft, this, MID_GNE_REROUTEDIALOG_TABLE_CLOSINGREROUTE, GUIDesignTableAdditionals);
    myClosingRerouteTable->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myClosingRerouteTable->setSelTextColor(FXRGBA(0, 0, 0, 255));

    FXHorizontalFrame* buttonAndLabelDestProbReroute = new FXHorizontalFrame(columnRight, GUIDesignAuxiliarHorizontalFrame);
    myAddDestProbReroutes = new FXButton(buttonAndLabelDestProbReroute, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_REROUTEDIALOG_ADD_DESTPROBREROUTE, GUIDesignButtonIcon);
    new FXLabel(buttonAndLabelDestProbReroute, ("Add new " + toString(SUMO_TAG_DEST_PROB_REROUTE) + "s").c_str(), 0, GUIDesignLabelThick);
    myDestProbRerouteTable = new FXTable(columnRight, this, MID_GNE_REROUTEDIALOG_TABLE_DESTPROBREROUTE, GUIDesignTableAdditionals);
    myDestProbRerouteTable->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myDestProbRerouteTable->setSelTextColor(FXRGBA(0, 0, 0, 255));

    FXHorizontalFrame* buttonAndLabelRouteProbReroute = new FXHorizontalFrame(columnRight, GUIDesignAuxiliarHorizontalFrame);
    myAddRouteProbReroute = new FXButton(buttonAndLabelRouteProbReroute, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_REROUTEDIALOG_ADD_ROUTEPROBREROUTE, GUIDesignButtonIcon);
    FXLabel* routeProbRerouteLabel = new FXLabel(buttonAndLabelRouteProbReroute, ("Add new " + toString(SUMO_TAG_ROUTE_PROB_REROUTE) + "s").c_str(), 0, GUIDesignLabelThick);
    myRouteProbRerouteTable = new FXTable(columnRight, this, MID_GNE_REROUTEDIALOG_TABLE_ROUTEPROBREROUTE, GUIDesignTableAdditionals);
    myRouteProbRerouteTable->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myRouteProbRerouteTable->setSelTextColor(FXRGBA(0, 0, 0, 255));

    FXHorizontalFrame* buttonAndLabelParkingAreaReroute = new FXHorizontalFrame(columnRight2, GUIDesignAuxiliarHorizontalFrame);
    FXButton* parkingAreaRerouteButton = myAddParkingAreaReroute = new FXButton(buttonAndLabelParkingAreaReroute, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_REROUTEDIALOG_ADD_PARKINGAREAREROUTE, GUIDesignButtonIcon);
    FXLabel* parkingAreaRerouteLabel = new FXLabel(buttonAndLabelParkingAreaReroute, ("Add new " + toString(SUMO_TAG_PARKING_ZONE_REROUTE) + "s").c_str(), 0, GUIDesignLabelThick);
    myParkingAreaRerouteTable = new FXTable(columnRight2, this, MID_GNE_REROUTEDIALOG_TABLE_PARKINGAREAREROUTE, GUIDesignTableAdditionals);
    myParkingAreaRerouteTable->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myParkingAreaRerouteTable->setSelTextColor(FXRGBA(0, 0, 0, 255));

    // disable add parkingAreaReroute Button and change label if there isn't parkingAreas in net
    if (rerouterInterval->getViewNet()->getNet()->getAdditionalByType(SUMO_TAG_PARKING_AREA).size() == 0) {
        parkingAreaRerouteButton->disable();
        parkingAreaRerouteLabel->setText(("There isn't " + toString(SUMO_TAG_PARKING_AREA) + "s in net").c_str());
    }

    // disable add routeProbReroute Button and change label if the rerouter has multiple edges (random routes can only work from one edge)
    if (rerouterInterval->getFirstAdditionalParent()->getEdgeChilds().size() > 1) {
        myAddRouteProbReroute->disable();
        routeProbRerouteLabel->setText("Rerouter has more than one edge");
    }

    // update tables
    updateClosingLaneReroutesTable();
    updateClosingReroutesTable();
    updateDestProbReroutesTable();
    updateRouteProbReroutesTable();
    updateParkingAreaReroutesTable();

    // start a undo list for editing local to this additional
    initChanges();

    // add element if we aren't updating an existent element
    if (myUpdatingElement == false) {
        myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myEditedAdditional, true), true);
    }

    // Open as modal dialog
    openAsModalDialog();
}


GNERerouterIntervalDialog::~GNERerouterIntervalDialog() {}


long
GNERerouterIntervalDialog::onCmdAccept(FXObject*, FXSelector, void*) {
    // set strings for dialogs
    std::string errorTitle = "Error" + toString(myUpdatingElement ? "updating" : "creating") + " " + toString(myEditedAdditional->getTag()) + " of " + toString(myEditedAdditional->getFirstAdditionalParent()->getTag());
    std::string operationType = toString(myEditedAdditional->getFirstAdditionalParent()->getTag()) + "'s " + toString(myEditedAdditional->getTag()) + " cannot be " + (myUpdatingElement ? "updated" : "created") + " because ";
    if (myBeginEndValid == false) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + toString(myEditedAdditional->getTag()) + " defined by " + toString(SUMO_ATTR_BEGIN) + " and " + toString(SUMO_ATTR_END) + " is invalid.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if (myClosingLaneReroutesEdited.empty() &&
               myClosingReroutesEdited.empty() &&
               myDestProbReroutesEdited.empty() &&
               myParkingAreaRerouteEdited.empty() &&
               myRouteProbReroutesEdited.empty()) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + "at least one " + toString(myEditedAdditional->getTag()) + "'s element must be defined.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if ((myClosingLaneReroutesEdited.size() > 0) && (myClosingLaneReroutesValid == false)) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + "there are invalid " + toString(SUMO_TAG_CLOSING_LANE_REROUTE) + "s.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if ((myClosingLaneReroutesEdited.size() > 0) && (myClosingReroutesValid == false)) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + "there are invalid " + toString(SUMO_TAG_CLOSING_REROUTE) + "s.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if ((myDestProbReroutesEdited.size() > 0) && (myDestProbReroutesValid == false)) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + "there are invalid " + toString(SUMO_TAG_PARKING_ZONE_REROUTE) + "s.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if ((myParkingAreaRerouteEdited.size() > 0) && (myParkingAreaReroutesValid == false)) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + "there are invalid " + toString(SUMO_TAG_DEST_PROB_REROUTE) + "s.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else if ((myRouteProbReroutesEdited.size() > 0) && (myRouteProbReroutesValid == false)) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox of type 'warning'");
        // open warning Box
        FXMessageBox::warning(getApp(), MBOX_OK, errorTitle.c_str(), "%s", (operationType + "there are invalid " + toString(SUMO_TAG_ROUTE_PROB_REROUTE) + "s.").c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox of type 'warning' with 'OK'");
        return 0;
    } else {
        // accept changes before closing dialog
        acceptChanges();
        // Stop Modal
        getApp()->stopModal(this, TRUE);
        return 1;
    }
}


long
GNERerouterIntervalDialog::onCmdCancel(FXObject*, FXSelector, void*) {
    // cancel changes
    cancelChanges();
    // Stop Modal
    getApp()->stopModal(this, FALSE);
    return 1;
}


long
GNERerouterIntervalDialog::onCmdReset(FXObject*, FXSelector, void*) {
    // reset changes
    resetChanges();
    // update tables
    updateClosingLaneReroutesTable();
    updateClosingReroutesTable();
    updateDestProbReroutesTable();
    updateRouteProbReroutesTable();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdAddClosingLaneReroute(FXObject*, FXSelector, void*) {
    // create closing lane reroute
    GNEClosingLaneReroute* closingLaneReroute = new GNEClosingLaneReroute(this);
    myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(closingLaneReroute, true), true);
    myClosingLaneReroutesEdited.push_back(closingLaneReroute);
    // update closing lane reroutes table
    updateClosingLaneReroutesTable();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdAddClosingReroute(FXObject*, FXSelector, void*) {
    // create closing reroute
    GNEClosingReroute* closingReroute = new GNEClosingReroute(this);
    myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(closingReroute, true), true);
    myClosingReroutesEdited.push_back(closingReroute);
    // update closing reroutes table
    updateClosingReroutesTable();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdAddDestProbReroute(FXObject*, FXSelector, void*) {
    // create closing reroute and add it to table
    GNEDestProbReroute* destProbReroute = new GNEDestProbReroute(this);
    myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(destProbReroute, true), true);
    myDestProbReroutesEdited.push_back(destProbReroute);
    // update dest Prob reroutes table
    updateDestProbReroutesTable();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdAddRouteProbReroute(FXObject*, FXSelector, void*) {
    // create route Prob Reroute
    GNERouteProbReroute* routeProbReroute = new GNERouteProbReroute(this);
    myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(routeProbReroute, true), true);
    myRouteProbReroutesEdited.push_back(routeProbReroute);
    // update route prob reroutes table
    updateRouteProbReroutesTable();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdAddParkingAreaReroute(FXObject*, FXSelector, void*) {
    // create parkingAreaReroute and add it to table
    GNEParkingAreaReroute* parkingAreaReroute = new GNEParkingAreaReroute(this);
    myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(parkingAreaReroute, true), true);
    myParkingAreaRerouteEdited.push_back(parkingAreaReroute);
    // update dest Prob reroutes table
    updateParkingAreaReroutesTable();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdClickedClosingLaneReroute(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myClosingLaneReroutesEdited.size(); i++) {
        if (myClosingLaneRerouteTable->getItem(i, 4)->hasFocus()) {
            myClosingLaneRerouteTable->removeRows(i);
            myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myClosingLaneReroutesEdited.at(i), false), true);
            myClosingLaneReroutesEdited.erase(myClosingLaneReroutesEdited.begin() + i);
            updateClosingLaneReroutesTable();
            return 1;
        }
    }
    return 0;
}


long
GNERerouterIntervalDialog::onCmdClickedClosingReroute(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myClosingReroutesEdited.size(); i++) {
        if (myClosingRerouteTable->getItem(i, 4)->hasFocus()) {
            myClosingRerouteTable->removeRows(i);
            myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myClosingReroutesEdited.at(i), false), true);
            myClosingReroutesEdited.erase(myClosingReroutesEdited.begin() + i);
            updateClosingReroutesTable();
            return 1;
        }
    }
    return 1;
}


long
GNERerouterIntervalDialog::onCmdClickedDestProbReroute(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myDestProbReroutesEdited.size(); i++) {
        if (myDestProbRerouteTable->getItem(i, 3)->hasFocus()) {
            myDestProbRerouteTable->removeRows(i);
            myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myDestProbReroutesEdited.at(i), false), true);
            myDestProbReroutesEdited.erase(myDestProbReroutesEdited.begin() + i);
            updateDestProbReroutesTable();
            return 1;
        }
    }
    return 0;
}


long
GNERerouterIntervalDialog::onCmdClickedRouteProbReroute(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myRouteProbReroutesEdited.size(); i++) {
        if (myRouteProbRerouteTable->getItem(i, 3)->hasFocus()) {
            myRouteProbRerouteTable->removeRows(i);
            myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myRouteProbReroutesEdited.at(i), false), true);
            myRouteProbReroutesEdited.erase(myRouteProbReroutesEdited.begin() + i);
            updateRouteProbReroutesTable();
            return 1;
        }
    }
    return 0;
}


long
GNERerouterIntervalDialog::onCmdClickedParkingAreaReroute(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myParkingAreaRerouteEdited.size(); i++) {
        if (myParkingAreaRerouteTable->getItem(i, 3)->hasFocus()) {
            ;
        } else if (myParkingAreaRerouteTable->getItem(i, 4)->hasFocus()) {
            myParkingAreaRerouteTable->removeRows(i);
            myEditedAdditional->getViewNet()->getUndoList()->add(new GNEChange_Additional(myParkingAreaRerouteEdited.at(i), false), true);
            myParkingAreaRerouteEdited.erase(myParkingAreaRerouteEdited.begin() + i);
            updateParkingAreaReroutesTable();
            return 1;
        }
    }
    return 0;
}


long
GNERerouterIntervalDialog::onCmdEditClosingLaneReroute(FXObject*, FXSelector, void*) {
    myClosingLaneReroutesValid = true;
    // iterate over table and check that all parameters are correct
    for (int i = 0; i < myClosingLaneRerouteTable->getNumRows(); i++) {
        GNEAdditional* closingLaneReroute = myClosingLaneReroutesEdited.at(i);
        if (!SUMOXMLDefinitions::isValidNetID(myClosingLaneRerouteTable->getItem(i, 0)->getText().text())) {
            myClosingLaneReroutesValid = false;
            myClosingLaneRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (closingLaneReroute->isValid(SUMO_ATTR_ALLOW, myClosingLaneRerouteTable->getItem(i, 1)->getText().text()) == false) {
            myClosingLaneReroutesValid = false;
            myClosingLaneRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (closingLaneReroute->isValid(SUMO_ATTR_DISALLOW, myClosingLaneRerouteTable->getItem(i, 2)->getText().text()) == false) {
            myClosingLaneReroutesValid = false;
            myClosingLaneRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else {
            // check if allow/disallow should be changed
            bool changeAllow = myClosingLaneRerouteTable->getItem(i, 1)->getText().text() != closingLaneReroute->getAttribute(SUMO_ATTR_ALLOW);
            bool changeDisallow = myClosingLaneRerouteTable->getItem(i, 2)->getText().text() != closingLaneReroute->getAttribute(SUMO_ATTR_DISALLOW);
            // set new values in Closing  reroute
            closingLaneReroute->setAttribute(SUMO_ATTR_LANE, myClosingLaneRerouteTable->getItem(i, 0)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            if (changeAllow) {
                closingLaneReroute->setAttribute(SUMO_ATTR_ALLOW, myClosingLaneRerouteTable->getItem(i, 1)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
                myClosingLaneRerouteTable->getItem(i, 2)->setText(closingLaneReroute->getAttribute(SUMO_ATTR_DISALLOW).c_str());

            }
            if (changeDisallow) {
                closingLaneReroute->setAttribute(SUMO_ATTR_DISALLOW, myClosingLaneRerouteTable->getItem(i, 2)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
                myClosingLaneRerouteTable->getItem(i, 1)->setText(closingLaneReroute->getAttribute(SUMO_ATTR_ALLOW).c_str());
            }
            // set Correct label
            myClosingLaneRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        }
    }
    // update list
    myClosingLaneRerouteTable->update();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdEditClosingReroute(FXObject*, FXSelector, void*) {
    myClosingReroutesValid = true;
    // iterate over table and check that all parameters are correct
    for (int i = 0; i < myClosingRerouteTable->getNumRows(); i++) {
        GNEAdditional* closingReroute = myClosingReroutesEdited.at(i);
        if (!SUMOXMLDefinitions::isValidNetID(myClosingRerouteTable->getItem(i, 0)->getText().text())) {
            myClosingReroutesValid = false;
            myClosingRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (closingReroute->isValid(SUMO_ATTR_ALLOW, myClosingRerouteTable->getItem(i, 1)->getText().text()) == false) {
            myClosingReroutesValid = false;
            myClosingRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (closingReroute->isValid(SUMO_ATTR_DISALLOW, myClosingRerouteTable->getItem(i, 2)->getText().text()) == false) {
            myClosingReroutesValid = false;
            myClosingRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else {
            // check if allow/disallow should be changed
            bool changeAllow = myClosingRerouteTable->getItem(i, 1)->getText().text() != closingReroute->getAttribute(SUMO_ATTR_ALLOW);
            bool changeDisallow = myClosingRerouteTable->getItem(i, 2)->getText().text() != closingReroute->getAttribute(SUMO_ATTR_DISALLOW);
            // set new values in Closing  reroute
            closingReroute->setAttribute(SUMO_ATTR_EDGE, myClosingRerouteTable->getItem(i, 0)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            if (changeAllow) {
                closingReroute->setAttribute(SUMO_ATTR_ALLOW, myClosingRerouteTable->getItem(i, 1)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
                myClosingRerouteTable->getItem(i, 2)->setText(closingReroute->getAttribute(SUMO_ATTR_DISALLOW).c_str());

            }
            if (changeDisallow) {
                closingReroute->setAttribute(SUMO_ATTR_DISALLOW, myClosingRerouteTable->getItem(i, 2)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
                myClosingRerouteTable->getItem(i, 1)->setText(closingReroute->getAttribute(SUMO_ATTR_ALLOW).c_str());
            }
            // set Correct label
            myClosingRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        }
    }
    // update list
    myClosingRerouteTable->update();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdEditDestProbReroute(FXObject*, FXSelector, void*) {
    myDestProbReroutesValid = true;
    // iterate over table and check that all parameters are correct
    for (int i = 0; i < myDestProbRerouteTable->getNumRows(); i++) {
        GNEAdditional* destProbReroute = myDestProbReroutesEdited.at(i);
        if (!SUMOXMLDefinitions::isValidNetID(myDestProbRerouteTable->getItem(i, 0)->getText().text())) {
            myDestProbReroutesValid = false;
            myDestProbRerouteTable->getItem(i, 2)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (destProbReroute->isValid(SUMO_ATTR_PROB, myDestProbRerouteTable->getItem(i, 1)->getText().text()) == false) {
            myDestProbReroutesValid = false;
            myDestProbRerouteTable->getItem(i, 2)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else {
            // set new values in Closing  reroute
            destProbReroute->setAttribute(SUMO_ATTR_EDGE, myDestProbRerouteTable->getItem(i, 0)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            destProbReroute->setAttribute(SUMO_ATTR_PROB, myDestProbRerouteTable->getItem(i, 1)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            // set Correct label
            myDestProbRerouteTable->getItem(i, 2)->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        }
    }
    // update list
    myDestProbRerouteTable->update();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdEditRouteProbReroute(FXObject*, FXSelector, void*) {
    myRouteProbReroutesValid = true;
    // iterate over table and check that all parameters are correct
    for (int i = 0; i < myRouteProbRerouteTable->getNumRows(); i++) {
        GNEAdditional* routeProbReroute = myRouteProbReroutesEdited.at(i);
        if (!SUMOXMLDefinitions::isValidNetID(myRouteProbRerouteTable->getItem(i, 0)->getText().text())) {
            myRouteProbReroutesValid = false;
            myRouteProbRerouteTable->getItem(i, 2)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (routeProbReroute->isValid(SUMO_ATTR_PROB, myRouteProbRerouteTable->getItem(i, 1)->getText().text()) == false) {
            myRouteProbReroutesValid = false;
            myRouteProbRerouteTable->getItem(i, 2)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else {
            // set new values in Closing  reroute
            routeProbReroute->setAttribute(SUMO_ATTR_ROUTE, myRouteProbRerouteTable->getItem(i, 0)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            routeProbReroute->setAttribute(SUMO_ATTR_PROB, myRouteProbRerouteTable->getItem(i, 1)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            // set Correct label
            myRouteProbRerouteTable->getItem(i, 2)->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        }
    }
    // update list
    myRouteProbRerouteTable->update();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdEditParkingAreaReroute(FXObject*, FXSelector, void*) {
    myParkingAreaReroutesValid = true;
    // iterate over table and check that all parameters are correct
    for (int i = 0; i < myParkingAreaRerouteTable->getNumRows(); i++) {
        GNEAdditional* parkingAreaReroute = myParkingAreaRerouteEdited.at(i);
        if (!SUMOXMLDefinitions::isValidNetID(myParkingAreaRerouteTable->getItem(i, 0)->getText().text())) {
            myParkingAreaReroutesValid = false;
            myParkingAreaRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (parkingAreaReroute->isValid(SUMO_ATTR_PROB, myParkingAreaRerouteTable->getItem(i, 1)->getText().text()) == false) {
            myParkingAreaReroutesValid = false;
            myParkingAreaRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else if (parkingAreaReroute->isValid(SUMO_ATTR_VISIBLE, myParkingAreaRerouteTable->getItem(i, 2)->getText().text()) == false) {
            myParkingAreaReroutesValid = false;
            myParkingAreaRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
        } else {
            // set new values in Closing  reroute
            parkingAreaReroute->setAttribute(SUMO_ATTR_PARKING, myParkingAreaRerouteTable->getItem(i, 0)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            parkingAreaReroute->setAttribute(SUMO_ATTR_PROB, myParkingAreaRerouteTable->getItem(i, 1)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            parkingAreaReroute->setAttribute(SUMO_ATTR_VISIBLE, myParkingAreaRerouteTable->getItem(i, 2)->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
            // set Correct label
            myParkingAreaRerouteTable->getItem(i, 3)->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        }
    }
    // update list
    myParkingAreaRerouteTable->update();
    return 1;
}


long
GNERerouterIntervalDialog::onCmdChangeBeginEnd(FXObject*, FXSelector, void*) {
    if (myEditedAdditional->isValid(SUMO_ATTR_BEGIN, myBeginTextField->getText().text()) &&
            myEditedAdditional->isValid(SUMO_ATTR_END, myEndTextField->getText().text())) {
        // set new values in rerouter interval
        myEditedAdditional->setAttribute(SUMO_ATTR_BEGIN, myBeginTextField->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
        myEditedAdditional->setAttribute(SUMO_ATTR_END, myEndTextField->getText().text(), myEditedAdditional->getViewNet()->getUndoList());
        // sort intervals of rerouter
        myEditedAdditional->sortAdditionalChilds();
        // change icon
        myBeginEndValid = true;
        myCheckLabel->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
    } else {
        myBeginEndValid = false;
        myCheckLabel->setIcon(GUIIconSubSys::getIcon(ICON_ERROR));
    }
    return 0;
}


void
GNERerouterIntervalDialog::updateClosingLaneReroutesTable() {
    // clear table
    myClosingLaneRerouteTable->clearItems();
    // set number of rows
    myClosingLaneRerouteTable->setTableSize(int(myClosingLaneReroutesEdited.size()), 5);
    // Configure list
    myClosingLaneRerouteTable->setVisibleColumns(5);
    myClosingLaneRerouteTable->setColumnWidth(0, 83);
    myClosingLaneRerouteTable->setColumnWidth(1, 83);
    myClosingLaneRerouteTable->setColumnWidth(2, 82);
    myClosingLaneRerouteTable->setColumnWidth(3, GUIDesignTableIconCellWidth);
    myClosingLaneRerouteTable->setColumnWidth(4, GUIDesignTableIconCellWidth);
    myClosingLaneRerouteTable->setColumnText(0, toString(SUMO_ATTR_LANE).c_str());
    myClosingLaneRerouteTable->setColumnText(1, toString(SUMO_ATTR_ALLOW).c_str());
    myClosingLaneRerouteTable->setColumnText(2, toString(SUMO_ATTR_DISALLOW).c_str());
    myClosingLaneRerouteTable->setColumnText(3, "");
    myClosingLaneRerouteTable->setColumnText(4, "");
    myClosingLaneRerouteTable->getRowHeader()->setWidth(0);
    // Declare pointer to FXTableItem
    FXTableItem* item = 0;
    // iterate over values
    for (int i = 0; i < (int)myClosingLaneReroutesEdited.size(); i++) {
        // Set closing edge
        item = new FXTableItem(myClosingLaneReroutesEdited.at(i)->getAttribute(SUMO_ATTR_LANE).c_str());
        myClosingLaneRerouteTable->setItem(i, 0, item);
        // set allow vehicles
        item = new FXTableItem(myClosingLaneReroutesEdited.at(i)->getAttribute(SUMO_ATTR_ALLOW).c_str());
        myClosingLaneRerouteTable->setItem(i, 1, item);
        // set disallow vehicles
        item = new FXTableItem(myClosingLaneReroutesEdited.at(i)->getAttribute(SUMO_ATTR_DISALLOW).c_str());
        myClosingLaneRerouteTable->setItem(i, 2, item);
        // set valid icon
        item = new FXTableItem("");
        item->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myClosingLaneRerouteTable->setItem(i, 3, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myClosingLaneRerouteTable->setItem(i, 4, item);
    }
}


void
GNERerouterIntervalDialog::updateClosingReroutesTable() {
    // clear table
    myClosingRerouteTable->clearItems();
    // set number of rows
    myClosingRerouteTable->setTableSize(int(myClosingReroutesEdited.size()), 5);
    // Configure list
    myClosingRerouteTable->setVisibleColumns(5);
    myClosingRerouteTable->setColumnWidth(0, 83);
    myClosingRerouteTable->setColumnWidth(1, 83);
    myClosingRerouteTable->setColumnWidth(2, 82);
    myClosingRerouteTable->setColumnWidth(3, GUIDesignTableIconCellWidth);
    myClosingRerouteTable->setColumnWidth(4, GUIDesignTableIconCellWidth);
    myClosingRerouteTable->setColumnText(0, toString(SUMO_ATTR_EDGE).c_str());
    myClosingRerouteTable->setColumnText(1, toString(SUMO_ATTR_ALLOW).c_str());
    myClosingRerouteTable->setColumnText(2, toString(SUMO_ATTR_DISALLOW).c_str());
    myClosingRerouteTable->setColumnText(3, "");
    myClosingRerouteTable->setColumnText(4, "");
    myClosingRerouteTable->getRowHeader()->setWidth(0);
    // Declare pointer to FXTableItem
    FXTableItem* item = 0;
    // iterate over values
    for (int i = 0; i < (int)myClosingReroutesEdited.size(); i++) {
        // Set closing edge
        item = new FXTableItem(myClosingReroutesEdited.at(i)->getAttribute(SUMO_ATTR_EDGE).c_str());
        myClosingRerouteTable->setItem(i, 0, item);
        // set allow vehicles
        item = new FXTableItem(myClosingReroutesEdited.at(i)->getAttribute(SUMO_ATTR_ALLOW).c_str());
        myClosingRerouteTable->setItem(i, 1, item);
        // set disallow vehicles
        item = new FXTableItem(myClosingReroutesEdited.at(i)->getAttribute(SUMO_ATTR_DISALLOW).c_str());
        myClosingRerouteTable->setItem(i, 2, item);
        // set valid icon
        item = new FXTableItem("");
        item->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myClosingRerouteTable->setItem(i, 3, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myClosingRerouteTable->setItem(i, 4, item);
    }
}


void
GNERerouterIntervalDialog::updateDestProbReroutesTable() {
    // clear table
    myDestProbRerouteTable->clearItems();
    // set number of rows
    myDestProbRerouteTable->setTableSize(int(myDestProbReroutesEdited.size()), 4);
    // Configure list
    myDestProbRerouteTable->setVisibleColumns(4);
    myDestProbRerouteTable->setColumnWidth(0, 124);
    myDestProbRerouteTable->setColumnWidth(1, 124);
    myDestProbRerouteTable->setColumnWidth(2, GUIDesignTableIconCellWidth);
    myDestProbRerouteTable->setColumnWidth(3, GUIDesignTableIconCellWidth);
    myDestProbRerouteTable->setColumnText(0, toString(SUMO_ATTR_EDGE).c_str());
    myDestProbRerouteTable->setColumnText(1, toString(SUMO_ATTR_PROB).c_str());
    myDestProbRerouteTable->setColumnText(2, "");
    myDestProbRerouteTable->setColumnText(3, "");
    myDestProbRerouteTable->getRowHeader()->setWidth(0);
    // Declare pointer to FXTableItem
    FXTableItem* item = 0;
    // iterate over values
    for (int i = 0; i < (int)myDestProbReroutesEdited.size(); i++) {
        // Set new destination
        item = new FXTableItem(myDestProbReroutesEdited.at(i)->getAttribute(SUMO_ATTR_EDGE).c_str());
        myDestProbRerouteTable->setItem(i, 0, item);
        // Set probability
        item = new FXTableItem(myDestProbReroutesEdited.at(i)->getAttribute(SUMO_ATTR_PROB).c_str());
        myDestProbRerouteTable->setItem(i, 1, item);
        // set valid icon
        item = new FXTableItem("");
        item->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myDestProbRerouteTable->setItem(i, 2, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myDestProbRerouteTable->setItem(i, 3, item);
    }
}


void
GNERerouterIntervalDialog::updateRouteProbReroutesTable() {
    // clear table
    myRouteProbRerouteTable->clearItems();
    // set number of rows
    myRouteProbRerouteTable->setTableSize(int(myRouteProbReroutesEdited.size()), 4);
    // Configure list
    myRouteProbRerouteTable->setVisibleColumns(4);
    myRouteProbRerouteTable->setColumnWidth(0, 124);
    myRouteProbRerouteTable->setColumnWidth(1, 124);
    myRouteProbRerouteTable->setColumnWidth(2, GUIDesignTableIconCellWidth);
    myRouteProbRerouteTable->setColumnWidth(3, GUIDesignTableIconCellWidth);
    myRouteProbRerouteTable->setColumnText(0, toString(SUMO_ATTR_ROUTE).c_str());
    myRouteProbRerouteTable->setColumnText(1, toString(SUMO_ATTR_PROB).c_str());
    myRouteProbRerouteTable->setColumnText(2, "");
    myRouteProbRerouteTable->setColumnText(3, "");
    myRouteProbRerouteTable->getRowHeader()->setWidth(0);
    // Declare pointer to FXTableItem
    FXTableItem* item = 0;
    // iterate over values
    for (int i = 0; i < (int)myRouteProbReroutesEdited.size(); i++) {
        // Set new route
        item = new FXTableItem(myRouteProbReroutesEdited.at(i)->getAttribute(SUMO_ATTR_ROUTE).c_str());
        myRouteProbRerouteTable->setItem(i, 0, item);
        // Set probability
        item = new FXTableItem(myRouteProbReroutesEdited.at(i)->getAttribute(SUMO_ATTR_PROB).c_str());
        myRouteProbRerouteTable->setItem(i, 1, item);
        // set valid icon
        item = new FXTableItem("");
        item->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myRouteProbRerouteTable->setItem(i, 2, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myRouteProbRerouteTable->setItem(i, 3, item);
    }
}


void
GNERerouterIntervalDialog::updateParkingAreaReroutesTable() {
    // clear table
    myParkingAreaRerouteTable->clearItems();
    // set number of rows
    myParkingAreaRerouteTable->setTableSize(int(myParkingAreaRerouteEdited.size()), 5);
    // Configure list
    myParkingAreaRerouteTable->setVisibleColumns(4);
    myParkingAreaRerouteTable->setColumnWidth(0, 124);
    myParkingAreaRerouteTable->setColumnWidth(1, 90);
    myParkingAreaRerouteTable->setColumnWidth(2, 35);
    myParkingAreaRerouteTable->setColumnWidth(3, GUIDesignTableIconCellWidth);
    myParkingAreaRerouteTable->setColumnWidth(4, GUIDesignTableIconCellWidth);
    myParkingAreaRerouteTable->setColumnText(0, toString(SUMO_ATTR_PARKING).c_str());
    myParkingAreaRerouteTable->setColumnText(1, toString(SUMO_ATTR_PROB).c_str());
    myParkingAreaRerouteTable->setColumnText(2, "vis.");
    myParkingAreaRerouteTable->setColumnText(3, "");
    myParkingAreaRerouteTable->setColumnText(4, "");
    myParkingAreaRerouteTable->getRowHeader()->setWidth(0);
    // Declare pointer to FXTableItem
    FXTableItem* item = 0;
    // iterate over values
    for (int i = 0; i < (int)myParkingAreaRerouteEdited.size(); i++) {
        // Set new destination
        item = new FXTableItem(myParkingAreaRerouteEdited.at(i)->getAttribute(SUMO_ATTR_PARKING).c_str());
        myParkingAreaRerouteTable->setItem(i, 0, item);
        // Set probability
        item = new FXTableItem(myParkingAreaRerouteEdited.at(i)->getAttribute(SUMO_ATTR_PROB).c_str());
        myParkingAreaRerouteTable->setItem(i, 1, item);
        // Set visible
        item = new FXTableItem(myParkingAreaRerouteEdited.at(i)->getAttribute(SUMO_ATTR_VISIBLE) == "1" ? "true" : "false");
        myParkingAreaRerouteTable->setItem(i, 2, item);
        // set valid icon
        item = new FXTableItem("");
        item->setIcon(GUIIconSubSys::getIcon(ICON_CORRECT));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myParkingAreaRerouteTable->setItem(i, 3, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myParkingAreaRerouteTable->setItem(i, 4, item);
    }
}


/****************************************************************************/
