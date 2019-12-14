/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNETLSEditorFrame.cpp
/// @author  Jakob Erdmann
/// @date    May 2011
/// @version $Id$
///
// The Widget for modifying traffic lights
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <iostream>
#include <utils/foxtools/fxexdefs.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIIOGlobals.h>
#include <utils/gui/div/GUIDesigns.h>
#include <netbuild/NBTrafficLightDefinition.h>
#include <netbuild/NBLoadedSUMOTLDef.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/xml/XMLSubSys.h>
#include <netwrite/NWWriter_SUMO.h>
#include <netimport/NIXMLTrafficLightsHandler.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBNetBuilder.h>
#include <netbuild/NBOwnTLDef.h>
#include <netedit/changes/GNEChange_TLS.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNENet.h>
#include <netedit/netelements/GNEJunction.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/GNEUndoList.h>
#include <netedit/netelements/GNEInternalLane.h>

#include "GNETLSEditorFrame.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNETLSEditorFrame) GNETLSEditorFrameMap[] = {
    FXMAPFUNC(SEL_COMMAND,    MID_CANCEL,                       GNETLSEditorFrame::onCmdCancel),
    FXMAPFUNC(SEL_UPDATE,     MID_CANCEL,                       GNETLSEditorFrame::onUpdModified),
    FXMAPFUNC(SEL_COMMAND,    MID_OK,                           GNETLSEditorFrame::onCmdOK),
    FXMAPFUNC(SEL_UPDATE,     MID_OK,                           GNETLSEditorFrame::onUpdModified),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_CREATE,          GNETLSEditorFrame::onCmdDefCreate),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_CREATE,          GNETLSEditorFrame::onUpdDefCreate),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_DELETE,          GNETLSEditorFrame::onCmdDefDelete),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_DELETE,          GNETLSEditorFrame::onUpdDefSwitch),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_SWITCH,          GNETLSEditorFrame::onCmdDefSwitch),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_SWITCH,          GNETLSEditorFrame::onUpdDefSwitch),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_OFFSET,          GNETLSEditorFrame::onCmdDefOffset),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_OFFSET,          GNETLSEditorFrame::onUpdNeedsDef),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_RENAME,          GNETLSEditorFrame::onCmdDefRename),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_SUBRENAME,       GNETLSEditorFrame::onCmdDefSubRename),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_ADDOFF,          GNETLSEditorFrame::onCmdDefAddOff),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_GUESSPROGRAM,    GNETLSEditorFrame::onCmdGuess),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_PHASE_CREATE,    GNETLSEditorFrame::onCmdPhaseCreate),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_PHASE_CREATE,    GNETLSEditorFrame::onUpdNeedsDef),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_PHASE_DELETE,    GNETLSEditorFrame::onCmdPhaseDelete),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_PHASE_DELETE,    GNETLSEditorFrame::onUpdNeedsDefAndPhase),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_CLEANUP,         GNETLSEditorFrame::onCmdCleanup),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_CLEANUP,         GNETLSEditorFrame::onUpdNeedsDef),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_ADDUNUSED,       GNETLSEditorFrame::onCmdAddUnused),
    FXMAPFUNC(SEL_UPDATE,     MID_GNE_TLSFRAME_ADDUNUSED,       GNETLSEditorFrame::onUpdNeedsDef),
    FXMAPFUNC(SEL_SELECTED,   MID_GNE_TLSFRAME_PHASE_TABLE,     GNETLSEditorFrame::onCmdPhaseSwitch),
    FXMAPFUNC(SEL_DESELECTED, MID_GNE_TLSFRAME_PHASE_TABLE,     GNETLSEditorFrame::onCmdPhaseSwitch),
    FXMAPFUNC(SEL_CHANGED,    MID_GNE_TLSFRAME_PHASE_TABLE,     GNETLSEditorFrame::onCmdPhaseSwitch),
    FXMAPFUNC(SEL_REPLACED,   MID_GNE_TLSFRAME_PHASE_TABLE,     GNETLSEditorFrame::onCmdPhaseEdit),
};

FXDEFMAP(GNETLSEditorFrame::TLSFile) TLSFileMap[] = {
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_LOAD_PROGRAM,    GNETLSEditorFrame::TLSFile::onCmdLoadTLSProgram),
    FXMAPFUNC(SEL_COMMAND,    MID_GNE_TLSFRAME_SAVE_PROGRAM,    GNETLSEditorFrame::TLSFile::onCmdSaveTLSProgram),
};

// Object implementation
FXIMPLEMENT(GNETLSEditorFrame,          FXVerticalFrame,    GNETLSEditorFrameMap,   ARRAYNUMBER(GNETLSEditorFrameMap))
FXIMPLEMENT(GNETLSEditorFrame::TLSFile, FXGroupBox,         TLSFileMap,             ARRAYNUMBER(TLSFileMap))


// ===========================================================================
// method definitions
// ===========================================================================

GNETLSEditorFrame::GNETLSEditorFrame(FXHorizontalFrame* horizontalFrameParent, GNEViewNet* viewNet):
    GNEFrame(horizontalFrameParent, viewNet, "Edit Traffic Light"),
    myEditedDef(0) {

    // create TLSJunction modul
    myTLSJunction = new GNETLSEditorFrame::TLSJunction(this);

    // create TLSDefinition modul
    myTLSDefinition = new GNETLSEditorFrame::TLSDefinition(this);

    // create TLSAttributes modul
    myTLSAttributes = new GNETLSEditorFrame::TLSAttributes(this);

    // create TLSModifications modul
    myTLSModifications = new GNETLSEditorFrame::TLSModifications(this);

    // create TLSPhases modul
    myTLSPhases = new GNETLSEditorFrame::TLSPhases(this);

    // create TLSFile modul
    myTLSFile = new GNETLSEditorFrame::TLSFile(this);

    // "Add 'off' program"
    /*
    new FXButton(myContentFrame, "Add \"Off\"-Program\t\tAdds a program for switching off this traffic light",
            0, this, MID_GNE_TLSFRAME_ADDOFF, GUIDesignButton);
    */
}


GNETLSEditorFrame::~GNETLSEditorFrame() {
    cleanup();
}


void
GNETLSEditorFrame::editJunction(GNEJunction* junction) {
    if (myTLSJunction->getCurrentJunction() == 0 || (!myTLSModifications->checkHaveModifications() && (junction != myTLSJunction->getCurrentJunction()))) {
        onCmdCancel(0, 0, 0);
        myViewNet->getUndoList()->p_begin("modifying traffic light definition");
        myTLSJunction->setCurrentJunction(junction);
        myTLSJunction->getCurrentJunction()->selectTLS(true);
        myTLSAttributes->initTLSAttributes(myTLSJunction->getCurrentJunction());
        myTLSJunction->updateJunctionDescription();
    } else {
        myViewNet->setStatusBarText("Unsaved modifications. Abort or Save");
    }
}


bool
GNETLSEditorFrame::isTLSSaved() {
    if (myTLSModifications->checkHaveModifications()) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening question FXMessageBox 'save TLS'");
        // open question box
        FXuint answer = FXMessageBox::question(this, MBOX_YES_NO_CANCEL,
                                               "Save TLS Changes", "%s",
                                               "There is unsaved changes in current edited traffic light.\nDo you want to save it before changing mode?");
        if (answer == MBOX_CLICKED_YES) { //1:yes, 2:no, 4:esc/cancel
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'save TLS' with 'YES'");
            // save modifications
            onCmdOK(0, 0, 0);
            return true;
        } else if (answer == MBOX_CLICKED_NO) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'save TLS' with 'No'");
            // cancel modifications
            onCmdCancel(0, 0, 0);
            return true;
        } else {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'save TLS' with 'Cancel'");
            // abort changing mode
            return false;
        }
    } else {
        return true;
    }
}


bool
GNETLSEditorFrame::parseTLSPrograms(const std::string& file) {
    myViewNet->getUndoList()->p_begin("Loading TLS");
    XMLSubSys::runParser(*myTLSFile, file);
    if (myTLSFile->checkTLSValids()) {
        for (auto i : myTLSFile->getLoadedTLS()) {
            GNEJunction* junction = myViewNet->getNet()->retrieveJunction(i->getID());
            myViewNet->getUndoList()->add(new GNEChange_TLS(junction, i, true), true);
        }
        myViewNet->getUndoList()->p_end();
    } else {
        myViewNet->getUndoList()->p_abort();
    }
    return true;
}


long
GNETLSEditorFrame::onCmdCancel(FXObject*, FXSelector, void*) {
    if (myTLSJunction->getCurrentJunction() != 0) {
        myViewNet->getUndoList()->p_abort();
        cleanup();
        myViewNet->update();
        // disable TLS File
        myTLSFile->disableTLSFile();
    }
    return 1;
}


long
GNETLSEditorFrame::onCmdOK(FXObject*, FXSelector, void*) {
    if (myTLSJunction->getCurrentJunction() != 0) {
        if (myTLSModifications->checkHaveModifications()) {
            NBTrafficLightDefinition* oldDefinition = myTLSAttributes->getCurrentTLSDefinition();
            std::vector<NBNode*> nodes = oldDefinition->getNodes();
            for (auto it : nodes) {
                GNEJunction* junction = myViewNet->getNet()->retrieveJunction(it->getID());
                myViewNet->getUndoList()->add(new GNEChange_TLS(junction, oldDefinition, false), true);
                myViewNet->getUndoList()->add(new GNEChange_TLS(junction, myEditedDef, true), true);
            }
            myEditedDef = nullptr;
            myViewNet->getUndoList()->p_end();
            cleanup();
            myViewNet->update();
        } else {
            onCmdCancel(0, 0, 0);
        }
    }
    return 1;
}


long
GNETLSEditorFrame::onCmdDefCreate(FXObject*, FXSelector, void*) {
    GNEJunction* junction = myTLSJunction->getCurrentJunction();
    // abort because we onCmdOk assumes we wish to save an edited definition
    onCmdCancel(0, 0, 0);
    // check that current junction has two or more edges
    if ((junction->getGNEIncomingEdges().size() > 0) && (junction->getGNEOutgoingEdges().size() > 0)) {
        if (junction->getAttribute(SUMO_ATTR_TYPE) != toString(NODETYPE_TRAFFIC_LIGHT)) {
            junction->setAttribute(SUMO_ATTR_TYPE, toString(NODETYPE_TRAFFIC_LIGHT), myViewNet->getUndoList());
        } else {
            myViewNet->getUndoList()->add(new GNEChange_TLS(junction, 0, true, true), true);
        }
        editJunction(junction);
    } else {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening warning FXMessageBox 'invalid TLS'");
        // open question box
        FXMessageBox::warning(this, MBOX_OK,
                              "TLS cannot be created", "%s",
                              "Traffic Light cannot be created because junction must have\n at least one incoming edge and one outgoing edge.");
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox 'invalid TLS'");
    }
    return 1;
}


long
GNETLSEditorFrame::onCmdDefDelete(FXObject*, FXSelector, void*) {
    GNEJunction* junction = myTLSJunction->getCurrentJunction();
    const bool changeType = myTLSAttributes->getNumberOfTLSDefinitions() == 1;
    NBTrafficLightDefinition* tlDef = myTLSAttributes->getCurrentTLSDefinition();
    onCmdCancel(0, 0, 0); // abort because onCmdOk assumes we wish to save an edited definition
    if (changeType) {
        junction->setAttribute(SUMO_ATTR_TYPE, toString(NODETYPE_PRIORITY), myViewNet->getUndoList());
    } else {
        myViewNet->getUndoList()->add(new GNEChange_TLS(junction, tlDef, false), true);
    }
    return 1;
}


long
GNETLSEditorFrame::onCmdDefSwitch(FXObject*, FXSelector, void*) {
    assert(myTLSJunction->getCurrentJunction() != 0);
    assert(myTLSAttributes->getNumberOfTLSDefinitions() == myTLSAttributes->getNumberOfPrograms());
    NBTrafficLightDefinition* tlDef = myTLSAttributes->getCurrentTLSDefinition();
    // logic may not have been recomputed yet. recompute to be sure
    NBTrafficLightLogicCont& tllCont = myViewNet->getNet()->getTLLogicCont();
    myViewNet->getNet()->computeJunction(myTLSJunction->getCurrentJunction());
    NBTrafficLightLogic* tllogic = tllCont.getLogic(tlDef->getID(), tlDef->getProgramID());
    if (tllogic != 0) {
        // now we can be sure that the tlDef is up to date (i.e. re-guessed)
        buildIinternalLanes(tlDef);
        // create working copy from original def
        delete myEditedDef;
        myEditedDef = new NBLoadedSUMOTLDef(tlDef, tllogic);
        myTLSAttributes->setOffset(myEditedDef->getLogic()->getOffset());
        myTLSPhases->initPhaseTable();
        myTLSPhases->updateCycleDuration();
        myTLSPhases->showCycleDuration();
    } else {
        // tlDef has no valid logic (probably because id does not control any links
        onCmdCancel(0, 0, 0);
        myViewNet->setStatusBarText("Traffic light does not control any links");
    }
    return 1;
}


long
GNETLSEditorFrame::onUpdDefSwitch(FXObject* o, FXSelector, void*) {
    const bool enable = myTLSAttributes->getNumberOfTLSDefinitions() > 0 && !myTLSModifications->checkHaveModifications();
    o->handle(this, FXSEL(SEL_COMMAND, enable ? FXWindow::ID_ENABLE : FXWindow::ID_DISABLE), 0);
    return 1;
}


long
GNETLSEditorFrame::onUpdNeedsDef(FXObject* o, FXSelector, void*) {
    const bool enable = myTLSAttributes->getNumberOfTLSDefinitions() > 0;
    o->handle(this, FXSEL(SEL_COMMAND, enable ? FXWindow::ID_ENABLE : FXWindow::ID_DISABLE), 0);
    return 1;
}


long
GNETLSEditorFrame::onUpdNeedsDefAndPhase(FXObject* o, FXSelector, void*) {
    // do not delete the last phase
    const bool enable = myTLSAttributes->getNumberOfTLSDefinitions() > 0 && myTLSPhases->getPhaseTable()->getNumRows() > 1;
    o->handle(this, FXSEL(SEL_COMMAND, enable ? FXWindow::ID_ENABLE : FXWindow::ID_DISABLE), 0);
    return 1;
}


long
GNETLSEditorFrame::onUpdDefCreate(FXObject* o, FXSelector, void*) {
    const bool enable = myTLSJunction->getCurrentJunction() != 0 && !myTLSModifications->checkHaveModifications();
    o->handle(this, FXSEL(SEL_COMMAND, enable ? FXWindow::ID_ENABLE : FXWindow::ID_DISABLE), 0);
    return 1;
}


long
GNETLSEditorFrame::onUpdModified(FXObject* o, FXSelector, void*) {
    bool enable = myTLSModifications->checkHaveModifications();
    o->handle(this, FXSEL(SEL_COMMAND, enable ? FXWindow::ID_ENABLE : FXWindow::ID_DISABLE), 0);
    return 1;
}



long
GNETLSEditorFrame::onCmdDefOffset(FXObject*, FXSelector, void*) {
    myTLSModifications->setHaveModifications(true);
    myEditedDef->setOffset(myTLSAttributes->getOffset());
    return 1;
}


long
GNETLSEditorFrame::onCmdDefRename(FXObject*, FXSelector, void*) {
    return 1;
}


long
GNETLSEditorFrame::onCmdDefSubRename(FXObject*, FXSelector, void*) {
    return 1;
}


long
GNETLSEditorFrame::onCmdDefAddOff(FXObject*, FXSelector, void*) {
    return 1;
}


long
GNETLSEditorFrame::onCmdGuess(FXObject*, FXSelector, void*) {
    return 1;
}


long
GNETLSEditorFrame::onCmdPhaseSwitch(FXObject*, FXSelector, void*) {
    const int index = myTLSPhases->getPhaseTable()->getCurrentRow();
    const NBTrafficLightLogic::PhaseDefinition& phase = getPhases()[index];
    myTLSPhases->getPhaseTable()->selectRow(index);
    // need not hold since links could have been deleted somewhere else and indices may be reused
    // assert(phase.state.size() == myInternalLanes.size());
    for (auto it : myInternalLanes) {
        int tlIndex = it.first;
        std::vector<GNEInternalLane*> lanes = it.second;
        LinkState state = LINKSTATE_DEADEND;
        if (tlIndex >= 0 && tlIndex < (int)phase.state.size()) {
            state = (LinkState)phase.state[tlIndex];
        }
        for (auto it_lane : lanes) {
            it_lane->setLinkState(state);
        }
    }
    myViewNet->update();
    return 1;
}


long
GNETLSEditorFrame::onCmdPhaseCreate(FXObject*, FXSelector, void*) {
    myTLSModifications->setHaveModifications(true);
    // allows insertion at first position by deselecting via arrow keys
    int newIndex = myTLSPhases->getPhaseTable()->getSelStartRow() + 1;
    int oldIndex = MAX2(0, myTLSPhases->getPhaseTable()->getSelStartRow());
    // copy current row
    const bool fixed = myEditedDef->getType() == TLTYPE_STATIC;
    SUMOTime duration = getSUMOTime(myTLSPhases->getPhaseTable()->getItemText(oldIndex, 0));
    std::string state = myTLSPhases->getPhaseTable()->getItemText(oldIndex, fixed ? 1 : 3).text();

    // smart adapations for new state
    bool haveGreen = false;
    bool haveYellow = false;
    for (char c : state) {
        if (c == LINKSTATE_TL_GREEN_MAJOR || c == LINKSTATE_TL_GREEN_MINOR) {
            haveGreen = true;
        } else if (c == LINKSTATE_TL_YELLOW_MAJOR || c == LINKSTATE_TL_YELLOW_MINOR) {
            haveYellow = true;
        }
    }
    const OptionsCont& oc = OptionsCont::getOptions();
    if (haveGreen && haveYellow) {
        // guess left-mover state
        duration = TIME2STEPS(oc.getInt("tls.left-green.time"));
        for (int i = 0; i < (int)state.size(); i++) {
            if (state[i] == LINKSTATE_TL_YELLOW_MAJOR || state[i] == LINKSTATE_TL_YELLOW_MINOR) {
                state[i] = LINKSTATE_TL_RED;
            } else if (state[i] == LINKSTATE_TL_GREEN_MINOR) {
                state[i] = LINKSTATE_TL_GREEN_MAJOR;
            }
        }
    } else if (haveGreen) {
        // guess yellow state
        myEditedDef->setParticipantsInformation();
        duration = TIME2STEPS(myEditedDef->computeBrakingTime(oc.getFloat("tls.yellow.min-decel")));
        for (int i = 0; i < (int)state.size(); i++) {
            if (state[i] == LINKSTATE_TL_GREEN_MAJOR || state[i] == LINKSTATE_TL_GREEN_MINOR) {
                state[i] = LINKSTATE_TL_YELLOW_MINOR;
            }
        }
    } else if (haveYellow) {
        duration = TIME2STEPS(oc.isDefault("tls.allred.time") ? 2 :  oc.getInt("tls.allred.time"));
        // guess all-red state
        for (int i = 0; i < (int)state.size(); i++) {
            if (state[i] == LINKSTATE_TL_YELLOW_MAJOR || state[i] == LINKSTATE_TL_YELLOW_MINOR) {
                state[i] = LINKSTATE_TL_RED;
            }
        }
    }

    myEditedDef->getLogic()->addStep(duration, state, newIndex);
    myTLSPhases->getPhaseTable()->setCurrentItem(newIndex, 0);
    myTLSPhases->initPhaseTable(newIndex);
    myTLSPhases->getPhaseTable()->setFocus();
    return 1;
}


long
GNETLSEditorFrame::onCmdPhaseDelete(FXObject*, FXSelector, void*) {
    myTLSModifications->setHaveModifications(true);
    const int newRow = MAX2((int)0, (int)myTLSPhases->getPhaseTable()->getCurrentRow() - 1);
    myEditedDef->getLogic()->deletePhase(myTLSPhases->getPhaseTable()->getCurrentRow());
    myTLSPhases->initPhaseTable(newRow);
    myTLSPhases->getPhaseTable()->setFocus();
    return 1;
}


long
GNETLSEditorFrame::onCmdCleanup(FXObject*, FXSelector, void*) {
    myTLSModifications->setHaveModifications(myEditedDef->cleanupStates());
    myTLSPhases->initPhaseTable(0);
    myTLSPhases->getPhaseTable()->setFocus();
    return 1;
}


long
GNETLSEditorFrame::onCmdAddUnused(FXObject*, FXSelector, void*) {
    myEditedDef->getLogic()->setStateLength(
        myEditedDef->getLogic()->getNumLinks() + 1);
    myTLSModifications->setHaveModifications(true);
    myTLSPhases->initPhaseTable(0);
    myTLSPhases->getPhaseTable()->setFocus();
    return 1;
}


long
GNETLSEditorFrame::onCmdPhaseEdit(FXObject*, FXSelector, void* ptr) {
    /* @note: there is a bug when copying/pasting rows: when this handler is
     * called the value of the cell is not yet updated. This means you have to
     * click inside the cell and hit enter to actually update the value */
    FXTablePos* tp = (FXTablePos*)ptr;
    FXString value = myTLSPhases->getPhaseTable()->getItemText(tp->row, tp->col);
    const bool fixed = myEditedDef->getType() == TLTYPE_STATIC;
    if (tp->col == 0) {
        // duration edited
        if (GNEAttributeCarrier::canParse<double>(value.text())) {
            SUMOTime duration = getSUMOTime(value);
            if (duration > 0) {
                myEditedDef->getLogic()->setPhaseDuration(tp->row, duration);
                myTLSModifications->setHaveModifications(true);
                myTLSPhases->updateCycleDuration();
                return 1;
            }
        }
        // input error, reset value
        myTLSPhases->getPhaseTable()->setItemText(tp->row, 0, toString(STEPS2TIME(getPhases()[tp->row].duration)).c_str());
    } else if (!fixed && tp->col == 1) {
        // minDur edited
        if (GNEAttributeCarrier::canParse<double>(value.text())) {
            SUMOTime minDur = getSUMOTime(value);
            if (minDur > 0) {
                myEditedDef->getLogic()->setPhaseMinDuration(tp->row, minDur);
                myTLSModifications->setHaveModifications(true);
                return 1;
            }
        } else if (StringUtils::prune(value.text()).empty()) {
            myEditedDef->getLogic()->setPhaseMinDuration(tp->row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
            myTLSModifications->setHaveModifications(true);
            return 1;
        }
        // input error, reset value
        myTLSPhases->getPhaseTable()->setItemText(tp->row, 1, varDurString(getPhases()[tp->row].minDur).c_str());
    } else if (!fixed && tp->col == 2) {
        // minDur edited
        if (GNEAttributeCarrier::canParse<double>(value.text())) {
            SUMOTime maxDur = getSUMOTime(value);
            if (maxDur > 0) {
                myEditedDef->getLogic()->setPhaseMaxDuration(tp->row, maxDur);
                myTLSModifications->setHaveModifications(true);
                return 1;
            }
        } else if (StringUtils::prune(value.text()).empty()) {
            myEditedDef->getLogic()->setPhaseMaxDuration(tp->row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
            myTLSModifications->setHaveModifications(true);
            return 1;
        }
        // input error, reset value
        myTLSPhases->getPhaseTable()->setItemText(tp->row, 2, varDurString(getPhases()[tp->row].maxDur).c_str());
    } else {
        // state edited
        try {
            // insert phase with new step and delete the old phase
            myEditedDef->getLogic()->addStep(getPhases()[tp->row].duration, value.text(), tp->row);
            myEditedDef->getLogic()->deletePhase(tp->row + 1);
            myTLSModifications->setHaveModifications(true);
            onCmdPhaseSwitch(0, 0, 0);
        } catch (ProcessError&) {
            // input error, reset value
            myTLSPhases->getPhaseTable()->setItemText(tp->row, 1, getPhases()[tp->row].state.c_str());
        }
    }
    return 1;
}


void
GNETLSEditorFrame::cleanup() {
    if (myTLSJunction->getCurrentJunction()) {
        myTLSJunction->getCurrentJunction()->selectTLS(false);
    }
    // clean data structures
    myTLSJunction->setCurrentJunction(nullptr);
    myTLSModifications->setHaveModifications(false);
    delete myEditedDef;
    myEditedDef = nullptr;
    buildIinternalLanes(0); // only clears
    // clean up controls
    myTLSAttributes->clearTLSAttributes();
    myTLSPhases->initPhaseTable(); // only clears when there are no definitions
    myTLSPhases->hideCycleDuration();
    myTLSJunction->updateJunctionDescription();
}


void
GNETLSEditorFrame::buildIinternalLanes(NBTrafficLightDefinition* tlDef) {
    SUMORTree& rtree = myViewNet->getNet()->getVisualisationSpeedUp();
    // clean up previous objects
    for (auto it : myInternalLanes) {
        for (auto it_intLanes : it.second) {
            rtree.removeAdditionalGLObject(it_intLanes);
            delete it_intLanes;
        }
    }
    myInternalLanes.clear();
    // create new internal lanes
    if (tlDef != 0) {
        const int NUM_POINTS = 10;
        assert(myTLSJunction->getCurrentJunction());
        NBNode* nbn = myTLSJunction->getCurrentJunction()->getNBNode();
        std::string innerID = ":" + nbn->getID(); // see NWWriter_SUMO::writeInternalEdges
        const NBConnectionVector& links = tlDef->getControlledLinks();
        for (auto it : links) {
            int tlIndex = it.getTLIndex();
            PositionVector shape = it.getFrom()->getToNode()->computeInternalLaneShape(it.getFrom(), NBEdge::Connection(it.getFromLane(),
                                   it.getTo(), it.getToLane()), NUM_POINTS);
            GNEInternalLane* ilane = new GNEInternalLane(this, innerID + '_' + toString(tlIndex),  shape, tlIndex);
            rtree.addAdditionalGLObject(ilane);
            myInternalLanes[tlIndex].push_back(ilane);
        }
        for (auto c : nbn->getCrossings()) {
            if (c->tlLinkIndex2 > 0 && c->tlLinkIndex2 != c->tlLinkIndex) {
                // draw both directions
                PositionVector forward = c->shape;
                forward.move2side(c->width / 4);
                GNEInternalLane* ilane = new GNEInternalLane(this, c->id, forward, c->tlLinkIndex);
                rtree.addAdditionalGLObject(ilane);
                myInternalLanes[c->tlLinkIndex].push_back(ilane);

                PositionVector backward = c->shape.reverse();
                backward.move2side(c->width / 4);
                GNEInternalLane* ilane2 = new GNEInternalLane(this, c->id + "_r", backward, c->tlLinkIndex2);
                rtree.addAdditionalGLObject(ilane2);
                myInternalLanes[c->tlLinkIndex2].push_back(ilane2);
            } else {
                // draw only one lane for both directions
                GNEInternalLane* ilane = new GNEInternalLane(this, c->id, c->shape, c->tlLinkIndex);
                rtree.addAdditionalGLObject(ilane);
                myInternalLanes[c->tlLinkIndex].push_back(ilane);
            }
        }
    }
}


std::string
GNETLSEditorFrame::varDurString(SUMOTime dur) {
    return dur == NBTrafficLightDefinition::UNSPECIFIED_DURATION ? "   " : toString(STEPS2TIME(dur));
}


const std::vector<NBTrafficLightLogic::PhaseDefinition>&
GNETLSEditorFrame::getPhases() {
    return myEditedDef->getLogic()->getPhases();
}


void
GNETLSEditorFrame::handleChange(GNEInternalLane* lane) {
    myTLSModifications->setHaveModifications(true);
    if (myViewNet->changeAllPhases()) {
        const std::vector<NBTrafficLightLogic::PhaseDefinition>& phases = getPhases();
        for (int row = 0; row < (int)phases.size(); row++) {
            myEditedDef->getLogic()->setPhaseState(row, lane->getTLIndex(), lane->getLinkState());
        }
    } else {
        myEditedDef->getLogic()->setPhaseState(myTLSPhases->getPhaseTable()->getCurrentRow(), lane->getTLIndex(), lane->getLinkState());
    }
    myTLSPhases->initPhaseTable(myTLSPhases->getPhaseTable()->getCurrentRow());
    myTLSPhases->getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::handleMultiChange(GNELane* lane, FXObject* obj, FXSelector sel, void* eventData) {
    if (myEditedDef != 0) {
        myTLSModifications->setHaveModifications(true);
        const NBConnectionVector& links = myEditedDef->getControlledLinks();
        std::set<std::string> fromIDs;
        fromIDs.insert(lane->getMicrosimID());
        GNEEdge& edge = lane->getParentEdge();
        // if neither the lane nor its edge are selected, apply changes to the whole edge
        if (!edge.isAttributeCarrierSelected() && !lane->isAttributeCarrierSelected()) {
            for (auto it_lane : edge.getLanes()) {
                fromIDs.insert(it_lane->getMicrosimID());
            }
        } else {
            // if the edge is selected, apply changes to all lanes of all selected edges
            if (edge.isAttributeCarrierSelected()) {
                std::vector<GNEEdge*> edges = myViewNet->getNet()->retrieveEdges(true);
                for (auto it : edges) {
                    for (auto it_lane : it->getLanes()) {
                        fromIDs.insert(it_lane->getMicrosimID());
                    }
                }
            }
            // if the lane is selected, apply changes to all selected lanes
            if (lane->isAttributeCarrierSelected()) {
                std::vector<GNELane*> lanes = myViewNet->getNet()->retrieveLanes(true);
                for (auto it_lane : lanes) {
                    fromIDs.insert(it_lane->getMicrosimID());
                }
            }

        }
        // set new state for all connections from the chosen lane IDs
        for (auto it : links) {
            if (fromIDs.count(it.getFrom()->getLaneID(it.getFromLane())) > 0) {
                std::vector<GNEInternalLane*> lanes = myInternalLanes[it.getTLIndex()];
                for (auto it_lane : lanes) {
                    it_lane->onDefault(obj, sel, eventData);
                }
            }
        }
    }
}


bool
GNETLSEditorFrame::controlsEdge(GNEEdge& edge) const {
    if (myEditedDef != 0) {
        const NBConnectionVector& links = myEditedDef->getControlledLinks();
        for (auto it : links) {
            if (it.getFrom()->getID() == edge.getMicrosimID()) {
                return true;
            }
        }
    }
    return false;
}


SUMOTime
GNETLSEditorFrame::getSUMOTime(const FXString& string) {
    assert(GNEAttributeCarrier::canParse<double>(string.text()));
    return TIME2STEPS(GNEAttributeCarrier::parse<double>(string.text()));
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSAttributes - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSAttributes::TLSAttributes(GNETLSEditorFrame* TLSEditorParent) :
    FXGroupBox(TLSEditorParent->myContentFrame, "Traffic light Attributes", GUIDesignGroupBoxFrame),
    myTLSEditorParent(TLSEditorParent) {

    // create frame, label and textfield for name (By default disabled)
    FXHorizontalFrame* nameFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myNameLabel = new FXLabel(nameFrame, "ID", 0, GUIDesignLabelAttribute);
    myNameTextField = new FXTextField(nameFrame, GUIDesignTextFieldNCol, myTLSEditorParent, MID_GNE_TLSFRAME_SWITCH, GUIDesignTextField);
    myNameTextField->disable();

    // create frame, label and comboBox for Program (By default hidden)
    FXHorizontalFrame* programFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myProgramLabel = new FXLabel(programFrame, "Program", 0, GUIDesignLabelAttribute);
    myProgramComboBox = new FXComboBox(programFrame, GUIDesignComboBoxNCol, myTLSEditorParent, MID_GNE_TLSFRAME_SWITCH, GUIDesignComboBoxAttribute);
    myProgramComboBox->disable();

    // create frame, label and TextField for Offset (By default disabled)
    FXHorizontalFrame* offsetFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myOffsetLabel = new FXLabel(offsetFrame, "Offset", 0, GUIDesignLabelAttribute);
    myOffsetTextField = new FXTextField(offsetFrame, GUIDesignTextFieldNCol, myTLSEditorParent, MID_GNE_TLSFRAME_OFFSET, GUIDesignTextFieldReal);
    myOffsetTextField->disable();
}


GNETLSEditorFrame::TLSAttributes::~TLSAttributes() {}


void
GNETLSEditorFrame::TLSAttributes::initTLSAttributes(GNEJunction* junction) {
    assert(junction);
    myTLSDefinitions.clear();
    // enable name TextField
    myNameTextField->enable();
    // enable Offset
    myOffsetTextField->enable();
    // obtain TLSs
    for (auto it : junction->getNBNode()->getControllingTLS()) {
        myTLSDefinitions.push_back(it);
        myNameTextField->setText(it->getID().c_str());
        myNameTextField->enable();
        myProgramComboBox->appendItem(it->getProgramID().c_str());
        // enable TLS
        myTLSEditorParent->myTLSFile->enableTLSFile();
    }
    if (myTLSDefinitions.size() > 0) {
        myProgramComboBox->enable();
        myProgramComboBox->setCurrentItem(0);
        myProgramComboBox->setNumVisible(myProgramComboBox->getNumItems());
        myTLSEditorParent->onCmdDefSwitch(0, 0, 0);
        // enable TLS
        myTLSEditorParent->myTLSFile->enableTLSFile();
    }
}


void
GNETLSEditorFrame::TLSAttributes::clearTLSAttributes() {
    // clear definitions
    myTLSDefinitions.clear();
    // clear and disable name TextField
    myNameTextField->setText("");
    myNameTextField->disable();
    // clear and disable myProgramComboBox
    myProgramComboBox->clearItems();
    myProgramComboBox->disable();
    // clear and disable Offset TextField
    myOffsetTextField->setText("");
    myOffsetTextField->disable();
}


NBTrafficLightDefinition*
GNETLSEditorFrame::TLSAttributes::getCurrentTLSDefinition() const {
    return myTLSDefinitions.at(myProgramComboBox->getCurrentItem());
}


int
GNETLSEditorFrame::TLSAttributes::getNumberOfTLSDefinitions() const {
    return (int)myTLSDefinitions.size();
}


int
GNETLSEditorFrame::TLSAttributes::getNumberOfPrograms() const {
    return myProgramComboBox->getNumItems();
}


SUMOTime
GNETLSEditorFrame::TLSAttributes::getOffset() const {
    return getSUMOTime(myOffsetTextField->getText());
}


void
GNETLSEditorFrame::TLSAttributes::setOffset(SUMOTime offset) {
    myOffsetTextField->setText(toString(STEPS2TIME(offset)).c_str());
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSJunction - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSJunction::TLSJunction(GNETLSEditorFrame* TLSEditorParent) :
    FXGroupBox(TLSEditorParent->myContentFrame, "Junction", GUIDesignGroupBoxFrame),
    myTLSEditorParent(TLSEditorParent),
    myCurrentJunction(nullptr) {
    // Create frame for junction ID
    FXHorizontalFrame* junctionIDFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myLabelJunctionID = new FXLabel(junctionIDFrame, "Junction ID", 0, GUIDesignLabelAttribute);
    myTextFieldJunctionID = new FXTextField(junctionIDFrame, GUIDesignTextFieldNCol, this, MID_GNE_TLSFRAME_SELECT_JUNCTION, GUIDesignTextField);
    myTextFieldJunctionID->setEditable(false);
    // create frame for junction status
    FXHorizontalFrame* junctionIDStatus = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myLabelJunctionStatus = new FXLabel(junctionIDStatus, "Status", 0, GUIDesignLabelAttribute);
    myTextFieldJunctionStatus = new FXTextField(junctionIDStatus, GUIDesignTextFieldNCol, this, MID_GNE_TLSFRAME_UPDATE_STATUS, GUIDesignTextField);
    myTextFieldJunctionStatus->setEditable(false);
    // update junction description after creation
    updateJunctionDescription();
    // show TLS Junction
    show();
}


GNETLSEditorFrame::TLSJunction::~TLSJunction() {}


GNEJunction*
GNETLSEditorFrame::TLSJunction::getCurrentJunction() const {
    return myCurrentJunction;
}


void
GNETLSEditorFrame::TLSJunction::setCurrentJunction(GNEJunction* junction) {
    myCurrentJunction = junction;
}


void
GNETLSEditorFrame::TLSJunction::updateJunctionDescription() const {
    if (myCurrentJunction == 0) {
        myTextFieldJunctionID->setText("");
        myTextFieldJunctionStatus->setText("");
    } else {
        NBNode* nbn = myCurrentJunction->getNBNode();
        myTextFieldJunctionID->setText(nbn->getID().c_str());
        if (!nbn->isTLControlled()) {
            myTextFieldJunctionStatus->setText("uncontrolled");
        } else {
            myTextFieldJunctionStatus->setText(myTLSEditorParent->myTLSModifications->checkHaveModifications() ? "modified" : "unmodified");
        }
    }
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSDefinition - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSDefinition::TLSDefinition(GNETLSEditorFrame* TLSEditorParent) :
    FXGroupBox(TLSEditorParent->myContentFrame, "Traffic lights definition", GUIDesignGroupBoxFrame),
    myTLSEditorParent(TLSEditorParent) {
    // create create tlDef button
    myNewTLProgram = new FXButton(this, "Create TLS\t\tCreate a new traffic light program",
                                  GUIIconSubSys::getIcon(ICON_MODETLS), TLSEditorParent, MID_GNE_TLSFRAME_CREATE, GUIDesignButton);
    // create delete tlDef button
    myDeleteTLProgram = new FXButton(this, "Delete TLS\t\tDelete a traffic light program. If all programs are deleted the junction turns into a priority junction.",
                                     GUIIconSubSys::getIcon(ICON_REMOVE), TLSEditorParent, MID_GNE_TLSFRAME_DELETE, GUIDesignButton);
    // show TLS TLSDefinition
    show();
}


GNETLSEditorFrame::TLSDefinition::~TLSDefinition() {}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSPhases - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSPhases::TLSPhases(GNETLSEditorFrame* TLSEditorParent) :
    FXGroupBox(TLSEditorParent->myContentFrame, "Phases", GUIDesignGroupBoxFrame),
    myTLSEditorParent(TLSEditorParent),
    myTableFont(new FXFont(getApp(), "Courier New", 9)) {

    // create and configure phase table
    myTableScroll = new FXScrollWindow(this, LAYOUT_FILL_X | LAYOUT_FIX_HEIGHT);
    myPhaseTable = new FXTable(myTableScroll, myTLSEditorParent, MID_GNE_TLSFRAME_PHASE_TABLE, GUIDesignTableLimitedHeight);
    myPhaseTable->setColumnHeaderMode(LAYOUT_FIX_HEIGHT);
    myPhaseTable->setColumnHeaderHeight(0);
    myPhaseTable->setRowHeaderMode(LAYOUT_FIX_WIDTH);
    myPhaseTable->setRowHeaderWidth(0);
    myPhaseTable->hide();
    myPhaseTable->setFont(myTableFont);
    myPhaseTable->setHelpText("phase duration in seconds | phase state");

    // create total duration info label
    myCycleDuration = new FXLabel(this, "", 0, GUIDesignLabelLeft);

    // create new phase button
    myInsertDuplicateButton = new FXButton(this, "Insert Phase\t\tInsert new phase after the selected phase. The new state is deduced from the selected phase.", 0, myTLSEditorParent, MID_GNE_TLSFRAME_PHASE_CREATE, GUIDesignButton);

    // create delete phase button
    myDeleteSelectedPhaseButton = new FXButton(this, "Delete Phase\t\tDelete selected phase", 0, myTLSEditorParent, MID_GNE_TLSFRAME_PHASE_DELETE, GUIDesignButton);

    // create cleanup states button
    new FXButton(this, "Cleanup States\t\tClean unused states from all phase.", 0, myTLSEditorParent, MID_GNE_TLSFRAME_CLEANUP, GUIDesignButton);

    // add unused states button
    new FXButton(this, "Add Unused States\t\tExtend the state vector for all phases by one entry.", 0, myTLSEditorParent, MID_GNE_TLSFRAME_ADDUNUSED, GUIDesignButton);

    // show TLSFile
    show();
}


GNETLSEditorFrame::TLSPhases::~TLSPhases() {
    delete myTableFont;
}


FXTable*
GNETLSEditorFrame::TLSPhases::getPhaseTable() const {
    return myPhaseTable;
}


void
GNETLSEditorFrame::TLSPhases::initPhaseTable(int index) {
    myPhaseTable->setVisibleRows(1);
    myPhaseTable->setVisibleColumns(2);
    myPhaseTable->hide();
    if (myTLSEditorParent->myTLSAttributes->getNumberOfTLSDefinitions() > 0) {
        const bool fixed = myTLSEditorParent->myEditedDef->getType() == TLTYPE_STATIC;
        const std::vector<NBTrafficLightLogic::PhaseDefinition>& phases = myTLSEditorParent->getPhases();
        myPhaseTable->setTableSize((int)phases.size(), fixed ? 2 : 4);
        myPhaseTable->setVisibleRows((int)phases.size());
        myPhaseTable->setVisibleColumns(fixed ? 2 : 4);
        for (int row = 0; row < (int)phases.size(); row++) {
            myPhaseTable->setItemText(row, 0, toString(STEPS2TIME(phases[row].duration)).c_str());
            if (!fixed) {
                myPhaseTable->setItemText(row, 1, varDurString(phases[row].minDur).c_str());
                myPhaseTable->setItemText(row, 2, varDurString(phases[row].maxDur).c_str());
            }
            myPhaseTable->setItemText(row, fixed ? 1 : 3, phases[row].state.c_str());
            myPhaseTable->getItem(row, 1)->setJustify(FXTableItem::LEFT);
        }
        myPhaseTable->fitColumnsToContents(0, fixed ? 2 : 4);
        myPhaseTable->setHeight((int)phases.size() * 21); // experimental
        myPhaseTable->setCurrentItem(index, 0);
        myPhaseTable->selectRow(index, true);
        myPhaseTable->show();
        myPhaseTable->setFocus();
        myTableScroll->setHeight(myPhaseTable->getHeight() + 10);
    }
    update();
}


void
GNETLSEditorFrame::TLSPhases::showCycleDuration() {
    myCycleDuration->show();
}


void
GNETLSEditorFrame::TLSPhases::hideCycleDuration() {
    myCycleDuration->hide();
}

void
GNETLSEditorFrame::TLSPhases::updateCycleDuration() {
    SUMOTime cycleDuration = 0;
    for (auto it : myTLSEditorParent->getPhases()) {
        cycleDuration += it.duration;
    }
    std::string text = "Cycle time: " + toString(STEPS2TIME(cycleDuration));
    myCycleDuration->setText(text.c_str());
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSModifications - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSModifications::TLSModifications(GNETLSEditorFrame* TLSEditorParent) :
    FXGroupBox(TLSEditorParent->myContentFrame, "Modifications", GUIDesignGroupBoxFrame),
    myTLSEditorParent(TLSEditorParent),
    myHaveModifications(false) {
    // create save modifications button
    mySaveModificationsButtons = new FXButton(this, "Save\t\tSave program modifications (Enter)",
            GUIIconSubSys::getIcon(ICON_OK), myTLSEditorParent, MID_OK, GUIDesignButton);
    // create discard modifications buttons
    myDiscardModificationsButtons = new FXButton(this, "Cancel\t\tDiscard program modifications (Esc)",
            GUIIconSubSys::getIcon(ICON_CANCEL), myTLSEditorParent, MID_CANCEL, GUIDesignButton);
    // show TLSModifications
    show();
}


GNETLSEditorFrame::TLSModifications::~TLSModifications() {}


bool
GNETLSEditorFrame::TLSModifications::checkHaveModifications() const {
    return myHaveModifications;
}


void
GNETLSEditorFrame::TLSModifications::setHaveModifications(bool value) {
    myHaveModifications = value;
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSFile - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSFile::TLSFile(GNETLSEditorFrame* TLSEditorParent) :
    FXGroupBox(TLSEditorParent->myContentFrame, "TLS Program", GUIDesignGroupBoxFrame),
    SUMOSAXHandler("TLS-Program"),
    myTLSEditorParent(TLSEditorParent) {
    // create create tlDef button
    myLoadTLSProgramButton = new FXButton(this, "Load TLS Program", 0, this, MID_GNE_TLSFRAME_LOAD_PROGRAM, GUIDesignButton);
    // create create tlDef button
    mySaveTLSProgramButton = new FXButton(this, "Save TLS Program", 0, this, MID_GNE_TLSFRAME_SAVE_PROGRAM, GUIDesignButton);
    // by default TLSFile is disabled
    disableTLSFile();
    // show TLSFile
    show();
}


GNETLSEditorFrame::TLSFile::~TLSFile() {}


void
GNETLSEditorFrame::TLSFile::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    bool ok = true;
    switch (element) {
        case SUMO_TAG_TLLOGIC: {
            std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
            if (myTLSEditorParent->getViewNet()->getNet()->retrieveJunction(id, false) == nullptr) {
                WRITE_ERROR("ERROR, Junction doesn't exist");
            } else {
                std::string programID = attrs.getOpt<std::string>(SUMO_ATTR_PROGRAMID, id.c_str(), ok, "<unknown>");
                TrafficLightType type = TLTYPE_STATIC;
                std::string typeS = attrs.get<std::string>(SUMO_ATTR_TYPE, 0, ok);
                if (SUMOXMLDefinitions::TrafficLightTypes.hasString(typeS)) {
                    type = SUMOXMLDefinitions::TrafficLightTypes.get(typeS);
                } else {
                    WRITE_ERROR("Traffic light '" + id + "' has unknown type '" + typeS + "'.");
                }
                const SUMOTime offset = attrs.getOptSUMOTimeReporting(SUMO_ATTR_OFFSET, id.c_str(), ok, 0);
                // avoid to insert duplicated TLSs
                myLoadedTLS.push_back(new NBLoadedSUMOTLDef(id, programID, offset, type));
            }
            break;
        }
        case SUMO_TAG_PHASE: {
            if (myLoadedTLS.size() > 0) {
                double duration = attrs.get<double>(SUMO_ATTR_DURATION, 0, ok);
                std::string state = attrs.get<std::string>(SUMO_ATTR_STATE, 0, ok);
                myLoadedTLS.back()->getLogic()->addStep(TIME2STEPS(duration), state);
                break;
            }
        }
        default:
            break;
    }
}


void
GNETLSEditorFrame::TLSFile::enableTLSFile() {
    // enable buttons
    myLoadTLSProgramButton->enable();
    mySaveTLSProgramButton->enable();
}


void
GNETLSEditorFrame::TLSFile::disableTLSFile() {
    // disable buttons
    myLoadTLSProgramButton->disable();
    mySaveTLSProgramButton->disable();
}


void
GNETLSEditorFrame::TLSFile::clearLoadedTLS() {
    for (auto i : myLoadedTLS) {
        delete i;
    }
    myLoadedTLS.clear();
}


const std::vector<NBLoadedSUMOTLDef*>&
GNETLSEditorFrame::TLSFile::getLoadedTLS() const {
    return myLoadedTLS;
}


bool
GNETLSEditorFrame::TLSFile::checkTLSValids() {
    std::vector<NBLoadedSUMOTLDef*> definitionsJunctionsWithoutTL;
    for (auto i : myLoadedTLS) {
        if (myTLSEditorParent->getViewNet()->getNet()->retrieveJunction(i->getID())->getNBNode()->isTLControlled() == false) {
            definitionsJunctionsWithoutTL.push_back(i);
        }
    }
    // check if there is definitions loaded with an uncontrolled junction asociated
    if (definitionsJunctionsWithoutTL.size() > 0) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening question FXMessageBox 'create TLS in uncontrolled junctions'");
        // open question box
        FXuint answer = FXMessageBox::question(myTLSEditorParent, MBOX_YES_NO_CANCEL,
                                               "Create TLS", "%s",
                                               ("There is " + toString(definitionsJunctionsWithoutTL.size()) + " TLS without controlled junction.\nDo you want to create TLSs in uncontrolled Junctions?").c_str());
        if (answer == MBOX_CLICKED_YES) { //1:yes, 2:no, 4:esc/cancel
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'create TLS in uncontrolled junctions' with 'Yes'");
            // convert Junctions in TLS
            for (auto i : definitionsJunctionsWithoutTL) {
                GNEJunction* junction = myTLSEditorParent->getViewNet()->getNet()->retrieveJunction(i->getID());
                junction->setAttribute(SUMO_ATTR_TYPE, "traffic_light", myTLSEditorParent->getViewNet()->getUndoList());
            }
        } else if (answer == MBOX_CLICKED_NO) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'create TLS in uncontrolled junctions' with 'No'");
            // delete loaded TLSs without associated controlled Junction
            for (auto i : definitionsJunctionsWithoutTL) {
                myLoadedTLS.erase(std::find(myLoadedTLS.begin(), myLoadedTLS.end(), i));
                delete i;
            }
        } else {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'create TLS in uncontrolled junctions' with 'Cancel'");
            // return false to abort loading of TLSs
            return false;
        }
    }

    // return true to continue loading TLSs
    return true;
}


long
GNETLSEditorFrame::TLSFile::onCmdLoadTLSProgram(FXObject*, FXSelector, void*) {
    FXFileDialog opendialog(this, "Load TLS Program");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODETLS));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("*.xml");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        // first clear loaded phases
        clearLoadedTLS();
        // run parser
        XMLSubSys::runParser(*this, opendialog.getFilename().text(), true);

        myTLSEditorParent->myEditedDef->cleanupStates();

        // check that only a phase was loaded
        /* for(auto i : myLoadedTLS) {
            if(i->getID() == myTLSEditorParent->myTLSJunction->getCurrentJunction()->getID()) {
                myTLSEditorParent->myEditedDef->getProgramID();

                for (auto j : i.second) {
                    myTLSEditorParent->myEditedDef->getLogic()->addStep(j.duration, j.state, j.minDur, j.maxDur);
                }

            }
        }
        */
        myTLSEditorParent->myTLSPhases->initPhaseTable();
        myTLSEditorParent->myTLSModifications->setHaveModifications(true);
    }
    return 0;
}


long
GNETLSEditorFrame::TLSFile::onCmdSaveTLSProgram(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(this,
                    "Save TLS Program as", ".xml",
                    GUIIconSubSys::getIcon(ICON_MODETLS),
                    gCurrentFolder);
    if (file == "") {
        return 1;
    }
    OutputDevice& device = OutputDevice::getDevice(file.text());

    // save program
    device.openTag(SUMO_TAG_TLLOGIC);
    device.writeAttr(SUMO_ATTR_ID, myTLSEditorParent->myEditedDef->getLogic()->getID());
    device.writeAttr(SUMO_ATTR_TYPE, myTLSEditorParent->myEditedDef->getLogic()->getType());
    device.writeAttr(SUMO_ATTR_PROGRAMID, myTLSEditorParent->myEditedDef->getLogic()->getProgramID());
    device.writeAttr(SUMO_ATTR_OFFSET, writeSUMOTime(myTLSEditorParent->myEditedDef->getLogic()->getOffset()));
    // write the phases
    const bool varPhaseLength = myTLSEditorParent->myEditedDef->getLogic()->getType() != TLTYPE_STATIC;
    const std::vector<NBTrafficLightLogic::PhaseDefinition>& phases = myTLSEditorParent->myEditedDef->getLogic()->getPhases();
    for (auto j : phases) {
        device.openTag(SUMO_TAG_PHASE);
        device.writeAttr(SUMO_ATTR_DURATION, writeSUMOTime(j.duration));
        device.writeAttr(SUMO_ATTR_STATE, j.state);
        if (varPhaseLength) {
            if (j.minDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
                device.writeAttr(SUMO_ATTR_MINDURATION, writeSUMOTime(j.minDur));
            }
            if (j.maxDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
                device.writeAttr(SUMO_ATTR_MAXDURATION, writeSUMOTime(j.maxDur));
            }
        }
        device.closeTag();
    }
    device.close();
    return 1;
}


std::string
GNETLSEditorFrame::TLSFile::writeSUMOTime(SUMOTime steps) {
    double time = STEPS2TIME(steps);
    if (time == std::floor(time)) {
        return toString(int(time));
    } else {
        return toString(time);
    }
}
/****************************************************************************/
