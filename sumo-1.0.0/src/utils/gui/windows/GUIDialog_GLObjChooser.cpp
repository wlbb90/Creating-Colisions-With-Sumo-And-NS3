/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GUIDialog_GLObjChooser.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// Class for the window that allows to choose a street, junction or vehicle
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <vector>
#include <fxkeys.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/windows/GUIGlChildWindow.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/globjects/GUIGlObject_AbstractAdd.h>
#include "GUIDialog_GLObjChooser.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GUIDialog_GLObjChooser) GUIDialog_GLObjChooserMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSER_CENTER, GUIDialog_GLObjChooser::onCmdCenter),
    FXMAPFUNC(SEL_COMMAND,  MID_CANCEL,         GUIDialog_GLObjChooser::onCmdClose),
    FXMAPFUNC(SEL_CHANGED,  MID_CHOOSER_TEXT,   GUIDialog_GLObjChooser::onChgText),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSER_TEXT,   GUIDialog_GLObjChooser::onCmdText),
    FXMAPFUNC(SEL_KEYPRESS, MID_CHOOSER_LIST,   GUIDialog_GLObjChooser::onListKeyPress),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSER_FILTER, GUIDialog_GLObjChooser::onCmdFilter),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_INVERT,  GUIDialog_GLObjChooser::onCmdToggleSelection),
};

FXIMPLEMENT(GUIDialog_GLObjChooser, FXMainWindow, GUIDialog_GLObjChooserMap, ARRAYNUMBER(GUIDialog_GLObjChooserMap))


// ===========================================================================
// method definitions
// ===========================================================================
GUIDialog_GLObjChooser::GUIDialog_GLObjChooser(GUIGlChildWindow* parent, FXIcon* icon, const FXString& title, const std::vector<GUIGlID>& ids, GUIGlObjectStorage& /*glStorage*/) :
    FXMainWindow(parent->getApp(), title, icon, NULL, GUIDesignChooserDialog),
    myParent(parent) {
    FXHorizontalFrame* hbox = new FXHorizontalFrame(this, GUIDesignAuxiliarFrame);
    // build the list
    FXVerticalFrame* layoutLeft = new FXVerticalFrame(hbox, GUIDesignChooserLayoutLeft);
    myTextEntry = new FXTextField(layoutLeft, 0, this, MID_CHOOSER_TEXT, GUIDesignChooserTextField);
    FXVerticalFrame* layoutList = new FXVerticalFrame(layoutLeft, GUIDesignChooserLayoutList);
    myList = new FXList(layoutList, this, MID_CHOOSER_LIST, GUIDesignChooserListSingle);
    refreshList(ids);
    // build the buttons
    FXVerticalFrame* layoutRight = new FXVerticalFrame(hbox, GUIDesignChooserLayoutRight);
    myCenterButton = new FXButton(layoutRight, "Center\t\t", GUIIconSubSys::getIcon(ICON_RECENTERVIEW), this, MID_CHOOSER_CENTER, GUIDesignChooserButtons);
    new FXHorizontalSeparator(layoutRight, GUIDesignHorizontalSeparator);
    new FXButton(layoutRight, "&Hide Unselected\t\t", GUIIconSubSys::getIcon(ICON_FLAG), this, MID_CHOOSER_FILTER, GUIDesignChooserButtons);
    new FXButton(layoutRight, "&Select/deselect\tSelect/deselect current object\t", GUIIconSubSys::getIcon(ICON_FLAG), this, MID_CHOOSEN_INVERT, GUIDesignChooserButtons);
    new FXHorizontalSeparator(layoutRight, GUIDesignHorizontalSeparator);
    new FXButton(layoutRight, "&Close\t\t", GUIIconSubSys::getIcon(ICON_NO), this, MID_CANCEL, GUIDesignChooserButtons);

    myParent->getParent()->addChild(this);
    // create and show dialog
    create();
    show();
}


GUIDialog_GLObjChooser::~GUIDialog_GLObjChooser() {
    myParent->getParent()->removeChild(this);
}


void
GUIDialog_GLObjChooser::show() {
    FXMainWindow::show();
    myTextEntry->setFocus();
}


long
GUIDialog_GLObjChooser::onCmdCenter(FXObject*, FXSelector, void*) {
    int selected = myList->getCurrentItem();
    if (selected >= 0) {
        myParent->setView(*static_cast<GUIGlID*>(myList->getItemData(selected)));
    }
    return 1;
}


long
GUIDialog_GLObjChooser::onCmdClose(FXObject*, FXSelector, void*) {
    close(true);
    return 1;
}


long
GUIDialog_GLObjChooser::onChgText(FXObject*, FXSelector, void*) {
    int id = myList->findItem(myTextEntry->getText(), -1, SEARCH_PREFIX);
    if (id < 0) {
        if (myList->getNumItems() > 0) {
            myList->deselectItem(myList->getCurrentItem());
        }
        myCenterButton->disable();
        return 1;
    }
    myList->deselectItem(myList->getCurrentItem());
    myList->makeItemVisible(id);
    myList->selectItem(id);
    myList->setCurrentItem(id, true);
    myCenterButton->enable();
    return 1;
}


long
GUIDialog_GLObjChooser::onCmdText(FXObject*, FXSelector, void*) {
    int current = myList->getCurrentItem();
    if (current >= 0 && myList->isItemSelected(current)) {
        myParent->setView(*static_cast<GUIGlID*>(myList->getItemData(current)));
    }
    return 1;
}



long
GUIDialog_GLObjChooser::onListKeyPress(FXObject*, FXSelector, void* ptr) {
    FXEvent* event = (FXEvent*)ptr;
    switch (event->code) {
        case KEY_Return:
            onCmdText(0, 0, 0);
            break;
        default:
            break;
    }
    return 1;
}


long
GUIDialog_GLObjChooser::onCmdFilter(FXObject*, FXSelector, void*) {
    FXIcon* flag = GUIIconSubSys::getIcon(ICON_FLAG);
    std::vector<GUIGlID> selectedGlIDs;
    const int numItems = myList->getNumItems();
    for (int i = 0; i < numItems; i++) {
        const GUIGlID glID = *static_cast<GUIGlID*>(myList->getItemData(i));
        if (myList->getItemIcon(i) == flag) {
            selectedGlIDs.push_back(glID);
        }
    }
    refreshList(selectedGlIDs);
    return 1;
}


void
GUIDialog_GLObjChooser::refreshList(const std::vector<GUIGlID>& ids) {
    myList->clearItems();
    for (auto i : ids) {
        GUIGlObject* o = GUIGlObjectStorage::gIDStorage.getObjectBlocking(i);
        if (o == 0) {
            continue;
        }
        const std::string& name = o->getMicrosimID();
        bool selected = myParent->isSelected(o);
        FXIcon* icon = selected ? GUIIconSubSys::getIcon(ICON_FLAG) : 0;
        myIDs.insert(o->getGlID());
        myList->appendItem(name.c_str(), icon, (void*) & (*myIDs.find(o->getGlID())));
        GUIGlObjectStorage::gIDStorage.unblockObject(i);
    }
    myList->update();
}


long
GUIDialog_GLObjChooser::onCmdToggleSelection(FXObject*, FXSelector, void*) {
    FXIcon* flag = GUIIconSubSys::getIcon(ICON_FLAG);
    int i = myList->getCurrentItem();
    if (i >= 0) {
        toggleSelection(i);
        if (myList->getItemIcon(i) == flag) {
            myList->setItemIcon(i, 0);
        } else {
            myList->setItemIcon(i, flag);
        }
    }
    myList->update();
    myParent->getView()->update();
    return 1;
}


void
GUIDialog_GLObjChooser::toggleSelection(int listIndex) {
    GUIGlID* glID = static_cast<GUIGlID*>(myList->getItemData(listIndex));
    gSelected.toggleSelection(*glID);
}




/****************************************************************************/

