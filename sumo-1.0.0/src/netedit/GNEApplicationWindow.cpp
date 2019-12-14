/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEApplicationWindow.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
/// @version $Id$
///
// The main window of Netedit (adapted from GUIApplicationWindow)
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <string>
#include <sstream>
#include <algorithm>

#include <utils/common/ToString.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/foxtools/FXLinkLabel.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/options/OptionsCont.h>
#include <utils/gui/div/GUIMessageWindow.h>
#include <utils/gui/div/GUIDialog_GLChosenEditor.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GUIIOGlobals.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/div/GUIUserIO.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/events/GUIEvent_Message.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/images/GUITextureSubSys.h>
#include <utils/gui/settings/GUICompleteSchemeStorage.h>
#include <utils/gui/settings/GUISettingsHandler.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <netimport/NIFrame.h>
#include <netbuild/NBFrame.h>
#include <netwrite/NWFrame.h>
#include <utils/common/SystemFrame.h>
#include <netimport/NIImporter_SUMO.h>
#include <netedit/frames/GNETLSEditorFrame.h>
#include <netedit/dialogs/GNEDialog_About.h>
#include <utils/gui/windows/GUIDialog_Options.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNEJunction.h>
#include <netedit/additionals/GNEPOI.h>
#include <netedit/additionals/GNEAdditionalHandler.h>

#include "GNEApplicationWindow.h"
#include "GNELoadThread.h"
#include "GNEEvent_NetworkLoaded.h"
#include "GNEViewParent.h"
#include "GNEViewNet.h"
#include "GNENet.h"
#include "GNEUndoList.h"


// ===========================================================================
// FOX-declarations
// ===========================================================================
FXDEFMAP(GNEApplicationWindow) GNEApplicationWindowMap[] = {
    // quit calls
    FXMAPFUNC(SEL_COMMAND,  MID_QUIT,                                       GNEApplicationWindow::onCmdQuit),
    FXMAPFUNC(SEL_SIGNAL,   MID_QUIT,                                       GNEApplicationWindow::onCmdQuit),
    FXMAPFUNC(SEL_CLOSE,    MID_WINDOW,                                     GNEApplicationWindow::onCmdQuit),

    // toolbar file
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_NEWNETWORK,                 GNEApplicationWindow::onCmdNewNetwork),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_NEWNETWORK,                 GNEApplicationWindow::onUpdOpen),
    FXMAPFUNC(SEL_COMMAND,  MID_OPEN_NETWORK,                               GNEApplicationWindow::onCmdOpenNetwork),
    FXMAPFUNC(SEL_UPDATE,   MID_OPEN_NETWORK,                               GNEApplicationWindow::onUpdOpen),
    FXMAPFUNC(SEL_COMMAND,  MID_OPEN_CONFIG,                                GNEApplicationWindow::onCmdOpenConfiguration),
    FXMAPFUNC(SEL_UPDATE,   MID_OPEN_CONFIG,                                GNEApplicationWindow::onUpdOpen),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_OPENFOREIGN,                GNEApplicationWindow::onCmdOpenForeign),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_OPENFOREIGN,                GNEApplicationWindow::onUpdOpen),
    FXMAPFUNC(SEL_COMMAND,  MID_OPEN_SHAPES,                                GNEApplicationWindow::onCmdOpenShapes),
    FXMAPFUNC(SEL_UPDATE,   MID_OPEN_SHAPES,                                GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_OPEN_ADDITIONALS,                           GNEApplicationWindow::onCmdOpenAdditionals),
    FXMAPFUNC(SEL_UPDATE,   MID_OPEN_ADDITIONALS,                           GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_OPEN_TLSPROGRAMS,                           GNEApplicationWindow::onCmdOpenTLSPrograms),
    FXMAPFUNC(SEL_UPDATE,   MID_OPEN_TLSPROGRAMS,                           GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_RECENTFILE,                                 GNEApplicationWindow::onCmdOpenRecent),
    FXMAPFUNC(SEL_UPDATE,   MID_RECENTFILE,                                 GNEApplicationWindow::onUpdOpen),
    FXMAPFUNC(SEL_COMMAND,  MID_RELOAD,                                     GNEApplicationWindow::onCmdReload),
    FXMAPFUNC(SEL_UPDATE,   MID_RELOAD,                                     GNEApplicationWindow::onUpdReload),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVENETWORK,                GNEApplicationWindow::onCmdSaveNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVENETWORK,                GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVENETWORK_AS,             GNEApplicationWindow::onCmdSaveAsNetwork),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_SAVENETWORK_AS,             GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVEPLAINXML,               GNEApplicationWindow::onCmdSaveAsPlainXML),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_SAVEPLAINXML,               GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVEJOINED,                 GNEApplicationWindow::onCmdSaveJoined),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_SAVEJOINED,                 GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVESHAPES,                 GNEApplicationWindow::onCmdSaveShapes),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVESHAPES_AS,              GNEApplicationWindow::onCmdSaveShapesAs),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVEADDITIONALS,            GNEApplicationWindow::onCmdSaveAdditionals),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVEADDITIONALS_AS,         GNEApplicationWindow::onCmdSaveAdditionalsAs),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS,            GNEApplicationWindow::onCmdSaveTLSPrograms),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS,            GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS_AS,         GNEApplicationWindow::onCmdSaveTLSProgramsAs),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS_AS,         GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_CLOSE,                                      GNEApplicationWindow::onCmdClose),
    FXMAPFUNC(SEL_UPDATE,   MID_CLOSE,                                      GNEApplicationWindow::onUpdNeedsNetwork),

    // Toolbar edit
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_CREATE_EDGE,                    GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_CREATE_EDGE,                    GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_MOVE,                           GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_MOVE,                           GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_DELETE,                         GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_DELETE,                         GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_INSPECT,                        GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_INSPECT,                        GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_SELECT,                         GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_SELECT,                         GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_CONNECT,                        GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_CONNECT,                        GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_TLS,                            GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_TLS,                            GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_ADDITIONAL,                     GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_ADDITIONAL,                     GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_CROSSING,                       GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_CROSSING,                       GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_POLYGON,                        GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_POLYGON,                        GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SETMODE_PROHIBITION,                    GNEApplicationWindow::onCmdSetMode),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_SETMODE_PROHIBITION,                    GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_EDITVIEWSCHEME,                             GNEApplicationWindow::onCmdEditViewScheme),
    FXMAPFUNC(SEL_UPDATE,   MID_EDITVIEWSCHEME,                             GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_EDITVIEWPORT,                               GNEApplicationWindow::onCmdEditViewport),
    FXMAPFUNC(SEL_UPDATE,   MID_EDITVIEWPORT,                               GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_SUMOGUI,                                    GNEApplicationWindow::onCmdOpenSUMOGUI),
    FXMAPFUNC(SEL_UPDATE,   MID_SUMOGUI,                                    GNEApplicationWindow::onUpdNeedsNetwork),

    // Toolbar processing
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROCESSING_COMPUTEJUNCTIONS,            GNEApplicationWindow::onCmdComputeJunctions),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_PROCESSING_COMPUTEJUNCTIONS,            GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROCESSING_COMPUTEJUNCTIONS_VOLATILE,   GNEApplicationWindow::onCmdComputeJunctionsVolatile),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_PROCESSING_COMPUTEJUNCTIONS_VOLATILE,   GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROCESSING_CLEANJUNCTIONS,              GNEApplicationWindow::onCmdCleanJunctions),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_PROCESSING_CLEANJUNCTIONS,              GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROCESSING_JOINJUNCTIONS,               GNEApplicationWindow::onCmdJoinJunctions),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_PROCESSING_JOINJUNCTIONS,               GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROCESSING_CLEANINVALIDCROSSINGS,       GNEApplicationWindow::onCmdCleanInvalidCrossings),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_PROCESSING_CLEANINVALIDCROSSINGS,       GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROCESSING_OPTIONS,                     GNEApplicationWindow::onCmdOptions),

    // Toolbar locate
    FXMAPFUNC(SEL_COMMAND,  MID_LOCATEJUNCTION,                             GNEApplicationWindow::onCmdLocate),
    FXMAPFUNC(SEL_UPDATE,   MID_LOCATEJUNCTION,                             GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_LOCATEEDGE,                                 GNEApplicationWindow::onCmdLocate),
    FXMAPFUNC(SEL_UPDATE,   MID_LOCATEEDGE,                                 GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_LOCATETLS,                                  GNEApplicationWindow::onCmdLocate),
    FXMAPFUNC(SEL_UPDATE,   MID_LOCATETLS,                                  GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_LOCATEADD,                                  GNEApplicationWindow::onCmdLocate),
    FXMAPFUNC(SEL_UPDATE,   MID_LOCATEADD,                                  GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_LOCATEPOI,                                  GNEApplicationWindow::onCmdLocate),
    FXMAPFUNC(SEL_UPDATE,   MID_LOCATEPOI,                                  GNEApplicationWindow::onUpdNeedsNetwork),
    FXMAPFUNC(SEL_COMMAND,  MID_LOCATEPOLY,                                 GNEApplicationWindow::onCmdLocate),
    FXMAPFUNC(SEL_UPDATE,   MID_LOCATEPOLY,                                 GNEApplicationWindow::onUpdNeedsNetwork),

    // toolbar windows
    FXMAPFUNC(SEL_COMMAND,  MID_CLEARMESSAGEWINDOW,                         GNEApplicationWindow::onCmdClearMsgWindow),

    // toolbar help
    FXMAPFUNC(SEL_COMMAND,  MID_ABOUT,                                      GNEApplicationWindow::onCmdAbout),

    // key events
    FXMAPFUNC(SEL_KEYPRESS,     0,                                          GNEApplicationWindow::onKeyPress),
    FXMAPFUNC(SEL_KEYRELEASE,   0,                                          GNEApplicationWindow::onKeyRelease),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_HOTKEY_ESC,                             GNEApplicationWindow::onCmdAbort),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_HOTKEY_DEL,                             GNEApplicationWindow::onCmdDel),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_HOTKEY_ENTER,                           GNEApplicationWindow::onCmdEnter),

    // threads events
    FXMAPFUNC(FXEX::SEL_THREAD_EVENT, ID_LOADTHREAD_EVENT,                  GNEApplicationWindow::onLoadThreadEvent),
    FXMAPFUNC(FXEX::SEL_THREAD,       ID_LOADTHREAD_EVENT,                  GNEApplicationWindow::onLoadThreadEvent),

    // Other
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_HOTKEY_FOCUSFRAME,                      GNEApplicationWindow::onCmdFocusFrame),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_HOTKEY_TOOGLE_GRID,                     GNEApplicationWindow::onCmdToogleGrid),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,                                       GNEApplicationWindow::onCmdHelp),
    FXMAPFUNC(SEL_COMMAND,  MID_EDITVIEWPORT,                               GNEApplicationWindow::onCmdEditViewport),
    FXMAPFUNC(SEL_CLIPBOARD_REQUEST, 0,                                     GNEApplicationWindow::onClipboardRequest),
};

// Object implementation
FXIMPLEMENT(GNEApplicationWindow, FXMainWindow, GNEApplicationWindowMap, ARRAYNUMBER(GNEApplicationWindowMap))

// ===========================================================================
// member method definitions
// ===========================================================================
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4355)
#endif
GNEApplicationWindow::GNEApplicationWindow(FXApp* a, const std::string& configPattern) :
    GUIMainWindow(a),
    myLoadThread(0),
    myAmLoading(false),
    myRecentNets(a, "nets"),
    myConfigPattern(configPattern),
    hadDependentBuild(false),
    myNet(nullptr),
    myUndoList(new GNEUndoList(this)),
    myTitlePrefix("NETEDIT " VERSION_STRING) {
    // init icons
    GUIIconSubSys::initIcons(a);
    // init Textures
    GUITextureSubSys::initTextures(a);
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif


void
GNEApplicationWindow::dependentBuild() {
    // do this not twice
    if (hadDependentBuild) {
        WRITE_ERROR("DEBUG: GNEApplicationWindow::dependentBuild called twice");
        return;
    }
    hadDependentBuild = true;
    setTarget(this);
    setSelector(MID_WINDOW);
    // build menu bar
    myMenuBarDrag = new FXToolBarShell(this, GUIDesignToolBarShell3);
    myMenuBar = new FXMenuBar(myTopDock, myMenuBarDrag, GUIDesignBar);
    new FXToolBarGrip(myMenuBar, myMenuBar, FXMenuBar::ID_TOOLBARGRIP, GUIDesignToolBarGrip);
    // build the thread - io
    myLoadThreadEvent.setTarget(this), myLoadThreadEvent.setSelector(ID_LOADTHREAD_EVENT);
    // build the status bar
    myStatusbar = new FXStatusBar(this, GUIDesignStatusBar);
    {
        myGeoFrame =
            new FXHorizontalFrame(myStatusbar, GUIDesignHorizontalFrameStatusBar);
        myGeoCoordinate = new FXLabel(myGeoFrame, "N/A\t\tOriginal coordinate (before coordinate transformation in NETCONVERT)", 0, LAYOUT_CENTER_Y);
        myCartesianFrame =
            new FXHorizontalFrame(myStatusbar, GUIDesignHorizontalFrameStatusBar);
        myCartesianCoordinate = new FXLabel(myCartesianFrame, "N/A\t\tNetwork coordinate", 0, LAYOUT_CENTER_Y);
    }
    // make the window a mdi-window
    myMainSplitter = new FXSplitter(this, GUIDesignSplitter | SPLITTER_VERTICAL | SPLITTER_REVERSED);
    myMDIClient = new FXMDIClient(myMainSplitter, GUIDesignSplitterMDI);
    myMDIMenu = new FXMDIMenu(this, myMDIClient);
    // Due netedit only have a view, this buttons must be disabled (see #2807)
    //new FXMDIWindowButton(myMenuBar, myMDIMenu, myMDIClient, FXMDIClient::ID_MDI_MENUWINDOW, GUIDesignMDIButtonLeft);
    //new FXMDIDeleteButton(myMenuBar, myMDIClient, FXMDIClient::ID_MDI_MENUCLOSE, GUIDesignMDIButtonRight);
    //new FXMDIRestoreButton(myMenuBar, myMDIClient, FXMDIClient::ID_MDI_MENURESTORE, GUIDesignMDIButtonRight);
    //new FXMDIMinimizeButton(myMenuBar, myMDIClient, FXMDIClient::ID_MDI_MENUMINIMIZE, GUIDesignMDIButtonRight);
    // build the message window
    myMessageWindow = new GUIMessageWindow(myMainSplitter);
    myMainSplitter->setSplit(1, 65);
    // fill menu and tool bar
    fillMenuBar();
    // build additional threads
    myLoadThread = new GNELoadThread(getApp(), this, myEvents, myLoadThreadEvent);
    // set the status bar
    myStatusbar->getStatusLine()->setText("Ready.");
    // set the caption
    setTitle(myTitlePrefix);
    // set Netedit ICON
    setIcon(GUIIconSubSys::getIcon(ICON_NETEDIT));
    // initialize single hotkeys using decimal code (to avoid problems in Linux)
    getAccelTable()->addAccel(101, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_CREATE_EDGE));  // e
    getAccelTable()->addAccel(69,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_CREATE_EDGE));  // E
    getAccelTable()->addAccel(109, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_MOVE));         // m
    getAccelTable()->addAccel(77,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_MOVE));         // M
    getAccelTable()->addAccel(100, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_DELETE));       // d
    getAccelTable()->addAccel(68,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_DELETE));       // D
    getAccelTable()->addAccel(105, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_INSPECT));      // i
    getAccelTable()->addAccel(73,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_INSPECT));      // I
    getAccelTable()->addAccel(115, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_SELECT));       // s
    getAccelTable()->addAccel(83,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_SELECT));       // S
    getAccelTable()->addAccel(99,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_CONNECT));      // c
    getAccelTable()->addAccel(67,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_CONNECT));      // C
    getAccelTable()->addAccel(119, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_PROHIBITION));  // w
    getAccelTable()->addAccel(87,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_PROHIBITION));  // W
    getAccelTable()->addAccel(116, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_TLS));          // t
    getAccelTable()->addAccel(94,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_TLS));          // T
    getAccelTable()->addAccel(97,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_ADDITIONAL));   // a
    getAccelTable()->addAccel(65,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_ADDITIONAL));   // A
    getAccelTable()->addAccel(114, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_CROSSING));     // r
    getAccelTable()->addAccel(82,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_CROSSING));     // R
    getAccelTable()->addAccel(112, this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_POLYGON));      // p
    getAccelTable()->addAccel(80,  this, FXSEL(SEL_COMMAND, MID_GNE_SETMODE_POLYGON));      // P
    getAccelTable()->addAccel(118, this, FXSEL(SEL_COMMAND, MID_EDITVIEWPORT));             // v
    getAccelTable()->addAccel(86,  this, FXSEL(SEL_COMMAND, MID_EDITVIEWPORT));             // V
    // initialize Ctrl hotkeys with Caps Lock enabled using decimal code (to avoid problems in Linux)
    getAccelTable()->addAccel(262222, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_NEWNETWORK));        // Ctrl + N
    getAccelTable()->addAccel(262223, this, FXSEL(SEL_COMMAND, MID_OPEN_NETWORK));                      // Ctrl + O
    getAccelTable()->addAccel(327691, this, FXSEL(SEL_COMMAND, MID_OPEN_CONFIG));                       // Ctrl + Shift + O
    getAccelTable()->addAccel(262226, this, FXSEL(SEL_COMMAND, MID_RELOAD));                            // Ctrl + R
    getAccelTable()->addAccel(262227, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVENETWORK));       // Ctrl + S
    getAccelTable()->addAccel(327695, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVENETWORK_AS));    // Ctrl + Shift + S
    getAccelTable()->addAccel(262220, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVEPLAINXML));      // Ctrl + L
    getAccelTable()->addAccel(262218, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVEJOINED));        // Ctrl + J
    getAccelTable()->addAccel(262224, this, FXSEL(SEL_COMMAND, MID_OPEN_SHAPES));                       // Ctrl + P
    getAccelTable()->addAccel(327692, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVESHAPES));        // Ctrl + Shift + P
    getAccelTable()->addAccel(262212, this, FXSEL(SEL_COMMAND, MID_OPEN_ADDITIONALS));                  // Ctrl + D
    getAccelTable()->addAccel(327780, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVEADDITIONALS));   // Ctrl + Shift + D
    getAccelTable()->addAccel(262219, this, FXSEL(SEL_COMMAND, MID_OPEN_TLSPROGRAMS));                  // Ctrl + K
    getAccelTable()->addAccel(327787, this, FXSEL(SEL_COMMAND, MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS));   // Ctrl + Shift + K
    getAccelTable()->addAccel(262230, this, FXSEL(SEL_COMMAND, MID_CLOSE));                             // Ctrl + W
    getAccelTable()->addAccel(262225, this, FXSEL(SEL_COMMAND, MID_QUIT));                              // Ctrl + Q
    getAccelTable()->addAccel(262234, this, FXSEL(SEL_COMMAND, FXUndoList::ID_UNDO));                   // Ctrl + Z
    getAccelTable()->addAccel(262233, this, FXSEL(SEL_COMMAND, FXUndoList::ID_REDO));                   // Ctrl + Y
    getAccelTable()->addAccel(262230, this, FXSEL(SEL_COMMAND, MID_EDITVIEWSCHEME));                    // Ctrl + V
    getAccelTable()->addAccel(262217, this, FXSEL(SEL_COMMAND, MID_EDITVIEWPORT));                      // Ctrl + I
    getAccelTable()->addAccel(262215, this, FXSEL(SEL_COMMAND, MID_GNE_HOTKEY_TOOGLE_GRID));            // Ctrl + G
    getAccelTable()->addAccel(262228, this, FXSEL(SEL_COMMAND, MID_SUMOGUI));                           // Ctrl + S
    // initialize Shift hotkeys with Caps Lock enabled using decimal code (to avoid problems in Linux)
    getAccelTable()->addAccel(65642, this, FXSEL(SEL_COMMAND, MID_LOCATEJUNCTION)); // Shift + J
    getAccelTable()->addAccel(65637, this, FXSEL(SEL_COMMAND, MID_LOCATEEDGE));     // Shift + E
    getAccelTable()->addAccel(65652, this, FXSEL(SEL_COMMAND, MID_LOCATETLS));      // Shift + T
    getAccelTable()->addAccel(65633, this, FXSEL(SEL_COMMAND, MID_LOCATEADD));      // Shift + A
    getAccelTable()->addAccel(65647, this, FXSEL(SEL_COMMAND, MID_LOCATEPOI));      // Shift + O
    getAccelTable()->addAccel(65644, this, FXSEL(SEL_COMMAND, MID_LOCATEPOLY));     // Shift + L
    // initialize rest of hotkeys
    getAccelTable()->addAccel(parseAccel("Esc"), this, FXSEL(SEL_COMMAND, MID_GNE_HOTKEY_ESC));
    getAccelTable()->addAccel(parseAccel("Del"), this, FXSEL(SEL_COMMAND, MID_GNE_HOTKEY_DEL));
    getAccelTable()->addAccel(parseAccel("Enter"), this, FXSEL(SEL_COMMAND, MID_GNE_HOTKEY_ENTER));
    getAccelTable()->addAccel(parseAccel("F12"), this, FXSEL(SEL_COMMAND, MID_GNE_HOTKEY_FOCUSFRAME));
}


void
GNEApplicationWindow::create() {
    setWindowSizeAndPos();
    gCurrentFolder = getApp()->reg().readStringEntry("SETTINGS", "basedir", "");
    FXMainWindow::create();
    myMenuBarDrag->create();
    myFileMenu->create();
    myEditMenu->create();
    myFileMenuShapes->create();
    myFileMenuAdditionals->create();
    myFileMenuTLS->create();
    //mySettingsMenu->create();
    myWindowsMenu->create();
    myHelpMenu->create();

    FXint textWidth = getApp()->getNormalFont()->getTextWidth("8", 1) * 22;
    myCartesianFrame->setWidth(textWidth);
    myGeoFrame->setWidth(textWidth);

    show(PLACEMENT_DEFAULT);
    if (!OptionsCont::getOptions().isSet("window-size")) {
        if (getApp()->reg().readIntEntry("SETTINGS", "maximized", 0) == 1) {
            maximize();
        }
    }

}


GNEApplicationWindow::~GNEApplicationWindow() {
    closeAllWindows();
    // Close icons
    GUIIconSubSys::close();
    // Close gifs (Textures)
    GUITextureSubSys::close();
    // delete visuals
    delete myGLVisual;
    // must delete menus to avoid segfault on removing accelerators
    // (http://www.fox-toolkit.net/faq#TOC-What-happens-when-the-application-s)
    delete myFileMenuShapes,
           delete myFileMenuAdditionals,
           delete myFileMenuTLS,
           delete myFileMenu;
    delete myEditMenu;
    delete myLocatorMenu;
    delete myProcessingMenu;
    delete myWindowsMenu;
    delete myHelpMenu;
    // Delete load thread
    delete myLoadThread;
    // drop all events
    while (!myEvents.empty()) {
        // get the next event
        GUIEvent* e = myEvents.top();
        myEvents.pop();
        delete e;
    }
    // delte undo list
    delete myUndoList;
}


void
GNEApplicationWindow::detach() {
    FXMainWindow::detach();
    myMenuBarDrag->detach();
}


void
GNEApplicationWindow::fillMenuBar() {
    // build file menu
    myFileMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar, "&File", 0, myFileMenu);
    new FXMenuCommand(myFileMenu,
                      "&New Network...\tCtrl+N\tCreate a new network.",
                      GUIIconSubSys::getIcon(ICON_OPEN_NET), this, MID_GNE_TOOLBARFILE_NEWNETWORK);
    new FXMenuCommand(myFileMenu,
                      "&Open Network...\tCtrl+O\tOpen a SUMO network.",
                      GUIIconSubSys::getIcon(ICON_OPEN_NET), this, MID_OPEN_NETWORK);
    new FXMenuCommand(myFileMenu,
                      "Open Configura&tion...\tCtrl+Shift+O\tOpen a NETCONVERT configuration file.",
                      GUIIconSubSys::getIcon(ICON_OPEN_CONFIG), this, MID_OPEN_CONFIG);
    new FXMenuCommand(myFileMenu,
                      "Import &Foreign Network...\t\tImport a foreign network such as OSM.",
                      GUIIconSubSys::getIcon(ICON_OPEN_NET), this, MID_GNE_TOOLBARFILE_OPENFOREIGN);
    new FXMenuCommand(myFileMenu,
                      "&Reload\tCtrl+R\tReloads the network.",
                      GUIIconSubSys::getIcon(ICON_RELOAD), this, MID_RELOAD);
    new FXMenuCommand(myFileMenu,
                      "&Save Network...\tCtrl+S\tSave the network.",
                      GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVENETWORK);
    new FXMenuCommand(myFileMenu,
                      "Save Net&work As...\tCtrl+Shift+S\tSave the network in another file.",
                      GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVENETWORK_AS);
    new FXMenuCommand(myFileMenu,
                      "Save plain XM&L...\tCtrl+L\tSave plain xml representation the network.",
                      GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVEPLAINXML);
    new FXMenuCommand(myFileMenu,
                      "Save &joined junctions...\tCtrl+J\tSave log of joined junctions (allows reproduction of joins).",
                      GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVEJOINED);
    // create Shapes menu options
    myFileMenuShapes = new FXMenuPane(this);
    new FXMenuCommand(myFileMenuShapes,
                      "Load S&hapes...\tCtrl+P\tLoad shapes into the network view.",
                      GUIIconSubSys::getIcon(ICON_OPEN_SHAPES), this, MID_OPEN_SHAPES);
    mySaveShapesMenuCommand = new FXMenuCommand(myFileMenuShapes,
            "Save Shapes\tCtrl+Shift+P\tSave shapes elements.",
            GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVESHAPES);
    mySaveShapesMenuCommand->disable();
    mySaveShapesMenuCommandAs = new FXMenuCommand(myFileMenuShapes,
            "Save Shapes As...\t\tSave shapes elements in another files.",
            GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVESHAPES_AS);
    mySaveShapesMenuCommandAs->disable();
    new FXMenuCascade(myFileMenu, "Shapes", GUIIconSubSys::getIcon(ICON_MODEPOLYGON), myFileMenuShapes);
    // create Additionals menu options
    myFileMenuAdditionals = new FXMenuPane(this);
    new FXMenuCommand(myFileMenuAdditionals,
                      "Load A&dditionals...\tCtrl+D\tLoad additional elements.",
                      GUIIconSubSys::getIcon(ICON_OPEN_ADDITIONALS), this, MID_OPEN_ADDITIONALS);
    mySaveAdditionalsMenuCommand = new FXMenuCommand(myFileMenuAdditionals,
            "Save Additionals\tCtrl+Shift+D\tSave additional elements.",
            GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVEADDITIONALS);
    mySaveAdditionalsMenuCommand->disable();
    mySaveAdditionalsMenuCommandAs = new FXMenuCommand(myFileMenuAdditionals,
            "Save Additionals As...\t\tSave additional elements in another file.",
            GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVEADDITIONALS_AS);
    mySaveAdditionalsMenuCommandAs->disable();
    new FXMenuCascade(myFileMenu, "Additionals", GUIIconSubSys::getIcon(ICON_MODEADDITIONAL), myFileMenuAdditionals);
    // create TLS menu options
    myFileMenuTLS = new FXMenuPane(this);
    new FXMenuCommand(myFileMenuTLS,
                      "load TLS Programs...\tCtrl+K\tload TLS Programs in all Traffic Lights of the net.",
                      GUIIconSubSys::getIcon(ICON_OPEN_TLSPROGRAMS), this, MID_OPEN_TLSPROGRAMS);
    mySaveTLSProgramsMenuCommand = new FXMenuCommand(myFileMenuTLS,
            "Save TLS Programs \tCtrl+Shift+K\tSave TLS Programs of all Traffic Lights of the current net.",
            GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS);
    mySaveTLSProgramsMenuCommand->disable();
    new FXMenuCommand(myFileMenuTLS,
                      "Save TLS Programs As...\t\tSave TLS Programs of all Traffic Lights of the current net in another file.",
                      GUIIconSubSys::getIcon(ICON_SAVE), this, MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS_AS);
    new FXMenuCascade(myFileMenu, "Traffic Lights", GUIIconSubSys::getIcon(ICON_MODETLS), myFileMenuTLS);
    // close network
    new FXMenuSeparator(myFileMenu);
    new FXMenuCommand(myFileMenu,
                      "Close\tCtrl+W\tClose the net&work.",
                      GUIIconSubSys::getIcon(ICON_CLOSE), this, MID_CLOSE);
    // Recent files
    FXMenuSeparator* sep1 = new FXMenuSeparator(myFileMenu);
    sep1->setTarget(&myRecentConfigs);
    sep1->setSelector(FXRecentFiles::ID_ANYFILES);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_1);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_2);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_3);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_4);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_5);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_6);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_7);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_8);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_9);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentConfigs, FXRecentFiles::ID_FILE_10);
    new FXMenuCommand(myFileMenu, "Clear Recent Configurat&ions", 0, &myRecentConfigs, FXRecentFiles::ID_CLEAR);
    myRecentConfigs.setTarget(this);
    myRecentConfigs.setSelector(MID_RECENTFILE);
    FXMenuSeparator* sep2 = new FXMenuSeparator(myFileMenu);
    sep2->setTarget(&myRecentNets);
    sep2->setSelector(FXRecentFiles::ID_ANYFILES);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_1);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_2);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_3);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_4);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_5);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_6);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_7);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_8);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_9);
    new FXMenuCommand(myFileMenu, "", 0, &myRecentNets, FXRecentFiles::ID_FILE_10);
    new FXMenuCommand(myFileMenu, "Cl&ear Recent Networks", 0, &myRecentNets, FXRecentFiles::ID_CLEAR);
    myRecentNets.setTarget(this);
    myRecentNets.setSelector(MID_RECENTFILE);
    new FXMenuSeparator(myFileMenu);
    new FXMenuCommand(myFileMenu,
                      "&Quit\tCtrl+Q\tQuit the Application.",
                      0, this, MID_QUIT, 0);

    // build edit menu
    myEditMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar, "&Edit", 0, myEditMenu);

    // build undo/redo command
    new FXMenuCommand(myEditMenu,
                      "&Undo\tCtrl+Z\tUndo the last change.",
                      GUIIconSubSys::getIcon(ICON_UNDO), myUndoList, FXUndoList::ID_UNDO);
    new FXMenuCommand(myEditMenu,
                      "&Redo\tCtrl+Y\tRedo the last change.",
                      GUIIconSubSys::getIcon(ICON_REDO), myUndoList, FXUndoList::ID_REDO);

    new FXMenuSeparator(myEditMenu);

    // build modes command
    new FXMenuCommand(myEditMenu,
                      "&Edge mode\tE\tCreate junction and edges.",
                      GUIIconSubSys::getIcon(ICON_MODECREATEEDGE), this, MID_GNE_SETMODE_CREATE_EDGE);
    new FXMenuCommand(myEditMenu,
                      "&Move mode\tM\tMove elements.",
                      GUIIconSubSys::getIcon(ICON_MODEMOVE), this, MID_GNE_SETMODE_MOVE);
    new FXMenuCommand(myEditMenu,
                      "&Delete mode\tD\tDelete elements.",
                      GUIIconSubSys::getIcon(ICON_MODEDELETE), this, MID_GNE_SETMODE_DELETE);
    new FXMenuCommand(myEditMenu,
                      "&Inspect mode\tI\tInspect elements and change their attributes.",
                      GUIIconSubSys::getIcon(ICON_MODEINSPECT), this, MID_GNE_SETMODE_INSPECT);
    new FXMenuCommand(myEditMenu,
                      "&Select mode\tS\tSelect elements.",
                      GUIIconSubSys::getIcon(ICON_MODESELECT), this, MID_GNE_SETMODE_SELECT);
    new FXMenuCommand(myEditMenu,
                      "&Connection mode\tC\tEdit connections between lanes.",
                      GUIIconSubSys::getIcon(ICON_MODECONNECTION), this, MID_GNE_SETMODE_CONNECT);
    new FXMenuCommand(myEditMenu,
                      "Pro&hibition mode\tW\tEdit connection prohibitions.",
                      GUIIconSubSys::getIcon(ICON_MODEPROHIBITION), this, MID_GNE_SETMODE_PROHIBITION);
    new FXMenuCommand(myEditMenu,
                      "&Traffic light mode\tT\tEdit traffic lights over junctions.",
                      GUIIconSubSys::getIcon(ICON_MODETLS), this, MID_GNE_SETMODE_TLS);
    new FXMenuCommand(myEditMenu,
                      "&Additional mode\tA\tCreate additional elements.",
                      GUIIconSubSys::getIcon(ICON_MODEADDITIONAL), this, MID_GNE_SETMODE_ADDITIONAL);
    new FXMenuCommand(myEditMenu,
                      "C&rossing mode\tR\tCreate crossings between edges.",
                      GUIIconSubSys::getIcon(ICON_MODECROSSING), this, MID_GNE_SETMODE_CROSSING);
    new FXMenuCommand(myEditMenu,
                      "&POI-Poly mode\tP\tCreate Points-Of-Interest and polygons.",
                      GUIIconSubSys::getIcon(ICON_MODEPOLYGON), this, MID_GNE_SETMODE_POLYGON);

    new FXMenuSeparator(myEditMenu);
    new FXMenuCommand(myEditMenu,
                      "Edit Visualisation ...\tCtrl+V\tOpens a dialog for editing visualization settings.",
                      0, this, MID_EDITVIEWSCHEME);
    new FXMenuCommand(myEditMenu,
                      "Edit Viewport...\tCtrl+I\tOpens a dialog for editing viewing are, zoom and rotation.",
                      0, this, MID_EDITVIEWPORT);
    new FXMenuCommand(myEditMenu,
                      "Toggle Grid...\tCtrl+G\tToggles background grid (and snap-to-grid functionality).",
                      0, this, MID_GNE_HOTKEY_TOOGLE_GRID);
    new FXMenuSeparator(myEditMenu);
    new FXMenuCommand(myEditMenu,
                      "Open in SUMO GUI...\tCtrl+T\tOpens the SUMO GUI application with the current network.",
                      GUIIconSubSys::getIcon(ICON_APP), this, MID_SUMOGUI);

    // processing menu (trigger netbuild computations)
    myProcessingMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar, "&Processing", 0, myProcessingMenu);
    new FXMenuCommand(myProcessingMenu,
                      "Compute Junctions\tF5\tComputes junction shape and logic.",
                      GUIIconSubSys::getIcon(ICON_COMPUTEJUNCTIONS), this, MID_GNE_PROCESSING_COMPUTEJUNCTIONS);
    new FXMenuCommand(myProcessingMenu,
                      "Compute Junctions with volatile options\tShift+F5\tComputes junction shape and logic using volatile junctions.",
                      GUIIconSubSys::getIcon(ICON_COMPUTEJUNCTIONS), this, MID_GNE_PROCESSING_COMPUTEJUNCTIONS_VOLATILE);
    new FXMenuCommand(myProcessingMenu,
                      "Clean Junctions\tF6\tRemoves solitary junctions.",
                      GUIIconSubSys::getIcon(ICON_CLEANJUNCTIONS), this, MID_GNE_PROCESSING_CLEANJUNCTIONS);
    new FXMenuCommand(myProcessingMenu,
                      "Join Selected Junctions\tF7\tJoins selected junctions into a single junction.",
                      GUIIconSubSys::getIcon(ICON_JOINJUNCTIONS), this, MID_GNE_PROCESSING_JOINJUNCTIONS);
    new FXMenuCommand(myProcessingMenu,
                      "Clean invalid crossings\tF8\tClear invalid crossings.",
                      GUIIconSubSys::getIcon(ICON_JOINJUNCTIONS), this, MID_GNE_PROCESSING_CLEANINVALIDCROSSINGS);
    new FXMenuCommand(myProcessingMenu,
                      "Options\tF10\t\tConfigure Processing Options.",
                      GUIIconSubSys::getIcon(ICON_OPTIONS), this, MID_GNE_PROCESSING_OPTIONS);
    // build settings menu
    /*
    mySettingsMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar,"&Settings",0,mySettingsMenu);
    new FXMenuCheck(mySettingsMenu,
                    "Gaming Mode\t\tToggle gaming mode on/off.",
                    this,MID_GAMING);
    */
    // build Locate menu
    myLocatorMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar, "&Locate", nullptr, myLocatorMenu);
    new FXMenuCommand(myLocatorMenu,
                      "Locate &Junctions\tShift+J\tOpen a Dialog for Locating a Junction.",
                      GUIIconSubSys::getIcon(ICON_LOCATEJUNCTION), this, MID_LOCATEJUNCTION);
    new FXMenuCommand(myLocatorMenu,
                      "Locate &Edges\tShift+E\tOpen a Dialog for Locating an Edge.",
                      GUIIconSubSys::getIcon(ICON_LOCATEEDGE), this, MID_LOCATEEDGE);
    new FXMenuCommand(myLocatorMenu,
                      "Locate &TLS\tShift+T\tOpen a Dialog for Locating a Traffic Light.",
                      GUIIconSubSys::getIcon(ICON_LOCATETLS), this, MID_LOCATETLS);
    new FXMenuCommand(myLocatorMenu,
                      "Locate &Additional\tShift+A\tOpen a Dialog for Locating an Additional Structure.",
                      GUIIconSubSys::getIcon(ICON_LOCATEADD), this, MID_LOCATEADD);
    new FXMenuCommand(myLocatorMenu,
                      "Locate P&oI\tShift+O\tOpen a Dialog for Locating a Point of Intereset.",
                      GUIIconSubSys::getIcon(ICON_LOCATEPOI), this, MID_LOCATEPOI);
    new FXMenuCommand(myLocatorMenu,
                      "Locate Po&lygon\tShift+L\tOpen a Dialog for Locating a Polygon.",
                      GUIIconSubSys::getIcon(ICON_LOCATEPOLY), this, MID_LOCATEPOLY);
    // build windows menu
    myWindowsMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar, "&Windows", 0, myWindowsMenu);
    new FXMenuCheck(myWindowsMenu,
                    "&Show Status Line\t\tToggle this Status Bar on/off.",
                    myStatusbar, FXWindow::ID_TOGGLESHOWN);
    new FXMenuCheck(myWindowsMenu,
                    "Show &Message Window\t\tToggle the Message Window on/off.",
                    myMessageWindow, FXWindow::ID_TOGGLESHOWN);
    /*
    new FXMenuSeparator(myWindowsMenu);
    new FXMenuCommand(myWindowsMenu,"Tile &Horizontally",
                      GUIIconSubSys::getIcon(ICON_WINDOWS_TILE_HORI),
                      myMDIClient,FXMDIClient::ID_MDI_TILEHORIZONTAL);
    new FXMenuCommand(myWindowsMenu,"Tile &Vertically",
                      GUIIconSubSys::getIcon(ICON_WINDOWS_TILE_VERT),
                      myMDIClient,FXMDIClient::ID_MDI_TILEVERTICAL);
    new FXMenuCommand(myWindowsMenu,"C&ascade",
                      GUIIconSubSys::getIcon(ICON_WINDOWS_CASCADE),
                      myMDIClient,FXMDIClient::ID_MDI_CASCADE);
    new FXMenuCommand(myWindowsMenu,"&Close",0,
                      myMDIClient,FXMDIClient::ID_MDI_CLOSE);
    sep1=new FXMenuSeparator(myWindowsMenu);
    sep1->setTarget(myMDIClient);
    sep1->setSelector(FXMDIClient::ID_MDI_ANY);
    new FXMenuCommand(myWindowsMenu,"",0,myMDIClient,FXMDIClient::ID_MDI_1);
    new FXMenuCommand(myWindowsMenu,"",0,myMDIClient,FXMDIClient::ID_MDI_2);
    new FXMenuCommand(myWindowsMenu,"",0,myMDIClient,FXMDIClient::ID_MDI_3);
    new FXMenuCommand(myWindowsMenu,"",0,myMDIClient,FXMDIClient::ID_MDI_4);
    new FXMenuCommand(myWindowsMenu,"&Others...",0,myMDIClient,FXMDIClient::ID_MDI_OVER_5);
    new FXMenuSeparator(myWindowsMenu);
    */
    new FXMenuCommand(myWindowsMenu,
                      "&Clear Message Window\t\tClear the message window.",
                      0, this, MID_CLEARMESSAGEWINDOW);

    // build help menu
    myHelpMenu = new FXMenuPane(this);
    new FXMenuTitle(myMenuBar, "&Help", 0, myHelpMenu);
    new FXMenuCommand(myHelpMenu,
                      "&Online Documentation\tF1\tOpen Online documentation.",
                      0, this, MID_HELP);
    new FXMenuCommand(myHelpMenu,
                      "&About\tF2\tAbout netedit.",
                      0, this, MID_ABOUT);
}


long
GNEApplicationWindow::onCmdQuit(FXObject*, FXSelector, void*) {
    if (continueWithUnsavedChanges()) {
        storeWindowSizeAndPos();
        getApp()->reg().writeStringEntry("SETTINGS", "basedir", gCurrentFolder.text());
        if (isMaximized()) {
            getApp()->reg().writeIntEntry("SETTINGS", "maximized", 1);
        } else {
            getApp()->reg().writeIntEntry("SETTINGS", "maximized", 0);
        }
        getApp()->exit(0);
    }
    return 1;
}


long
GNEApplicationWindow::onCmdEditChosen(FXObject*, FXSelector, void*) {
    GUIDialog_GLChosenEditor* chooser =
        new GUIDialog_GLChosenEditor(this, &gSelected);
    chooser->create();
    chooser->show();
    return 1;
}


long
GNEApplicationWindow::onCmdNewNetwork(FXObject*, FXSelector, void*) {
    // ask before we clobber options
    if (!continueWithUnsavedChanges()) {
        return 1;
    }
    OptionsCont& oc = OptionsCont::getOptions();
    GNELoadThread::fillOptions(oc);
    GNELoadThread::setDefaultOptions(oc);
    loadConfigOrNet("", true, false, true, true);
    return 1;
}


long
GNEApplicationWindow::onCmdOpenConfiguration(FXObject*, FXSelector, void*) {
    // get the new file name
    FXFileDialog opendialog(this, "Open Netconvert Configuration");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODECREATEEDGE));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList(myConfigPattern.c_str());
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();
        loadConfigOrNet(file, false);
        myRecentConfigs.appendFile(file.c_str());
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenNetwork(FXObject*, FXSelector, void*) {
    // get the new file name
    FXFileDialog opendialog(this, "Open Network");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODECREATEEDGE));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("SUMO nets (*.net.xml)\nAll files (*)");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();
        loadConfigOrNet(file, true);
        myRecentNets.appendFile(file.c_str());
        // when a net is loaded, save additional, shapes an TLSPrograms are disabled
        disableSaveAdditionalsMenu();
        disableSaveShapesMenu();
        mySaveTLSProgramsMenuCommand->disable();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenForeign(FXObject*, FXSelector, void*) {
    // ask before we clobber options
    if (!continueWithUnsavedChanges()) {
        return 1;
    }
    // get the new file name
    FXFileDialog opendialog(this, "Import Foreign Network");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODECREATEEDGE));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    FXString osmPattern("OSM net (*.osm.xml,*.osm)");
    opendialog.setPatternText(0, osmPattern);
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();

        OptionsCont& oc = OptionsCont::getOptions();
        GNELoadThread::fillOptions(oc);
        if (osmPattern.contains(opendialog.getPattern())) {
            oc.set("osm-files", file);
            oc.set("ramps.guess", "true");
            oc.set("tls.guess", "true");
        } else {
            throw ProcessError("Attempted to import unknown file format '" + file + "'.");
        }

        GUIDialog_Options* wizard =
            new GUIDialog_Options(this, "Select Import Options", getWidth(), getHeight());

        if (wizard->execute()) {
            NIFrame::checkOptions(); // needed to set projection parameters
            loadConfigOrNet(file, false, false, false);
        }
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenShapes(FXObject*, FXSelector, void*) {
    // get the shape file name
    FXFileDialog opendialog(this, "Open Shapes file");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODEPOLYGON));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("Shape files (*.xml)\nAll files (*)");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();
        GNEShapeHandler handler(file, myNet);
        // disable validation for shapes
        XMLSubSys::setValidation("never", "auto");
        // begin undo operation
        myUndoList->p_begin("Loading shapes from '" + file + "'");
        // run parser for shapes
        if (!XMLSubSys::runParser(handler, file, false)) {
            WRITE_MESSAGE("Loading of shapes failed.");
        }
        // end undoList operation and update view
        myUndoList->p_end();
        update();
        // enable validation for shapes
        XMLSubSys::setValidation("auto", "auto");
        update();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenAdditionals(FXObject*, FXSelector, void*) {
    // get the Additional file name
    FXFileDialog opendialog(this, "Open Additionals file");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODEADDITIONAL));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("Additional files (*.xml)\nAll files (*)");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();
        // disable validation for additionals
        XMLSubSys::setValidation("never", "auto");
        // Create additional handler
        GNEAdditionalHandler additionalHandler(file, myNet->getViewNet());
        // begin undoList operation
        myUndoList->p_begin("Loading additionals from '" + file + "'");
        // Run parser for additionals
        if (!XMLSubSys::runParser(additionalHandler, file, false)) {
            WRITE_MESSAGE("Loading of " + file + " failed.");
        }
        // end undoList operation and update view
        myUndoList->p_end();
        update();
        // restore validation for additionals
        XMLSubSys::setValidation("auto", "auto");
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenTLSPrograms(FXObject*, FXSelector, void*) {
    // get the shape file name
    FXFileDialog opendialog(this, "Open TLSPrograms file");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_MODETLS));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("TLSProgram files (*.xml)\nAll files (*)");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();
        // Run parser
        myUndoList->p_begin("Loading TLS Programs from '" + file + "'");
        if (myNet->getViewNet()->getViewParent()->getTLSEditorFrame()->parseTLSPrograms(file) == false) {
            // Abort undo/redo
            myUndoList->abort();
        } else {
            // commit undo/redo operation
            myUndoList->p_end();
            update();
        }
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenRecent(FXObject* sender, FXSelector, void* fileData) {
    if (myAmLoading) {
        myStatusbar->getStatusLine()->setText("Already loading!");
        return 1;
    }
    std::string file((const char*)fileData);
    loadConfigOrNet(file, sender == &myRecentNets);
    return 1;
}


long
GNEApplicationWindow::onCmdReload(FXObject*, FXSelector, void*) {
    // @note. If another network has been load during this session, it might not be desirable to set useStartupOptions
    loadConfigOrNet(OptionsCont::getOptions().getString("sumo-net-file"), true, true);
    return 1;
}


long
GNEApplicationWindow::onCmdClose(FXObject*, FXSelector, void*) {
    if (continueWithUnsavedChanges()) {
        closeAllWindows();
        // disable save additionals, shapes and TLS menu
        disableSaveAdditionalsMenu();
        disableSaveShapesMenu();
        mySaveTLSProgramsMenuCommand->disable();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdLocate(FXObject*, FXSelector sel, void*) {
    if (myMDIClient->numChildren() > 0) {
        GNEViewParent* w = dynamic_cast<GNEViewParent*>(myMDIClient->getActiveChild());
        if (w != 0) {
            w->onCmdLocate(0, sel, 0);
        }
    }
    return 1;
}

long
GNEApplicationWindow::onUpdOpen(FXObject* sender, FXSelector, void*) {
    sender->handle(this, myAmLoading ? FXSEL(SEL_COMMAND, ID_DISABLE) : FXSEL(SEL_COMMAND, ID_ENABLE), 0);
    return 1;
}


long
GNEApplicationWindow::onCmdClearMsgWindow(FXObject*, FXSelector, void*) {
    myMessageWindow->clear();
    return 1;
}


long
GNEApplicationWindow::onCmdAbout(FXObject*, FXSelector, void*) {
    // write warning if netedit is running in testing mode
    WRITE_DEBUG("Opening about dialog");
    // create and open about dialog
    GNEDialog_About* about = new GNEDialog_About(this);
    about->create();
    about->show(PLACEMENT_OWNER);
    // write warning if netedit is running in testing mode
    WRITE_DEBUG("Closed about dialog");
    return 1;
}


long GNEApplicationWindow::onClipboardRequest(FXObject*, FXSelector, void* ptr) {
    FXEvent* event = (FXEvent*)ptr;
    FXString string = GUIUserIO::clipped.c_str();
    setDNDData(FROM_CLIPBOARD, event->target, string);
    return 1;
}


long
GNEApplicationWindow::onLoadThreadEvent(FXObject*, FXSelector, void*) {
    eventOccurred();
    return 1;
}


void
GNEApplicationWindow::eventOccurred() {
    while (!myEvents.empty()) {
        // get the next event
        GUIEvent* e = myEvents.top();
        myEvents.pop();
        // process
        switch (e->getOwnType()) {
            case EVENT_SIMULATION_LOADED:
                handleEvent_NetworkLoaded(e);
                break;
            case EVENT_MESSAGE_OCCURRED:
            case EVENT_WARNING_OCCURRED:
            case EVENT_ERROR_OCCURRED:
            case EVENT_DEBUG_OCCURRED:
            case EVENT_GLDEBUG_OCCURRED:
                handleEvent_Message(e);
                break;
            default:
                break;
        }
        delete e;
    }
}


void
GNEApplicationWindow::handleEvent_NetworkLoaded(GUIEvent* e) {
    OptionsCont& oc = OptionsCont::getOptions();
    myAmLoading = false;
    GNEEvent_NetworkLoaded* ec = static_cast<GNEEvent_NetworkLoaded*>(e);
    // check whether the loading was successfull
    if (ec->myNet == nullptr) {
        // report failure
        setStatusBarText("Loading of '" + ec->myFile + "' failed!");
    } else {
        // set new Net
        myNet = ec->myNet;
        // report success
        setStatusBarText("'" + ec->myFile + "' loaded.");
        setWindowSizeAndPos();
        // initialise views
        myViewNumber = 0;
        GUISUMOAbstractView* view = openNewView();
        if (view && ec->mySettingsFile != "") {
            GUISettingsHandler settings(ec->mySettingsFile, true, true);
            std::string settingsName = settings.addSettings(view);
            view->addDecals(settings.getDecals());
            settings.applyViewport(view);
            settings.setSnapshots(view);
        }
        // set network name on the caption
        setTitle(MFXUtils::getTitleText(myTitlePrefix, ec->myFile.c_str()));
        getView()->setEditModeFromHotkey(MID_GNE_SETMODE_INSPECT);
        if (ec->myViewportFromRegistry) {
            Position off;
            off.set(getApp()->reg().readRealEntry("viewport", "x"), getApp()->reg().readRealEntry("viewport", "y"), getApp()->reg().readRealEntry("viewport", "z"));
            Position p(off.x(), off.y(), 0);
            getView()->setViewportFromToRot(off, p, 0);
        }
    }
    getApp()->endWaitCursor();
    myMessageWindow->registerMsgHandlers();
    // check if additionals has to be loaded at start
    if (oc.isSet("sumo-additionals-file") && !oc.getString("sumo-additionals-file").empty() && myNet) {
        myAdditionalsFile = oc.getString("sumo-additionals-file");
        WRITE_MESSAGE("Loading additionals from '" + myAdditionalsFile + "'");
        GNEAdditionalHandler additionalHandler(myAdditionalsFile, myNet->getViewNet());
        // disable validation for additionals
        XMLSubSys::setValidation("never", "auto");
        // Run parser
        myUndoList->p_begin("Loading additionals from '" + myAdditionalsFile + "'");
        if (!XMLSubSys::runParser(additionalHandler, myAdditionalsFile, false)) {
            WRITE_ERROR("Loading of " + myAdditionalsFile + " failed.");
        }
        // disable validation for additionals
        XMLSubSys::setValidation("auto", "auto");
        myUndoList->p_end();
    }
    // check if shapes has to be loaded at start
    if (oc.isSet("sumo-shapes-file") && !oc.getString("sumo-shapes-file").empty() && myNet) {
        myShapesFile = oc.getString("sumo-shapes-file");
        WRITE_MESSAGE("Loading shapes");
        GNEShapeHandler shapeHandler(myShapesFile, myNet);
        // disable validation for shapes
        XMLSubSys::setValidation("never", "auto");
        // Run parser
        myUndoList->p_begin("Loading shapes from '" + myShapesFile + "'");
        if (!XMLSubSys::runParser(shapeHandler, myShapesFile, false)) {
            WRITE_ERROR("Loading of shapes failed.");
        }
        // enable validation for shapes
        XMLSubSys::setValidation("auto", "auto");
        myUndoList->p_end();
    }
    // check if additionals output must be changed
    if (oc.isSet("additionals-output")) {
        myAdditionalsFile = oc.getString("additionals-output");
    }
    // check if shapes output must be changed
    if (oc.isSet("shapes-output")) {
        myShapesFile = oc.getString("shapes-output");
    }
    // check if TLSPrograms output must be changed
    if (oc.isSet("TLSPrograms-output")) {
        myTLSProgramsFile = oc.getString("TLSPrograms-output");
    }
    // after loading net shouldn't be saved
    if (myNet) {
        myNet->requiereSaveNet(false);
    }
    update();
}


void
GNEApplicationWindow::handleEvent_Message(GUIEvent* e) {
    GUIEvent_Message* ec = static_cast<GUIEvent_Message*>(e);
    myMessageWindow->appendMsg(ec->getOwnType(), ec->getMsg());
}


void
GNEApplicationWindow::loadConfigOrNet(const std::string file, bool isNet, bool isReload, bool useStartupOptions, bool newNet) {
    if (!continueWithUnsavedChanges()) {
        return;
    }
    storeWindowSizeAndPos();
    getApp()->beginWaitCursor();
    myAmLoading = true;
    closeAllWindows();
    if (isReload) {
        myLoadThread->start();
        setStatusBarText("Reloading.");
    } else {
        gSchemeStorage.saveViewport(0, 0, -1); // recenter view
        myLoadThread->loadConfigOrNet(file, isNet, useStartupOptions, newNet);
        setStatusBarText("Loading '" + file + "'.");
    }
    update();
}



GUISUMOAbstractView*
GNEApplicationWindow::openNewView() {
    std::string caption = "View #" + toString(myViewNumber++);
    FXuint opts = MDI_TRACKING;
    // create view parent
    GNEViewParent* viewParent = new GNEViewParent(myMDIClient, myMDIMenu, FXString(caption.c_str()), this, getBuildGLCanvas(), myNet, myUndoList, nullptr, opts, 10, 10, 300, 200);
    if (myMDIClient->numChildren() == 1) {
        viewParent->maximize();
    } else {
        myMDIClient->vertical(true);
    }
    myMDIClient->setActiveChild(viewParent);
    //v->grabKeyboard();
    return viewParent->getView();
}


FXGLCanvas*
GNEApplicationWindow::getBuildGLCanvas() const {
    if (myMDIClient->numChildren() == 0) {
        return 0;
    }
    GNEViewParent* share_tmp1 =
        static_cast<GNEViewParent*>(myMDIClient->childAtIndex(0));
    return share_tmp1->getBuildGLCanvas();
}


SUMOTime
GNEApplicationWindow::getCurrentSimTime() const {
    return 0;
}


double
GNEApplicationWindow::getTrackerInterval() const {
    return 1;
}


GNEUndoList*
GNEApplicationWindow::getUndoList() {
    return myUndoList;
}


void
GNEApplicationWindow::closeAllWindows() {
    myTrackerLock.lock();
    // remove trackers and other external windows
    for (int i = 0; i < (int)mySubWindows.size(); ++i) {
        mySubWindows[i]->destroy();
    }
    for (int i = 0; i < (int)myTrackerWindows.size(); ++i) {
        myTrackerWindows[i]->destroy();
    }
    // reset the caption
    setTitle(myTitlePrefix);
    // delete other children
    while (myTrackerWindows.size() != 0) {
        delete myTrackerWindows[0];
    }
    while (mySubWindows.size() != 0) {
        delete mySubWindows[0];
    }
    mySubWindows.clear();
    // add a separator to the log
    myMessageWindow->addSeparator();
    myTrackerLock.unlock();
    // remove coordinate information
    myGeoCoordinate->setText("N/A");
    myCartesianCoordinate->setText("N/A");

    myUndoList->p_clear();
    // check if net can be deleted
    if (myNet != nullptr) {
        delete myNet;
        myNet = nullptr;
        GeoConvHelper::resetLoaded();
    }
    myMessageWindow->unregisterMsgHandlers();
    // Reset textures
    GUITextureSubSys::resetTextures();
    // reset fonts
    GLHelper::resetFont();
    // disable saving commmand
    disableSaveAdditionalsMenu();
    disableSaveShapesMenu();
}


FXCursor*
GNEApplicationWindow::getDefaultCursor() {
    return getApp()->getDefaultCursor(DEF_ARROW_CURSOR);
}


void
GNEApplicationWindow::loadOptionOnStartup() {
    const OptionsCont& oc = OptionsCont::getOptions();
    loadConfigOrNet("", true, false, true, oc.getBool("new"));
}


void
GNEApplicationWindow::setStatusBarText(const std::string& statusBarText) {
    myStatusbar->getStatusLine()->setText(statusBarText.c_str());
    myStatusbar->getStatusLine()->setNormalText(statusBarText.c_str());
}


void
GNEApplicationWindow::setAdditionalsFile(const std::string& additionalsFile) {
    myAdditionalsFile = additionalsFile;
}


void
GNEApplicationWindow::setShapesFile(const std::string& shapesFile) {
    myShapesFile = shapesFile;
}


void
GNEApplicationWindow::setTLSProgramsFile(const std::string& TLSProgramsFile) {
    myTLSProgramsFile = TLSProgramsFile;
}


void
GNEApplicationWindow::enableSaveAdditionalsMenu() {
    mySaveAdditionalsMenuCommand->enable();
    mySaveAdditionalsMenuCommandAs->enable();
}


void
GNEApplicationWindow::disableSaveAdditionalsMenu() {
    mySaveAdditionalsMenuCommand->disable();
    mySaveAdditionalsMenuCommandAs->disable();
}


void
GNEApplicationWindow::enableSaveShapesMenu() {
    mySaveShapesMenuCommand->enable();
    mySaveShapesMenuCommandAs->enable();
}


void
GNEApplicationWindow::disableSaveShapesMenu() {
    mySaveShapesMenuCommand->disable();
    mySaveShapesMenuCommandAs->disable();
}


void
GNEApplicationWindow::enableSaveTLSProgramsMenu() {
    mySaveTLSProgramsMenuCommand->enable();
}


long
GNEApplicationWindow::onCmdSetMode(FXObject*, FXSelector sel, void*) {
    if (getView()) {
        getView()->setEditModeFromHotkey(FXSELID(sel));
    }
    return 1;
}


long
GNEApplicationWindow::onCmdOpenSUMOGUI(FXObject*, FXSelector, void*) {
    if (mySubWindows.empty()) {
        return 1;
    }
    std::string sumogui = "sumo-gui";
    const char* sumoPath = getenv("SUMO_HOME");
    if (sumoPath != 0) {
        std::string newPath = std::string(sumoPath) + "/bin/sumo-gui";
        if (FileHelpers::isReadable(newPath) || FileHelpers::isReadable(newPath + ".exe")) {
            sumogui = "\"" + newPath + "\"";
        }
    }
    std::string cmd = sumogui + " -n "  + OptionsCont::getOptions().getString("output-file");
    // start in background
#ifndef WIN32
    cmd = cmd + " &";
#else
    // see "help start" for the parameters
    cmd = "start /B \"\" " + cmd;
#endif
    WRITE_MESSAGE("Running " + cmd + ".");
    // yay! fun with dangerous commands... Never use this over the internet
    SysUtils::runHiddenCommand(cmd);
    return 1;
}


long
GNEApplicationWindow::onCmdAbort(FXObject*, FXSelector, void*) {
    if (getView()) {
        // show extra information for tests
        WRITE_DEBUG("Key ESC (abort) pressed");
        // abort current operation
        getView()->abortOperation();
        getView()->update();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdDel(FXObject*, FXSelector, void*) {
    if (getView()) {
        // show extra information for tests
        WRITE_DEBUG("Key DEL (delete) pressed");
        getView()->hotkeyDel();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdEnter(FXObject*, FXSelector, void*) {
    if (getView()) {
        // show extra information for tests
        WRITE_DEBUG("Key ENTER pressed");
        getView()->hotkeyEnter();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdFocusFrame(FXObject*, FXSelector, void*) {
    if (getView()) {
        getView()->hotkeyFocusFrame();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdEditViewport(FXObject*, FXSelector, void*) {
    if (getView()) {
        getView()->showViewportEditor();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdEditViewScheme(FXObject*, FXSelector, void*) {
    if (getView()) {
        getView()->showViewschemeEditor();
    }
    return 1;
}


long
GNEApplicationWindow::onCmdToogleGrid(FXObject*, FXSelector, void*) {
    // only toogle grid if there is a GNEViewNet
    if (getView() != nullptr) {
        // Toogle getMenuCheckShowGrid of GNEViewNet
        if (getView()->getMenuCheckShowGrid()->getCheck() == 1) {
            getView()->getMenuCheckShowGrid()->setCheck(0);
            // show extra information for tests
            WRITE_DEBUG("Disabled grid throught Ctrl+g hotkey");
        } else {
            getView()->getMenuCheckShowGrid()->setCheck(1);
            // show extra information for tests
            WRITE_WARNING("Enabled grid throught Ctrl+g hotkey");
        }
        // Call manually show grid function
        getView()->onCmdShowGrid(0, 0, 0);
    }
    return 1;
}


long
GNEApplicationWindow::onCmdHelp(FXObject*, FXSelector, void*) {
    FXLinkLabel::fxexecute("http://sumo.dlr.de/wiki/NETEDIT");
    return 1;
}


long
GNEApplicationWindow::onCmdComputeJunctions(FXObject*, FXSelector, void*) {
    // show extra information for tests
    WRITE_DEBUG("Key F5 (Compute) pressed");
    myNet->computeEverything(this, true, false);
    updateControls();
    return 1;
}


long
GNEApplicationWindow::onCmdComputeJunctionsVolatile(FXObject*, FXSelector, void*) {
    // declare variable to save FXMessageBox outputs.
    FXuint answer = 0;
    // declare string to save paths in wich additionals and shapes will be saved
    std::string additionalSavePath = myAdditionalsFile;
    std::string shapeSavePath = myShapesFile;
    // write warning if netedit is running in testing mode
    WRITE_DEBUG("Keys Shift + F5 (Compute with volatile options) pressed");
    WRITE_DEBUG("Opening FXMessageBox 'Volatile Recomputing'");
    // open question dialog box
    answer = FXMessageBox::question(myNet->getViewNet()->getApp(), MBOX_YES_NO, "Recompute with volatile options",
                                    "Changes produced in the net due a recomputing with volatile options cannot be undone. Continue?");
    if (answer != 1) { //1:yes, 2:no, 4:esc
        // write warning if netedit is running in testing mode
        if (answer == 2) {
            WRITE_DEBUG("Closed FXMessageBox 'Volatile Recomputing' with 'No'");
        } else if (answer == 4) {
            WRITE_DEBUG("Closed FXMessageBox 'Volatile Recomputing' with 'ESC'");
        }
        // abort recompute with volatile options
        return 0;
    } else {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox 'Volatile Recomputing' with 'Yes'");
        // Check if there are additionals in our net
        if (myNet->getNumberOfAdditionals() > 0) {
            // ask user if want to save additionals if weren't saved previously
            if (myAdditionalsFile == "") {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Opening FXMessageBox 'Save additionals before recomputing'");
                // open question dialog box
                answer = FXMessageBox::question(myNet->getViewNet()->getApp(), MBOX_YES_NO, "Save additionals before recomputing with volatile options",
                                                "Would you like to save additionals before recomputing?");
                if (answer != 1) { //1:yes, 2:no, 4:esc
                    // write warning if netedit is running in testing mode
                    if (answer == 2) {
                        WRITE_DEBUG("Closed FXMessageBox 'Save additionals before recomputing' with 'No'");
                    } else if (answer == 4) {
                        WRITE_DEBUG("Closed FXMessageBox 'Save additionals before recomputing' with 'ESC'");
                    }
                } else {
                    // write warning if netedit is running in testing mode
                    WRITE_DEBUG("Closed FXMessageBox 'Save additionals before recomputing' with 'Yes'");
                    // Open a dialog to set filename output
                    myAdditionalsFile = MFXUtils::getFilename2Write(this,
                                        "Select name of the additional file", ".xml",
                                        GUIIconSubSys::getIcon(ICON_MODETLS),
                                        gCurrentFolder).text();
                    // set obtanied filename output into additionalSavePath (can be "")
                    additionalSavePath = myAdditionalsFile;
                }
            }
            // Check if additional must be saved in a temporal directory, if user didn't define a directory for additionals
            if (myAdditionalsFile == "") {
                // Obtain temporal directory provided by FXSystem::getCurrentDirectory()
                additionalSavePath = FXSystem::getTempDirectory().text() + std::string("/tmpAdditionalsNetedit.xml");
            }
            // Start saving additionals
            getApp()->beginWaitCursor();
            try {
                myNet->saveAdditionals(additionalSavePath);
            } catch (IOError& e) {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Opening FXMessageBox 'Error saving additionals before recomputing'");
                // open error message box
                FXMessageBox::error(this, MBOX_OK, "Saving additionals in temporal folder failed!", "%s", e.what());
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Closed FXMessageBox 'Error saving additionals before recomputing' with 'OK'");
            }
            // end saving additionals
            myMessageWindow->addSeparator();
            getApp()->endWaitCursor();
        } else {
            // clear additional path
            additionalSavePath = "";
        }
        // Check if there are shapes in our net
        if (myNet->getNumberOfShapes() > 0) {
            // ask user if want to save shapes if weren't saved previously
            if (myShapesFile == "") {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Opening FXMessageBox 'Save shapes before recomputing'");
                // open question dialog box
                answer = FXMessageBox::question(myNet->getViewNet()->getApp(), MBOX_YES_NO, "Save shapes before recomputing with volatile options",
                                                "Would you like to save shapes before recomputing?");
                if (answer != 1) { //1:yes, 2:no, 4:esc
                    // write warning if netedit is running in testing mode
                    if (answer == 2) {
                        WRITE_DEBUG("Closed FXMessageBox 'Save shapes before recomputing' with 'No'");
                    } else if (answer == 4) {
                        WRITE_DEBUG("Closed FXMessageBox 'Save shapes before recomputing' with 'ESC'");
                    }
                } else {
                    // write warning if netedit is running in testing mode
                    WRITE_DEBUG("Closed FXMessageBox 'Save shapes before recomputing' with 'Yes'");
                    // Open a dialog to set filename output
                    myShapesFile = MFXUtils::getFilename2Write(this,
                                   "Select name of the shape file", ".xml",
                                   GUIIconSubSys::getIcon(ICON_MODEPOLYGON),
                                   gCurrentFolder).text();
                    // set obtanied filename output into shapeSavePath (can be "")
                    shapeSavePath = myShapesFile;
                }
            }
            // Check if shape must be saved in a temporal directory, if user didn't define a directory for shapes
            if (myShapesFile == "") {
                // Obtain temporal directory provided by FXSystem::getCurrentDirectory()
                shapeSavePath = FXSystem::getTempDirectory().text() + std::string("/tmpShapesNetedit.xml");
            }
            // Start saving shapes
            getApp()->beginWaitCursor();
            try {
                myNet->saveShapes(shapeSavePath);
            } catch (IOError& e) {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Opening FXMessageBox 'Error saving shapes before recomputing'");
                // open error message box
                FXMessageBox::error(this, MBOX_OK, "Saving shapes in temporal folder failed!", "%s", e.what());
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Closed FXMessageBox 'Error saving shapes before recomputing' with 'OK'");
            }
            // end saving shapes
            myMessageWindow->addSeparator();
            getApp()->endWaitCursor();
        } else {
            // clear save path
            shapeSavePath = "";
        }
        // compute with volatile options
        myNet->computeEverything(this, true, true, additionalSavePath, shapeSavePath);
        updateControls();
        return 1;
    }
}


long
GNEApplicationWindow::onCmdCleanJunctions(FXObject*, FXSelector, void*) {
    // show extra information for tests
    WRITE_DEBUG("Key F6 (Clean junction) pressed");
    myNet->removeSolitaryJunctions(myUndoList);
    return 1;
}


long
GNEApplicationWindow::onCmdJoinJunctions(FXObject*, FXSelector, void*) {
    // show extra information for tests
    WRITE_DEBUG("Key F7 (Join junctions) pressed");
    myNet->joinSelectedJunctions(myUndoList);
    return 1;
}


long
GNEApplicationWindow::onCmdCleanInvalidCrossings(FXObject*, FXSelector, void*) {
    // show extra information for tests
    WRITE_DEBUG("Key F8 (Clean invalid crossings) pressed");
    myNet->cleanInvalidCrossings(myUndoList);
    return 1;
}


long
GNEApplicationWindow::onCmdOptions(FXObject*, FXSelector, void*) {
    GUIDialog_Options* wizard =
        new GUIDialog_Options(this, "Configure Options", getWidth(), getHeight());

    if (wizard->execute()) {
        NIFrame::checkOptions(); // needed to set projection parameters
        NBFrame::checkOptions();
        NWFrame::checkOptions();
        SystemFrame::checkOptions(); // needed to set precision
    }
    return 1;
}


long
GNEApplicationWindow::onCmdSaveAsNetwork(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(this,
                    "Save Network as", ".net.xml",
                    GUIIconSubSys::getIcon(ICON_MODECREATEEDGE),
                    gCurrentFolder);
    if (file == "") {
        return 1;
    }
    OptionsCont& oc = OptionsCont::getOptions();
    oc.resetWritable();
    oc.set("output-file", file.text());
    setTitle(MFXUtils::getTitleText(myTitlePrefix, file));
    onCmdSaveNetwork(0, 0, 0);
    return 1;
}


long
GNEApplicationWindow::onCmdSaveAsPlainXML(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(this,
                    "Select name of the plain-xml edge-file (other names will be deduced from this)", "",
                    GUIIconSubSys::getIcon(ICON_MODECREATEEDGE),
                    gCurrentFolder);
    if (file == "") {
        return 1;
    }
    OptionsCont& oc = OptionsCont::getOptions();
    bool wasSet = oc.isSet("plain-output-prefix");
    std::string oldPrefix = oc.getString("plain-output-prefix");
    oc.resetWritable();
    std::string prefix = file.text();
    // if the name of an edg.xml file was given, remove the suffix
    if (StringUtils::endsWith(prefix, ".edg.xml")) {
        prefix = prefix.substr(0, prefix.size() - 8);
    }
    if (StringUtils::endsWith(prefix, ".")) {
        prefix = prefix.substr(0, prefix.size() - 1);
    }
    oc.set("plain-output-prefix", prefix);
    getApp()->beginWaitCursor();
    try {
        myNet->savePlain(oc);
        myUndoList->unmark();
        myUndoList->mark();
    } catch (IOError& e) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox 'Error saving plainXML'");
        // open message box
        FXMessageBox::error(this, MBOX_OK, "Saving plain xml failed!", "%s", e.what());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox 'Error saving plainXML' with 'OK'");
    }
    myMessageWindow->appendMsg(EVENT_MESSAGE_OCCURRED, "Plain XML saved with prefix '" + prefix + "'.\n");
    myMessageWindow->addSeparator();
    if (wasSet) {
        oc.resetWritable();
        oc.set("plain-output-prefix", oldPrefix);
    } else {
        oc.unSet("plain-output-prefix");
    }
    getApp()->endWaitCursor();
    return 1;
}


long
GNEApplicationWindow::onCmdSaveJoined(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(this,
                    "Select name of the joined-junctions file", ".nod.xml",
                    GUIIconSubSys::getIcon(ICON_MODECREATEEDGE),
                    gCurrentFolder);
    if (file == "") {
        return 1;
    }
    OptionsCont& oc = OptionsCont::getOptions();
    bool wasSet = oc.isSet("junctions.join-output");
    std::string oldFile = oc.getString("junctions.join-output");
    oc.resetWritable();
    std::string filename = file.text();
    oc.set("junctions.join-output", filename);
    getApp()->beginWaitCursor();
    try {
        myNet->saveJoined(oc);
    } catch (IOError& e) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox 'error saving joined'");
        // opening error message
        FXMessageBox::error(this, MBOX_OK, "Saving joined junctions failed!", "%s", e.what());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox 'error saving joined' with 'OK'");
    }
    myMessageWindow->appendMsg(EVENT_MESSAGE_OCCURRED, "Joined junctions saved to '" + filename + "'.\n");
    myMessageWindow->addSeparator();
    if (wasSet) {
        oc.resetWritable();
        oc.set("junctions.join-output", oldFile);
    } else {
        oc.unSet("junctions.join-output");
    }
    getApp()->endWaitCursor();
    return 1;
}


long
GNEApplicationWindow::onCmdSaveShapes(FXObject*, FXSelector, void*) {
    // check if save shapes menu is enabled
    if (mySaveShapesMenuCommand->isEnabled()) {
        // Check if shapes file was already set at start of netedit or with a previous save
        if (myShapesFile == "") {
            FXString file = MFXUtils::getFilename2Write(this,
                            "Select name of the shape file", ".xml",
                            GUIIconSubSys::getIcon(ICON_MODEPOLYGON),
                            gCurrentFolder);
            if (file == "") {
                // None shapes file was selected, then stop function
                return 0;
            } else {
                myShapesFile = file.text();
            }
        }
        getApp()->beginWaitCursor();
        try {
            myNet->saveShapes(myShapesFile);
            myMessageWindow->appendMsg(EVENT_MESSAGE_OCCURRED, "Shapes saved in " + myShapesFile + ".\n");
            mySaveShapesMenuCommand->disable();
        } catch (IOError& e) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Opening FXMessageBox 'Error saving shapes'");
            // open error dialog box
            FXMessageBox::error(this, MBOX_OK, "Saving POIs failed!", "%s", e.what());
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'Error saving shapes' with 'OK'");
        }
        myMessageWindow->addSeparator();
        getApp()->endWaitCursor();
        return 1;
    } else {
        return 0;
    }
}


long GNEApplicationWindow::onCmdSaveShapesAs(FXObject*, FXSelector, void*) {
    // Open window to select shape file
    FXString file = MFXUtils::getFilename2Write(this,
                    "Select name of the shape file", ".xml",
                    GUIIconSubSys::getIcon(ICON_MODEPOLYGON),
                    gCurrentFolder);
    if (file != "") {
        // Set new shape file
        myShapesFile = file.text();
        // save shapes
        return onCmdSaveShapes(0, 0, 0);
    } else {
        return 1;
    }
}


long
GNEApplicationWindow::onUpdNeedsNetwork(FXObject* sender, FXSelector, void*) {
    sender->handle(this, myNet == 0 ? FXSEL(SEL_COMMAND, ID_DISABLE) : FXSEL(SEL_COMMAND, ID_ENABLE), 0);
    return 1;
}


long
GNEApplicationWindow::onUpdReload(FXObject* sender, FXSelector, void*) {
    sender->handle(this, myNet == 0 || !OptionsCont::getOptions().isSet("sumo-net-file") ? FXSEL(SEL_COMMAND, ID_DISABLE) : FXSEL(SEL_COMMAND, ID_ENABLE), 0);
    return 1;
}


long
GNEApplicationWindow::onCmdSaveNetwork(FXObject*, FXSelector, void*) {
    OptionsCont& oc = OptionsCont::getOptions();
    // function onCmdSaveAsNetwork must be executed if this is the first save
    if (oc.getString("output-file") == "") {
        return onCmdSaveAsNetwork(0, 0, 0);
    } else {
        getApp()->beginWaitCursor();
        try {
            myNet->save(oc);
            myUndoList->unmark();
            myUndoList->mark();
        } catch (IOError& e) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Opening FXMessageBox 'error saving network'");
            // open error message box
            FXMessageBox::error(this, MBOX_OK, "Saving Network failed!", "%s", e.what());
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'error saving network' with 'OK'");
        }
        myMessageWindow->appendMsg(EVENT_MESSAGE_OCCURRED, "Network saved in " + oc.getString("output-file") + ".\n");
        // After saveing a net sucesfully, add it into Recent Nets list.
        myRecentNets.appendFile(oc.getString("output-file").c_str());
        myMessageWindow->addSeparator();
        getApp()->endWaitCursor();
        return 1;
    }
}


long
GNEApplicationWindow::onCmdSaveAdditionals(FXObject*, FXSelector, void*) {
    // check if save additional menu is enabled
    if (mySaveAdditionalsMenuCommand->isEnabled()) {
        // Check if additionals file was already set at start of netedit or with a previous save
        if (myAdditionalsFile == "") {
            FXString file = MFXUtils::getFilename2Write(this,
                            "Select name of the additional file", ".xml",
                            GUIIconSubSys::getIcon(ICON_MODEADDITIONAL),
                            gCurrentFolder);
            if (file == "") {
                // None additionals file was selected, then stop function
                return 0;
            } else {
                myAdditionalsFile = file.text();
            }
        }
        // Start saving additionals
        getApp()->beginWaitCursor();
        try {
            myNet->saveAdditionals(myAdditionalsFile);
            myMessageWindow->appendMsg(EVENT_MESSAGE_OCCURRED, "Additionals saved in " + myAdditionalsFile + ".\n");
            mySaveAdditionalsMenuCommand->disable();
        } catch (IOError& e) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Opening FXMessageBox 'error saving additionals'");
            // open error message box
            FXMessageBox::error(this, MBOX_OK, "Saving additionals failed!", "%s", e.what());
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'error saving additionals' with 'OK'");
        }
        myMessageWindow->addSeparator();
        getApp()->endWaitCursor();
        return 1;
    } else {
        return 0;
    }
}


long
GNEApplicationWindow::onCmdSaveAdditionalsAs(FXObject*, FXSelector, void*) {
    // Open window to select additionasl file
    FXString file = MFXUtils::getFilename2Write(this,
                    "Select name of the additional file", ".xml",
                    GUIIconSubSys::getIcon(ICON_MODEADDITIONAL),
                    gCurrentFolder);
    if (file != "") {
        // Set new additional file
        myAdditionalsFile = file.text();
        // save additionals
        return onCmdSaveAdditionals(0, 0, 0);
    } else {
        return 1;
    }
}


long
GNEApplicationWindow::onCmdSaveTLSPrograms(FXObject*, FXSelector, void*) {
    // check if save additional menu is enabled
    if (mySaveTLSProgramsMenuCommand->isEnabled()) {
        // Check if TLS Programs file was already set at start of netedit or with a previous save
        if (myTLSProgramsFile == "") {
            FXString file = MFXUtils::getFilename2Write(this,
                            "Select name of the additional file", ".xml",
                            GUIIconSubSys::getIcon(ICON_MODETLS),
                            gCurrentFolder);
            if (file == "") {
                // None TLS Programs file was selected, then stop function
                return 0;
            } else {
                myTLSProgramsFile = file.text();
            }
        }
        // Start saving TLS Programs
        getApp()->beginWaitCursor();
        try {
            myNet->saveTLSPrograms(myTLSProgramsFile);
            myMessageWindow->appendMsg(EVENT_MESSAGE_OCCURRED, "TLS Programs saved in " + myTLSProgramsFile + ".\n");
            mySaveTLSProgramsMenuCommand->disable();
        } catch (IOError& e) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Opening FXMessageBox 'error saving TLS Programs'");
            // open error message box
            FXMessageBox::error(this, MBOX_OK, "Saving TLS Programs failed!", "%s", e.what());
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'error saving TLS Programs' with 'OK'");
        }
        myMessageWindow->addSeparator();
        getApp()->endWaitCursor();
        return 1;
    } else {
        return 0;
    }
}


long
GNEApplicationWindow::onCmdSaveTLSProgramsAs(FXObject*, FXSelector, void*) {
    // Open window to select TLS Programs file
    FXString file = MFXUtils::getFilename2Write(this,
                    "Select name of the TLS Progarm file", ".xml",
                    GUIIconSubSys::getIcon(ICON_MODETLS),
                    gCurrentFolder);
    if (file != "") {
        // Set new TLS Program file
        myTLSProgramsFile = file.text();
        // save TLS Programs
        return onCmdSaveTLSPrograms(0, 0, 0);
    } else {
        return 1;
    }
}


long
GNEApplicationWindow::onUpdSaveNetwork(FXObject* sender, FXSelector, void*) {
    OptionsCont& oc = OptionsCont::getOptions();
    bool enable = myNet != 0 && oc.isSet("output-file");
    sender->handle(this, FXSEL(SEL_COMMAND, enable ? ID_ENABLE : ID_DISABLE), 0);
    if (enable) {
        FXString caption = ("Save " + oc.getString("output-file")).c_str();
        sender->handle(this, FXSEL(SEL_COMMAND, FXMenuCaption::ID_SETSTRINGVALUE), (void*)&caption);
    }
    return 1;
}


GNEViewNet*
GNEApplicationWindow::getView() {
    if (mySubWindows.size() != 0) {
        GUIGlChildWindow* childWindows = dynamic_cast<GUIGlChildWindow*>(mySubWindows[0]);
        if (childWindows != nullptr) {
            return dynamic_cast<GNEViewNet*>(childWindows->getView());
        } else {
            return nullptr;
        }
    } else {
        return nullptr;
    }
}


bool
GNEApplicationWindow::continueWithUnsavedChanges() {
    FXuint answer = 0;
    if (myNet && !myNet->isNetSaved()) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox 'Confirm closing network'");
        // open question box
        answer = FXMessageBox::question(getApp(), MBOX_QUIT_SAVE_CANCEL,
                                        "Confirm closing Network", "%s",
                                        "You have unsaved changes in the network. Do you wish to quit and discard all changes?");
        // restore focus to view net
        getView()->setFocus();
        // if user close dialog box, check additionasl and shapes
        if (answer == MBOX_CLICKED_QUIT) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'Confirm closing network' with 'Quit'");
            if (continueWithUnsavedAdditionalChanges() && continueWithUnsavedShapeChanges()) {
                // clear undo list and return true to continue with closing/reload
                myUndoList->p_clear();
                return true;
            } else {
                return false;
            }
        } else if (answer == MBOX_CLICKED_SAVE) {
            // save newtork
            onCmdSaveNetwork(0, 0, 0);
            if (!myUndoList->marked()) {
                // saving failed
                return false;
            }
            if (continueWithUnsavedAdditionalChanges() && continueWithUnsavedShapeChanges()) {
                // clear undo list and return true to continue with closing/reload
                myUndoList->p_clear();
                return true;
            } else {
                return false;
            }
        } else {
            // write warning if netedit is running in testing mode
            if (answer == 2) {
                WRITE_DEBUG("Closed FXMessageBox 'Confirm closing network' with 'No'");
            } else if (answer == 4) {
                WRITE_DEBUG("Closed FXMessageBox 'Confirm closing network' with 'ESC'");
            }
            // return false to stop closing/reloading
            return false;
        }
    } else {
        if (continueWithUnsavedAdditionalChanges() && continueWithUnsavedShapeChanges()) {
            // clear undo list and return true to continue with closing/reload
            myUndoList->p_clear(); //only ask once
            return true;
        } else {
            // return false to stop closing/reloading
            return false;
        }
    }
}


bool
GNEApplicationWindow::continueWithUnsavedAdditionalChanges() {
    // Check if there are non saved additionals
    if (mySaveAdditionalsMenuCommand->isEnabled()) {
        WRITE_DEBUG("Opening FXMessageBox 'Save additionals before exit'");
        // open question box
        FXuint answer = FXMessageBox::question(getApp(), MBOX_QUIT_SAVE_CANCEL,
                                               "Save additionals before exit", "%s",
                                               "You have unsaved additionals. Do you wish to quit and discard all changes?");
        // restore focus to view net
        getView()->setFocus();
        // if answer was affirmative, but there was an error during saving additional, return false to stop closing/reloading
        if (answer == MBOX_CLICKED_QUIT) {
            WRITE_DEBUG("Closed FXMessageBox 'Save additionals before exit' with 'Quit'");
            // nothing to save, return true
            return true;
        } else if (answer == MBOX_CLICKED_SAVE) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'Save additionals before exit' with 'Yes'");
            if (onCmdSaveAdditionals(0, 0, 0) == 1) {
                // additionals sucesfully saved
                return true;
            } else {
                // error saving additionals, abort saving
                return false;
            }
        } else {
            // write warning if netedit is running in testing mode
            if (answer == 2) {
                WRITE_DEBUG("Closed FXMessageBox 'Save additionals before exit' with 'No'");
            } else if (answer == 4) {
                WRITE_DEBUG("Closed FXMessageBox 'Save additionals before exit' with 'ESC'");
            }
            // abort saving
            return false;
        }
    } else {
        // nothing to save, return true
        return true;
    }
}


bool
GNEApplicationWindow::continueWithUnsavedShapeChanges() {
    // Check if there are non saved additionals
    if (mySaveShapesMenuCommand->isEnabled()) {
        WRITE_DEBUG("Opening FXMessageBox 'Save shapes before exit'");
        // open question box
        FXuint answer = FXMessageBox::question(getApp(), MBOX_QUIT_SAVE_CANCEL,
                                               "Save shapes before exit", "%s",
                                               "You have unsaved shapes. Do you wish to quit and discard all changes?");
        // restore focus to view net
        getView()->setFocus();
        // if answer was affirmative, but there was an error during saving additional, return false to stop closing/reloading
        if (answer == MBOX_CLICKED_QUIT) {
            WRITE_DEBUG("Closed FXMessageBox 'Save shapes before exit' with 'Quit'");
            return true;
        } else if (answer == MBOX_CLICKED_SAVE) {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'Save shapes before exit' with 'Yes'");
            if (onCmdSaveShapes(0, 0, 0) == 1) {
                // shapes sucesfully saved
                return true;
            } else {
                // error saving shapes, abort saving
                return false;
            }
        } else {
            // write warning if netedit is running in testing mode
            if (answer == 2) {
                WRITE_DEBUG("Closed FXMessageBox 'Save shapes before exit' with 'No'");
            } else if (answer == 4) {
                WRITE_DEBUG("Closed FXMessageBox 'Save shapes before exit' with 'ESC'");
            }
            // abort saving
            return false;
        }
    } else {
        // nothing to save, then return true
        return true;
    }
}


void
GNEApplicationWindow::updateControls() {
    GNEViewNet* view = getView();
    if (view != 0) {
        view->updateControls();
    }
}


long
GNEApplicationWindow::onKeyPress(FXObject* o, FXSelector sel, void* eventData) {
    const long handled = FXMainWindow::onKeyPress(o, sel, eventData);
    if (handled == 0 && myMDIClient->numChildren() > 0) {
        GNEViewParent* w = dynamic_cast<GNEViewParent*>(myMDIClient->getActiveChild());
        if (w != 0) {
            w->onKeyPress(0, sel, eventData);
        }
    }
    return 0;
}


long
GNEApplicationWindow::onKeyRelease(FXObject* o, FXSelector sel, void* eventData) {
    const long handled = FXMainWindow::onKeyRelease(o, sel, eventData);
    if (handled == 0 && myMDIClient->numChildren() > 0) {
        GNEViewParent* w = dynamic_cast<GNEViewParent*>(myMDIClient->getActiveChild());
        if (w != 0) {
            w->onKeyRelease(0, sel, eventData);
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// GNEApplicationWindow::GNEShapeHandler - methods
// ---------------------------------------------------------------------------

GNEApplicationWindow::GNEShapeHandler::GNEShapeHandler(const std::string& file, GNENet* net) :
    ShapeHandler(file, *net),
    myNet(net) {}


GNEApplicationWindow::GNEShapeHandler::~GNEShapeHandler() {}


Position
GNEApplicationWindow::GNEShapeHandler::getLanePos(const std::string& poiID, const std::string& laneID, double lanePos, double lanePosLat) {
    std::string edgeID;
    int laneIndex;
    NBHelpers::interpretLaneID(laneID, edgeID, laneIndex);
    NBEdge* edge = myNet->retrieveEdge(edgeID)->getNBEdge();
    if (edge == 0 || laneIndex < 0 || edge->getNumLanes() <= laneIndex) {
        WRITE_ERROR("Lane '" + laneID + "' to place poi '" + poiID + "' on is not known.");
        return Position::INVALID;
    }
    if (lanePos < 0) {
        lanePos = edge->getLength() + lanePos;
    }
    if (lanePos < 0 || lanePos > edge->getLength()) {
        WRITE_WARNING("lane position " + toString(lanePos) + " for poi '" + poiID + "' is not valid.");
    }
    return edge->getLanes()[laneIndex].shape.positionAtOffset(lanePos, -lanePosLat);
}

/****************************************************************************/
