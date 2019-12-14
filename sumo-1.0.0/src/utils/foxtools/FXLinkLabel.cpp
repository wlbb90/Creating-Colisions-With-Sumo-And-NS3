/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2006-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    FXLinkLabel.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    2006-03-08
/// @version $Id$
///
//
/****************************************************************************/

/* =========================================================================
 * included modules
 * ======================================================================= */
#include <config.h>

#ifdef WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

#include "FXLinkLabel.h"


FXint
FXLinkLabel::fxexecute(FXString link) {
#ifdef WIN32
    return (int)ShellExecute(NULL, "open", link.text(), NULL, NULL, SW_SHOWNORMAL) > 32;
#else
    FXString ext = FXPath::extension(link);
    FXString list;
    if (comparecase(link.section(':', 0), "http") == 0 ||
            comparecase(link.section(':', 0), "ftp") == 0 ||
            comparecase(ext, "htm") == 0 || comparecase(ext, "html") == 0 ||
            comparecase(ext, "php") == 0 || comparecase(ext, "asp") == 0) {
        list = "mozilla-firefox\tmozilla\tnetscape\tkonqueror\tdillo\tlynx";
    } else if (comparecase(ext, "pdf") == 0) {
        list = "acroread\tkghostview\tgpdf\txpdf";
    }

    if (list.length()) {
        FXString software;
        FXint index = 0;
        FXString path = FXSystem::getExecPath();

        software = list.section("\t", index);
        while (!software.empty()) {
            software = FXPath::search(path, software);
            if (software.length())
                return system(FXString().format("%s \"%s\" >/dev/null 2>&1 & ",
                                                software.text(), link.text()).text()) > 0 ? 0 : 1;
            index++;
            software = list.section("\t", index);
        }
    } else if (FXStat::isExecutable(link)) {
        return system((link + " >/dev/null 2>&1 & ").text()) > 0 ? 0 : 1;
    }
    return 0;
#endif
}



FXDEFMAP(FXLinkLabel) FXLinkLabelMap[] = {
    FXMAPFUNC(SEL_LEFTBUTTONPRESS, 0, FXLinkLabel::onLeftBtnPress),
    FXMAPFUNC(SEL_TIMEOUT, FXLinkLabel::ID_TIMER, FXLinkLabel::onTimer),
};
FXIMPLEMENT(FXLinkLabel, FXLabel, FXLinkLabelMap, ARRAYNUMBER(FXLinkLabelMap))


FXLinkLabel::FXLinkLabel(FXComposite* p, const FXString& text, FXIcon* ic, FXuint opts, FXint x, FXint y, FXint w, FXint h, FXint pl, FXint pr, FXint pt, FXint pb) : FXLabel(p, text, ic, opts, x, y, w, h, pl, pr, pt, pb) {
    setDefaultCursor(getApp()->getDefaultCursor(DEF_HAND_CURSOR));
    setTextColor(FXRGB(0, 0, 255));
}

FXLinkLabel::~FXLinkLabel() {
    getApp()->removeTimeout(this, ID_TIMER);
}

long FXLinkLabel::onLeftBtnPress(FXObject*, FXSelector, void*) {
    FXString link = getTipText();
    if (link.length()) {
        getApp()->beginWaitCursor();
        if (fxexecute(link)) {
            getApp()->addTimeout(this, ID_TIMER, 2000);  // 2 seconds of way cursor
        } else {
            getApp()->endWaitCursor();
            getApp()->beep();
        }
    }
    return 1;
}

long FXLinkLabel::onTimer(FXObject*, FXSelector, void*) {
    getApp()->endWaitCursor();
    return 1;
}