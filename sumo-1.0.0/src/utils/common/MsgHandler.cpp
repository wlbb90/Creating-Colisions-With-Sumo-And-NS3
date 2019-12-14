/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MsgHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 17 Jun 2003
/// @version $Id$
///
// Retrieves messages about the process and gives them further to output
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <cassert>
#include <vector>
#include <algorithm>
#include <iostream>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/UtilExceptions.h>

#include "MsgHandler.h"
#include "AbstractMutex.h"


// ===========================================================================
// static member variables
// ===========================================================================

MsgHandler* MsgHandler::myDebugInstance = 0;
MsgHandler* MsgHandler::myGLDebugInstance = 0;
MsgHandler* MsgHandler::myErrorInstance = 0;
MsgHandler* MsgHandler::myWarningInstance = 0;
MsgHandler* MsgHandler::myMessageInstance = 0;
bool MsgHandler::myAmProcessingProcess = false;
AbstractMutex* MsgHandler::myLock = 0;
bool MsgHandler::myWriteDebugMessages(false);
bool MsgHandler::myWriteDebugGLMessages(false);


// ===========================================================================
// method definitions
// ===========================================================================

MsgHandler*
MsgHandler::getMessageInstance() {
    if (myMessageInstance == 0) {
        myMessageInstance = new MsgHandler(MT_MESSAGE);
    }
    return myMessageInstance;
}


MsgHandler*
MsgHandler::getWarningInstance() {
    if (myWarningInstance == 0) {
        myWarningInstance = new MsgHandler(MT_WARNING);
    }
    return myWarningInstance;
}


MsgHandler*
MsgHandler::getErrorInstance() {
    if (myErrorInstance == 0) {
        myErrorInstance = new MsgHandler(MT_ERROR);
    }
    return myErrorInstance;
}


MsgHandler*
MsgHandler::getDebugInstance() {
    if (myDebugInstance == 0) {
        myDebugInstance = new MsgHandler(MT_DEBUG);
    }
    return myDebugInstance;
}


MsgHandler*
MsgHandler::getGLDebugInstance() {
    if (myGLDebugInstance == 0) {
        myGLDebugInstance = new MsgHandler(MT_GLDEBUG);
    }
    return myGLDebugInstance;
}


void
MsgHandler::enableDebugMessages(bool enable) {
    myWriteDebugMessages = enable;
}

void
MsgHandler::enableDebugGLMessages(bool enable) {
    myWriteDebugGLMessages = enable;
}

void
MsgHandler::inform(std::string msg, bool addType) {
    if (myLock != 0) {
        myLock->lock();
    }
    // beautify progress output
    if (myAmProcessingProcess) {
        myAmProcessingProcess = false;
        MsgHandler::getMessageInstance()->inform("");
    }
    msg = build(msg, addType);
    // inform all receivers
    for (auto i : myRetrievers) {
        i->inform(msg);
    }
    // set the information that something occurred
    myWasInformed = true;
    if (myLock != 0) {
        myLock->unlock();
    }
}


void
MsgHandler::beginProcessMsg(std::string msg, bool addType) {
    if (myLock != 0) {
        myLock->lock();
    }
    msg = build(msg, addType);
    // inform all other receivers
    for (auto i : myRetrievers) {
        i->inform(msg, ' ');
        myAmProcessingProcess = true;
    }
    // set the information that something occurred
    myWasInformed = true;
    if (myLock != 0) {
        myLock->unlock();
    }
}


void
MsgHandler::endProcessMsg(std::string msg) {
    if (myLock != 0) {
        myLock->lock();
    }
    // inform all other receivers
    for (auto i : myRetrievers) {
        i->inform(msg);
    }
    // set the information that something occurred
    myWasInformed = true;
    myAmProcessingProcess = false;
    if (myLock != 0) {
        myLock->unlock();
    }
}


void
MsgHandler::clear() {
    if (myLock != 0) {
        myLock->lock();
    }
    myWasInformed = false;
    if (myLock != 0) {
        myLock->unlock();
    }
}


void
MsgHandler::addRetriever(OutputDevice* retriever) {
    if (myLock != 0) {
        myLock->lock();
    }
    if (!isRetriever(retriever)) {
        myRetrievers.push_back(retriever);
    }
    if (myLock != 0) {
        myLock->unlock();
    }
}


void
MsgHandler::removeRetriever(OutputDevice* retriever) {
    if (myLock != 0) {
        myLock->lock();
    }
    RetrieverVector::iterator i = find(myRetrievers.begin(), myRetrievers.end(), retriever);
    if (i != myRetrievers.end()) {
        myRetrievers.erase(i);
    }
    if (myLock != 0) {
        myLock->unlock();
    }
}


bool
MsgHandler::isRetriever(OutputDevice* retriever) const {
    return find(myRetrievers.begin(), myRetrievers.end(), retriever) != myRetrievers.end();
}


void
MsgHandler::removeRetrieverFromAllInstances(OutputDevice* out) {
    if (myDebugInstance != nullptr) {
        myDebugInstance->removeRetriever(out);
    }
    if (myGLDebugInstance != nullptr) {
        myGLDebugInstance->removeRetriever(out);
    }
    if (myErrorInstance != nullptr) {
        myErrorInstance->removeRetriever(out);
    }
    if (myWarningInstance != nullptr) {
        myWarningInstance->removeRetriever(out);
    }
    if (myMessageInstance != nullptr) {
        myMessageInstance->removeRetriever(out);
    }
}

void
MsgHandler::initOutputOptions() {
    // initialize console properly
    OutputDevice::getDevice("stdout");
    OutputDevice::getDevice("stderr");
    OptionsCont& oc = OptionsCont::getOptions();
    if (oc.getBool("no-warnings")) {
        getWarningInstance()->removeRetriever(&OutputDevice::getDevice("stderr"));
    }
    // build the logger if possible
    if (oc.isSet("log", false)) {
        OutputDevice* logFile = &OutputDevice::getDevice(oc.getString("log"));
        getErrorInstance()->addRetriever(logFile);
        if (!oc.getBool("no-warnings")) {
            getWarningInstance()->addRetriever(logFile);
        }
        getMessageInstance()->addRetriever(logFile);
    }
    if (oc.isSet("message-log", false)) {
        OutputDevice* logFile = &OutputDevice::getDevice(oc.getString("message-log"));
        getMessageInstance()->addRetriever(logFile);
    }
    if (oc.isSet("error-log", false)) {
        OutputDevice* logFile = &OutputDevice::getDevice(oc.getString("error-log"));
        getErrorInstance()->addRetriever(logFile);
        getWarningInstance()->addRetriever(logFile);
    }
    if (!oc.getBool("verbose")) {
        getMessageInstance()->removeRetriever(&OutputDevice::getDevice("stdout"));
    }
}


void
MsgHandler::cleanupOnEnd() {
    if (myLock != 0) {
        myLock->lock();
    }
    delete myMessageInstance;
    myMessageInstance = 0;
    delete myWarningInstance;
    myWarningInstance = 0;
    delete myErrorInstance;
    myErrorInstance = 0;
    delete myDebugInstance;
    myDebugInstance = 0;
    delete myGLDebugInstance;
    myGLDebugInstance = 0;
    if (myLock != 0) {
        myLock->unlock();
    }
}


MsgHandler::MsgHandler(MsgType type) :
    myType(type), myWasInformed(false) {
    if (type == MT_MESSAGE) {
        addRetriever(&OutputDevice::getDevice("stdout"));
    } else {
        addRetriever(&OutputDevice::getDevice("stderr"));
    }
}


MsgHandler::~MsgHandler() {
}


bool
MsgHandler::wasInformed() const {
    return myWasInformed;
}


void
MsgHandler::assignLock(AbstractMutex* lock) {
    assert(myLock == 0);
    myLock = lock;
}

/****************************************************************************/

