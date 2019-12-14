/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    SUMORTree.h
/// @author  Daniel Krajzewicz
/// @date    27.10.2008
/// @version $Id$
///
// A RT-tree for efficient storing of SUMO's GL-objects
/****************************************************************************/
#ifndef SUMORTree_h
#define SUMORTree_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/settings/GUIVisualizationSettings.h>
#include <utils/gui/div/GUIIOGlobals.h>
#include <utils/geom/Boundary.h>
#include <utils/foxtools/MFXMutex.h>

#include "RTree.h"


#define GUI_RTREE_QUAL RTree<GUIGlObject*, GUIGlObject, float, 2, GUIVisualizationSettings>

// specialized implementation for speedup and avoiding warnings

template<>
inline float GUI_RTREE_QUAL::RectSphericalVolume(Rect* a_rect) {
  ASSERT(a_rect);
  const float extent0 = a_rect->m_max[0] - a_rect->m_min[0];
  const float extent1 = a_rect->m_max[1] - a_rect->m_min[1];
  return .78539816f * (extent0 * extent0 + extent1 * extent1);
}

template<>
inline GUI_RTREE_QUAL::Rect GUI_RTREE_QUAL::CombineRect(Rect* a_rectA, Rect* a_rectB) {
    ASSERT(a_rectA && a_rectB);
    Rect newRect;
    newRect.m_min[0] = rtree_min(a_rectA->m_min[0], a_rectB->m_min[0]);
    newRect.m_max[0] = rtree_max(a_rectA->m_max[0], a_rectB->m_max[0]);
    newRect.m_min[1] = rtree_min(a_rectA->m_min[1], a_rectB->m_min[1]);
    newRect.m_max[1] = rtree_max(a_rectA->m_max[1], a_rectB->m_max[1]);
    return newRect;
}


// ===========================================================================
// class definitions
// ===========================================================================
/** @class SUMORTree
 * @brief A RT-tree for efficient storing of SUMO's GL-objects
 * 
 * This class specialises the used RT-tree implementation from "rttree.h" and
 *  extends it by a mutex for avoiding parallel change and traversal of the tree.
 */
class SUMORTree : private GUI_RTREE_QUAL, public Boundary {
public:
    /// @brief Constructor
    SUMORTree() : GUI_RTREE_QUAL(&GUIGlObject::drawGL) {
    }

    /// @brief Destructor
    virtual ~SUMORTree() {
        // check if lock is locked before insert objects
        if(myLock.locked()) {
            ProcessError("Mutex of SUMORTree is locked during call destructor (Lock value = " + toString(myLock.lockCount())+ ")");
        }
        // show information in gui testing debug gl mode
        WRITE_GLDEBUG("Number of objects in SUMORTree during call destructor: " + toString(myTreeDebug.size()));
    }

    /** @brief Insert entry
     * @param a_min Min of bounding rect
     * @param a_max Max of bounding rect
     * @param a_dataId Positive Id of data.  Maybe zero, but negative numbers not allowed.
     * @see RTree::Insert
     */
    virtual void Insert(const float a_min[2], const float a_max[2], GUIGlObject* const & a_dataId) {
        AbstractMutex::ScopedLocker locker(myLock);
        GUI_RTREE_QUAL::Insert(a_min, a_max, a_dataId);
    }

    /** @brief Remove entry
     * @param a_min Min of bounding rect
     * @param a_max Max of bounding rect
     * @param a_dataId Positive Id of data.  Maybe zero, but negative numbers not allowed.
     * @see RTree::Remove
     */
    virtual void Remove(const float a_min[2], const float a_max[2], GUIGlObject* const & a_dataId) {
        AbstractMutex::ScopedLocker locker(myLock);
        GUI_RTREE_QUAL::Remove(a_min, a_max, a_dataId);
    }

    /** @brief Find all within search rectangle
     * @param a_min Min of search bounding rect
     * @param a_max Max of search bounding rect
     * @param a_searchResult Search result array.  Caller should set grow size. Function will reset, not append to array.
     * @param a_resultCallback Callback function to return result.  Callback should return 'true' to continue searching
     * @param a_context User context to pass as parameter to a_resultCallback
     * @return Returns the number of entries found
     * @see RTree::Search
     */
    virtual int Search(const float a_min[2], const float a_max[2], const GUIVisualizationSettings& c) const {
        AbstractMutex::ScopedLocker locker(myLock);
        return GUI_RTREE_QUAL::Search(a_min, a_max, c);
    }

    /** @brief Adds an additional object (detector/shape/trigger) for visualisation
     * @param[in] o The object to add
     */
    void addAdditionalGLObject(GUIGlObject *o) {
        // check if lock is locked before insert objects
        if(myLock.locked()) {
            ProcessError("Mutex of SUMORTree is locked before object insertion (Lock value = " + toString(myLock.lockCount())+ ")");
        }
        // lock mutex
        AbstractMutex::ScopedLocker locker(myLock);
        // obtain boundary of object
        Boundary b = o->getCenteringBoundary();
        // show information in gui testing debug gl mode
        if (MsgHandler::writeDebugGLMessages()) {
            if(myTreeDebug.count(o) != 0) {
                throw ProcessError("GUIGlObject " + o->getMicrosimID() + " was already inserted");
            } else if ((b.getWidth() == 0) || (b.getHeight() == 0)) {
                throw ProcessError("boundary of GUIGlObject " + o->getMicrosimID() + " has an invalid size");
            } else {
                myTreeDebug[o] = b;
                // write GL Debug
                WRITE_GLDEBUG("Inserted " + o->getFullName() + " into SUMORTree with boundary " + toString(b));
            }
        }
        // insert it in Tree
        const float cmin[2] = {(float) b.xmin(), (float) b.ymin()};
        const float cmax[2] = {(float) b.xmax(), (float) b.ymax()};
        Insert(cmin, cmax, o);
    }

    /** @brief Removes an additional object (detector/shape/trigger) from being visualised
     * @param[in] o The object to remove
     */
    void removeAdditionalGLObject(GUIGlObject *o) {
        // check if lock is locked remove insert objects
        if(myLock.locked()) {
            ProcessError("Mutex of SUMORTree is locked before object remove (Lock value = " + toString(myLock.lockCount())+ ")");
        }
        // lock mutex
        AbstractMutex::ScopedLocker locker(myLock);
        // obtain boundary of object
        Boundary b = o->getCenteringBoundary();
        // show information in gui testing debug gl mode
        if (MsgHandler::writeDebugGLMessages()) {
            if(myTreeDebug.count(o) == 0) {
                throw ProcessError("GUIGlObject " + o->getMicrosimID() + " wasn't inserted");
            } else if (b != myTreeDebug.at(o)) {
                 throw ProcessError("add boundary of GUIGlObject " + o->getMicrosimID() + " is different of remove boundary (" + toString(b) + " != " + toString(myTreeDebug.at(o)) + ")");
            } else {
                myTreeDebug.erase(o);
                WRITE_GLDEBUG("Removed object " + o->getFullName() + " from SUMORTree with boundary " + toString(b));
            }
        }
        // remove it from Tree
        const float cmin[2] = {(float) b.xmin(), (float) b.ymin()};
        const float cmax[2] = {(float) b.xmax(), (float) b.ymax()};
        Remove(cmin, cmax, o);
    }

protected:
    /// @brief A mutex avoiding parallel change and traversal of the tree
    mutable MFXMutex myLock;

private:
    /**@brief Map only used for check that SUMORTree works as expected, only is used if option "gui-testing-debug-gl" is enabled.
     * @note Warning: DO NOT USE in release mode and use it in debug mode carefully, due it produces a slowdown.
     */
    std::map<GUIGlObject*, Boundary> myTreeDebug;
};


#endif

/****************************************************************************/

