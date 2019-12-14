/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNETLSEditorFrame.h
/// @author  Jakob Erdmann
/// @date    May 2011
/// @version $Id$
///
// The Widget for modifying traffic lights
/****************************************************************************/
#ifndef GNETLSEditorFrame_h
#define GNETLSEditorFrame_h

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "GNEFrame.h"

// ===========================================================================
// class declarations
// ===========================================================================
class NBTrafficLightDefinition;
class NBLoadedSUMOTLDef;
class GNEEdge;
class GNELane;
class GNEInternalLane;
class GNEJunction;
class NBOwnTLDef;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNETLSEditorFrame
 * The Widget for modifying Traffic Lights (TLS)
 */
class GNETLSEditorFrame : public GNEFrame {
    /// @brief FOX-declaration
    FXDECLARE(GNETLSEditorFrame)

public:

    // ===========================================================================
    // class TLSJunction
    // ===========================================================================

    class TLSJunction : protected FXGroupBox {

    public:
        /// @brief constructor
        TLSJunction(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSJunction();

        /// @brief get current modified junction
        GNEJunction* getCurrentJunction() const;

        /// @brief set current junction
        void setCurrentJunction(GNEJunction* junction);

        /// @brief update descrition
        void updateJunctionDescription() const;

    private:
        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief label for junction ID
        FXLabel* myLabelJunctionID;

        /// @brief text field for junction ID
        FXTextField* myTextFieldJunctionID;

        /// @brief label for junction status
        FXLabel* myLabelJunctionStatus;

        /// @brief text field for junction status
        FXTextField* myTextFieldJunctionStatus;

        /// @brief the junction of the tls is being modified
        GNEJunction* myCurrentJunction;
    };

    // ===========================================================================
    // class TLSDefinition
    // ===========================================================================

    class TLSDefinition : protected FXGroupBox {

    public:
        /// @brief constructor
        TLSDefinition(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSDefinition();

    private:
        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief button for create new Traffic light program
        FXButton* myNewTLProgram;

        /// @brief button for delete traffic light program
        FXButton* myDeleteTLProgram;
    };

    // ===========================================================================
    // class TLSAttributes
    // ===========================================================================

    class TLSAttributes : protected FXGroupBox {

    public:
        /// @brief constructor
        TLSAttributes(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSAttributes();

        /// @brief initializes the definitions and corresponding listbox
        void initTLSAttributes(GNEJunction* junction);

        /// @brief clear TLS attributes
        void clearTLSAttributes();

        /// @brief get current definition
        NBTrafficLightDefinition* getCurrentTLSDefinition() const;

        /// @brief get current offset in SUMOTIme
        SUMOTime getOffset() const;

        /// @brief set new offset
        void setOffset(SUMOTime offset);

        /// @brief get number of definitions
        int getNumberOfTLSDefinitions() const;

        /// @brief get number of programs
        int getNumberOfPrograms() const;

    private:
        /// @brief pointer to TLSEditorParent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief the list of Definitions for the current junction
        std::vector<NBTrafficLightDefinition*> myTLSDefinitions;

        /// @brief name label
        FXLabel* myNameLabel;

        /// @brief name text field
        FXTextField* myNameTextField;

        /// @brief program label
        FXLabel* myProgramLabel;

        /// @brief the comboBox for selecting the tl-definition to edit
        FXComboBox* myProgramComboBox;

        /// @brief offset label
        FXLabel* myOffsetLabel;

        /// @brief the control for modifying offset
        FXTextField* myOffsetTextField;
    };

    // ===========================================================================
    // class TLSPhases
    // ===========================================================================

    class TLSPhases : protected FXGroupBox {

    public:
        /// @brief constructor
        TLSPhases(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSPhases();

        /// @brief get phase table
        FXTable* getPhaseTable() const;

        /**@brief initialies the phase table
         * @param[in] index The index to select
         */
        void initPhaseTable(int index = 0);

        /// @brief show cycle duration
        void showCycleDuration();

        /// @brief hide cycle duration
        void hideCycleDuration();

        /// @brief recomputes cycle duration and updates label
        void updateCycleDuration();

    private:
        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief font for the phase table
        FXFont* myTableFont;

        /// @brief window for oversized phase tables
        FXScrollWindow* myTableScroll;

        /// @brief table for selecting and rearranging phases and for changing duration
        FXTable* myPhaseTable;

        /// @brief label with the cycle duration
        FXLabel* myCycleDuration;

        /// @brief insert new phase button
        FXButton* myInsertDuplicateButton;

        /// @brief delete phase button
        FXButton* myDeleteSelectedPhaseButton;
    };

    // ===========================================================================
    // class TLSModifications
    // ===========================================================================

    class TLSModifications : protected FXGroupBox {

    public:
        /// @brief constructor
        TLSModifications(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSModifications();

        /// @brief check if current TLS was modified
        bool checkHaveModifications() const;

        /// @brief set if current TLS was modified
        void setHaveModifications(bool value);

    private:
        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief button for cancel modifications
        FXButton* myDiscardModificationsButtons;

        /// @brief button for save modifications
        FXButton* mySaveModificationsButtons;

        /// @brief whether the current tls was modified
        bool myHaveModifications;
    };

    // ===========================================================================
    // class TLSFile
    // ===========================================================================

    class TLSFile : protected FXGroupBox, public SUMOSAXHandler {
        /// @brief FOX-declaration
        FXDECLARE(GNETLSEditorFrame::TLSFile)

    public:
        /// @brief constructor
        TLSFile(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSFile();

        /// @name inherited from GenericSAXHandler
        //@{

        /** @brief Called on the opening of a tag
            * @param[in] element ID of the currently opened element
            * @param[in] attrs Attributes within the currently opened element
            * @exception ProcessError If something fails
            * @see GenericSAXHandler::myStartElement
            */
        void myStartElement(int element, const SUMOSAXAttributes& attrs);
        //@}

        /// @brief enable all elements TLSFile
        void enableTLSFile();

        /// @brief disable all elements TLSFile
        void disableTLSFile();

        /// @brief clear loaded phases
        void clearLoadedTLS();

        /// @brief check if loaded TrafficLights corresponds to a TL controled junction in net
        bool checkTLSValids();

        /// @brief get loaded phases
        const std::vector<NBLoadedSUMOTLDef*>& getLoadedTLS() const;

        /// @name FOX-callbacks
        /// @{
        /// @brief load TLS Program from an additional file
        long onCmdLoadTLSProgram(FXObject*, FXSelector, void*);

        /// @brief save TLS Programm to an additional file
        long onCmdSaveTLSProgram(FXObject*, FXSelector, void*);
        /// @}

    protected:
        /// @brief FOX needs this
        TLSFile() {}

    private:
        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief button for load TLS Programs
        FXButton* myLoadTLSProgramButton;

        /// @brief button for save TLS Programs
        FXButton* mySaveTLSProgramButton;

        /// @brief container used to save loaded TLSs
        std::vector<NBLoadedSUMOTLDef*> myLoadedTLS;

        /// @brief convert SUMOTime into string
        std::string writeSUMOTime(SUMOTime steps);
    };


    /**@brief Constructor
     * @brief parent FXHorizontalFrame in which this GNEFrame is placed
     * @brief viewNet viewNet that uses this GNEFrame
     */
    GNETLSEditorFrame(FXHorizontalFrame* horizontalFrameParent, GNEViewNet* viewNet);

    /// @brief Destructor
    ~GNETLSEditorFrame();

    /// @brief edits the traffic light for the given junction
    /// @param[in] junction The junction of which the traffic light shall be edited
    void editJunction(GNEJunction* junction);

    /// @brief check if modifications in TLS was saved
    bool isTLSSaved();

    /// @brief parse TLS Programs from a file
    bool parseTLSPrograms(const std::string& file);

    /// @name FOX-callbacks
    /// @{
    /// @brief Called when the user presses the OK-Button
    /// @note saves any modifications
    long onCmdOK(FXObject*, FXSelector, void*);

    /// @brief Called when the user presses the Cancel-button
    /// @note discards any modifications
    long onCmdCancel(FXObject*, FXSelector, void*);

    /// @brief Called when the user presses the button Toogle
    long onCmdToggle(FXObject*, FXSelector, void*);

    /// @brief Called when the user presses the button Guess
    long onCmdGuess(FXObject*, FXSelector, void*);

    /// @brief Called when the user creates a TLS
    long onCmdDefCreate(FXObject*, FXSelector, void*);

    /// @brief Called when the user deletes a TLS
    long onCmdDefDelete(FXObject*, FXSelector, void*);

    /// @brief Called when the user changes the offset of a TLS
    long onCmdDefOffset(FXObject*, FXSelector, void*);

    /// @brief Called when the user switchs a TLS
    long onCmdDefSwitch(FXObject*, FXSelector, void*);

    /// @brief Called when the user renames a TLS
    long onCmdDefRename(FXObject*, FXSelector, void*);

    /// @brief Called when the user sub-renames a TLS
    long onCmdDefSubRename(FXObject*, FXSelector, void*);

    /// @brief Called when the user adds a OFF
    long onCmdDefAddOff(FXObject*, FXSelector, void*);

    /// @brief Called when the user switchs a Phase
    long onCmdPhaseSwitch(FXObject*, FXSelector, void*);

    /// @brief Called when the user creates a Phase
    long onCmdPhaseCreate(FXObject*, FXSelector, void*);

    /// @brief Called when the user deletes a Phase
    long onCmdPhaseDelete(FXObject*, FXSelector, void*);

    /// @brief Called when the user cleans up states
    long onCmdCleanup(FXObject*, FXSelector, void*);

    /// @brief Called when the user cleans up states
    long onCmdAddUnused(FXObject*, FXSelector, void*);

    /// @brief Called when the user edits a Phase
    long onCmdPhaseEdit(FXObject*, FXSelector, void*);

    /// @brief Called when the user makes RILSA
    long onCmdMakeRILSAConforming(FXObject*, FXSelector, void*);

    /// @brief Called when occurs an update of switch definition
    long onUpdDefSwitch(FXObject*, FXSelector, void*);

    /// @brief Called when occurs an update of needs definition
    long onUpdNeedsDef(FXObject*, FXSelector, void*);

    /// @brief Called when occurs an update of needs definition an dphase
    long onUpdNeedsDefAndPhase(FXObject*, FXSelector, void*);

    /// @brief Called when occurs an update of create definition
    long onUpdDefCreate(FXObject*, FXSelector, void*);

    /// @brief Called when occurs an update of modified
    long onUpdModified(FXObject*, FXSelector, void*);
    /// @}

    /// @brief update phase definition for the current traffic light and phase
    void handleChange(GNEInternalLane* lane);

    /// @brief update phase definition for the current traffic light and phase
    void handleMultiChange(GNELane* lane, FXObject* obj, FXSelector sel, void* data);

    /// @brief whether the given edge is controlled by the currently edited tlDef
    bool controlsEdge(GNEEdge& edge) const;

protected:
    /// @brief FOX needs this
    GNETLSEditorFrame() {}

    /// @brief converts to SUMOTime
    static SUMOTime getSUMOTime(const FXString& string);

private:
    /// @brief modul for TLS Junction
    GNETLSEditorFrame::TLSJunction* myTLSJunction;

    /// @brief modul for TLS Definition
    GNETLSEditorFrame::TLSDefinition* myTLSDefinition;

    /// @brief modul for TLS attributes
    GNETLSEditorFrame::TLSAttributes* myTLSAttributes;

    /// @brief modul for load/Save TLS Modifications
    GNETLSEditorFrame::TLSModifications* myTLSModifications;

    /// @brief modul for TLS Phases
    GNETLSEditorFrame::TLSPhases* myTLSPhases;

    /// @brief modul for load/Save TLS Programs
    GNETLSEditorFrame::TLSFile* myTLSFile;

    /// @brief the internal lanes belonging the the current junction indexed by their tl-index
    typedef std::map<int, std::vector<GNEInternalLane*> > TLIndexMap;
    TLIndexMap myInternalLanes;

    /// @brief the traffic light definition being edited
    NBLoadedSUMOTLDef* myEditedDef;

    /// @brief index of the phase being shown
    int myPhaseIndex;

    /// @brief cleans up previous lanes
    void cleanup();

    /// @brief builds internal lanes for the given tlDef
    void buildIinternalLanes(NBTrafficLightDefinition* tlDef);

    /// @brief the phase of the current traffic light
    const std::vector<NBTrafficLightLogic::PhaseDefinition>& getPhases();

    /// @brief convert duration (potentially undefined) to string
    static std::string varDurString(SUMOTime dur);
};


#endif

/****************************************************************************/

