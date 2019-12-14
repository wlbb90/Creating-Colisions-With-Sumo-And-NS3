/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEInspectorFrame.h
/// @author  Jakob Erdmann
/// @author  Pablo Alvarez Lopez
/// @date    Mar 2011
/// @version $Id$
///
// The Widget for modifying network-element attributes (i.e. lane speed)
/****************************************************************************/
#ifndef GNEInspectorFrame_h
#define GNEInspectorFrame_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "GNEFrame.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNEAttributeCarrier;
class GNEAdditional;
class GNEEdge;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEInspectorFrame
 * The Widget for modifying network-element attributes (i.e. lane speed)
 */
class GNEInspectorFrame : public GNEFrame {
    /// @brief FOX-declaration
    FXDECLARE(GNEInspectorFrame)

public:

    // ===========================================================================
    // class AttributesEditor
    // ===========================================================================

    class AttributesEditor : private FXGroupBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEInspectorFrame::AttributesEditor)

    public:

        // ===========================================================================
        // class AttributeInput
        // ===========================================================================

        class AttributeInput : private FXHorizontalFrame {
            /// @brief FOX-declaration
            FXDECLARE(GNEInspectorFrame::AttributesEditor::AttributeInput)

        public:
            /// @brief constructor
            AttributeInput(GNEInspectorFrame::AttributesEditor* attributeEditorParent);

            /// @brief show attribute of ac
            void showAttribute(SumoXMLTag ACTag, SumoXMLAttr ACAttribute, const std::string& value);

            /// @brief show attribute
            void hideAttribute();

            /// @brief refresh current attribute input
            void refreshAttributeInput(const std::string& value, bool forceRefresh);

            /// @brief check if current attribute of TextField/ComboBox is valid
            bool isCurrentAttributeValid() const;

            /// @name FOX-callbacks
            /// @{

            /// @brief try to set new attribute value
            long onCmdSetAttribute(FXObject*, FXSelector, void*);

            /// @brief open model dialog for more comfortable attribute editing
            long onCmdOpenAttributeDialog(FXObject*, FXSelector, void*);
            /// @}

        protected:
            /// @brief FOX needs this
            AttributeInput() {}

            /// @brief removed invalid spaces of Positions and shapes
            std::string stripWhitespaceAfterComma(const std::string& stringValue);

        private:
            /// @brief enable attribute input elements
            void enableAttributeInputElements();

            /// @brief disable attribute input elements
            void disableAttributeInputElements();

            /// @brief pointer to AttributesEditor parent
            GNEInspectorFrame::AttributesEditor* myAttributesEditorParent;

            /// @brief current tag
            SumoXMLTag myTag;

            /// @brief current Attr
            SumoXMLAttr myAttr;

            /// @brief flag to check if input element contains multiple values
            bool myMultiple;

            /// @brief pointer to attribute label
            FXLabel* myLabel;

            /// @brief textField to modify the value of int attributes
            FXTextField* myTextFieldInt;

            /// @brief textField to modify the value of real/Time attributes
            FXTextField* myTextFieldReal;

            /// @brief textField to modify the value of string attributes
            FXTextField* myTextFieldStrings;

            /// @brief pointer to combo box choices
            FXComboBox* myChoicesCombo;

            /// @brief pointer to menu check
            FXCheckButton* myBoolCheckButton;

            /// @brief pointer to buttonCombinableChoices
            FXButton* myButtonCombinableChoices;

            /// @brief Button for open color editor
            FXButton* myColorEditor;
        };

        /// @brief constructor
        AttributesEditor(GNEInspectorFrame* inspectorFrameParent);

        /// @brief show attributes of ac
        void showAttributeEditor();

        /// @brief hide attribute editor
        void hideAttributesEditor();

        /// @brief refresh attribute editor (only the valid values will be refresh)
        void refreshAttributeEditor(bool forceRefreshShape, bool forceRefreshPosition);

        /// @brief get InspectorFrame Parent
        GNEInspectorFrame* getInspectorFrameParent() const;

        /// @name FOX-callbacks
        /// @{

        /// @brief Called when user press the help button
        long onCmdAttributeHelp(FXObject*, FXSelector, void*);
        /// @}

    protected:
        /// @brief FOX needs this
        AttributesEditor() {}

    private:
        /// @brief pointer to GNEInspectorFrame parent
        GNEInspectorFrame* myInspectorFrameParent;

        /// @brief list of Attribute inputs
        std::vector<GNEInspectorFrame::AttributesEditor::AttributeInput*> myVectorOfAttributeInputs;

        /// @brief button for help
        FXButton* myHelpButton;
    };

    // ===========================================================================
    // class NeteditAttributesEditor
    // ===========================================================================

    class NeteditAttributesEditor : private FXGroupBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEInspectorFrame::NeteditAttributesEditor)

    public:
        /// @brief constructor
        NeteditAttributesEditor(GNEInspectorFrame* inspectorFrameParent);

        /// @brief destructor
        ~NeteditAttributesEditor();

        /// @brief show netedit attributes editor
        void showNeteditAttributesEditor();

        /// @brief hide netedit attributes editor
        void hideNeteditAttributesEditor();

        /// @brief refresh netedit attributes
        void refreshNeteditAttributesEditor(bool forceRefresh);

        /// @name FOX-callbacks
        /// @{

        /// @brief Called when user change the current GEO Attribute
        long onCmdSetNeteditAttribute(FXObject*, FXSelector, void*);

        /// @brief Called when user press the help button
        long onCmdNeteditAttributeHelp(FXObject*, FXSelector, void*);
        /// @}

    protected:
        /// @brief FOX needs this
        NeteditAttributesEditor() {}

    private:
        /// @brief pointer to inspector frame parent
        GNEInspectorFrame* myInspectorFrameParent;

        /// @frame horizontal frame for change additional parent
        FXHorizontalFrame* myHorizontalFrameAdditionalParent;

        /// @brief Label for additional parent
        FXLabel* myLabelAdditionalParent;

        /// @brief pointer for change additional parent
        FXTextField* myTextFieldAdditionalParent;

        /// @frame horizontal frame for block movement
        FXHorizontalFrame* myHorizontalFrameBlockMovement;

        /// @brief Label for Check blocked movement
        FXLabel* myLabelBlockMovement;

        /// @brief pointer to check box "Block movement"
        FXCheckButton* myCheckBoxBlockMovement;

        /// @frame horizontal frame for block shape
        FXHorizontalFrame* myHorizontalFrameBlockShape;

        /// @brief Label for Check blocked shape
        FXLabel* myLabelBlockShape;

        /// @brief pointer to check box "Block Shape"
        FXCheckButton* myCheckBoxBlockShape;

        /// @frame horizontal frame for close shape
        FXHorizontalFrame* myHorizontalFrameCloseShape;

        /// @brief Label for close shape
        FXLabel* myLabelCloseShape;

        /// @brief pointer to check box "Block movement"
        FXCheckButton* myCheckBoxCloseShape;

        /// @brief button for help
        FXButton* myHelpButton;
    };

    // ===========================================================================
    // class GEOAttributesEditor
    // ===========================================================================

    class GEOAttributesEditor : private FXGroupBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEInspectorFrame::GEOAttributesEditor)

    public:
        /// @brief constructor
        GEOAttributesEditor(GNEInspectorFrame* inspectorFrameParent);

        /// @brief destructor
        ~GEOAttributesEditor();

        /// @brief show GEO attributes editor
        void showGEOAttributesEditor();

        /// @brief hide GEO attributes editor
        void hideGEOAttributesEditor();

        /// @brief refresh GEO attributes editor
        void refreshGEOAttributesEditor(bool forceRefresh);

        /// @name FOX-callbacks
        /// @{

        /// @brief Called when user change the current GEO Attribute
        long onCmdSetGEOAttribute(FXObject*, FXSelector, void*);

        /// @brief Called when user press the help button
        long onCmdGEOAttributeHelp(FXObject*, FXSelector, void*);
        /// @}

    protected:
        /// @brief FOX needs this
        GEOAttributesEditor() {}

    private:
        /// @brief current GNEInspectorFrame parent
        GNEInspectorFrame* myInspectorFrameParent;

        /// @brief horizontal frame for GEOAttribute
        FXHorizontalFrame* myGEOAttributeFrame;

        /// @brief Label for GEOAttribute
        FXLabel* myGEOAttributeLabel;

        /// @brief textField for GEOAttribute
        FXTextField* myGEOAttributeTextField;

        /// @brief horizontal frame for use GEO
        FXHorizontalFrame* myUseGEOFrame;

        /// @brief Label for use GEO
        FXLabel* myUseGEOLabel;

        /// @brief checkBox for use GEO
        FXCheckButton* myUseGEOCheckButton;

        /// @brief button for help
        FXButton* myHelpButton;
    };

    // ===========================================================================
    // class TemplateEditor
    // ===========================================================================

    class TemplateEditor : private FXGroupBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEInspectorFrame::TemplateEditor)

    public:
        /// @brief constructor
        TemplateEditor(GNEInspectorFrame* inspectorFrameParent);

        /// @brief destructor
        ~TemplateEditor();

        /// @brief show template editor
        void showTemplateEditor();

        /// @brief hide template editor
        void hideTemplateEditor();

        /// @brief get the template edge (to copy attributes from)
        GNEEdge* getEdgeTemplate() const;

        /// @brief seh the template edge (we assume shared responsibility via reference counting)
        void setEdgeTemplate(GNEEdge* tpl);

        /// @name FOX-callbacks
        /// @{

        /// @brief copy edge attributes from edge template
        long onCmdCopyTemplate(FXObject*, FXSelector, void*);

        /// @brief set current edge as new template
        long onCmdSetTemplate(FXObject*, FXSelector, void*);

        /// @brief update the copy button with the name of the template
        long onUpdCopyTemplate(FXObject*, FXSelector, void*);
        /// @}

    protected:
        /// @brief FOX needs this
        TemplateEditor() {}

    private:
        /// @brief current GNEInspectorFrame parent
        GNEInspectorFrame* myInspectorFrameParent;

        /// @brief copy template button
        FXButton* myCopyTemplateButton;

        /// @brief set template button
        FXButton* mySetTemplateButton;

        /// @brief the edge template
        GNEEdge* myEdgeTemplate;
    };

    /**@brief Constructor
     * @brief parent FXHorizontalFrame in which this GNEFrame is placed
     * @brief viewNet viewNet that uses this GNEFrame
     */
    GNEInspectorFrame(FXHorizontalFrame* horizontalFrameParent, GNEViewNet* viewNet);

    /// @brief Destructor
    ~GNEInspectorFrame();

    /// @brief show inspector frame
    void show();

    /// @brief hide inspector frame
    void hide();

    /// @brief Inspect a single element
    void inspectElement(GNEAttributeCarrier* AC);

    /// @brief Inspect the given multi-selection
    void inspectMultisection(const std::vector<GNEAttributeCarrier*>& ACs);

    /// @brief inspect child of already inspected element
    void inspectChild(GNEAttributeCarrier* AC, GNEAttributeCarrier* previousElement);

    /// @brief inspect called from DeleteFrame
    void inspectFromDeleteFrame(GNEAttributeCarrier* AC, GNEAttributeCarrier* previousElement, bool previousElementWasMarked);

    /// @brief remove AC from current inspected ACs
    void removeInspectedAC(GNEAttributeCarrier* ac);

    /// @brief Clear all current inspected ACs
    void clearInspectedAC();

    /// @brief get ACHierarchy
    GNEFrame::ACHierarchy* getACHierarchy() const;

    /// @brief get template editor
    TemplateEditor* getTemplateEditor() const;

    /// @name FOX-callbacks
    /// @{

    /// @brief called when user toogle the go back button
    long onCmdGoBack(FXObject*, FXSelector, void*);
    /// @}

    /// @brief get current list of inspected ACs
    const std::vector<GNEAttributeCarrier*>& getInspectedACs() const;

protected:
    /// @brief FOX needs this
    GNEInspectorFrame() {}

private:
    /// @brief Attribute editor
    AttributesEditor* myAttributesEditor;

    /// @brief Netedit Attributes editor
    NeteditAttributesEditor* myNeteditAttributesEditor;

    /// @brief GEO Attributes editor
    GEOAttributesEditor* myGEOAttributesEditor;

    /// @brief Generic parameters editor
    GenericParametersEditor* myGenericParametersEditor;

    /// @brief Template editor
    TemplateEditor* myTemplateEditor;

    /// @brief Attribute Carrier Hierarchy
    GNEFrame::ACHierarchy* myACHierarchy;

    /// @brief back Button
    FXButton* myBackButton;

    /// @brief pointer to previous element called by Inspector Frame
    GNEAttributeCarrier* myPreviousElementInspect;

    /// @brief pointer to previous element called by Delete Frame
    GNEAttributeCarrier* myPreviousElementDelete;

    /// @brief flag to ckec if myPreviousElementDelete was marked in Delete Frame
    bool myPreviousElementDeleteWasMarked;

    /// @brief the multi-selection currently being inspected
    std::vector<GNEAttributeCarrier*> myInspectedACs;
};


#endif

/****************************************************************************/

