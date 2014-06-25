// Copyright 2009 Distributed Systems Group, University of Kassel
// This program is distributed under the GNU Lesser General Public License (LGPL).
//
// This file is part of the Carpe Noctem Software Framework.
//
//    The Carpe Noctem Software Framework is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    The Carpe Noctem Software Framework is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
package de.uni_kassel.vs.cn.planDesigner.ui.parts;

import java.util.Collections;
import java.util.List;

import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.MouseEvent;
import org.eclipse.draw2d.MouseListener;
import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.gef.DragTracker;
import org.eclipse.gef.Request;
import org.eclipse.gef.tools.DragEditPartsTracker;
import org.eclipse.jface.resource.ImageRegistry;

import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.figures.CollapsableFigure;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public abstract class CollapsableEditPart extends PlanElementEditPart {
	
	private CollapsableFigure collapsableFigure;
	
	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();
		
		// Register the editPart with the ui extensionmodel
		PmlUiExtension extension = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		if(extension != null)
			this.adapter.addToObject(extension);
	}
	
	@Override
	protected IFigure createFigure() {
		this.collapsableFigure = new CollapsableFigure(){
			@Override
			public IFigure createDescriptionFigure() {
				return CollapsableEditPart.this.createDescriptionFigure();
			}
		};
		
		this.collapsableFigure.getArrowLabel().addMouseListener(new MouseListener(){

			public void mouseDoubleClicked(MouseEvent me) {
				handleCollapseState(me);
			}

			public void mousePressed(MouseEvent me) {
				handleCollapseState(me);
			}

			public void mouseReleased(MouseEvent me) {
				handleCollapseState(me);
			}
			
		});
		
		configureCollapsableFigure();
		
		return this.collapsableFigure;
	}

	
	protected abstract IFigure createDescriptionFigure();
	
	/**
	 * Convenience method which casts this EditParts figure to a CollapsableFigure.
	 * @return
	 */
	protected CollapsableFigure getCollapsableFigure(){
		return (CollapsableFigure)getFigure();
	}
	
	@Override
	protected List<Object> getModelChildren() {
		return getExpandedChildren();
	}
	
	/**
	 * Subclasses should override this method to return their
	 * model children in expanded state.
	 * @return
	 */
	protected List<Object> getExpandedChildren(){
		return Collections.EMPTY_LIST;
	}
	
	protected void notifyCollapsed(boolean collapsed){
		configureCollapsableFigure();
		refreshChildren();
	}
	
	public boolean isCollapsed() {
		// Try to get the ui extension 
		PmlUiExtension ext = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		
		// If the ui extension is available return the collapsed feature, else fall back.
		return ext != null ? ext.isCollapsed() : true;
	}

	public void setCollapsed(boolean collapsed) {
		// Try to get the ui extension 
		PmlUiExtension ext = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		
		if(ext != null)
		 ext.setCollapsed(collapsed);
	}

	@Override
	public IFigure getContentPane() {
		return ((CollapsableFigure)getFigure()).getChildrenFigure();
	}
	
	@Override
	protected void refreshUIExtension(Notification n) {
		int feature = n.getFeatureID(null);
		
		switch(feature){
		case PmlUIExtensionModelPackage.PML_UI_EXTENSION__COLLAPSED:
			notifyCollapsed(isCollapsed());
			break;
		default:
			super.refreshUIExtension(n);
		}
	}
	
	protected void handleCollapseState(MouseEvent me){
		if(me.button == 1 && getExpandedChildren().size() > 0){
			toggleCollapseState();
		}
		
	}
	
	private void toggleCollapseState(){
		// Collapse / Expand only makes sense if we have children
		PlanEditor editor = PlanEditorUtils.getPlanEditor(this);
		PmlUiExtension ext = editor.getUIExtension(getEObjectModel(), false);
		
		boolean collapsed = !isCollapsed();
		AbstractCommand cmd = (SetCommand) SetCommand.create(editor.getEditingDomain(), 
				ext, 
				PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
				collapsed);
		cmd.setLabel(collapsed ? "Collapse" : "Expand");
		editor.getEMFCommandStack().execute(cmd);
	}
	
	@Override
	public DragTracker getDragTracker(Request request) {
		return new DragEditPartsTracker(this){
			@Override
			protected boolean handleDoubleClick(int button) {
				if(button == 1 && getExpandedChildren().size() > 0){
					toggleCollapseState();
				}
				return true;
			}
		};
	}
	
	@Override
	protected void refreshVisuals() {
		super.refreshVisuals();
		
		configureCollapsableFigure();
	}
	
	protected void configureCollapsableFigure(){
		State s = null;
		ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
		if(getEObjectModel() instanceof State){
			s = (State) getEObjectModel();
		}
		if (this.collapsableFigure != null) {
//			if(!(getExpandedChildren().size() > 0))
			if(isCollapsed()){
				if(s != null){
					if(s.getPlans().size() > 0){
						this.collapsableFigure.hideCollapseControls();
						this.collapsableFigure.getArrowLabel().setIcon(reg.get(PlanDesignerConstants.ICON_ARROW_RIGHT_10));
					}else{
						this.collapsableFigure.hideCollapseControls();
					}
				}else{
					this.collapsableFigure.hideCollapseControls();
				}
			}else{
				this.collapsableFigure.unhideCollapsControls();
				this.collapsableFigure.setCollapsed(isCollapsed());
			}
		}
	}
}
