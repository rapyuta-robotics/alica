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
package de.uni_kassel.vs.cn.planDesigner.ui.figures;

import org.eclipse.draw2d.ColorConstants;
import org.eclipse.draw2d.Figure;
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.GridData;
import org.eclipse.draw2d.GridLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.draw2d.OrderedLayout;
import org.eclipse.draw2d.RoundedRectangle;
import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.gef.handles.HandleBounds;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Font;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.PMLXYLayout;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class PlanFigure extends Figure implements HandleBounds
{
	
	private IFigure mainFigure;
	
	private IFigure exitPointFigure;
	
	private IFigure entryPointFigure;
	
	private IFigure headerFigure;
	
	private IFigure nameHolder;
	
	private IFigure resultHolder;
	
	private IFigure preconditionHolder;
	
	private IFigure runtimeConditionHolder;
	
	private Label nameLabel;
	
	private IFigure handleBoundsFigure;
	
	private PlanEditPart planEditPart;

	public PlanFigure(PlanEditPart planEditPart)
	{
		this.planEditPart = planEditPart; // to determine if it is an master plan or not
		
		GridLayout gl = new GridLayout();
		gl.numColumns = 3;
		gl.horizontalSpacing = 0;
		gl.verticalSpacing = 0;
		gl.marginHeight = 1;
		gl.marginWidth = 0;
		setLayoutManager(gl);
		
		// Add the entrypoint figure
		add(getEntryPointFigure(), new GridData(SWT.CENTER,SWT.CENTER,false,true));
		
		handleBoundsFigure = new RoundedRectangle();
		
		gl = new GridLayout();
		gl.marginHeight = 0;
		gl.marginWidth = 0;
		handleBoundsFigure.setLayoutManager(gl);
		
		
		// Add the name label
		handleBoundsFigure.add(getHeaderFigure(), new GridData(SWT.FILL,SWT.BEGINNING,true,false));
		
		// Add the mainFigure;
		handleBoundsFigure.add(getMainFigure(), new GridData(SWT.FILL,SWT.FILL,true,true));
		
		// Add the runtimeConditionFigure
		handleBoundsFigure.add(getRuntimeConditionHolder(),new GridData(SWT.FILL,SWT.END,true,false));
		
		// Add the middle figure which will hold the main content
		add(handleBoundsFigure, new GridData(SWT.FILL,SWT.FILL,true,true));
		
		// Add the exitpoint figure
		add(getExitPointFigure(), new GridData(SWT.CENTER,SWT.CENTER,false,true));
		
		setSize(new Dimension(200,200));
	}
	
	public Rectangle getHandleBounds()
	{
		return handleBoundsFigure.getBounds().getCopy();
	}

	public IFigure getMainFigure()
	{
		if(this.mainFigure == null){
			this.mainFigure = new Figure();
			this.mainFigure.setLayoutManager(new PMLXYLayout(this.planEditPart));
		}
		return mainFigure;
	}

	public IFigure getExitPointFigure()
	{
		if(this.exitPointFigure == null){
			this.exitPointFigure = new Figure();
			this.exitPointFigure.setLayoutManager(new FlowLayout(false));
		}
		return exitPointFigure;
	}

	public IFigure getEntryPointFigure()
	{
		if(this.entryPointFigure == null){
			this.entryPointFigure = new Figure();
			FlowLayout fl = new FlowLayout(false);
			fl.setMinorAlignment(OrderedLayout.ALIGN_BOTTOMRIGHT);
			this.entryPointFigure.setLayoutManager(fl);
		}
		return entryPointFigure;
	}

	public IFigure getHeaderFigure()
	{
		if(this.headerFigure == null){
			this.headerFigure = new Figure();
			this.headerFigure.setLayoutManager(new GridLayout(3,false));
			this.headerFigure.setPreferredSize(new Dimension(-1,30));
			this.headerFigure.add(getPreconditionHolder(),new GridData(SWT.BEGINNING,SWT.BEGINNING,false,false));
			this.headerFigure.add(getNameHolder(),new GridData(SWT.FILL,SWT.FILL,true,true));
			this.headerFigure.add(getResultHolder(),new GridData(SWT.END,SWT.BEGINNING,false,false));
		}
		return headerFigure;
	}
	
	public IFigure getNameHolder(){
		if (this.nameHolder == null) {
			this.nameHolder = new Figure();
			PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
			if (((Plan)planEditPart.getModel()).isMasterPlan()) {
				nameHolder.setBackgroundColor(plugin.getColorRegistry().get(PlanDesignerConstants.MASTER_PLAN_LABEL_BACKGROUND_COLOR));
			} else {	
				nameHolder.setBackgroundColor(plugin.getColorRegistry().get(PlanDesignerConstants.PLAN_LABEL_BACKGROUND_COLOR));
			}
			GridLayout gLayout = new GridLayout();
			gLayout.marginHeight = 0;
			gLayout.marginWidth = 0;
			nameHolder.setLayoutManager(gLayout);
			nameHolder.setOpaque(true);
			nameHolder.add(getNameLabel(), new GridData(SWT.CENTER,SWT.CENTER,true,true));
		}
		return this.nameHolder;
	}
	
	public Label getNameLabel()
	{
		if(this.nameLabel == null)
		{
			this.nameLabel = new Label();
			this.nameLabel.setFont(new Font(null,"",11,SWT.NORMAL));
			this.nameLabel.setForegroundColor(ColorConstants.white);
		}
		return nameLabel;
	}

	public IFigure getResultHolder()
	{
		if(this.resultHolder == null){
			this.resultHolder = new Figure();
			this.resultHolder.setLayoutManager(new FlowLayout());
		}
		return resultHolder;
	}

	public IFigure getPreconditionHolder()
	{
		if(this.preconditionHolder == null){
			this.preconditionHolder = new Figure();
			this.preconditionHolder.setLayoutManager(new FlowLayout());
		}
		return preconditionHolder;
	}

	public IFigure getRuntimeConditionHolder()
	{
		if(runtimeConditionHolder == null){
			this.runtimeConditionHolder = new PlanRuntimeConditionFigure();
			this.runtimeConditionHolder.setBorder(new MarginBorder(7));
			
			FlowLayout fl = new FlowLayout();
			fl.setMajorAlignment(FlowLayout.ALIGN_CENTER);
			this.runtimeConditionHolder.setLayoutManager(fl);
		}
		return runtimeConditionHolder;
	}

}
