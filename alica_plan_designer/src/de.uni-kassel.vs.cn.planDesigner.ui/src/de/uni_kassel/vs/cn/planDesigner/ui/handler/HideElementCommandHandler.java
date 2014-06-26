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
package de.uni_kassel.vs.cn.planDesigner.ui.handler;

import java.util.Iterator;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.gef.EditPart;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.ui.handlers.HandlerUtil;

import de.uni_kassel.vs.cn.planDesigner.ui.commands.ExecuteAndRefreshPlanCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.HideElementCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanElementEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UIAwareEditor;

public class HideElementCommandHandler extends AbstractHandler
{

	public Object execute(ExecutionEvent event) throws ExecutionException
	{
		IStructuredSelection selection = (IStructuredSelection)HandlerUtil.getCurrentSelection(event);
		if(!selection.isEmpty())
		{
			PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
			
			CompoundCommand cmp = new CompoundCommand("Hide element");

			final UIAwareEditor editor = findPlanEditor(selection);
			
			for(Iterator<PlanElementEditPart> partIterator = selection.iterator(); partIterator.hasNext();)
			{
				final PlanElementEditPart part = partIterator.next();
				final EObject object = part.getEObjectModel();

				cmp.append(new HideElementCommand(editingDomain, editor, object, true));
			}
			
			// Add a command which will refresh the plan
			ExecuteAndRefreshPlanCommand cmd = new ExecuteAndRefreshPlanCommand(cmp, editor.getRootEditPart());
			editingDomain.getCommandStack().execute(cmd);
		}
		
		return null;
	}

	private UIAwareEditor findPlanEditor(IStructuredSelection selection)
	{
		EditPart part = (EditPart)selection.getFirstElement();
		return CommonUtils.getUIAwareEditorAdapter(part);
	}
	
	

}
