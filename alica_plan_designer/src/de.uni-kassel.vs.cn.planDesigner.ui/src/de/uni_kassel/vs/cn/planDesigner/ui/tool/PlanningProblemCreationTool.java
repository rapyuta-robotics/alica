package de.uni_kassel.vs.cn.planDesigner.ui.tool;

import java.util.Collections;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.tools.CreationTool;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.wizard.WizardDialog;
import org.eclipse.swt.graphics.Cursor;
import org.eclipse.swt.graphics.Point;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.StateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLNewPlanningProblemWizard;

public class PlanningProblemCreationTool extends CreationTool {
	
	@Override
	protected void performCreation(int button) {
		EditPart target = getTargetEditPart();
		// If target isn't a state we return
		if(target == null || !(target.getModel() instanceof State))
			return;
		
		final EditPartViewer viewer = getCurrentViewer();
		
		PlanningProblem createdPP = (PlanningProblem)getCreateRequest().getNewObject();
		
		// Create a NewPlan wizard and initialize it with the created plan
		PMLNewPlanningProblemWizard wiz = new PMLNewPlanningProblemWizard(createdPP);
		
		WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz){
			@Override
			protected Point getInitialSize() {
				return new Point(600,400);
			}
		};
		
		dialog.setBlockOnOpen(true);
		if(dialog.open() == Dialog.OK){
			// Get the created pp
			final PlanningProblem pp = wiz.getPlanningProblem();
			final Plan p = wiz.getPlan(); 
			
			final PlanEditor editor = PlanEditorUtils.getPlanEditor(target);
			PMLTransactionalEditingDomain editingDomain = editor.getEditingDomain();
			
			if( p != null ){
				CompoundCommand compound = new CompoundCommand(0);
				
				org.eclipse.emf.common.command.Command createChildCommandFirst = null;
				
				//This doesnt work unexecutable commands for one reason that I didnt know
				// Add the new Configuration to the state
//				compound.append(CreateChildCommand.create(
//						editingDomain, 
//						target.getModel(), 
//						new CommandParameter(null,null,pp),
//						Collections.EMPTY_LIST));
				
				createChildCommandFirst = CreateChildCommand.create(
						editingDomain, 
						target.getModel(), 
						new CommandParameter(target.getModel(), AlicaPackage.eINSTANCE.getState_Plans(),p), 
						Collections.emptyList());
				
				// Expand the state
				compound.append(SetCommand.create(
						editingDomain, 
						editor.getUIExtension((EObject)target.getModel(), true), 
						PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
						Boolean.FALSE));
				
				
				
				// Add a command which selects the new configuration
				compound.append(new RecordingCommand(editingDomain){
					@Override
					protected void doExecute() {
						EditPart part = (EditPart)viewer.getEditPartRegistry().get(p);
						if(part != null){
							// Activate the editor
							editor.getSite().getPage().activate(editor);
							viewer.flush();
							viewer.select(part);
						}
					}
				});
				
				// Execute the whole thing
				editingDomain.getCommandStack().execute(createChildCommandFirst);
				editingDomain.getCommandStack().execute(compound);
				
				// Create command which adds the configuration to the targeteditpart
			}
			else {
				
				CompoundCommand compound = new CompoundCommand(0);
				
				org.eclipse.emf.common.command.Command createChildCommandFirst = null;
				
				//This doesnt work unexecutable commands for one reason that I didnt know
				// Add the new Configuration to the state
	//			compound.append(CreateChildCommand.create(
	//					editingDomain, 
	//					target.getModel(), 
	//					new CommandParameter(null,null,pp),
	//					Collections.EMPTY_LIST));
				
				createChildCommandFirst = CreateChildCommand.create(
						editingDomain, 
						target.getModel(), 
						new CommandParameter(target.getModel(), AlicaPackage.eINSTANCE.getState_Plans(),pp), 
						Collections.emptyList());
				
				// Expand the state
				compound.append(SetCommand.create(
						editingDomain, 
						editor.getUIExtension((EObject)target.getModel(), true), 
						PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
						Boolean.FALSE));
				
				
				
				// Add a command which selects the new configuration
				compound.append(new RecordingCommand(editingDomain){
					@Override
					protected void doExecute() {
						EditPart part = (EditPart)viewer.getEditPartRegistry().get(pp);
						if(part != null){
							// Activate the editor
							editor.getSite().getPage().activate(editor);
							viewer.flush();
							viewer.select(part);
						}
					}
				});
				
				// Execute the whole thing
				editingDomain.getCommandStack().execute(createChildCommandFirst);
				editingDomain.getCommandStack().execute(compound);
				
				// Create command which adds the configuration to the targeteditpart
			}
		}	
	}
	
	/**
	 * Overridden to enable the tool when moving over a state
	 */
	@Override
	protected Cursor calculateCursor() {
		if(getTargetEditPart() instanceof StateEditPart)
			return getDefaultCursor();
		else
			return getDisabledCursor();
	}
	
	/**
	 * Overridden to not bother editParts with creation of commands. This was done because
	 * the command doesn't reuse the command's created by the editParts. It builds
	 * it's ownes ones.
	 */
	@Override
	protected Command getCommand() {
		return UnexecutableCommand.INSTANCE;
	}
}
