package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.commands.operations.IOperationHistoryListener;
import org.eclipse.core.commands.operations.IUndoableOperation;
import org.eclipse.core.commands.operations.OperationHistoryEvent;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.workspace.IWorkspaceCommandStack;
import org.eclipse.emf.workspace.ResourceUndoContext;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;
import org.eclipse.gef.ui.actions.WorkbenchPartAction;
import org.eclipse.ui.IEditorPart;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.PartInitException;

import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.NonUndoableCommandWrap;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.SwitchResourceContentsCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.exception.PlanNotFoundException;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskGraphEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.GraphDelta;
import de.uni_kassel.vs.cn.planDesigner.ui.util.RolesetEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.TaskGraphBuilder;

public class UpdateRoleSetAction extends WorkbenchPartAction {

	public static final String ID = "updateaction";
	private Resource rolesetResource;

	public UpdateRoleSetAction(IWorkbenchPart part, Resource rolesetResource) {
		super(part);
		setId(ID);
		setText("Update Plan");
		this.rolesetResource = rolesetResource;
	}

	@Override
	public void run() {
		RolesetEditor editor = (RolesetEditor) getWorkbenchPart();
		try {
			editor.updatePlan();
		} catch (PartInitException e) {

			e.printStackTrace();
		}

		Map<Object, AbstractGraphicalEditPart> registry = editor.getGraphicalViewer().getEditPartRegistry();
		TaskGraphEditPart diagram = (TaskGraphEditPart)registry.get(editor.getTaskGraph());
		diagram.expandOrCollapseAll(false);
		diagram.doAutoLayout();
	}

	@Override
	protected boolean calculateEnabled() {
		return true;
	}
}
