package de.uni_kassel.vs.cn.planDesigner.ui.handler;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.commands.IHandler;
import org.eclipse.core.resources.IFile;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.util.EcoreUtil.UsageCrossReferencer;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.handlers.HandlerUtil;

import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UsageDialog;

public class ShowUsageHandler extends AbstractHandler implements IHandler {

	public Object execute(ExecutionEvent event) throws ExecutionException {
		IStructuredSelection selection = (IStructuredSelection) HandlerUtil.getCurrentSelection(event);
		if (selection.isEmpty())
			return null;

		// find all references to the selected elements
		PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionalEditingDomain.Registry.INSTANCE
				.getEditingDomain(PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);

		List<EObject> resolvedList = loadUsages(selection, editingDomain);

		Map<EObject, Collection<EStructuralFeature.Setting>> referenceMap = UsageCrossReferencer.findAll(resolvedList, editingDomain.getResourceSet());

		// filtered map (only PlanElements)
		List<TreeNode> usages = new ArrayList<TreeNode>();
		for (EObject obj : referenceMap.keySet()) {
			// Create children list out of references 
			List<TreeNode> childrenList = new ArrayList<TreeNode>();
			for (EStructuralFeature.Setting setting : referenceMap.get(obj)) {
				if (setting.getEObject() instanceof PlanElement) {
					childrenList.add(new TreeNode((PlanElement) setting.getEObject()));
				}
			}
			if (childrenList.isEmpty()) 
				childrenList.add(new TreeNode("Isn't used anywhere."));
			
			
			if (obj instanceof BehaviourConfiguration) {
				// Search for the Behaviour-TreeNode in usages
				PlanElement beh = ((BehaviourConfiguration) obj).getBehaviour();
				TreeNode behTn = null;
				for (TreeNode searchTn : usages) {
					if (((PlanElement)searchTn.getValue()).equals(beh)) {
						behTn = searchTn;
						break;
					}
				}
				
				// Create the behaviourConf-TreeNode
				TreeNode behConfTn = new TreeNode(obj);
				behConfTn.setChildren(childrenList.toArray(new TreeNode[0]));
				
				if (behTn == null) { 
					// Found nothing, so create it and add the first child
					behTn = new TreeNode (beh);
					behTn.setChildren(new TreeNode[]{behConfTn});
					usages.add (behTn);
				} else {
					// Found one, so create a new child array and add the additional child
					TreeNode[] combinedChildren = new TreeNode[behTn.getChildren().length + 1];
					int i = 0;
					for (TreeNode child : behTn.getChildren()) {
						combinedChildren[i++] = child;
					}
					combinedChildren[combinedChildren.length-1] = behConfTn;
					behTn.setChildren(combinedChildren);
				}
				
			} else if (obj instanceof PlanElement) {
				TreeNode tn = new TreeNode((PlanElement) obj);
				
				tn.setChildren(childrenList.toArray(new TreeNode[0]));
				usages.add(tn);
			}
		}

		// for debugging
//		for (TreeNode tn : usages) {
//			System.out.println(tn.toString());
//			System.out.println(((PlanElement) tn.getValue()).getName() + ":");
//			if (tn.hasChildren()) {
//				for (TreeNode child : tn.getChildren()) {
//					if (child.getValue() instanceof PlanElement) {
//						System.out.println("\t" + ((PlanElement) child.getValue()).getName());
//						if (child.hasChildren()) {
//							for (TreeNode child1 : child.getChildren()) {
//								if (child1.getValue() instanceof PlanElement)
//									System.out.println("\t\t" + ((PlanElement) child1.getValue()).getName());
//							}
//						}
//					} else { 
//						System.out.println("\t" + child.getValue());
//					}
//				}
//			}
//		}

		// some strings for the usage dialog
		String titleString;
		String subString;
		if (usages.size() > 1) {
			titleString = "Usages of " + usages.size() + " PlanElements";
			subString = "The PlanElements are used at the following places.";
		} else if (usages.size() == 1) {
			String name = ((PlanElement) usages.get(0).getValue()).getName();
			titleString = "Usages of " + name;
			subString = "The PlanElement \"" + name + "\" is used at the following places.";
		} else {
			titleString = "No Usages found";
			subString = "Maybe you want to delete it? :-)";
		}

		// generate the dialog
		Shell activeShell = HandlerUtil.getActiveShell(event);
		final UsageDialog dialog = new UsageDialog(activeShell, usages, titleString, subString, MessageDialog.INFORMATION);

		// run the dialog
		activeShell.getDisplay().asyncExec(new Runnable() {
			public void run() {
				dialog.open();
			}
		});

		return null;
	}

	/**
	 * Load all Elements which could refer to the selected Elements.
	 */
	private List<EObject> loadUsages(IStructuredSelection selection, PMLTransactionalEditingDomain editingDomain) {
		
		@SuppressWarnings("unchecked")
		List<EObject> selectionList = selection.toList();
		ArrayList<EObject> resolvedList = new ArrayList<EObject>();
		for (EObject obj : selectionList) {
			resolvedList.add(EcoreUtil.resolve(obj, editingDomain.getResourceSet()));
			if (obj instanceof Behaviour) {
				for (EObject behConf : ((Behaviour) obj).getConfigurations()) {
					resolvedList.add(behConf);					
				}
			}
		}

		String ext;
		IFile fileToChange;
		Resource ownerResource;
		Set<IFile> files = new HashSet<IFile>();
		
		for (EObject obj : resolvedList) {
			ownerResource = editingDomain.loadResource(EcoreUtil.getURI(obj).toPlatformString(true));
			fileToChange = WorkspaceSynchronizer.getUnderlyingFile(ownerResource);
			ext = fileToChange.getFileExtension();

			/*
			 * Load everything which could reference to the file, you want to move: 
			 * Plans (.pml) -> [.pml, .pty, .pmlex]
			 * Behaviours (.beh) -> [.pml, .pmlex] 
			 * PlanTypes (.pty) -> [.pml, .pmlex] 
			 * RoleDefSet (.rdefset) -> [.rset, .graph, .pmlex]
			 * RolesetGraph (.graph) -> [.pmlex] 
			 * UI-Arrangements (.pmlex) -> [nothing] 
			 * Roleset (.rset) -> [nothing]
			 */
			if (ext.equals("pml")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("pml", "pty", "pmlex"));
			} else if (ext.equals("beh") || ext.equals("pty")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("pml", "pmlex"));
			} else if (ext.equals("rdefset")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("rset", "graph", "pmlex"));
			} else if (ext.equals("graph")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("pmlex"));
			}
		}

		// The load operation is idempotent.
		for (IFile file : files) {
			((PMLTransactionalEditingDomain) editingDomain).load(file);
		}

		return resolvedList;
	}

}
