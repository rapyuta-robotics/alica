package de.uni_kassel.vs.cn.planDesigner.ui.ltk;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.mapping.IResourceChangeDescriptionFactory;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.CompositeChange;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;
import org.eclipse.ltk.core.refactoring.participants.CheckConditionsContext;
import org.eclipse.ltk.core.refactoring.participants.MoveParticipant;
import org.eclipse.ltk.core.refactoring.participants.ResourceChangeChecker;
import org.eclipse.ltk.core.refactoring.resource.DeleteResourceChange;

import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

/**
 * MoveUIExtFileParticipant reacts to the movement of an .pml-File (Plans) and moves
 * the corresponding .pmlex-File (Description for the arrangements of the plan elements) accordingly.
 * @author Stephan Opfer
 */
public class MoveUIExtFileParticipant extends MoveParticipant {

	private IFile toMoveFile;
	private IFile toMoveUIExtFile;
	private Resource toMoveUIExtFileResource;
	
	private PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionalEditingDomain.Registry.INSTANCE
			.getEditingDomain(PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
	
	public MoveUIExtFileParticipant() {
	}

	@Override
	protected boolean initialize(Object element) {
		if (element instanceof IFile) {
			this.toMoveFile = (IFile) element;
			// only move pmlex files according to pml files (Plans)
			return this.toMoveFile.getFileExtension().equals("pml");
		}
		return false;
	}

	@Override
	public RefactoringStatus checkConditions(IProgressMonitor pm, CheckConditionsContext context)
			throws OperationCanceledException {
		RefactoringStatus result = new RefactoringStatus();

		// Add the .pmlex file which we modify to the deltaFactory.
		ResourceChangeChecker checker = (ResourceChangeChecker) context.getChecker(ResourceChangeChecker.class);
		IResourceChangeDescriptionFactory deltaFactory = checker.getDeltaFactory();

		toMoveUIExtFile = PlanEditorUtils.findUIExtensionFile(toMoveFile);
		// Resource uiExtResource = editingDomain.load(uiExtFile);
		//		
		// IFile fileToChange = WorkspaceSynchronizer.getUnderlyingFile(uiExtResource);
		deltaFactory.change(toMoveUIExtFile);

		return result;
	}

	@Override
	public Change createChange(IProgressMonitor pm) throws CoreException, OperationCanceledException {
		CompositeChange cmpChange = new CompositeChange("Move UI Extension File Composite Change");

		// create change for the file, which should be moved
		String newPath = ((IResource) this.getArguments().getDestination()).getFullPath().toString()+ java.io.File.separator + toMoveUIExtFile.getName();
		cmpChange.add(new MoveFileChange(this.getToMoveUIExtResource(), URI.createPlatformResourceURI(newPath, true)));
		cmpChange.add (new DeleteResourceChange(toMoveUIExtFile.getFullPath(), true));
		return cmpChange;
	}

	@Override
	public String getName() {
		return "Move File Participant";
	}
	
	/**
	 * Determines the Plan Element Resource, whose file should be moved.
	 * 
	 * @return planElementToMoveResource
	 */
	private Resource getToMoveUIExtResource() {
		if (toMoveUIExtFileResource == null) {
			toMoveUIExtFileResource = editingDomain.load(toMoveUIExtFile);
		}
		return toMoveUIExtFileResource;
	}

}
