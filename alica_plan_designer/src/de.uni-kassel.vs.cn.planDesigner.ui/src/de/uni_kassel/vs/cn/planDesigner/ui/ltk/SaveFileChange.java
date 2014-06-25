package de.uni_kassel.vs.cn.planDesigner.ui.ltk;

import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class SaveFileChange extends Change {
	
	private EObject owner;
	private PlanElement planElement;
	
	public SaveFileChange(EObject owner, PlanElement planElement) {
		this.owner = owner;
		this.planElement = planElement;
	}

	@Override
	public Object getModifiedElement() {
		return owner;
	}

	@Override
	public String getName() {
		return "Save " + owner.eResource().getURI().toFileString() + " because of a changed reference to " + planElement.toString();
	}

	@Override
	public void initializeValidationData(IProgressMonitor pm) {
	}

	@Override
	public RefactoringStatus isValid(IProgressMonitor arg0) throws CoreException, OperationCanceledException {
		return new RefactoringStatus();
	}

	@Override
	public Change perform(IProgressMonitor pm) throws CoreException {
		try {
			// Save the resource in which we have changed something
			Resource res = owner.eResource();
			if (res != null)
			{
				res.save(res.getResourceSet().getLoadOptions());
				
				// reload the resource
				res.unload();
				res.load(res.getResourceSet().getLoadOptions());
			}
		} catch (Exception e) {
			Status s = new Status(Status.ERROR, PlanDesignerActivator.PLUGIN_ID, 94, "Change command cannot be executed", e);
			throw new CoreException(s);
		}
		return null; // no undo possible! 
	}

}
