package de.uni_kassel.vs.cn.planDesigner.ui.ltk;

import java.util.Map;

import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class MoveFileChange extends Change {

	private Resource resourceToMove;
	private URI uri;

	public MoveFileChange(Resource resourceToMove, URI uri) {
		this.resourceToMove = resourceToMove;
		this.uri = uri;
	}

	@Override
	public Object getModifiedElement() {
		return resourceToMove;
	}

	@Override
	public String getName() {
		return "Changing Reference from " +resourceToMove.getURI() +" to " + uri;
	}

	@Override
	public void initializeValidationData(IProgressMonitor pm) {
//		System.out.println("initializeValidationData");
	}

	@Override
	public RefactoringStatus isValid(IProgressMonitor arg0) throws CoreException, OperationCanceledException {
//		System.out.println("isValid");
		RefactoringStatus status = new RefactoringStatus();
		return status;
	}

	@Override
	public Change perform(IProgressMonitor pm) throws CoreException {
		try {
//			System.out.println("perform");
			
			resourceToMove.setURI(uri);
			
			// Save the resource in which we have changed something
			Map<Object, Object> loadOptions = resourceToMove.getResourceSet().getLoadOptions();
			resourceToMove.save(loadOptions);
				
			// reload the resource, making the Plan Designer recognising the changes
			resourceToMove.unload();
			resourceToMove.load(loadOptions);
		
		} catch (Exception e) {
			Status s = new Status(Status.ERROR, PlanDesignerActivator.PLUGIN_ID, 94, "Change command cannot be executed", e);
			throw new CoreException(s);
		}
		return null;
	}

}
