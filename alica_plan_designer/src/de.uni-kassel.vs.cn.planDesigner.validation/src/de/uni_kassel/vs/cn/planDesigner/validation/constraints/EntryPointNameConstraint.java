package de.uni_kassel.vs.cn.planDesigner.validation.constraints;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.validation.AbstractModelConstraint;
import org.eclipse.emf.validation.IValidationContext;

import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;



public class EntryPointNameConstraint extends AbstractModelConstraint {

	@Override
	public IStatus validate(IValidationContext ctx) {
		EObject eObj = ctx.getTarget();
		if (eObj instanceof EntryPoint) {
			EntryPoint e = (EntryPoint)eObj;
			if (e.getTask() != null && ! e.getName().equals(e.getTask().getName())) {
				e.setName(e.getTask().getName());
			}
		
		}
		return ctx.createSuccessStatus();
	}

}
