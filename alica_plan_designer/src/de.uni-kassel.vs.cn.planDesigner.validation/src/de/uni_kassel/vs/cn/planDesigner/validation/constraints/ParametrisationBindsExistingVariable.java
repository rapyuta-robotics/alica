package de.uni_kassel.vs.cn.planDesigner.validation.constraints;

import java.util.LinkedList;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.validation.AbstractModelConstraint;
import org.eclipse.emf.validation.EMFEventType;
import org.eclipse.emf.validation.IValidationContext;

import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;

public class ParametrisationBindsExistingVariable extends AbstractModelConstraint {

	@Override
	public IStatus validate(IValidationContext ctx) {
		EObject eObj = ctx.getTarget();
		EMFEventType eType = ctx.getEventType();
		
		//System.out.println("validating target: "+eObj);
		if(eType != EMFEventType.NULL) System.out.println("new value"+ctx.getFeatureNewValue());
		
		LinkedList<Parametrisation> toRemove = new LinkedList<Parametrisation>();
		
		
		if (eObj instanceof Plan) {
				Plan plan = (Plan)eObj;
				EList<Variable> vars = plan.getVars();
				for(State s: plan.getStates()) {
					toRemove.clear();
					EList<Parametrisation> ps = s.getParametrisation();					
					for(Parametrisation p : ps) {					
						if(!vars.contains(p.getVar())) {
							toRemove.add(p);							
						}
	
					}
					s.getParametrisation().removeAll(toRemove);
				}
		}
		else if (eObj instanceof PlanType) {
			PlanType pt = (PlanType)eObj;
			EList<Parametrisation> ps = pt.getParametrisation();
			EList<Variable> vars = pt.getVars();
			for(Parametrisation p : ps) {					
				if(!vars.contains(p.getVar())) {
					toRemove.add(p);							
				}

			}
			pt.getParametrisation().removeAll(toRemove);
		}
		return ctx.createSuccessStatus();

	}

}
