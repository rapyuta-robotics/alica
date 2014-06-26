package de.uni_kassel.vs.cn.planDesigner.validation.constraints;

import java.util.LinkedList;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.validation.AbstractModelConstraint;
import org.eclipse.emf.validation.EMFEventType;
import org.eclipse.emf.validation.IValidationContext;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.State;


public class ReferencedSubPlanExists extends AbstractModelConstraint {

	@Override
	public IStatus validate(IValidationContext ctx) {
		EObject eObj = ctx.getTarget();
		EMFEventType eType = ctx.getEventType();
		
		//System.out.println("validating target: "+eObj);
		if(eType != EMFEventType.NULL) System.out.println("new value"+ctx.getFeatureNewValue());
		
		LinkedList<Parametrisation> toRemove = new LinkedList<Parametrisation>();
		
		if (eObj instanceof State) {
				State s = (State)eObj;
				EList<AbstractPlan> plans = s.getPlans();				
				for(Parametrisation p : s.getParametrisation()) {					
					if (!plans.contains(p.getSubplan())) {
						toRemove.add(p);
					} else {
						boolean found = false;
						for(AbstractPlan subp : plans) {
							if(subp.getVars().contains(p.getSubvar())) {
								found = true;
								break;
							}
						}
						if (!found) {
							toRemove.add(p);
						}
					}
					
				}
				s.getParametrisation().removeAll(toRemove);
		}
		else if (eObj instanceof PlanType) {
			PlanType pt = (PlanType)eObj;
			EList<AnnotatedPlan> subplans = pt.getPlans();
			
			for(Parametrisation p : pt.getParametrisation()) {					
				if (!subplans.contains(p.getSubplan())) {
					toRemove.add(p);
				} else {
					boolean found = false;
					for(AnnotatedPlan subpa : subplans) {
						AbstractPlan subp = subpa.getPlan();
						if(subp.getVars().contains(p.getSubvar())) {
							found = true;
							break;
						}
					}
					if (!found) {
						toRemove.add(p);
					}
				}				
			}
			pt.getParametrisation().removeAll(toRemove);
		}
		
			
		
		
		return ctx.createSuccessStatus();

	}
	
	
}
