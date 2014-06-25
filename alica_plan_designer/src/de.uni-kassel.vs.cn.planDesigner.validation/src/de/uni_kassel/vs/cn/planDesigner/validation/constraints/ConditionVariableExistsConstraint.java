package de.uni_kassel.vs.cn.planDesigner.validation.constraints;

import java.util.LinkedList;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.validation.AbstractModelConstraint;
import org.eclipse.emf.validation.EMFEventType;
import org.eclipse.emf.validation.IValidationContext;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;

public class ConditionVariableExistsConstraint extends AbstractModelConstraint {

	@Override
	public IStatus validate(IValidationContext ctx) {
		EObject eObj = ctx.getTarget();
		EMFEventType eType = ctx.getEventType();
		
		//System.out.println("validating target: "+eObj);
		if(eType != EMFEventType.NULL) System.out.println("new value"+ctx.getFeatureNewValue());
		
		LinkedList<Variable> toRemove = new LinkedList<Variable>();
		
		if (eObj instanceof AbstractPlan) {
				AbstractPlan p = (AbstractPlan)eObj;
				EList<Variable> vs = p.getVars();
				for(Condition c : p.getConditions()) {
					toRemove.clear();
					for (Variable cv : c.getVars()) {
						if(!vs.contains(cv)) {
							toRemove.add(cv);
							System.out.println("Removing Var "+cv.getName()+" from "+c.getConditionString());
						}
						
					}
					c.getVars().removeAll(toRemove);
				}
				if (p instanceof Plan) {
					Plan pl = (Plan)p;
					for(Transition t: pl.getTransitions()) {
						if (t.getPreCondition()!=null) {
							toRemove.clear();
							for(Variable cv : t.getPreCondition().getVars()) {
								if (!vs.contains(cv)) {
									toRemove.add(cv);
									System.out.println("Removing Var "+cv.getName()+" from "+t);								
								}
								
							}
							t.getPreCondition().getVars().removeAll(toRemove);
						}
					}
				}
		}
		
			/*if (name == null || name.length() == 0) {
				return ctx.createFailureStatus(new Object[] {eObj.eClass().getName()});
			}*/
		
		//System.out.println("Constraint called!");
		return ctx.createSuccessStatus();

	}

}
