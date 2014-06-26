package de.uni_kassel.vs.cn.planDesigner.validation.constraints;

import java.util.Collection;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.util.EcoreUtil.UsageCrossReferencer;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.validation.AbstractModelConstraint;
import org.eclipse.emf.validation.EMFEventType;
import org.eclipse.emf.validation.IValidationContext;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class VariableReferencedInParent extends AbstractModelConstraint {
	
	public IStatus validate(IValidationContext ctx) {
		EObject eObj = ctx.getTarget();
		EMFEventType eType = ctx.getEventType();
		
		//System.out.println("validating target: "+eObj);
		if(eType != EMFEventType.NULL) System.out.println("new value"+ctx.getFeatureNewValue());
		
		//LinkedList<Parametrisation> toRemove = new LinkedList<Parametrisation>();
		
		if (eObj instanceof AbstractPlan) {
			AbstractPlan p = (AbstractPlan)eObj;
			//EList<Variable> vars = p.getVars();
//GET ALL THE ABSTRACTPLANS!
			
			PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
			
			// We assume that at the time this action is invoked (since it can only 
			// be invoked from the repository) all necessary resources are loaded and we can 
			// find all cross-references
			Collection<EStructuralFeature.Setting> usages = UsageCrossReferencer.find(p, editingDomain.getResourceSet());
			
			//.findAll(preparedSelection, editingDomain.getResourceSet());
			System.out.println("----> UP THE TREE FOR PLAN: "+p.getName());
			for(EStructuralFeature.Setting setting : usages){
				System.out.println("------ FOUND Setting: "+setting +" With EOBJECT: "+setting.getEObject());
				if (setting.getEObject() instanceof Parametrisation) {
					Parametrisation toKill = (Parametrisation)setting.getEObject();
					EObject parent = toKill.eContainer();
					if (parent instanceof State) {
						((State)parent).getParametrisation().remove(toKill);
					} else if(parent instanceof PlanType) {
						((PlanType)parent).getParametrisation().remove(toKill);
					}
					else {
						throw new RuntimeException("Unexpected Container for a Parametrisation");
					}
					//((EList<Parametrisation>)toKill.eContainer()).remove(toKill);
				
				}
				
//					System.out.println("\tEObject: " +setting.getEObject() +", Feature: " +setting.getEStructuralFeature());
			}

		}
		
		return ctx.createSuccessStatus();

	}

}
