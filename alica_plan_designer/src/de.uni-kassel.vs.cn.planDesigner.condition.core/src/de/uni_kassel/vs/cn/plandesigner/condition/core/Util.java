package de.uni_kassel.vs.cn.plandesigner.condition.core;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.TransactionalEditingDomain;

import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class Util {
	
	/**
	 * Method which returns every condition from every plan.
	 * 
	 * @return
	 */
	public static Map<Plan, List<Condition>> getAllConditions() {
		TransactionalEditingDomain editingDomain = null;
		if (editingDomain == null) {
			editingDomain = (PMLTransactionalEditingDomain) TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		}
		// Set<IFile> filesInWorkspace =
		ResourceSet rSet = editingDomain.getResourceSet();

		// problem: condition hat zwar getAbstractPlan, aber für keine Condition
		// existiert Kante für plan, deswegen Map
		Map<Plan, List<Condition>> allConditionsMap = new HashMap<Plan, List<Condition>>();

		EList<Resource> resources = rSet.getResources();
		for (Resource r : resources) {
			EList<EObject> contents = r.getContents();

			// get every condition from plan...
			if (r.getContents().size() == 1 && r.getContents().get(0) instanceof Plan) {
				Plan plan = (Plan) contents.get(0);

				List<Condition> tempList = new ArrayList<Condition>();
				tempList.addAll(plan.getConditions());

				// and ever precondition from plans transistions
				EList<Transition> transitions = plan.getTransitions();
				for (Transition t : transitions) {
					Condition preCondition = t.getPreCondition();
					if (preCondition != null) {
						tempList.add(preCondition);
					}
				}

				allConditionsMap.put(plan, tempList);
			}
		}

		return allConditionsMap;
	}


}
