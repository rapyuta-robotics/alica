// Copyright 2009 Distributed Systems Group, University of Kassel
// This program is distributed under the GNU Lesser General Public License (LGPL).
//
// This file is part of the Carpe Noctem Software Framework.
//
//    The Carpe Noctem Software Framework is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    The Carpe Noctem Software Framework is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
package de.uni_kassel.vs.cn.planDesigner.alica.util;

import java.util.HashMap;
import java.util.Map;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.BasicExtendedMetaData;
import org.eclipse.emf.ecore.util.ExtendedMetaData;
import org.eclipse.emf.ecore.xmi.XMIResource;
import org.eclipse.emf.ecore.xmi.XMLResource;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;

public class AlicaSerializationHelper {
	
	private static AlicaSerializationHelper instance = null;
	
	private Map<Object, Object> optionsMap;
	
	private AlicaSerializationHelper(){
	}
	
	public Map<Object, Object> getLoadSaveOptions(){
		if(optionsMap == null){
			optionsMap = new HashMap<Object,Object>();
			
			// Create a new meta object
			ExtendedMetaData meta = new BasicExtendedMetaData();
			
			// Contribute some options			
			createEntryPointSaveOptions(meta);
			createTerminalStateSaveOptions(meta);
			createPlanSaveOptions(meta);
			createPlanTypeSaveOptions(meta);
			createPlanningProblemSaveOptions(meta);
			createStateSaveOptions(meta);
			createTransitionSaveOptions(meta);
			createRoleTaskMappingSaveOptions(meta);
			createConditionSaveOptions(meta);
			createTaskSaveOptions(meta);
			createParametrisationSaveOptions(meta);
			createAnnotatedPlanSaveOptions(meta);
			createCharateristicsSaveOptions(meta);
			
			// Save only resources that have actually changed.
			optionsMap.put(Resource.OPTION_SAVE_ONLY_IF_CHANGED, Resource.OPTION_SAVE_ONLY_IF_CHANGED_MEMORY_BUFFER);
			optionsMap.put(XMIResource.OPTION_EXTENDED_META_DATA, meta);
			
			// Save default attributes
			optionsMap.put(XMLResource.OPTION_KEEP_DEFAULT_CONTENT, Boolean.TRUE);

		}
		
		return optionsMap;
	}

	public static AlicaSerializationHelper getInstance() {
		if(instance == null){
			AlicaSerializationHelper.instance = new AlicaSerializationHelper();
		}
		return instance;
	}
	private void createCharateristicsSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getCharacteristic_Value(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getCharacteristic_Capability(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	private void createPlanSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlan_States(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlan_Transitions(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlan_Synchronisations(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlan_EntryPoints(),ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createPlanningProblemSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlanningProblem_Plans(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createTaskSaveOptions(ExtendedMetaData meta){
		
	}
	
	private void createParametrisationSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getParametrisation_Subplan(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getParametrisation_Subvar(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getParametrisation_Var(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	private void createAnnotatedPlanSaveOptions(ExtendedMetaData meta) {
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getAnnotatedPlan_Plan(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createRoleTaskMappingSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getRoleSet_Mappings(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getRoleTaskMapping_Role(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createPlanTypeSaveOptions(ExtendedMetaData meta){
		//meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlanType_Plans(), ExtendedMetaData.ELEMENT_FEATURE);
		//meta.setFeatureKind(AlicaPackage.eINSTANCE.getPlanType_Parametrisation(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createTransitionSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getTransition_PreCondition(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getTransition_InState(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getTransition_OutState(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getTransition_Synchronisation(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createStateSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getState_Plans(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getState_InTransitions(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getState_OutTransitions(), ExtendedMetaData.ELEMENT_FEATURE);
		//meta.setFeatureKind(AlicaPackage.eINSTANCE.getState_Parametrisation(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	/*
	private void createAbstractPlanSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getAbstractPlan_Conditions(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getAbstractPlan_Rating(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getAbstractPlan_Utilities(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	*/
	private void createTerminalStateSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getTerminalState_PostCondition(), ExtendedMetaData.ELEMENT_FEATURE);		
	}
	
	private void createEntryPointSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getEntryPoint_State(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getEntryPoint_Task(), ExtendedMetaData.ELEMENT_FEATURE);
	}
	
	private void createConditionSaveOptions(ExtendedMetaData meta){
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getCondition_AbstractPlan(), ExtendedMetaData.ELEMENT_FEATURE);
		meta.setFeatureKind(AlicaPackage.eINSTANCE.getCondition_Vars(), ExtendedMetaData.ELEMENT_FEATURE);
	}

}
