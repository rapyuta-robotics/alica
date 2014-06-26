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
package de.uni_kassel.vs.cn.planDesigner.ui;

import java.lang.reflect.Field;
import java.net.MalformedURLException;
import java.net.URL;

import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.jface.resource.ColorRegistry;
import org.eclipse.jface.resource.ImageDescriptor;
import org.eclipse.jface.resource.ImageRegistry;
import org.eclipse.swt.graphics.RGB;
import org.eclipse.ui.plugin.AbstractUIPlugin;
import org.osgi.framework.BundleContext;

import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

/**
 * The activator class controls the plug-in life cycle
 */
public class PlanDesignerActivator extends AbstractUIPlugin {

	// The plug-in ID
	public static final String PLUGIN_ID = "de.uni_kassel.vs.cn.planDesigner.ui";

	// The shared instance
	private static PlanDesignerActivator plugin;
	
	/** The color registry handles handles SWT colors for our editor */
	private ColorRegistry colorRegistry;
	
	/** boolean to indicate if colors and images have been initialize */
	private boolean imagesAndColorInitialized = false;
	
	/**
	 * The constructor
	 */
	public PlanDesignerActivator() {
	}
	
	/**2
	 * Creates an image descriptor and places it in the image registry.
	 */
	private void createImageDescriptor(String id, URL baseURL) {
		URL url = null;
//		System.out.println("PlanDesignerActivator.createImageDescriptor(): entry " + baseURL);
		try {
			url = new URL(baseURL, PlanDesignerConstants.ICON_PATH + id);
//			System.out.println("PlanDesignerActivator.createImageDescriptor(): url " + url);
		} catch (MalformedURLException e) {
			e.printStackTrace();
		}
		ImageDescriptor desc = ImageDescriptor.createFromURL(url);
		getImageRegistry().put(id, desc);
	}
	
	private void initializeColors(){
		colorRegistry = new ColorRegistry();
		
		colorRegistry.put(PlanDesignerConstants.ENTRY_POINT_BACKGROUND_COLOR, new RGB(68,145,224));
		colorRegistry.put(PlanDesignerConstants.FAILURE_POINT_BACKGROUND_COLOR, new RGB(255,120,0));
		colorRegistry.put(PlanDesignerConstants.PLAN_STATE_BACKGROUND_COLOR, new RGB(255,249,197));
		colorRegistry.put(PlanDesignerConstants.STATE_BACKGROUND_COLOR, new RGB(255,200,0));
		colorRegistry.put(PlanDesignerConstants.SUCCESS_POINT_BACKGROUND_COLOR, new RGB(169,255,3));
		colorRegistry.put(PlanDesignerConstants.PLAN_TYPE_POINT_BACKGROUND_COLOR, new RGB(255,118,106));
		colorRegistry.put(PlanDesignerConstants.PLAN_LABEL_BACKGROUND_COLOR, new RGB(96,137,255));
		colorRegistry.put(PlanDesignerConstants.MASTER_PLAN_LABEL_BACKGROUND_COLOR, new RGB(255,96,96));
		colorRegistry.put(PlanDesignerConstants.FAILURERANSITION_FOREGROUND_COLOR, new RGB(200,76,76));
		colorRegistry.put(PlanDesignerConstants.SUCCESSTRANSITION_FOREGROUND_COLOR, new RGB(91,192,102));
		
		colorRegistry.put(PlanDesignerConstants.PRIORITY_0_COLOR, new RGB(200,0,0));
		colorRegistry.put(PlanDesignerConstants.PRIORITY_1_COLOR, new RGB(255,204,51));
		colorRegistry.put(PlanDesignerConstants.PRIORITY_2_COLOR, new RGB(21,146,0));
		colorRegistry.put(PlanDesignerConstants.PRIORITY_DEFAULT_COLOR, new RGB(208,208,208));
	}
	
	/**
	 * Initializes the table of images used in this plugin.
	 */
	private void initializeImages() {
		URL baseURL = getBundle().getEntry("/"); //$NON-NLS-1$

		// A little reflection magic ... so that we don't
		// have to add the createImageDescriptor every time
		// we add it to the IBPELUIConstants ..
		Field fields[] = PlanDesignerConstants.class.getFields();	
		for(int i=0; i < fields.length; i++) {
			Field f = fields[i];
			if (f.getType() != String.class) { 
				continue;
			}
			String name = f.getName();
			if (name.startsWith("ICON_")) {   //$NON-NLS-1$ //$NON-NLS-2$
				try {
					String value = (String) f.get(null);
					createImageDescriptor(value, baseURL);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}			
		}
	}
	
	private void initialize(){
		if(!imagesAndColorInitialized){
			imagesAndColorInitialized = true;
			initializeColors();
			initializeImages();
		}
	}
	
	/*
	 * (non-Javadoc)
	 * @see org.eclipse.ui.plugin.AbstractUIPlugin#start(org.osgi.framework.BundleContext)
	 */
	public void start(BundleContext context) throws Exception {
		super.start(context);
		
		// Commented out due to bugs of reloading resources that are saved in editors
//		globalWorkspaceSynchronizer = new WorkspaceSynchronizer(TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
//				IPMLEditorConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID), createSynchronizationDelegate());
		plugin = this;
		
		// Refresh the workspace on startup
		ResourcesPlugin.getWorkspace().getRoot().refreshLocal(IResource.DEPTH_INFINITE, null);

//		ResourcesPlugin.getWorkspace().addResourceChangeListener(getWorkspaceListener(), IResourceDelta.ADDED);		
	}

	/*
	 * (non-Javadoc)
	 * @see org.eclipse.ui.plugin.AbstractUIPlugin#stop(org.osgi.framework.BundleContext)
	 */
	public void stop(BundleContext context) throws Exception {
//		ResourcesPlugin.getWorkspace().removeResourceChangeListener(getWorkspaceListener());
		plugin = null;
		ResourcesPlugin.getWorkspace().save(true,null);
//		globalWorkspaceSynchronizer.dispose();
		super.stop(context);
	}

	/**
	 * Returns the shared instance
	 *
	 * @return the shared instance
	 */
	public static PlanDesignerActivator getDefault() {
		return plugin;
	}

	public ColorRegistry getColorRegistry() {
		if(colorRegistry == null){
			initialize();
		}
		return colorRegistry;
	}
	
	@Override
	public ImageRegistry getImageRegistry() {
		ImageRegistry result = super.getImageRegistry();
		initialize();
		return result;
	}
}
