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
package de.uni_kassel.vs.cn.planDesigner.ui.edit;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.common.notify.AdapterFactory;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.xmi.impl.XMIResourceFactoryImpl;
import org.eclipse.emf.ecore.xmi.impl.XMLResourceImpl;
import org.eclipse.emf.transaction.TransactionalCommandStack;
import org.eclipse.emf.transaction.impl.TransactionalEditingDomainImpl;
import org.eclipse.ui.ide.undo.CreateFileOperation;

import de.uni_kassel.vs.cn.planDesigner.alica.impl.AlicaPackageImpl;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelFactory;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.EMF2GEFCommandStack;
//import de.uni_kassel.vs.cn.planDesigner.ui.uiextensionmodel.util.PmlUIExtensionModelResourceFactoryImpl;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

public class PMLTransactionalEditingDomain extends TransactionalEditingDomainImpl {
	
	public PMLTransactionalEditingDomain(AdapterFactory adapterFactory,
			TransactionalCommandStack stack, ResourceSet resourceSet) {
		super(adapterFactory, stack, resourceSet);
		
		initialize();
	}
	
	/**
	 * Loads the UIExtension resource which belongs to the given resource. 'Belongs' means the following:
	 * The filename of the given resource is the same as the one of the UIExtension resource, except it
	 * that is has the file extension .pmlex. If such a file is found in the same directory it will be used,
	 * otherwise one will be created. 
	 * @param res
	 * @return
	 */
	public Resource loadExtensionResource(Resource res){
		IFile pmlexfile = CommonUtils.findUIExtensionFile(res);
		
		if (!pmlexfile.exists()) {
			// The ui extension file doesn't exist yet, so create it.
			// System.out.println("pmlui extension file does not exist!");
			CreateFileOperation cfo = new CreateFileOperation(pmlexfile, null,
					null, "Create PMLDesigner UI Extensions");
			try {
				cfo.execute(null, null);
				
			} catch (ExecutionException e) {
				e.printStackTrace();
			}

			// Build the ui extensions
			try {
				pmlexfile.setContents(buildInitialExtensionContents(),
						IResource.NONE, null);
			} catch (CoreException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		return getResourceSet().getResource(URI.createPlatformResourceURI(pmlexfile.getFullPath()
						.toString(), true), true);
		
	}

	

	private InputStream buildInitialExtensionContents() {
		// Get a new Extensionmap
		PmlUiExtensionMap uiMap = PmlUIExtensionModelFactory.eINSTANCE
				.createPmlUiExtensionMap();

		// Create a new resource
		Resource res = new XMLResourceImpl();

		// Add the extension map to the contents
		res.getContents().add(uiMap);

		// Build a new outputstream which will hold our saved contents
		ByteArrayOutputStream outStream = new ByteArrayOutputStream();

		try {
			res.save(outStream, null);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return new ByteArrayInputStream(outStream.toByteArray());
	}
	
	
	
	/**
	 * Loads the given file and returns the parsed resource.
	 * 
	 * @param file
	 */
	public Resource load(IFile file) {
		Resource loaded = getResourceSet().getResource(URI.createPlatformResourceURI(file.getFullPath().toString(),
				true), true);
//		System.out.println("ResourceSet of " +loaded +": " +loaded.getResourceSet());
		return loaded;
	}
	
	private void initialize() {
		// Initialize the model and the extensionUI package
		AlicaPackageImpl.init();
		PmlUIExtensionModelPackageImpl.init();

		// Retrieve the extension to factory map
		Resource.Factory.Registry reg = Resource.Factory.Registry.INSTANCE;
		Map<String, Object> m = reg.getExtensionToFactoryMap();

		// Associate the .pmlex extension to a XMLResourceFactory
//		m.put("pmlex", new PmlUIExtensionModelResourceFactoryImpl());

		// TODO: Is it necessary to register all file extensions against
		// the XMIResourceFactory? Maybe in case of RCP we have to...!?
		
		// Associate the .pml extension to a PmlResourceFactory
		m.put("pml", new XMIResourceFactoryImpl());

		// Associate the .beh extension to a PmlResourceFactory
		m.put("beh", new XMIResourceFactoryImpl());

		// Associate the .pty extension to a PmlResourceFactory
		m.put("pty", new XMIResourceFactoryImpl());
	}
	
	@Override
	public EMF2GEFCommandStack getCommandStack() {
		return (EMF2GEFCommandStack)super.getCommandStack();
	}
}
