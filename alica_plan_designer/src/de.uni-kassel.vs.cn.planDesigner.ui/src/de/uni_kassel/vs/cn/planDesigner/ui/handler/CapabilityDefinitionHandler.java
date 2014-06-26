package de.uni_kassel.vs.cn.planDesigner.ui.handler;

import java.io.IOException;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.window.Window;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.handlers.HandlerUtil;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CapabilityDefinitionDialog;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class CapabilityDefinitionHandler extends AbstractHandler {
	IWorkbenchWindow activeWorkbenchWindow;
	@Override
	public Object execute(ExecutionEvent event) throws ExecutionException {
		// Try to open the CapabilityDefinitionSetFile
		IPath path = CommonUtils.getCapabilityDefinitionPath();
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IFile capabilityDefFile = root.getFile(path);
		activeWorkbenchWindow = HandlerUtil.getActiveWorkbenchWindow(event);

		PMLTransactionalEditingDomain domain = (PMLTransactionalEditingDomain) TransactionalEditingDomain.Registry.INSTANCE
				.getEditingDomain(PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);

		if (domain != null) {
			ResourceSet rSet = domain.getResourceSet();
			final Resource resource;

			if (!capabilityDefFile.exists()) {
				if (!createCapabilityDefinitionFile(capabilityDefFile))
					return null;
				else {
					resource = rSet.createResource(URI
							.createPlatformResourceURI(path.toString(), true));
					domain.getCommandStack().execute(
							new RecordingCommand(domain) {
								@Override
								protected void doExecute() {
									// Add a new CapabilityDefinitionSet
									resource.getContents()
											.add(AlicaFactory.eINSTANCE
													.createCapabilityDefinitionSet());
								}
							});

				}
			} else
				resource = rSet.getResource(
						URI.createPlatformResourceURI(path.toString(), true),
						true);
			CapabilityDefinitionDialog cdDialog = new CapabilityDefinitionDialog(
					activeWorkbenchWindow, domain);
			cdDialog.create();
			cdDialog.setInput(resource);
			if (cdDialog.open() == Window.OK) {
				try {
					resource.save(AlicaSerializationHelper.getInstance()
							.getLoadSaveOptions());
				} catch (IOException e) {
					e.printStackTrace();
				}
			}

			resource.eAdapters().clear();
			rSet.getResources().remove(resource);
			rSet = null;	
		}
		return null;
	}

	private boolean createCapabilityDefinitionFile(final IFile path){
		RunnableWithResult<Boolean> run = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				setResult(MessageDialog.openQuestion(activeWorkbenchWindow.getShell(), "File not found", "The capability definition file could not be found in\n\n" +
						"\t"+path.toString()+"\n\nShould a new file be created?"));
			}
		};
		
		activeWorkbenchWindow.getShell().getDisplay().syncExec(run);
		return run.getResult();
	}

}
