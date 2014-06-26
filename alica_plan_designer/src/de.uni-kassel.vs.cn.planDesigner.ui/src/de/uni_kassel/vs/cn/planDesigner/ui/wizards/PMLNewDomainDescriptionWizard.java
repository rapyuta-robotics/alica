package de.uni_kassel.vs.cn.planDesigner.ui.wizards;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Path;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.wizard.Wizard;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.List;
import org.eclipse.ui.INewWizard;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchWizard;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.DomainDescription;
import de.uni_kassel.vs.cn.planDesigner.alica.Planner;
import de.uni_kassel.vs.cn.planDesigner.alica.Planners;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewDomainDescriptionConfigurationWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewDomainDesctiptionWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlannerConfigurationWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlannerWizardPage;

public class PMLNewDomainDescriptionWizard extends Wizard implements INewWizard{

	private DomainDescription dm;
	private PMLTransactionalEditingDomain editingDomain;
	private ISelection selection;
	private PMLNewDomainDesctiptionWizardPage dmPage;
	private PMLNewDomainDescriptionConfigurationWizardPage configurationWizardPage;
	
	public PMLNewDomainDescriptionWizard(){
		this(AlicaFactory.eINSTANCE.createDomainDescription());
	}
	
	public PMLNewDomainDescriptionWizard(DomainDescription dm) {
		super();
		setNeedsProgressMonitor(true);
		setWindowTitle("New Domain Desctiption");
		
		// Connect to the editingDomain
		this.editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		this.dm =  dm;
	}
	
	/**
	 * We will accept the selection in the workbench to see if
	 * we can initialise from it.
	 * @see IWorkbenchWizard#init(IWorkbench, IStructuredSelection)
	 */
	public void init(IWorkbench workbench, IStructuredSelection selection) {
		this.selection = selection;
	}

	@Override
	public boolean performFinish() {
		final String containerName = dmPage.getContainerName();
		final String fileName = PlanEditorUtils.removeFileExtension(dmPage.getFileName()) + ".dm";
		System.out.println("JA ICH MACHE ES");
		IRunnableWithProgress op = new IRunnableWithProgress() {
			public void run(IProgressMonitor monitor) throws InvocationTargetException {
				try {
					doFinish(containerName, fileName, monitor);
				} catch (CoreException e) {
					throw new InvocationTargetException(e);
				} finally {
					monitor.done();
				}
			}
		};
		try {
			getContainer().run(true, false, op);
		} catch (InterruptedException e) {
			return false;
		} catch (InvocationTargetException e) {
			Throwable realException = e.getTargetException();
			MessageDialog.openError(getShell(), "Error", realException.getMessage());
			return false;
		}
		return true;
	}
	
	private void doFinish(String containerName, String fileName,IProgressMonitor monitor)
	throws CoreException {
		// create the planner file
		monitor.beginTask("Creating " + fileName, 1);
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IResource resource = root.findMember(new Path(containerName));
		if (!resource.exists() || !(resource instanceof IContainer)) {
			throwCoreException("Container \"" + containerName + "\" does not exist.");
		}
		
		IContainer container = (IContainer) resource;
		final IFile file = container.getFile(new Path(fileName));
		

		// Create an empty file
		file.create(null, true, monitor);
		// Init the file with the given planners
		monitor.worked(1);
		monitor.subTask("Creating dm contents");
		initFileWithPlanners(file);
		
		monitor.worked(1);
	}
	
	private void initFileWithPlanners(final IFile file){
		final Resource res = editingDomain.getResourceSet().createResource(URI.createPlatformResourceURI(file.getFullPath().toOSString(), true));
			
		editingDomain.getCommandStack().execute(new RecordingCommand(editingDomain){
			@Override
			protected void doExecute() {
				final DomainDescription dd = dm != null ? dm : AlicaFactory.eINSTANCE.createDomainDescription();
				dd.setName(file.getName().substring(0,file.getName().lastIndexOf(".")));
				
				res.getContents().add(dd);
				
				try {
					res.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
	}
	
	/**
	 * Adding the page to the wizard.
	 */
	public void addPages() {
		dmPage = new PMLNewDomainDesctiptionWizardPage(selection);
		configurationWizardPage = new PMLNewDomainDescriptionConfigurationWizardPage(getEditingDomain(), dm);
		addPage(dmPage);
		addPage(configurationWizardPage);
	}

	private PMLTransactionalEditingDomain getEditingDomain() {
		if(editingDomain == null)
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		
		return editingDomain;
	}
	
	private void throwCoreException(String message) throws CoreException {
		IStatus status =
			new Status(IStatus.ERROR, "de.uni_kassel.vs.cn.planDesigner.ui", IStatus.OK, message, null);
		throw new CoreException(status);
	}
	
	public DomainDescription getDm() {
		return dm;
	}
}
