package de.uni_kassel.vs.cn.planDesigner.ui.wizards;

import java.io.File;
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
import org.eclipse.ui.INewWizard;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchWizard;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Planner;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningType;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlanningProblemConfigurationWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlanningProblemWizardPage;

public class PMLNewPlanningProblemWizard extends Wizard implements INewWizard {

	private PlanningProblem planningProblem;
	private Plan plan;
	private PMLTransactionalEditingDomain editingDomain;
	private ISelection selection;
	private PMLNewPlanningProblemWizardPage planningProblemPage;
	private PMLNewPlanningProblemConfigurationWizardPage configurationWizardPage;
	
	public PMLNewPlanningProblemWizard(){
		this(AlicaFactory.eINSTANCE.createPlanningProblem());
	}
	
	public PMLNewPlanningProblemWizard(PlanningProblem problem) {
		super();
		setNeedsProgressMonitor(true);
		setWindowTitle("New PlanningProblem");
		
		// Connect to the editingDomain
		this.editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		this.planningProblem = problem;
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
		final String containerName = planningProblemPage.getContainerName();
		final String fileName = PlanEditorUtils.removeFileExtension(planningProblemPage.getFileName()) + ".pp";
		
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
		// create the behaviou file
		monitor.beginTask("Creating " + fileName, 1);
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IResource resource = root.findMember(new Path(containerName));
		if (!resource.exists() || !(resource instanceof IContainer)) {
			throwCoreException("Container \"" + containerName + "\" does not exist.");
		}
		
		if( configurationWizardPage.getPlanningType() == PlanningType.OFFLINE ){
			
			this.plan = AlicaFactory.eINSTANCE.createPlan();
			String filename = fileName.substring(0,fileName.indexOf(".")) + "_" + this.plan.getId() + ".pml";
			
			IContainer container = (IContainer) resource;
			final IFile file = container.getFile(new Path(filename));
			
			//file.create(null, true, monitor);
			initFileWithPlan(file);
		}
		else
		{		
		
			IContainer container = (IContainer) resource;
			final IFile file = container.getFile(new Path(fileName));
			
			if (file.exists()) {
				// This should not be the case since the user should have specified a filename
				// which is equivalent to the behaviourname
				// file.setContents(stream, true, true, monitor);
			System.err.println("Overwriting existing behaviour!!!");
			} else{
				// Create an empty file
				file.create(null, true, monitor);
				// Init the file with the given plan
				initFileWithPlanningProblems(file);
			}
		}
		monitor.worked(1);
	}
	
	private void initFileWithPlan(final IFile file){
		
		final Resource res = editingDomain.getResourceSet().createResource(URI.createPlatformResourceURI(file.getFullPath().toOSString(), true));
		
		editingDomain.getCommandStack().execute(new RecordingCommand(editingDomain){
			@Override
			protected void doExecute() {
				
//				editingDomain.getCommandStack().dispose();
//				editingDomain.getCommandStack().d
				
				Planner planner = configurationWizardPage.getPlanner();
				try {
					
					String command = planner.getCommand();
					
					String planName = file.getName().substring(0,file.getName().indexOf("."));
					String path = file.getLocation().removeLastSegments(1).toOSString();
					command += " -name " + planName + " -output " + path;
					Process p = Runtime.getRuntime().exec(command);
					p.waitFor();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				Plan createdPlan = (Plan)editingDomain.load((IFile)file).getContents().get(0);
				plan = createdPlan;
				
				res.getContents().add(createdPlan);
				
				try {
					res.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
//				System.out.println(createdPlan.getName());
			}
		});
	}
	
	private void initFileWithPlanningProblems(final IFile file){
		final Resource res = editingDomain.getResourceSet().createResource(URI.createPlatformResourceURI(file.getFullPath().toOSString(), true));
			
		editingDomain.getCommandStack().execute(new RecordingCommand(editingDomain){
			@Override
			protected void doExecute() {
				PlanningProblem p = planningProblem != null ? planningProblem : AlicaFactory.eINSTANCE.createPlanningProblem();
				p.setName(file.getName().substring(0,file.getName().lastIndexOf(".")));
				p.setComment(configurationWizardPage.getComment());
				p.setAlternativePlan(configurationWizardPage.getAlternativePlan());
				p.setWaitPlan(configurationWizardPage.getWaitPlan());
				p.setPlanner(configurationWizardPage.getPlanner());
				p.setPlannerParams(configurationWizardPage.getPlanningParameters());
				p.setUpdateRate(configurationWizardPage.getUpdateRate());
				p.setDistributeProblem(configurationWizardPage.getDistributePlan());
				p.setPlanningType(configurationWizardPage.getPlanningType());
				p.setRequirements(configurationWizardPage.getRequirements());
				
				Condition goal = AlicaFactory.eINSTANCE.createPostCondition();
				goal.setName(configurationWizardPage.getName());
				goal.setComment(configurationWizardPage.getComment());
				goal.setConditionString(configurationWizardPage.getGoal());
				p.getConditions().add(goal);
				if( configurationWizardPage.getRuntimecondition().length() > 0 ){
					Condition run = AlicaFactory.eINSTANCE.createRuntimeCondition();
					run.setConditionString(configurationWizardPage.getRuntimecondition());
					p.getConditions().add(run);
				}
				if( configurationWizardPage.getPrecondition().length() > 0 ){
					Condition pre = AlicaFactory.eINSTANCE.createPreCondition();
					pre.setConditionString(configurationWizardPage.getPrecondition());
					p.getConditions().add(pre);
				}
				
				for(AbstractPlan ap : configurationWizardPage.getPlanningProblemViewerList()) {
					p.getPlans().add(ap);
				}
				
				res.getContents().add(p);
				
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
		planningProblemPage = new PMLNewPlanningProblemWizardPage(selection);
		configurationWizardPage = new PMLNewPlanningProblemConfigurationWizardPage(
				getEditingDomain(), planningProblem);
		addPage(planningProblemPage);
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
	
	public PlanningProblem getPlanningProblem() {
		return planningProblem;
	}
	public Plan getPlan() {
		return plan;
	}
}
