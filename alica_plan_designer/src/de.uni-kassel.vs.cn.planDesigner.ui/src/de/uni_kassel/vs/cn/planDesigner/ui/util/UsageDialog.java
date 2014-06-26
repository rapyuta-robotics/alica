package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.wizard.WizardDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.ide.IDE;

import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLPlanTypeConfigurationWizard;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLPlanningProblemConfigurationWizard;

public class UsageDialog extends MessageDialog {

	List<TreeNode> usages;
	private TreeViewer viewer;

	public UsageDialog(Shell parent, List<TreeNode> usages, String title, String message, int dialogImageType) {
		super(parent, title, null, message, dialogImageType, new String[] { IDialogConstants.OK_LABEL, IDialogConstants.CANCEL_LABEL }, 0);
		this.usages = usages;
	}

	@Override
	protected Control createCustomArea(Composite parent) {
		Composite comp = new Composite(parent, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL);
		
		comp.setLayout(new FillLayout());
		comp.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

		viewer = new TreeViewer(comp);
		viewer.setContentProvider(new TreeUsageProvider());
		viewer.setLabelProvider(new UsageLabelProvider());
		viewer.setInput(usages);
		viewer.addDoubleClickListener(new IDoubleClickListener(){
			
			public void doubleClick(DoubleClickEvent event) {
				Object selection = ((IStructuredSelection)event.getSelection()).getFirstElement();
				if(selection instanceof TreeNode){
					TreeNode tn = (TreeNode) selection; 
					if (tn.getValue() instanceof State) {
						// open plan
						State state = (State) tn.getValue();
						try {
							IWorkbenchPage page =
								PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();
							IDE.openEditor(page, WorkspaceSynchronizer.getFile(state.eResource()));
						} catch (PartInitException e) {
							PlanDesignerActivator.getDefault().getLog().log(
									new Status(IStatus.ERROR, PlanDesignerActivator.PLUGIN_ID,"Error while opening plan",e));
						}
					} else if (tn.getValue() instanceof AnnotatedPlan) {
						// open plan type
						AnnotatedPlan anPlan = (AnnotatedPlan) tn.getValue();
						
						// Create a plantype configuration wizard and initialize it with the plantype
						PMLPlanTypeConfigurationWizard wiz = new PMLPlanTypeConfigurationWizard((PlanType) anPlan.eContainer());
						
						WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz);
						dialog.setBlockOnOpen(true);
						dialog.open();
					} else if (tn.getValue() instanceof PlanningProblem) {
						// open plan type
						PlanningProblem pp = (PlanningProblem) tn.getValue();
						
						// Create a plantype configuration wizard and initialize it with the plantype
						PMLPlanningProblemConfigurationWizard wiz = new PMLPlanningProblemConfigurationWizard((PlanningProblem) pp);
						
						WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz);
						dialog.setBlockOnOpen(true);
						dialog.open();
					} else if (tn.getValue() instanceof EntryPoint) {
						// open plan
						EntryPoint ep = (EntryPoint) tn.getValue();
						try {
							IWorkbenchPage page =
								PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();
							IDE.openEditor(page, WorkspaceSynchronizer.getFile(ep.eResource()));
						} catch (PartInitException e) {
							PlanDesignerActivator.getDefault().getLog().log(
									new Status(IStatus.ERROR, PlanDesignerActivator.PLUGIN_ID,"Error while opening plan",e));
						}
					}
				}
			}
			
		});
		
		viewer.expandAll();

		return comp;
	}

	@Override
	protected Point getInitialSize() {
		return new Point(500, 350);
	}

	protected TreeViewer getViewer() {
		return viewer;
	}

	protected void setViewer(TreeViewer viewer) {
		this.viewer = viewer;
	}

	public class TreeUsageProvider implements ITreeContentProvider {

		public Object[] getChildren(Object parentElement) {
			return ((TreeNode) parentElement).getChildren();
		}

		public Object getParent(Object element) {
			return ((TreeNode) element).getParent();
		}

		public boolean hasChildren(Object element) {
			return ((TreeNode) element).hasChildren();
		}

		public Object[] getElements(Object inputElement) {
			Object[] result = new String[] { "No usages found!" };
			if (inputElement instanceof ArrayList<?>) {
				result = ((ArrayList<?>) inputElement).toArray();
			}
			return result;
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

	}

	private class UsageLabelProvider extends LabelProvider {
		@Override
		public Image getImage(Object element) {
			Image img = PlanDesignerActivator.getDefault().getImageRegistry().get(
					PlanDesignerConstants.ICON_UNKNOWN_TYPE);
			
			if (element instanceof TreeNode) {
				Object value = ((TreeNode) element).getValue();
				if (value instanceof Plan) {
					if (((Plan) value).isMasterPlan()) {
						img = PlanDesignerActivator.getDefault().getImageRegistry().get(
								PlanDesignerConstants.ICON_MASTER_PLAN_16);
					} else {
						img = PlanDesignerActivator.getDefault().getImageRegistry().get(
								PlanDesignerConstants.ICON_PLAN_16);
					}
				} else if( value instanceof PlanningProblem ) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_PLANNING_PROBLEM_16);
				}
				else if (value instanceof BehaviourConfiguration) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_BEHAVIOUR_CONFIGURATION_16);
				} else if (value instanceof Behaviour) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_BEHAVIOUR_16);
				} else if (value instanceof PlanType || value instanceof AnnotatedPlan) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_PLANTYPE_16);
				} else if (value instanceof State) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_STATE_16);
				} else if (value instanceof EntryPoint) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_ENTRY_POINT_16);
				} else if (value instanceof Task) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_TASK_16);
				} else if (value instanceof Parametrisation) {
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_CAPABILITY_16); // TODO: create Icon for parametrisations
				}
			}
			
			return img;
		}

		@Override
		public String getText(Object element) {
			String result = element.toString();
			if (element instanceof TreeNode) {
				Object value = ((TreeNode) element).getValue();
				if (value instanceof PlanElement) {
					if (value instanceof State) {
						State state = (State) value;
						result = "State \"" + state.getName() + "\" in Plan \"" + state.getInPlan().getName() + "\"";		
					} else if (value instanceof AnnotatedPlan){
						// references from plan types (the plan is inserted there in an annotated plan
						AnnotatedPlan anPlan = (AnnotatedPlan) value;
						result = ((PlanElement)anPlan.eContainer()).getName();
					} else if (value instanceof EntryPoint){
						EntryPoint ep = (EntryPoint) value;
						result = "EntryPoint \"" + ((EntryPoint) value).getName() + "\" in Plan \"" + ep.getPlan().getName() + "\"";
					} else if (value instanceof Parametrisation) {
						Parametrisation param = (Parametrisation) value;
						State state = ((State)param.eContainer());
						result = "Parametrisation \"" + param.getId() + "\" of State \"" + state.getName() 
								+ "\" in Plan \"" + state.getInPlan().getName() + "\"";
					} else {
						result = ((PlanElement) value).getName();			
					}
				} else {
					result = value.toString();
				}
			}
			return result;
		}
	}
}
