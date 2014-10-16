package de.uni_kassel.vs.cn.planDesigner.codegeneration.commands;


import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.resources.WorkspaceJob;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.core.runtime.jobs.IJobChangeEvent;
import org.eclipse.core.runtime.jobs.JobChangeAdapter;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.jface.viewers.CheckStateChangedEvent;
import org.eclipse.jface.viewers.CheckboxTableViewer;
import org.eclipse.jface.viewers.ICheckStateListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.ScrolledComposite;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Table;
import org.eclipse.ui.handlers.HandlerUtil;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;


public class CleanupExpressionValidatorsCommandCPlusPlus extends AbstractHandler {

	private class FindObsoleteExpressionValidatorsJob extends WorkspaceJob
	{

		private Collection<IFile> filesToDelete;

		public FindObsoleteExpressionValidatorsJob()
		{
			super("Find obsolete expression validators");
			setUser(true);
		}

		@Override
		public IStatus runInWorkspace(IProgressMonitor monitor)
				throws CoreException
		{
			final Set<IFile> expressionValidatorFiles = CommonUtils.collectAllFilesWithExtension("cs","pl");
			Set<IFile> filesInWorkspace = CommonUtils.collectAllFilesWithExtension("pml","beh");

			monitor.beginTask("Searching for obsolete expression validators", expressionValidatorFiles.size()+expressionValidatorFiles.size());

			ResourceSet rSet = CommonUtils.getAlicaResourceSet();
			// Load all files into the resourceset
			monitor.subTask("Loading files...");
			for (IFile file : filesInWorkspace)
			{
				CommonUtils.load(rSet, file);
			}
			monitor.worked(filesInWorkspace.size());

			monitor.subTask("Fetching IDs");
			// Fetch all IDs
			Set<Object> existingIDs = fetchIDs(rSet);
			monitor.worked(1);
			
			filesToDelete = new HashSet<IFile>();

//			System.out.println("Checking " +collectedFiles.size() +" validators: ");
			
			monitor.subTask("Checking existence");
			for (IFile fileToCheck : expressionValidatorFiles)
			{
//				System.out.println("\t" +fileToCheck);
				if (!checkForExistence(fileToCheck, existingIDs))
				{
					filesToDelete.add(fileToCheck);
				}
			}
			monitor.worked(expressionValidatorFiles.size());

			monitor.subTask("Cleanup");
			// Cleanup -> Unload all resources
			for (Resource r : rSet.getResources())
			{
				r.unload();
			}
			
			rSet.getResources().clear();
			rSet = null;
			monitor.done();
			
			return Status.OK_STATUS;
		}

		public Collection<IFile> getFilesToDelete()
		{
			return filesToDelete;
		}
		
		private Set<Object> fetchIDs(ResourceSet set)
		{
			Set<Object> existingIDs = new HashSet<Object>();
			for (Resource r : set.getResources())
			{
				Object contents = r.getContents().get(0);
				if(contents instanceof Behaviour)
				{
					Behaviour behaviour = (Behaviour)contents;
					for (BehaviourConfiguration config : behaviour.getConfigurations())
					{
						existingIDs.add(config.getId());
					}
				}
				else if(contents instanceof Plan)
				{
					// For plans we'll compare against their name
					existingIDs.add(((Plan)contents).getName());
				}
			}
		
//			System.out.println("Fetched " +existingIDs.size() +" IDs");
			return existingIDs;
		}

		/**
		 * Checks if the given file can be matched to the given set of IDs
		 * @param fileToCheck
		 * @param existingIDs
		 * @return	<code>true</code> if the given file was matched, <code>false</code>
		 * otherwise
		 */
		private boolean checkForExistence(IFile fileToCheck, Set<Object> existingIDs)
		{
			boolean exists = true;
			
			// Check whether the validator was generated for a BehaviourConfiguration
			// or a plan. Decision is made by the name: If it was generated out of 
			// a BehaviourConfiguration it will have the name "Behaviour" in it.
			// Otherwise we assume that it is a plan
			
			String fileName = CommonUtils.removeFileExtension(fileToCheck.getName());
			if(fileName.endsWith("Constraints")) fileName = fileName.substring(0,fileName.indexOf("Constraints"));
			if(fileName.endsWith("Behaviour") && containsDigits(fileName,13)) // 13 is the number of digits in a long
			{
				// Extract the id
				String id = fileName.substring(fileName.length()-22,fileName.length()-9);
				
				if(id.length() > 0)
				{
					long longID = Long.parseLong(id);
					exists = existingIDs.contains(longID);
				}
			}
			else
			{
				exists = existingIDs.contains(fileName);
			}
			
			return exists;
		}

		private boolean containsDigits(String fileName, int minimumDigits)
		{
			int digitCount = 0;
			for (int i=0; i < fileName.length(); i++)
			{
				if(Character.isDigit(fileName.charAt(i)))
				{
					digitCount++;
					if(digitCount >= minimumDigits)
					{
						break;
					}
				}
			}
			
			
			return digitCount >= minimumDigits;
		}
	}
	
	private class DeleteObsoleteValidatorsDialog extends TitleAreaDialog
	{
		private class IFileContentProvider implements IStructuredContentProvider
		{
			public Object[] getElements(Object inputElement)
			{
				Object[] elements = null;
				if(inputElement instanceof Collection<?>)
				{
					elements = ((Collection<?>) inputElement).toArray();
				}
				else
				{
					elements = new Object[0];
				}
				
				return elements;
			}

			public void dispose()
			{
			}

			public void inputChanged(Viewer viewer, Object oldInput,
					Object newInput)
			{
			}
		}
		
		private class IFileLabelProvider extends LabelProvider
		{
			@Override
			public Image getImage(Object element)
			{
				if(element instanceof IFile)
				{
					return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CSHARP_16);
				}
				else
				{
					return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_UNKNOWN_TYPE);
				}
			}
			
			@Override
			public String getText(Object element)
			{
				if(element instanceof IFile)
				{
					return ((IFile) element).getName();
				}
				else
				{
					return super.getText(element);
				}
			}
		}
		
		private final Collection<IFile> filesToDelete;
		
		private CheckboxTableViewer viewer;

		private Label selectedLabel;
		
		private Collection<IFile> checkedFiles;

		public DeleteObsoleteValidatorsDialog(Shell parentShell, Collection<IFile> filesToDelete)
		{
			super(parentShell);
			this.filesToDelete = filesToDelete;
		}
		
		@Override
		protected void okPressed()
		{
			checkedFiles = new HashSet<IFile>();
			for (Object o : viewer.getCheckedElements())
			{
				if(o instanceof IFile)
				{
					checkedFiles.add((IFile)o);
				}
			}
			super.okPressed();
		}
		
		@Override
		protected Control createDialogArea(Composite parent)
		{
			Composite composite = new Composite(parent, SWT.NONE);
			composite.setLayout(new GridLayout(1,false));
			composite.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
			
			ScrolledComposite scroller = new ScrolledComposite(composite, SWT.H_SCROLL | SWT.V_SCROLL | SWT.BORDER);
			scroller.setExpandHorizontal(true);
			scroller.setExpandVertical(true);
			Table table = new Table(scroller, SWT.CHECK | SWT.MULTI | SWT.FULL_SELECTION);
			viewer = new CheckboxTableViewer(table);
			viewer.setContentProvider(new IFileContentProvider());
			viewer.setLabelProvider(new IFileLabelProvider());
			viewer.setInput(filesToDelete);
			viewer.setSorter(new ViewerSorter());
			viewer.addCheckStateListener(new ICheckStateListener()
			{
				public void checkStateChanged(CheckStateChangedEvent event)
				{
					selectedLabel.setText("Selected: " +viewer.getCheckedElements().length);
				}
			});
			scroller.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
			scroller.setContent(table);
			
			Composite buttonComposite = new Composite(composite, SWT.NONE);
			buttonComposite.setLayout(new GridLayout(3,false));
			buttonComposite.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, false));
			
			Button b = new Button(buttonComposite, SWT.PUSH);
			b.setText("Select all");
			b.setLayoutData(new GridData(SWT.BEGINNING,SWT.CENTER,false,true));
			b.addSelectionListener(new SelectionAdapter()
			{
				@Override
				public void widgetSelected(SelectionEvent e)
				{
					viewer.setAllChecked(true);
					refreshVisuals();
				}
			});
			
			b = new Button(buttonComposite, SWT.PUSH);
			b.setText("Select none");
			b.setLayoutData(new GridData(SWT.BEGINNING,SWT.CENTER,false,true));
			b.addSelectionListener(new SelectionAdapter()
			{
				@Override
				public void widgetSelected(SelectionEvent e)
				{
					viewer.setAllChecked(false);
					refreshVisuals();
				}
			}); 
			
			selectedLabel = new Label(buttonComposite, SWT.NONE);
			selectedLabel.setAlignment(SWT.RIGHT);
			GridData layoutData = new GridData(SWT.RIGHT,SWT.CENTER,true,true);
			layoutData.widthHint = 100;
			selectedLabel.setLayoutData(layoutData);

			setTitle("Obsolete expression validators");
			setMessage("Choose the validators to delete. Note: Once deleted, all manually inserted code is lost!");
			// Select all 
			viewer.setAllChecked(true);
			refreshVisuals();
			
			return composite;
		}
		
		private Collection<IFile> getSelectedFiles()
		{
			return checkedFiles;
		}
		
		@Override
		protected Point getInitialSize()
		{
			return new Point(400,500);
		}
		
		private void refreshVisuals()
		{
			selectedLabel.setText("Selected: " +viewer.getCheckedElements().length);	
		}
	}
	
	public Object execute(ExecutionEvent event) throws ExecutionException
	{
		final Shell activeShell = HandlerUtil.getActiveShell(event);
		final FindObsoleteExpressionValidatorsJob job = new FindObsoleteExpressionValidatorsJob();
		job.addJobChangeListener(new JobChangeAdapter()
		{
			@Override
			public void done(IJobChangeEvent event)
			{
				if(event.getResult().isOK() && event.getResult() != Status.CANCEL_STATUS)
				{
					final Collection<IFile> filesToDelete = job.getFilesToDelete();
					
					if(filesToDelete.isEmpty())
					{
						activeShell.getDisplay().asyncExec(new Runnable()
						{
							public void run()
							{
								MessageDialog.openInformation(activeShell, "Cleanup expression validators", "No obsolete validators found.");
							}
						});
					}
					else
					{
						activeShell.getDisplay().asyncExec(new Runnable()
						{
							public void run()
							{
								final DeleteObsoleteValidatorsDialog dialog = 
									new DeleteObsoleteValidatorsDialog(activeShell, filesToDelete);
								dialog.setBlockOnOpen(true);
								if (dialog.open() == IStatus.OK)
								{
									new WorkspaceJob(
											"Remove obsolete expression validators")
									{
										@Override
										public IStatus runInWorkspace(
												IProgressMonitor monitor)
												throws CoreException
										{
											for (IFile file : dialog
													.getSelectedFiles())
											{
												file.delete(true, monitor);
											}
											
											ResourcesPlugin.getWorkspace().getRoot().
													refreshLocal(IResource.DEPTH_INFINITE, monitor);
											return Status.OK_STATUS;
										}
									}.schedule();

								}
							}
						});	
					}
					
				}
				super.done(event);
			}
		});
		
		job.schedule();
		
		return null;
	}

	
}
