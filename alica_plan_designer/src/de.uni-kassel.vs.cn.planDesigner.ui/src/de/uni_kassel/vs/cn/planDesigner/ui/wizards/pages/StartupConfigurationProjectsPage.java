package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import java.io.File;

import org.eclipse.jface.preference.IPreferenceStore;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyAdapter;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.DirectoryDialog;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

/*
 * TODO: externalize strings
 * TODO: get ES_ROOT env-var 
 */
public class StartupConfigurationProjectsPage extends WizardPage
{
	public static final String	ROLES_LOCATION					= "/roles";

	public static final String	MISC_LOCATION					= "/Misc";

	public static final String	PLANS_LOCATION					= "/plans";

	public static final String	EXPRESSION_VALIDATORS_LOCATION	= "/Expr";

	private String				msg								= "Adjust project that will be generated";

	private String				planProject						= "1";

	private String				exprValProject					= "2";

	private String				rolesProject					= "3";

	private String				miscProject						= "4";

	private String				rootPath						= "...";
	private String				es_root							= "";

	private Text				rootPathText;
	private Text				planPathText;
	private Text				exprValPathText;
	private Text				rolesPathText;
	private Text				miscPathText;
	private Text				planNameText;
	private Text				exprValNameText;
	private Text				rolesNameText;
	private Text				miscNameText;

	public StartupConfigurationProjectsPage()
	{
		super("Projects Page");
		setTitle("Projects");
		setDescription(msg);

		// Note: you probably have to start the plan designer from console to get the es_root env variable
		es_root = System.getenv("ES_ROOT");
		if (es_root != null && !es_root.isEmpty()){
			rootPath =  es_root;
		} else {
			rootPath = System.getenv("HOME") + File.separator;
		}
		
		if (rootPath == null)
		{
			rootPath = "";
		}

		IPreferenceStore store = PlanDesignerActivator.getDefault().getPreferenceStore();

		planProject = "Plans";

		miscProject = store.getString(PlanDesignerConstants.PREF_MISC_PROJECT_WORKSPACE_PATH);
		rolesProject = store.getString(PlanDesignerConstants.PREF_ROLE_DEFINITION_CONTAINER);
		exprValProject = store.getString(PlanDesignerConstants.PREF_CODEGEN_BASE_PATH);

		miscProject = chopSlash(miscProject);
		rolesProject = chopSlash(rolesProject);
		exprValProject = chopSlash(exprValProject);
	}

	private String chopSlash(String what)
	{
		int lastIndexOf = what.lastIndexOf("/");
		if (lastIndexOf != -1 && what.length() > 1)
		{
			what = what.substring(lastIndexOf + 1);
		}

		return what;
	}

	public void createControl(Composite arg0)
	{
		Composite parent = new Composite(arg0, SWT.NONE);
		parent.setLayout(new GridLayout(1, false));

		// root path
		Group grp = new Group(parent, SWT.NONE);
		grp.setLayout(new FillLayout(SWT.VERTICAL));
		grp.setLayoutData(new GridData(GridData.FILL_BOTH));
		Composite top = new Composite(grp, SWT.NONE);
		top.setLayout(new GridLayout(3, false));
//		top.setLayoutData(new GridData(GridData.FILL_BOTH));

		Label l = new Label(top, SWT.NONE);
		l.setLayoutData(new GridData(SWT.FILL, SWT.NONE, true, false));
		l.setText("Root path:");

		rootPathText = new Text(top, SWT.BORDER);
		rootPathText.setLayoutData(new GridData(SWT.FILL, SWT.NONE, true, false));
		rootPathText.addKeyListener(new KeyAdapter()
		{
			@Override
			public void keyReleased(KeyEvent e)
			{
				updatePaths(rootPathText.getText());
				checkState();
			}
		});

		Button b = new Button(top, SWT.PUSH);
		b.setText("Browse...");
		b.setLayoutData(new GridData(SWT.FILL, SWT.NONE, false, false));
		b.addSelectionListener(new SelectionAdapter()
		{
			@Override
			public void widgetSelected(SelectionEvent e)
			{
				DirectoryDialog dlg = new DirectoryDialog(getShell());
				dlg.setFilterPath(rootPathText.getText());
				dlg.setText("Select root location");
				dlg.setMessage("Select root location");

				String dir = dlg.open();
				if (dir != null)
				{
					updatePaths(dir);
				}

				checkState();
			}
		});

		// Bottom
		Composite comp = new Composite(parent, SWT.NONE);
		comp.setLayout(new FillLayout(SWT.VERTICAL));
		comp.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

		// plans
		grp = new Group(comp, SWT.BORDER);
		grp.setLayout(new FillLayout(SWT.VERTICAL));
		grp.setText("Plans project");
		Composite compp1 = new Composite(grp, SWT.NONE);

		compp1.setLayout(new GridLayout(3,false));
		Label proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true,1,0));
		proj_l.setText("Project name:");
		planNameText = new Text(compp1, SWT.BORDER);
		planNameText.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 2, 0));
		planNameText.setText(planProject);


		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		proj_l.setText("Project location:");
		planPathText = new Text(compp1, SWT.BORDER);
		planPathText.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		planPathText.setEnabled(true);
		planPathText.addKeyListener(new KeyAdapter()
		{
			@Override
			public void keyReleased(KeyEvent e)
			{
				checkState();
			}
		});
		
		
		b = new Button(compp1, SWT.PUSH);
		b.setText("Browse...");
		b.setLayoutData(new GridData(SWT.FILL, SWT.NONE, false, false));
		b.addSelectionListener(new SelectionAdapter()
		{
			@Override
			public void widgetSelected(SelectionEvent e)
			{
				DirectoryDialog dlg = new DirectoryDialog(getShell());
				
				dlg.setFilterPath(planPathText.getText());				
				dlg.setText("Select Plans location");
				dlg.setMessage("Select location of Plans");

				String dir = dlg.open();
				if (dir != null)
				{
					planPathText.setText(dir);
				}

				checkState();
			}
		});

		// expression validators
		grp = new Group(comp, SWT.BORDER);
		grp.setLayout(new FillLayout(SWT.VERTICAL));
		grp.setText("Expression Validators project");
		compp1 = new Composite(grp, SWT.NONE);
		compp1.setLayout(new GridLayout(3,false));
		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true,1,0));
		proj_l.setText("Project name:");
		exprValNameText = new Text(compp1, SWT.BORDER);
		exprValNameText.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 2, 0));
		exprValNameText.setText(exprValProject);

		
		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		proj_l.setText("Project location:");
		exprValPathText = new Text(compp1, SWT.BORDER);
		exprValPathText.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		exprValPathText.setEnabled(true);
		exprValPathText.addKeyListener(new KeyAdapter()
		{
			@Override
			public void keyReleased(KeyEvent e)
			{
				checkState();
			}
		});
		b = new Button(compp1, SWT.PUSH);
		b.setText("Browse...");
		b.setLayoutData(new GridData(SWT.FILL, SWT.NONE, false, false));
		b.addSelectionListener(new SelectionAdapter()
		{
			@Override
			public void widgetSelected(SelectionEvent e)
			{
				DirectoryDialog dlg = new DirectoryDialog(getShell());
				dlg.setFilterPath(exprValPathText.getText());
				dlg.setText("Select expression folder");
				dlg.setMessage("Select location of expressions");

				String dir = dlg.open();
				if (dir != null)
				{
					exprValPathText.setText(dir);
				}

				checkState();
			}
		});

		// roles
		grp = new Group(comp, SWT.BORDER);
		grp.setLayout(new FillLayout(SWT.VERTICAL));
		grp.setText("Roles project");
		compp1 = new Composite(grp, SWT.NONE);
		compp1.setLayout(new GridLayout(3,false));
		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true,1,0));
		proj_l.setText("Project name:");
		rolesNameText = new Text(compp1, SWT.BORDER);
		rolesNameText.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 2, 0));
		rolesNameText.setText(rolesProject);

		
		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		proj_l.setText("Project location:");
		rolesPathText = new Text(compp1, SWT.BORDER);
		rolesPathText.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		rolesPathText.setEnabled(true);
		rolesPathText.addKeyListener(new KeyAdapter()
		{
			@Override
			public void keyReleased(KeyEvent e)
			{
				checkState();
			}
		});
		b = new Button(compp1, SWT.PUSH);
		b.setText("Browse...");
		b.setLayoutData(new GridData(SWT.FILL, SWT.NONE, false, false));
		b.addSelectionListener(new SelectionAdapter()
		{
			@Override
			public void widgetSelected(SelectionEvent e)
			{
				DirectoryDialog dlg = new DirectoryDialog(getShell());
				dlg.setFilterPath(rolesPathText.getText());
				dlg.setText("Select role folder");
				dlg.setMessage("Select location of roles");

				String dir = dlg.open();
				if (dir != null)
				{
					rolesPathText.setText(dir);
				}

				checkState();
			}
		});
		
		// misc
		grp = new Group(comp, SWT.BORDER);
		grp.setLayout(new FillLayout(SWT.VERTICAL));
		grp.setText("Misc project");
		compp1 = new Composite(grp, SWT.NONE);
		compp1.setLayout(new GridLayout(3,false));
		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true,1,0));
		proj_l.setText("Project name:");
		miscNameText = new Text(compp1, SWT.BORDER);
		miscNameText.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 2, 0));
		miscNameText.setText(miscProject);

		
		proj_l = new Label(compp1, SWT.NONE);
		proj_l.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		proj_l.setText("Project location:");
		miscPathText = new Text(compp1, SWT.BORDER);
		miscPathText.setLayoutData(new GridData(SWT.FILL, SWT.BEGINNING, true, true));
		miscPathText.setEnabled(true);
		miscPathText.addKeyListener(new KeyAdapter()
		{
			@Override
			public void keyReleased(KeyEvent e)
			{
				checkState();
			}
		});
		b = new Button(compp1, SWT.PUSH);
		b.setText("Browse...");
		b.setLayoutData(new GridData(SWT.FILL, SWT.NONE, false, false));
		b.addSelectionListener(new SelectionAdapter()
		{
			@Override
			public void widgetSelected(SelectionEvent e)
			{
				DirectoryDialog dlg = new DirectoryDialog(getShell());
				dlg.setFilterPath(miscPathText.getText());
				dlg.setText("Select Misc folder");
				dlg.setMessage("Select location for miscellaneous files.");

				String dir = dlg.open();
				if (dir != null)
				{
					miscPathText.setText(dir);
				}

				checkState();
			}
		});
		setControl(parent);
		setPageComplete(false);

		updatePaths(rootPath);

		checkState();
	}

	private void updatePaths(String root)
	{
		rootPath = root;
		rootPathText.setText(root);
		String pt = planPathText.getText();
		if (pt != null && !pt.isEmpty()) {
			planPathText.setText(rootPath + pt.subSequence(pt.lastIndexOf(File.separatorChar), pt.length()));
		} else {
			planPathText.setText(rootPath + PLANS_LOCATION);
		}
		
		String mt = miscPathText.getText();
		if (mt != null && !mt.isEmpty()) {
			miscPathText.setText(rootPath + mt.subSequence(mt.lastIndexOf(File.separatorChar), mt.length()));			
		} else {
			miscPathText.setText(rootPath + MISC_LOCATION);
		}
		
		String rt = rolesPathText.getText();
		if (rt != null && !rt.isEmpty()) {
			rolesPathText.setText(rootPath + rt.subSequence(rt.lastIndexOf(File.separatorChar), rt.length()));
		} else {
			rolesPathText.setText(rootPath + ROLES_LOCATION);
		}
		
		String et = exprValPathText.getText();
		if (et != null && !et.isEmpty()) {
			exprValPathText.setText(rootPath + et.subSequence(et.lastIndexOf(File.separatorChar), et.length()));
		} else {
			exprValPathText.setText(rootPath + EXPRESSION_VALIDATORS_LOCATION);
		}
	}

	private void checkState()
	{
		File fe = new File(exprValPathText.getText());
		File fp = new File(planPathText.getText());
		File fr = new File(rolesPathText.getText());
		File fm = new File(miscPathText.getText());
		
		
		boolean dirsThere = (dirIsThere(fe)) && (dirIsThere(fp)) && (dirIsThere(fr)) && (dirIsThere(fm));
		if (!dirsThere)
		{
			setErrorMessage("At least one project directory couldn't be found!");
		}
		else
		{
			setErrorMessage(null);
			setMessage(msg);
		}

		setPageComplete((!planPathText.getText().isEmpty()) && (!exprValPathText.getText().isEmpty()) && dirsThere);
	}

	private boolean dirIsThere(File fe)
	{
		return fe.exists() && fe.isDirectory();
	}

	/* Getter */

	public String getPlanProjectName()
	{
		return planNameText.getText();
		//return planProject;
	}

	public String getExprValProject()
	{
		return exprValNameText.getText();
		//return exprValProject;
	}

	public String getRolesProject()
	{
		return rolesNameText.getText();
		//return rolesProject;
	}

	public String getMiscProject()
	{
		return miscNameText.getText();
		//return miscProject;
	}

	public String getRootPath()
	{
		return rootPath;
	}
	public String getPlanPath()
	{
		return planPathText.getText();
	}

	public String getExprValPath()
	{
		return exprValPathText.getText();
	}

	public String getRolesPath()
	{
		return rolesPathText.getText();
	}

	public String getMiscPath()
	{
		return miscPathText.getText();
	}

	
}
