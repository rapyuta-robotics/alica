package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.util.ArrayList;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Shell;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;

public class PlanUsageDialog extends Dialog {

	private ArrayList<Plan> plans = new ArrayList<Plan>();
	public PlanUsageDialog(Shell parentShell, ArrayList plans){
		super(parentShell);
		this.plans = plans;
	}

	/**
	 * Create contents of the dialog.
	 * @param parent
	 */
	@Override
	protected Control createDialogArea(Composite parent) {
		Composite container = (Composite) super.createDialogArea(parent);
		container.setLayout(null);
		
		Label lblThereArePlans = new Label(container, SWT.NONE);
		lblThereArePlans.setBounds(10, 10, 430, 42);
		lblThereArePlans.setText("There are plans which reference the element you want to delete. \nIf you continue those references will be removed");
		
		List list = new List(container, SWT.BORDER);
		for(Plan p : plans){
			list.add(p.getName());
		}
		list.setBounds(10, 58, 430, 132);
		return container;
	}

	/**
	 * Create contents of the button bar.
	 * @param parent
	 */
	@Override
	protected void createButtonsForButtonBar(Composite parent) {
		createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL,
				true);
		createButton(parent, IDialogConstants.CANCEL_ID,
				IDialogConstants.CANCEL_LABEL, false);
	}
	@Override
	  protected void configureShell(Shell newShell) {
	    super.configureShell(newShell);
	    newShell.setText("More plans are affected");
	  }
	/**
	 * Return the initial size of the dialog.
	 */
	@Override
	protected Point getInitialSize() {
		return new Point(450, 300);
	}
}
