package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;

/**
 * Dialog with a tree viewer.
 * 
 * @author philipp
 * 
 */
public class TreeDialog extends Dialog {
	/**
	 * Content provider for the tree viewer
	 */
	private ITreeContentProvider contentProvider;
	/**
	 * Label provider for the tree viewer
	 */
	private LabelProvider labelProvider;
	/**
	 * Input of the tree viewer
	 */
	private Object input;

	/**
	 * 
	 * @param parentShell
	 * @param contentProvider
	 *            Content provider for the tree viewer
	 * @param labelProvider
	 *            Label provider for the tree viewer
	 * @param input
	 *            Input of the tree viewer
	 */
	protected TreeDialog(Shell parentShell, ITreeContentProvider contentProvider, LabelProvider labelProvider, Object input) {
		super(parentShell);
		this.contentProvider = contentProvider;
		this.labelProvider = labelProvider;
		this.input = input;
	}

	@Override
	protected Control createDialogArea(Composite parent) {
		Composite container = (Composite) super.createDialogArea(parent);
		container.setLayout(new GridLayout(1, false));

		TreeViewer viewer = new TreeViewer(container, SWT.H_SCROLL | SWT.V_SCROLL);
		viewer.getControl().setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		viewer.setContentProvider(contentProvider);
		viewer.setLabelProvider(labelProvider);
		viewer.setInput(input);

		// set size
		return container;
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

}
