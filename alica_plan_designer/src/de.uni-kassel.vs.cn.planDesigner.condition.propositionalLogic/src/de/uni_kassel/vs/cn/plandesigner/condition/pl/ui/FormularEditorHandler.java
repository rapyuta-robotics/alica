package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.swt.widgets.Display;

public class FormularEditorHandler extends AbstractHandler{

	@Override
	public Object execute(ExecutionEvent event) throws ExecutionException {
		FormularEditor editor = new FormularEditor(Display.getCurrent().getActiveShell());
		editor.open();
		return null;
	}

}
