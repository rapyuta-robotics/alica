package de.uni_kassel.vs.cn.plandesigner.condition.core;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.MessageBox;

/**
 * Handler which refreshs the Condition-Plugins in the pluginfolder
 * @author philipp
 *
 */
public class RefreshPluginsHandler extends AbstractHandler {

	@Override
	public Object execute(ExecutionEvent event) throws ExecutionException {
		//do the refresh
		ConditionPluginLoader.getInstance().refreshConditionPlugins();
		
		//inform user
		MessageBox mb = new MessageBox(Display.getCurrent().getActiveShell());
		mb.setText("Information");
		mb.setMessage("Refreshed Condition-Plugins");
		mb.open();
		
		return null;
	}

}
