package de.uni_kassel.vs.cn.planDesigner.ui.tool;

import org.eclipse.gef.Tool;
import org.eclipse.gef.palette.CombinedTemplateCreationEntry;
import org.eclipse.gef.requests.CreationFactory;
import org.eclipse.gef.tools.CreationTool;
import org.eclipse.jface.resource.ImageDescriptor;

public class PlanningProblemCreationToolEntry extends
		CombinedTemplateCreationEntry {

	public PlanningProblemCreationToolEntry(String label, String shortDesc,
			CreationFactory factory, ImageDescriptor iconSmall,
			ImageDescriptor iconLarge) {
		super(label, shortDesc, factory, iconSmall, iconLarge);
	}

	@Override
	public Tool createTool() {
		CreationTool tool = new PlanningProblemCreationTool();
		tool.setProperties(getToolProperties());
		return tool;
	}
}
