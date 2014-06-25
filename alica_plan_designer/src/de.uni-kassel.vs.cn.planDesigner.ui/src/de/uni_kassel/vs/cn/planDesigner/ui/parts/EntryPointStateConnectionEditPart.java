package de.uni_kassel.vs.cn.planDesigner.ui.parts;

import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.PolygonDecoration;
import org.eclipse.draw2d.PolylineConnection;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.GraphicalEditPart;
import org.eclipse.gef.editparts.AbstractConnectionEditPart;
import org.eclipse.gef.editpolicies.ConnectionEndpointEditPolicy;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.RGB;

import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;

public class EntryPointStateConnectionEditPart extends AbstractConnectionEditPart {
	
	@Override
	protected void createEditPolicies() {
		installEditPolicy(EditPolicy.CONNECTION_ENDPOINTS_ROLE, new ConnectionEndpointEditPolicy());
	}

	@Override
	protected IFigure createFigure() {
		PolylineConnection con = new PolylineConnection();
		con.setTargetDecoration(new PolygonDecoration());
		con.setLineWidth(2);
		con.setLineCap(SWT.CAP_FLAT);
		con.setLineJoin( SWT.JOIN_MITER);
		con.setLineStyle( SWT.LINE_DOT);
		con.setAntialias(SWT.ON);
		con.setForegroundColor(new Color(null, new RGB(0,0,255)));
		return con;
	}
	
	@Override
	public void refresh() {
		super.refresh();
		EntryPointStateDummyConnection epsdc = (EntryPointStateDummyConnection) getModel();
		GraphicalEditPart graphicalEditPart = (GraphicalEditPart) getViewer().getEditPartRegistry().get(epsdc.getTarget());
		if (graphicalEditPart != null){
			graphicalEditPart.refresh();
		}
	}
	
	@Override
	public void deactivate() {
		super.deactivate();
		
		setTarget(null);
	}
}
