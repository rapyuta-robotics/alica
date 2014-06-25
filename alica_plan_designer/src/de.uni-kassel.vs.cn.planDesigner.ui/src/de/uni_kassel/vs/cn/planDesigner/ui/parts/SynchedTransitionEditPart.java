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

import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;

public class SynchedTransitionEditPart extends AbstractConnectionEditPart {

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
		con.setLineStyle( SWT.LINE_SOLID);
		con.setAntialias(SWT.ON);
		con.setForegroundColor(new Color(null, new RGB(255,0,0)));
		return con;
	}
	
	@Override
	public void refresh() {
		super.refresh();
		SynchedTransitionDummyConnection stdc = (SynchedTransitionDummyConnection) getModel();
		if(stdc.getTarget()!=null && getViewer().getEditPartRegistry().get(stdc.getTarget())!=null) ((GraphicalEditPart) getViewer().getEditPartRegistry().get(stdc.getTarget())).refresh();
		if(stdc.getSource()!=null && getViewer().getEditPartRegistry().get(stdc.getSource())!=null) ((GraphicalEditPart) getViewer().getEditPartRegistry().get(stdc.getSource())).refresh();
	}
	
	@Override
	public void deactivate() {
		//System.out.println("dec recvd");
		/*SynchedTransitionDummyConnection stdc = (SynchedTransitionDummyConnection) getModel();
		if (stdc == null || stdc.getTarget()==null || stdc.getSource()==null || !stdc.getSource().getSynchedTransitions().contains(stdc.getTarget())) {
			System.out.println("n1");
			setTarget(null);
			setSource(null);
		}
		
		if (getTarget()==null || getSource()==null) {
			System.out.println("n2");
			setTarget(null);
			setSource(null);		
		}
		if (getTarget()==null || getSource()==null || !getSource().isActive() || !getTarget().isActive()) {
			System.out.println("n3");
			setTarget(null);
			setSource(null);		
			
		}*/
		
		super.deactivate();
		try {
			refresh();
		} catch(Exception e) {}
		
		//refreshVisuals();
//		setTarget(null);
//		setSource(null);
	}

}
