package de.uni_kassel.vs.cn.planDesigner.validation.actions;

import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.util.EContentAdapter;
import org.eclipse.emf.validation.model.EvaluationMode;
import org.eclipse.emf.validation.service.ILiveValidator;
import org.eclipse.emf.validation.service.ModelValidationService;
import org.eclipse.jface.dialogs.MessageDialog;

import de.uni_kassel.vs.cn.planDesigner.validation.constraints.ValidationDelegateClientSelector;

public class LiveValidationContentAdapter extends EContentAdapter {
	private final EnableLiveValidationDelegate actionDelegate;
	private ILiveValidator validator = null;



	LiveValidationContentAdapter(EnableLiveValidationDelegate delegate) {
		actionDelegate = delegate;
	}

	public void notifyChanged(final Notification notification) {
		super.notifyChanged(notification);
//System.out.println("Notify changed: "+notification);		
		actionDelegate.shell.getDisplay().asyncExec(new Runnable() {
			public void run() {
				if (validator == null) {
					validator = ModelValidationService.getInstance().newValidator(
						EvaluationMode.LIVE);
				}
				
				ValidationDelegateClientSelector.running = true;
				
				IStatus status = validator.validate(notification);
				
				if (!status.isOK()) {
					if (status.isMultiStatus()) {
						status = status.getChildren()[0];
					}
					
					MessageDialog.openError(LiveValidationContentAdapter.this.actionDelegate.shell,LiveValidationContentAdapter.this.actionDelegate.title,status.getMessage());
				}
				
				ValidationDelegateClientSelector.running = false;
			}
		});
	}	
	
	
	/*public void notifyChanged(final Notification notification) {
		super.notifyChanged(notification);
System.out.println("Notify changed: "+notification);		
		if (validator == null) {
			validator =	(ILiveValidator)ModelValidationService.getInstance().newValidator(EvaluationMode.LIVE);
			
		}
		
		ValidationDelegateClientSelector.running = true;
		
		IStatus status = validator.validate(notification);
		
		if (!status.isOK()) {
			if (status.isMultiStatus()) {
				status = status.getChildren()[0];
			}
			
			System.out.println("The current modification has violated one or more live constraints");
		}
		
		ValidationDelegateClientSelector.running = false;
	}*/
}
