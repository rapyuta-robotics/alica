package de.uni_kassel.vs.cn.planDesigner.validation.listeners;


	import java.util.List;

import org.eclipse.core.runtime.IStatus;
//import org.eclipse.emf.validation.examples.general.console.ConsoleUtil;
	//import org.eclipse.emf.validation.examples.general.internal.l10n.ValidationMessages;
	import org.eclipse.emf.validation.model.IConstraintStatus;
import org.eclipse.emf.validation.service.IValidationListener;
import org.eclipse.emf.validation.service.ValidationEvent;
import org.eclipse.osgi.util.NLS;
	//import org.eclipse.ui.console.ConsolePlugin;
	//import org.eclipse.ui.console.MessageConsole;


	/**
	 * Concrete implementation of a validation listener
	 */
	public class ValidationListener
		implements IValidationListener {
		
		private static final String PLATFORM_NEWLINE = System
			.getProperty("line.separator"); //$NON-NLS-1$
		
		/**
		 * Inner class that helps in the production of an output view message when
		 * live validation encounters problems.
		 */
		private static class OutputUtility {
		    /** Whether the last status that I processed had errors. */
		    private boolean hasErrors = false;
		
		    /**
		     * Appends the problems contained within the specified
		     * <code>status</code> collection to the specified <code>output</code>
		     * buffer.
		     * 
		     * @param event the live validation occurred event
		     * @param output the output
		     */
		    void appendProblems(
		        ValidationEvent event,
		        StringBuffer output) {
		        hasErrors = false;
		
		        appendProblemsRecursive(
		            toStatusArray(event),
		            output);
		
		        if (hasErrors()) {
		            output.append(VALIDATION_ROLLED_BACK);
		            output.append(PLATFORM_NEWLINE);
		        }
		    }
		
		    /**
		     * Queries whether any errors were found in the last processing of
		     * validation status.
		     * 
		     * @return whether any errors were found
		     */
		    boolean hasErrors() {
		        return hasErrors;
		    }
		
		    // private helper to appendProblems() that can be called recursively
		    private void appendProblemsRecursive(
		        IStatus[] statuses,
		        StringBuffer output) {
		        for (IStatus next : statuses) {
		            if (!next.isOK()) {
		                final String messagePattern;
		
		                switch (next.getSeverity()) {
		                    case IStatus.ERROR :
		                        hasErrors = true;
		                        messagePattern = VALIDATION_ERROR;
		                        break;
		                    case IStatus.WARNING :
		                        messagePattern = VALIDATION_WARNING;
		                        break;
		                    default :
		                        messagePattern = VALIDATION_NOTE;
		                        break;
		                }
		
		                output.append(
		                    NLS.bind(
		                        messagePattern,
		                        new Object[] { next.getMessage()}));
		                output.append(PLATFORM_NEWLINE);
		            }
		
		            if (next.isMultiStatus()) {
		                appendProblemsRecursive(
		                    next.getChildren(),
		                    output);
		            }
		        }
		    }
		}

	    /** Message indicating that problems were found in applying model changes. */
	    private static final String VALIDATION_PROBLEMS = "Problems";

	    /** Message indicating that model changes were rolled back. */
	    private static final String VALIDATION_ROLLED_BACK = "Rolled Back";
	    
	    /**
	     * Message reporting an error.  The arguments are:
	     * <ul>
	     *   <li><tt>{0}</tt> - the message from the validation service</li>
	     * </ul>
	     */
	    private static final String VALIDATION_ERROR = "Error";

	    /**
	     * Message reporting a warning.  The arguments are:
	     * <ul>
	     *   <li><tt>{0}</tt> - the message from the validation service</li>
	     * </ul>
	     */
	    private static final String VALIDATION_WARNING = "Warning";

	    /**
	     * Message reporting a note.  The arguments are:
	     * <ul>
	     *   <li><tt>{0}</tt> - the message from the validation service</li>
	     * </ul>
	     */
	    private static final String VALIDATION_NOTE = "Note";
		
		// flag to display events
		public static boolean displayEvents = false;
		// Create a name for the console
		

	    /**
	     * Helper object for creating message to output view.
	     */
	    private final OutputUtility outputUtility;

		/**
		 * Constructor.
		 */
		public ValidationListener() {
			this.outputUtility = new OutputUtility();
		}
		
		/*
		 * @see org.eclipse.emf.validation.service.IValidationListener#validationOccurred(org.eclipse.emf.validation.service.ValidationEvent)
		 */
		public void validationOccurred(ValidationEvent event) {
			if (displayEvents) {		
				// Display in the console				
		        String messages = getMessages(event);
		        if (messages.length() > 0) {
		        	System.out.println(VALIDATION_PROBLEMS);
		        	System.out.println(messages);
		        }

		        
			}
		}

	    /**
	     * Composes the string message to display based on the count of the various
	     * types of problems in the <code>status</code>.
	     * 
	     * @param event the live validation occurred event
	     * @return	A formulated message string.
	     */
	    private String getMessages(ValidationEvent event) {
	        StringBuffer buffer = new StringBuffer();

	        outputUtility.appendProblems(event, buffer);

	        return buffer.toString();
	    }
		
	    /**
	     * Converts a validation event to an array of statuses.
	     * 
	     * @param event the validation event
	     * @return its validation results, as a status array
	     */
	    private static IStatus[] toStatusArray(ValidationEvent event) {
	    	List<IConstraintStatus> results = event.getValidationResults();
	    	
	    	return results.toArray(new IStatus[results.size()]);
	    }    	
}

	

