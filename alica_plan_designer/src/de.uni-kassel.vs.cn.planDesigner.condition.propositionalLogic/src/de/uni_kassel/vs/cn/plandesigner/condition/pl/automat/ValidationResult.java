package de.uni_kassel.vs.cn.plandesigner.condition.pl.automat;

/**
 * An instance of this object holds information about the validated formular.
 * This means weather the formular is valid or not, the error message, and the
 * error position in the formular.
 * 
 * @author philipp
 * 
 */
public class ValidationResult {
	/**
	 * True if formular is valid
	 */
	private boolean isValid;

	/**
	 * Error message, null if formular is valid.
	 */
	private String statusmessage;
	/**
	 * Errorposition in validated String which represented the formular. -1 if
	 * formular is valid.
	 */
	private int errorPosition = -1;
	
	public ValidationResult(boolean isValid, String errormessage, int errorPosition) {
		this.isValid = isValid;
		this.statusmessage = errormessage;
		this.errorPosition = errorPosition;

	
	}

	/**
	 * Returns true if formular is valid
	 * @return
	 */
	public boolean isValid() {
		return isValid;
	}

	/**
	 * Returns error message, null if formular is valid.
	 * @return
	 */
	public String getStatusMessage() {
		return statusmessage;
	}

	/**
	 * Returns error position in validated String which represented the formular. -1 if
	 * formular is valid.
	 */
	public int getErrorPosition() {
		return errorPosition;
	}

	
	
}
