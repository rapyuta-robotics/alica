/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.eclipse.emf.common.util.Enumerator;

/**
 * <!-- begin-user-doc -->
 * A representation of the literals of the enumeration '<em><b>Planning Type</b></em>',
 * and utility methods for working with them.
 * <!-- end-user-doc -->
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningType()
 * @model
 * @generated
 */
public enum PlanningType implements Enumerator {
	/**
	 * The '<em><b>Online</b></em>' literal object.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #ONLINE_VALUE
	 * @generated
	 * @ordered
	 */
	ONLINE(0, "Online", "Online"),

	/**
	 * The '<em><b>Offline</b></em>' literal object.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #OFFLINE_VALUE
	 * @generated
	 * @ordered
	 */
	OFFLINE(1, "Offline", "Offline"),

	/**
	 * The '<em><b>Interactive</b></em>' literal object.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #INTERACTIVE_VALUE
	 * @generated
	 * @ordered
	 */
	INTERACTIVE(2, "Interactive", "Interactive");

	/**
	 * The '<em><b>Online</b></em>' literal value.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of '<em><b>Online</b></em>' literal object isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @see #ONLINE
	 * @model name="Online"
	 * @generated
	 * @ordered
	 */
	public static final int ONLINE_VALUE = 0;

	/**
	 * The '<em><b>Offline</b></em>' literal value.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of '<em><b>Offline</b></em>' literal object isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @see #OFFLINE
	 * @model name="Offline"
	 * @generated
	 * @ordered
	 */
	public static final int OFFLINE_VALUE = 1;

	/**
	 * The '<em><b>Interactive</b></em>' literal value.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of '<em><b>Interactive</b></em>' literal object isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @see #INTERACTIVE
	 * @model name="Interactive"
	 * @generated
	 * @ordered
	 */
	public static final int INTERACTIVE_VALUE = 2;

	/**
	 * An array of all the '<em><b>Planning Type</b></em>' enumerators.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	private static final PlanningType[] VALUES_ARRAY =
		new PlanningType[] {
			ONLINE,
			OFFLINE,
			INTERACTIVE,
		};

	/**
	 * A public read-only list of all the '<em><b>Planning Type</b></em>' enumerators.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public static final List<PlanningType> VALUES = Collections.unmodifiableList(Arrays.asList(VALUES_ARRAY));

	/**
	 * Returns the '<em><b>Planning Type</b></em>' literal with the specified literal value.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public static PlanningType get(String literal) {
		for (int i = 0; i < VALUES_ARRAY.length; ++i) {
			PlanningType result = VALUES_ARRAY[i];
			if (result.toString().equals(literal)) {
				return result;
			}
		}
		return null;
	}

	/**
	 * Returns the '<em><b>Planning Type</b></em>' literal with the specified name.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public static PlanningType getByName(String name) {
		for (int i = 0; i < VALUES_ARRAY.length; ++i) {
			PlanningType result = VALUES_ARRAY[i];
			if (result.getName().equals(name)) {
				return result;
			}
		}
		return null;
	}

	/**
	 * Returns the '<em><b>Planning Type</b></em>' literal with the specified integer value.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public static PlanningType get(int value) {
		switch (value) {
			case ONLINE_VALUE: return ONLINE;
			case OFFLINE_VALUE: return OFFLINE;
			case INTERACTIVE_VALUE: return INTERACTIVE;
		}
		return null;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	private final int value;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	private final String name;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	private final String literal;

	/**
	 * Only this class can construct instances.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	private PlanningType(int value, String name, String literal) {
		this.value = value;
		this.name = name;
		this.literal = literal;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public int getValue() {
	  return value;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getName() {
	  return name;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getLiteral() {
	  return literal;
	}

	/**
	 * Returns the literal value of the enumerator, which is its string representation.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public String toString() {
		return literal;
	}
	
} //PlanningType
