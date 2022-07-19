package de.unikassel.vs.alica.planDesigner.modelmanagement;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;

import java.io.File;
import java.nio.file.Paths;

public class FileSystemUtil {

    public static File getFile(ModelModificationQuery mmq) {
        switch (mmq.getElementType()) {
            case Types.PLAN:
            case Types.MASTERPLAN:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.PLAN).toFile();
            case Types.PLANTYPE:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.PLANTYPE).toFile();
            case Types.BEHAVIOUR:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.BEHAVIOUR).toFile();
            case Types.TASKREPOSITORY:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.TASKREPOSITORY).toFile();
            case Types.ROLESET:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.ROLESET).toFile();
            case Types.CONFIGURATION:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.CONFIGURATION).toFile();
            case Types.UIEXTENSION:
                return Paths.get(mmq.getAbsoluteDirectory(), mmq.getName() + "." + Extensions.PLAN_EXTENSION).toFile();
            default:
                System.err.println("FileSystemUtil: Unknown eventType gets ignored!");
                return null;
        }
    }

    public static File getFile(String absoluteDirectory, String name, String ending) {
        return Paths.get(absoluteDirectory, name + "." + ending).toFile();
    }

    /**
     * Determines the elementType string corresponding to the given PlanElement.
     *
     * @param planElement whose elementType is to be determined
     * @return elementType of the plan element
     */
    public static String getTypeString(PlanElement planElement) {
        if (planElement instanceof Plan) {
            Plan plan = (Plan) planElement;
            if (plan.getMasterPlan()) {
                return Types.MASTERPLAN;
            } else {
                return Types.PLAN;
            }
        } else if (planElement instanceof Behaviour) {
            return Types.BEHAVIOUR;
        } else if (planElement instanceof PlanType) {
            return Types.PLANTYPE;
        } else if (planElement instanceof Task) {
            return Types.TASK;
        } else if (planElement instanceof TaskRepository) {
            return Types.TASKREPOSITORY;
        } else if (planElement instanceof Role) {
            return Types.ROLE;
        } else if (planElement instanceof RoleSet) {
            return Types.ROLESET;
        } else {
            return null;
        }
    }

    /**
     * Determines the file ending string corresponding to the given SerializablePlanElement.
     *
     * @param planElement whose file ending is to be determined
     * @return file ending of the plan element
     */
    public static String getType(SerializablePlanElement planElement) {
        if (planElement instanceof Plan) {
            return Extensions.PLAN;
        } else if (planElement instanceof Behaviour) {
            return Extensions.BEHAVIOUR;
        } else if (planElement instanceof PlanType) {
            return Extensions.PLANTYPE;
        } else if (planElement instanceof TaskRepository) {
            return Extensions.TASKREPOSITORY;
        } else if (planElement instanceof Configuration) {
            return Extensions.CONFIGURATION;
        } else if (planElement instanceof RoleSet) {
            return Extensions.ROLESET;
        } else {
            return null;
        }
    }

    public static String getType(File file) {
        switch (getFileExtensionInternal(file)) {
            case Extensions.PLAN:
                return Types.PLAN;
            case Extensions.BEHAVIOUR:
                return Types.BEHAVIOUR;
            case Extensions.PLANTYPE:
                return Types.PLANTYPE;
            case Extensions.TASKREPOSITORY:
                return Types.TASKREPOSITORY;
            case Extensions.ROLESET:
                return Types.ROLESET;
            case Extensions.PLAN_EXTENSION:
                return Types.UIEXTENSION;
            case Extensions.CONFIGURATION:
                return Types.CONFIGURATION;
            case Extensions.CONDITIONS:
                return Types.CONDITIONS;
            default:
                return Types.UNSUPPORTED;
        }
    }

    public static Class getClassType(File modelFile) {
        switch (getFileExtensionInternal(modelFile)) {
            case Extensions.PLAN:
                return Plan.class;
            case Extensions.BEHAVIOUR:
                return Behaviour.class;
            case Extensions.PLANTYPE:
                return PlanType.class;
            case Extensions.TASKREPOSITORY:
                return TaskRepository.class;
            case Extensions.ROLESET:
                return RoleSet.class;
            case Extensions.CONFIGURATION:
                return Configuration.class;
            case Extensions.PLAN_EXTENSION:
                return UiExtension.class;
            case Extensions.CONDITIONS:
                return ConditionRepository.class;
            default:
                return null;
        }
    }

    private static String getFileExtensionInternal(File file) {
        String fileAsString = file.toString();
        int pointIdx = fileAsString.lastIndexOf('.');
        if (pointIdx == -1) {
            return Extensions.NO;
        }
        return fileAsString.substring(pointIdx + 1);
    }
}
