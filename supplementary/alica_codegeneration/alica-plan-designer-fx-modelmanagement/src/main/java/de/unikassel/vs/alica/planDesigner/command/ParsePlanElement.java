package de.unikassel.vs.alica.planDesigner.command;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.*;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;

import java.io.File;

/**
 * Parses a given file and adds the resulting object to the corresponding maps of the model manager.
 */
public class ParsePlanElement extends Command {
    SerializablePlanElement oldElement;
    SerializablePlanElement newElement;

    public ParsePlanElement(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        oldElement = null;
        newElement = null;
    }

    @Override
    public void doCommand() {
        // Searching for an existing element with the same id, because that will be replaced and needs to be stored for undo
        oldElement = (SerializablePlanElement) modelManager.getSerializablePlanElement(mmq);

        // Use method which is thought to load model at startup of plan designer
        newElement = modelManager.loadModelFile(FileSystemUtil.getFile(mmq), true);
    }

    @Override
    public void undoCommand() {
        if (oldElement != null) {
            // replace new object with former old one
            if (oldElement instanceof Plan && ((Plan) oldElement).getMasterPlan()) {
                modelManager.storePlanElement(Types.MASTERPLAN, oldElement, false);
            } else {
                modelManager.storePlanElement(mmq.getElementType(), oldElement, false);
            }
        } else {
            // remove new object
            if (newElement instanceof Plan && ((Plan) newElement).getMasterPlan()) {
                modelManager.dropPlanElement(Types.MASTERPLAN, newElement, false);
            } else {
                modelManager.dropPlanElement(mmq.getElementType(), newElement, false);
            }
        }
    }
}
