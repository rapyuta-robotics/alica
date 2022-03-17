package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateEntryPoint extends UiPositionCommand {

    protected EntryPoint entryPoint;

    public CreateEntryPoint(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        this.entryPoint = createEntryPoint();
        this.uiElement = createUiElement(mmq.getParentId(), this.entryPoint);
    }

    protected EntryPoint createEntryPoint() {
        EntryPoint entryPoint = new EntryPoint();
        entryPoint.setMaxCardinality(Integer.MAX_VALUE);
        entryPoint.setPlan((Plan) this.modelManager.getPlanElement(mmq.getParentId()));
        entryPoint.setTask((Task) this.modelManager.getPlanElement(mmq.getRelatedObjects().get(Types.TASK)));
        return entryPoint;
    }

    @Override
    public void doCommand() {
        this.uiExtension.getPlan().addEntryPoint(entryPoint);
        this.uiExtension.add(entryPoint.getId(), uiElement);
        this.modelManager.storePlanElement(Types.ENTRYPOINT, this.entryPoint, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.entryPoint);
    }

    @Override
    public void undoCommand() {
        this.uiExtension.getPlan().removeEntryPoint(entryPoint);
        this.uiExtension.remove(entryPoint.getId());
        this.modelManager.dropPlanElement(Types.ENTRYPOINT, this.entryPoint, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.entryPoint);
    }
}
