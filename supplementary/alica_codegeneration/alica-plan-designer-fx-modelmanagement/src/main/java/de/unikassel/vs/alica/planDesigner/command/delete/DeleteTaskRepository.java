package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.TaskRepository;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeleteTaskRepository extends Command {

    protected TaskRepository taskRepository;

    public DeleteTaskRepository(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.taskRepository = (TaskRepository) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        this.modelManager.dropPlanElement(Types.TASKREPOSITORY, this.taskRepository, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.taskRepository);
    }

    @Override
    public void undoCommand() {
        this.modelManager.storePlanElement(Types.TASKREPOSITORY, this.taskRepository, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.taskRepository);
    }
}
