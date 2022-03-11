package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.TaskRepository;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateTaskRepository extends Command {

    TaskRepository taskRepository;

    public CreateTaskRepository(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.taskRepository = createTaskRepository();
    }


    protected TaskRepository createTaskRepository() {
        TaskRepository repository = new TaskRepository();
        repository.setName(mmq.getName());
        repository.setRelativeDirectory(modelManager.makeRelativeDirectory(mmq.getAbsoluteDirectory(), repository.getName() + "." + Extensions.TASKREPOSITORY));
        repository.registerDirtyFlag();
        return repository;
    }

    @Override
    public void doCommand() {
        modelManager.storePlanElement(Types.TASKREPOSITORY, this.taskRepository, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.taskRepository);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.TASKREPOSITORY, this.taskRepository, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.taskRepository);
    }
}
