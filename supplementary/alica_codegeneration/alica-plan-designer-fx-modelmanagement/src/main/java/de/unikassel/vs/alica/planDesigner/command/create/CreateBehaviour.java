package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.PostCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.PreCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.RuntimeCondition;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateBehaviour extends Command {
    Behaviour behaviour;

    public CreateBehaviour(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.behaviour = createBehaviour();
    }

    protected Behaviour createBehaviour() {
        Behaviour behaviour = new Behaviour();
        behaviour.setName(mmq.getName());
        behaviour.setRelativeDirectory(modelManager.makeRelativeDirectory(mmq.getAbsoluteDirectory(), behaviour.getName()+ "." + Extensions.BEHAVIOUR));
        behaviour.registerDirtyFlag();
        return behaviour;
    }

    @Override
    public void doCommand() {
        modelManager.storePlanElement(Types.BEHAVIOUR, this.behaviour,true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.behaviour);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.BEHAVIOUR, this.behaviour, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.behaviour);
    }
}
