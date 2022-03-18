package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.RoleSet;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateRoleSet extends Command {

    private final RoleSet roleSet;

    public CreateRoleSet(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.roleSet = createRoleSet();
    }

    protected RoleSet createRoleSet() {
        RoleSet roleSet = new RoleSet();
        roleSet.setName(mmq.getName());
        roleSet.setDefaultRoleSet(true);
        roleSet.setRelativeDirectory(modelManager.makeRelativeDirectory(mmq.getAbsoluteDirectory(), roleSet.getName() + "." + Extensions.ROLESET));
        roleSet.registerDirtyFlag();
        return roleSet;
    }

    @Override
    public void doCommand() {
        modelManager.storePlanElement(Types.ROLESET, this.roleSet, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.roleSet);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.ROLESET, this.roleSet, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.roleSet);
    }
}
