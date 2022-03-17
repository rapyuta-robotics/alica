package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.Characteristic;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import de.unikassel.vs.alica.planDesigner.alicamodel.RoleSet;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateCharacteristic extends Command {

    Characteristic characteristic;
    Role role;

    public CreateCharacteristic(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.role = (Role) modelManager.getPlanElement(mmq.getParentId());
        this.characteristic = createCharacteristic(this.role);
    }

    protected Characteristic createCharacteristic(Role role) {
        Characteristic characteristic = new Characteristic();
        characteristic.setName(mmq.getName());
        characteristic.setRole(role);
        return characteristic;
    }

    @Override
    public void doCommand() {
        this.role.addCharacteristic(characteristic);
        modelManager.storePlanElement(Types.ROLE_CHARACTERISTIC, this.characteristic,  false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.characteristic);
    }

    @Override
    public void undoCommand() {
        this.role.removeCharacteristic(characteristic);
        modelManager.dropPlanElement(Types.ROLE_CHARACTERISTIC, this.characteristic, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.characteristic);
    }
}
