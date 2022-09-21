package de.unikassel.vs.alica.planDesigner.alicamodel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ConditionRepository extends SerializablePlanElement {

    protected final ArrayList<TransitionCondition> conditions = new ArrayList<>();

    public ConditionRepository() {}

    public ConditionRepository(long id) { this.id = id; }

    public void addTransitionCondition(TransitionCondition condition) {
        if (condition.getConditionRepository() != this) {
            condition.setConditionRepository(this);
        }
        condition.registerDirtyFlag(this.changeListenerForDirtyFlag);
        conditions.add(condition);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public void removeTransitionCondition(TransitionCondition condition) {
        conditions.remove(condition);
        condition.setConditionRepository(null);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public List<TransitionCondition> getConditions() {
        return Collections.unmodifiableList(conditions);
    }

    public TransitionCondition getTransitionCondition(long conditionID) {
        for (TransitionCondition condition : conditions) {
            if (conditionID == condition.getId()) {
                return condition;
            }
        }
        return null;
    }

    public boolean contains (TransitionCondition condition) {
        return conditions.contains(condition);
    }

    @Override
    public void registerDirtyFlag() {
        super.registerDirtyFlag();
        for (TransitionCondition condition : conditions) {
            condition.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
}
