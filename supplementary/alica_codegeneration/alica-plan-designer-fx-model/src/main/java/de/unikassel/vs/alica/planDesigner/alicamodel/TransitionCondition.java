package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleStringProperty;

public class TransitionCondition extends PlanElement {

    protected ConditionRepository conditionRepository;
    protected final SimpleStringProperty relativeDirectory = new SimpleStringProperty(this, "relativeDirectory", null);


    public TransitionCondition() {
        super();
    }

    public TransitionCondition (long id) {
        this.id = id;
    }

    public ConditionRepository getConditionRepository() {
        return conditionRepository;
    }

    public void setConditionRepository(ConditionRepository conditionRepository) {
        this.conditionRepository = conditionRepository;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        comment.addListener(listener);
        name.addListener(listener);
    }

    public String getRelativeDirectory() {
        return relativeDirectory.get();
    }
    public void setRelativeDirectory(String relativeDirectory) {
        this.relativeDirectory.set(relativeDirectory);
    }
    public SimpleStringProperty realativeDirectoryProperty() {
        return relativeDirectory;
    }
}
