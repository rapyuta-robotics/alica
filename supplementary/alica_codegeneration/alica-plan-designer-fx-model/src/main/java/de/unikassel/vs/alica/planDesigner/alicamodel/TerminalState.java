package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleObjectProperty;

public class TerminalState extends State {

    // whether it is a success state or failure state (not editable, so no dirty flag necessary)
    private boolean success;
    protected final SimpleObjectProperty<PostCondition> postCondition = new SimpleObjectProperty<>();

    private ChangeListenerForDirtyFlag changeListener;

    public TerminalState(){ this.success = false; }

    public TerminalState(boolean success) {
        this.success = success;
    }

    public boolean isSuccess(){
        return success;
    }

    public PostCondition getPostCondition() {
        return postCondition.get();
    }
    public void setPostCondition(PostCondition postCondition) {
        this.postCondition.set(postCondition);
        if(postCondition != null){
            postCondition.registerDirtyFlag(this.changeListener);
        }
    }
    public SimpleObjectProperty<PostCondition> postConditionProperty() {
        return postCondition;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        this.changeListener = listener;
        if (this.getPostCondition() != null) {
            this.getPostCondition().registerDirtyFlag(listener);
        }
    }
}
