package de.unikassel.vs.alica.planDesigner.alicamodel;

import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import javafx.beans.property.SimpleStringProperty;

@JsonIdentityInfo(
        generator = ObjectIdGenerators.PropertyGenerator.class,
        property = "id")
public class PlanElement {
    public static final String forbiddenCharacters = ".*[\\./\\*\\\\$§?\\[\\]!{}\\-äüö#\"%~'ÄÖÜß@,]+.*";
    public static final String NO_NAME = "NO_NAME";

    protected static int PLAN_ELEMENT_COUNTER = 0;

    protected long id;
    protected final SimpleStringProperty name = new SimpleStringProperty(null, "name", "");
    protected final SimpleStringProperty comment = new SimpleStringProperty(null, "comment", "");

    public PlanElement() {
        this.id = generateId();
    }

    public PlanElement(long id) {
        this.id = id;
    }

    public long getId() {
        return id;
    }

    protected long generateId() {
        return System.currentTimeMillis() + PLAN_ELEMENT_COUNTER++;
    }

    public String getName() {
        if (name.get() == null || name.get().isEmpty())
        {
            return Long.toString(id);
        }
        return name.get();
    }

    public void setName(String name) {
        if (name.matches(forbiddenCharacters)) {
            name.replaceAll(forbiddenCharacters, "");
        } else {
            this.name.set(name);
        }
    }

    public SimpleStringProperty nameProperty() {
        return name;
    }

    public String getComment() {
        return comment.get();
    }

    public void setComment(String comment) {
        this.comment.set(comment);
    }

    public SimpleStringProperty commentProperty() {
        return comment;
    }
}
