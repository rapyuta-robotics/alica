package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleFloatProperty;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RoleSet extends SerializablePlanElement {

    private SimpleFloatProperty defaultPriority = new SimpleFloatProperty();
    private SimpleBooleanProperty defaultRoleSet = new SimpleBooleanProperty();
    private final ArrayList<Role> roles = new ArrayList<>();

    public RoleSet() {}

    public float getDefaultPriority() {
        return defaultPriority.get();
    }
    public void setDefaultPriority(float defaultPriority) {
        this.defaultPriority.setValue(defaultPriority);
    }

    public boolean getDefaultRoleSet() {
        return defaultRoleSet.get();
    }
    public void setDefaultRoleSet(boolean defaultRoleSet) {
        this.defaultRoleSet.setValue(defaultRoleSet);
    }

    public Role getRole(long roleID) {
        for (Role role : roles) {
            if (roleID == role.getId()) {
                return role;
            }
        }
        return null;
    }
    public List<Role> getRoles() {
        return Collections.unmodifiableList(roles);
    }
    public void addRole(Role role) {
        role.registerDirtyFlag(this.changeListenerForDirtyFlag);
        roles.add(role);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public void removeRole(Role role) {
        roles.remove(role);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public boolean containsRole (Role role) {
        return roles.contains(role);
    }

    @Override
    public void registerDirtyFlag() {
        super.registerDirtyFlag();
        this.defaultPriority.addListener(this.changeListenerForDirtyFlag);
        this.defaultRoleSet.addListener(this.changeListenerForDirtyFlag);

        for (Role role : roles) {
            role.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
}
