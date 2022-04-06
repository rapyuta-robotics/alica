package de.unikassel.vs.alica.planDesigner.uiextensionmodel;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class UiExtension {

    protected Plan plan;
    protected HashMap<Long, UiElement> uiElementMap = null;

    /**
     * Default-constructor, only necessary for deserialization
     */
    private UiExtension() {
        this(null);
    }

    public UiExtension(Plan plan) {
        this.plan = plan;
        this.uiElementMap = new HashMap<>();
    }

    public Plan getPlan() {
        return plan;
    }

    public void setPlan(Plan plan) {
        this.plan = plan;
    }

    public Set<Long> getKeys() {
        return uiElementMap.keySet();
    }

    public void remove(Long key) {
        this.uiElementMap.remove(key);
    }

    public void add(Long key, UiElement value) {
        this.uiElementMap.put(key, value);
    }

    public Map<Long, UiElement> getUiElementMap() {
        return Collections.unmodifiableMap(this.uiElementMap);
    }

    /**
     * Method for simplifying the access to the {@link UiElement}-objects.
     * <p>
     * Whenever a {@link PlanElement}, which has no UiElement, is
     * requested, a  UiElement is created and put into the map. This Method
     * may not be required later, when PmlUiExtensions are saved, loaded and created
     * automatically
     *
     * @param planElementId the {@link PlanElement} to find a {@link UiElement} for
     * @return the corresponding {@link UiElement} or a new one
     */
    public UiElement getUiElement(Long planElementId) {
        UiElement uiElement = uiElementMap.get(planElementId);
        if (uiElement == null) {
            uiElement = new UiElement();
            uiElementMap.put(planElementId, uiElement);
        }
        return uiElement;
    }

    public void registerDirtyListeners() {
        for (UiElement uiElement : uiElementMap.values()) {
            registerDirtyListeners(uiElement);
        }
    }

    private void registerDirtyListeners(UiElement extension) {
        extension.xProperty().addListener((observable, oldValue, newValue) ->
                plan.setDirty(true)
        );
        extension.yProperty().addListener((observable, oldValue, newValue) ->
                plan.setDirty(true)
        );
    }

}
