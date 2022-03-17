package de.unikassel.vs.alica.planDesigner.alicamodel;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Configuration extends SerializablePlanElement {
    private final Map<String, String> parameters = new HashMap<>();

    public Map<String, String> getParameters() {
        return Collections.unmodifiableMap(parameters);
    }

    public Configuration () {
        super();
    }

    /**
     * For deserialization through Jackson only.
     */
    public Configuration (long id) {
        this.id = id;
    }

    public String putParameter(String key, String value) {
        setDirty(true);
        return parameters.put(key, value);
    }

    public String removeParameter(String key) {
        setDirty(true);
        return parameters.remove(key);
    }

    /**
     * Used for any kind of modification of the parameters: Insert, Change, Remove, etc.
     *
     * @param newEntry
     * @param oldEntry
     */
    public void modifyParameter(Map.Entry<String, String> newEntry, Map.Entry<String, String> oldEntry) {
        setDirty(true);

        // Insert new entry (no old entry)
        if (oldEntry == null) {
            this.parameters.put(newEntry.getKey(), newEntry.getValue());
            return;
        }

        // Remove parameter (no new entry)
        if (newEntry == null) {
            this.parameters.remove(oldEntry.getKey());
            return;
        }

        // Modify existing parameter (old and new entry given)
        if (newEntry.getKey() == oldEntry.getKey()) {
            this.parameters.put(newEntry.getKey(), newEntry.getValue());
        } else {
            this.parameters.remove(oldEntry.getKey());
            this.parameters.put(newEntry.getKey(), newEntry.getValue());
        }
    }
}
