package com.rapyutarobotics.alica;

import javafx.util.Pair;

import java.util.*;

public class ReferenceCollection<K,V> {

    boolean allowMultipleValuesPerKey;
    LinkedList<Pair<K,V>> referenceList;
    HashMap<K,V> referenceMap;

    ReferenceCollection(boolean allowMultipleValuesPerKey) {
        this.allowMultipleValuesPerKey = allowMultipleValuesPerKey;
        if (this.allowMultipleValuesPerKey) {
            referenceList = new LinkedList<>();
        } else {
            referenceMap = new HashMap<>();
        }
    }

    public void put(K key, V value) {
        if(allowMultipleValuesPerKey) {
            referenceList.add(new Pair<K,V> (key,value));
        } else {
            if (referenceMap.containsKey(key)) {
                throw new RuntimeException("[ReferenceCollection] Key " + key + " already is assigned to value " + value + "!");
            }
            referenceMap.put(key,value);
        }
    }

    public LinkedList<K> getKeys() {
        LinkedList<K> keys = new LinkedList<>();
        if(allowMultipleValuesPerKey) {
            for (Pair<K,V> entry : referenceList) {
                keys.add(entry.getKey());
            }
        } else {
            keys.addAll(referenceMap.keySet());
        }
        return keys;
    }

    public LinkedList<Pair<K,V>> getEntries() {
        if (allowMultipleValuesPerKey) {
            return referenceList;
        } else {
            LinkedList<Pair<K,V>> entryList = new LinkedList<>();
            for (Map.Entry<K, V> entry : referenceMap.entrySet()) {
                entryList.add(new Pair<K,V>(entry.getKey(), entry.getValue()));
            }
            return entryList;
        }
    }

    public V get(K key) {
        if (allowMultipleValuesPerKey) {
            for (Pair<K,V> entry : referenceList) {
                if (entry.getKey().equals(key)) {
                    return entry.getValue();
                }
            }
            return null;
        } else {
            return referenceMap.get(key);
        }
    }

    public boolean containsKey(K key) {
        if (allowMultipleValuesPerKey) {
            for (Pair<K,V> entry : referenceList) {
                if (entry.getKey().equals(key)) {
                    return true;
                }
            }
            return false;
        } else {
            return referenceMap.containsKey(key);
        }
    }

    public void clear() {
        if (allowMultipleValuesPerKey) {
            referenceList.clear();
        } else {
            referenceMap.clear();
        }
    }
}
