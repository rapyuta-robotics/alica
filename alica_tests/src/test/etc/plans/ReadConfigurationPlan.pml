{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1588069183324,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1588069183324",
      "plan": 1588061334567,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1588069177860,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1588061334567,
  "inheritBlackboard": false,
  "masterPlan": false,
  "name": "ReadConfigurationPlan",
  "preCondition": null,
  "relativeDirectory": "Configurations",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1588069183324,
      "id": 1588069177860,
      "inTransitions": [],
      "name": "DecisionState",
      "outTransitions": [
        1588069612659,
        1588069615552
      ],
      "parentPlan": 1588061334567,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1588069261047,
      "inTransitions": [
        1588069612659
      ],
      "name": "StateA",
      "outTransitions": [],
      "parentPlan": 1588061334567,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1588069265377,
      "inTransitions": [
        1588069615552
      ],
      "name": "StateB",
      "outTransitions": [],
      "parentPlan": 1588061334567,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "condition": "conditions/conditions.cnd#4281647834169813432",
      "id": 1588069612659,
      "inState": 1588069177860,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromDefault NameTo Default Name",
      "outState": 1588069261047,
      "pointsWeb": [],
      "preCondition": {
        "comment": "TestValue = 1",
        "conditionString": null,
        "enabled": true,
        "id": 1588069612661,
        "name": "1588069612661",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": "conditions/conditions.cnd#3684268241099966909",
      "id": 1588069615552,
      "inState": 1588069177860,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromDefault NameTo Default Name",
      "outState": 1588069265377,
      "pointsWeb": [],
      "preCondition": {
        "comment": "TestValue = 2",
        "conditionString": null,
        "enabled": true,
        "id": 1588069615553,
        "name": "1588069615553",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
