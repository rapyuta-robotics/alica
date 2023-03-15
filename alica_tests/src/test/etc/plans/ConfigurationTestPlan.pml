{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 2539111367556229723,
      "key": "numberOfCalls",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1588061024407,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1588061024407",
      "plan": 1588060981661,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1588060991102,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1588060981661,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "ConfigurationTestPlan",
  "preCondition": null,
  "relativeDirectory": "Configurations",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1588061024407,
      "id": 1588060991102,
      "inTransitions": [],
      "name": "Default Name",
      "outTransitions": [
        1588253347211
      ],
      "parentPlan": 1588060981661,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1588253341545,
      "inTransitions": [
        1588253347211
      ],
      "name": "Default Name",
      "outTransitions": [],
      "parentPlan": 1588060981661,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#2163654295690873706",
      "id": 1588253347211,
      "inState": 1588060991102,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": "numberOfCalls",
            "value": null
          }
        ],
        "output": []
      },
      "name": "FromDefault NameTo Default Name",
      "outState": 1588253341545,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1588253347213,
        "name": "1588253347213",
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
