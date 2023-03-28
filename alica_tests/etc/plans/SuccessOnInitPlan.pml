{
  "blackboard": [],
  "comment": "A plan that immediately transitions to success state on init",
  "entryPoints": [
    {
      "comment": "",
      "id": 3588095671831141137,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1863216812678266511,
      "positionWeb": {
        "x": 336,
        "y": 196
      },
      "state": 250474402398721721,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1863216812678266511,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SuccessOnInitPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 66024355821713834,
      "inTransitions": [
        4061633193878470475
      ],
      "name": "SuccessOnInitSuccessState",
      "outTransitions": [],
      "parentPlan": 1863216812678266511,
      "positionWeb": {
        "x": 728,
        "y": 328
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3588095671831141137,
      "id": 250474402398721721,
      "inTransitions": [],
      "name": "DummyInitState",
      "outTransitions": [
        4061633193878470475
      ],
      "parentPlan": 1863216812678266511,
      "positionWeb": {
        "x": 520,
        "y": 185
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2872265442510628524",
      "id": 4061633193878470475,
      "inState": 250474402398721721,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 66024355821713834,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4197030928062612573,
        "name": "",
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
