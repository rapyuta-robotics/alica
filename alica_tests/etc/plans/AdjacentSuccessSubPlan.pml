{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3470417373268048093,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "EntryPoint",
      "plan": 1682631238618360548,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 656998006978148289,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1682631238618360548,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "AdjacentSuccessSubPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 235276470945876557,
      "inTransitions": [
        1747408236004727286
      ],
      "name": "WaitState",
      "outTransitions": [
        4079672912751513705
      ],
      "parentPlan": 1682631238618360548,
      "positionWeb": {
        "x": 627.8733431516937,
        "y": 79.53460972017673
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 496520533178003845,
      "inTransitions": [
        4079672912751513705
      ],
      "name": "SucState",
      "outTransitions": [],
      "parentPlan": 1682631238618360548,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3470417373268048093,
      "id": 656998006978148289,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1747408236004727286
      ],
      "parentPlan": 1682631238618360548,
      "positionWeb": {
        "x": 427.15758468335787,
        "y": 332.21974181962383
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#19871606597697646",
      "id": 1747408236004727286,
      "inState": 656998006978148289,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 235276470945876557,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 597347780541336226,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3517323109117319233",
      "id": 4079672912751513705,
      "inState": 235276470945876557,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 496520533178003845,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1067314038887345208,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
