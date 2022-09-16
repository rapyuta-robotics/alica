{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2741715629576575326,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 1,
      "name": "",
      "plan": 2425328142973735249,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 3997532517592149463,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2425328142973735249,
  "inheritBlackboard": false,
  "masterPlan": true,
  "name": "Master",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Move.pml#1889749086610694100",
          "comment": "",
          "configuration": null,
          "id": 496472308860171093,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2405597980801916441,
      "inTransitions": [
        3486027875296378577
      ],
      "name": "Move",
      "outTransitions": [
        635844345274619238
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Go2RandomPosition.beh#4085572422059465423",
          "comment": "",
          "configuration": null,
          "id": 586073573470828748,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2741715629576575326,
      "id": 3997532517592149463,
      "inTransitions": [
        635844345274619238
      ],
      "name": "Init",
      "outTransitions": [
        3486027875296378577
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#974606107671315045",
      "id": 635844345274619238,
      "inState": 2405597980801916441,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3997532517592149463,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1136497454350831106,
        "name": "Move2Init",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#748720375848597116",
      "id": 3486027875296378577,
      "inState": 3997532517592149463,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2405597980801916441,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1597434482701133956,
        "name": "Init2Move",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": [],
  "libraryName": "alica_turtlesim_library"
}
