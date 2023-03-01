{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 53133849509772171,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1449843163261594615,
      "positionWeb": {
        "x": 522,
        "y": 421.99999237060547
      },
      "state": 1256477521278293720,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 1449843163261594615,
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": true,
  "name": "SurroundLeaderTutorial",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 4220525235259193568,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 53133849509772171,
      "id": 1256477521278293720,
      "inTransitions": [],
      "name": "SpawnTurtle",
      "outTransitions": [
        1701344904592867727
      ],
      "parentPlan": 1449843163261594615,
      "positionWeb": {
        "x": 762,
        "y": 408.99999237060547
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Simulation.pml#2425328142973735249",
          "comment": "",
          "configuration": null,
          "id": 814335721917579950,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3424972982848590838,
      "inTransitions": [
        1701344904592867727
      ],
      "name": "Simulation",
      "outTransitions": [],
      "parentPlan": 1449843163261594615,
      "positionWeb": {
        "x": 1147,
        "y": 406.99999237060547
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2190266318562141841",
      "id": 1701344904592867727,
      "inState": 1256477521278293720,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3424972982848590838,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1158412559413860996,
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
