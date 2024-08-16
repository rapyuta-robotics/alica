{
  "blackboard": [
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 610196170392400041,
      "key": "data",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1293324462022421085,
      "key": "target_x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1302855491540810679,
      "key": "target_y",
      "type": "double"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2271062550998909602,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3683805017237692753,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2578169245525112306,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 3683805017237692753,
  "implementationName": "PopulateBlackboard",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "MoveToWorkflow",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1376360864978758396,
      "inTransitions": [
        1410678085374684927
      ],
      "name": "Moved",
      "outTransitions": [],
      "parentPlan": 3683805017237692753,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 1851137403713380951,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": "target_x",
                "value": null
              },
              {
                "childKey": "goal_y",
                "parentKey": "target_y",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2271062550998909602,
      "id": 2578169245525112306,
      "inTransitions": [],
      "name": "GoTo",
      "outTransitions": [
        1410678085374684927
      ],
      "parentPlan": 3683805017237692753,
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
      "condition": "ConditionRepository.cnd#1",
      "id": 1410678085374684927,
      "inState": 2578169245525112306,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1376360864978758396,
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
        "id": 332397056034410795,
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
