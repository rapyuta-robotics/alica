{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 3047597954425160962,
      "key": "blackboardKeys",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2622393546596805822,
      "key": "blackboardBlueprint",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1293324462022421085,
      "key": "x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1302855491540810679,
      "key": "y",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3949955977224766241,
      "key": "blackboard",
      "type": "std::any"
    },
    {
      "access": "input",
      "comment": "",
      "id": 610196170392400041,
      "key": "task",
      "type": "std::string"
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
        "x": 578,
        "y": 436
      },
      "state": 2507524292527222213,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 3683805017237692753,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": false,
  "name": "AdhocMoveWorkflow",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PopulateBlackboardFromJson.beh#3218372711073374685",
          "comment": "",
          "configuration": null,
          "id": 2052369829007344956,
          "keyMapping": {
            "input": [
              {
                "childKey": "blackboard",
                "parentKey": "blackboard",
                "value": null
              },
              {
                "childKey": "blackboardBlueprint",
                "parentKey": "blackboardBlueprint",
                "value": null
              },
              {
                "childKey": "blackboardKeys",
                "parentKey": "blackboardKeys",
                "value": null
              },
              {
                "childKey": "json",
                "parentKey": "task",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2271062550998909602,
      "id": 2507524292527222213,
      "inTransitions": [],
      "name": "PopulateBlackboard",
      "outTransitions": [
        3522637572077184775
      ],
      "parentPlan": 3683805017237692753,
      "positionWeb": {
        "x": 795,
        "y": 422
      },
      "type": "State",
      "variableBindings": []
    },
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
        "x": 1496,
        "y": 421
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
                "parentKey": "x",
                "value": null
              },
              {
                "childKey": "goal_y",
                "parentKey": "y",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2578169245525112306,
      "inTransitions": [
        3522637572077184775
      ],
      "name": "GoTo",
      "outTransitions": [
        1410678085374684927
      ],
      "parentPlan": 3683805017237692753,
      "positionWeb": {
        "x": 1183,
        "y": 421
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
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 332397056034410795,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 3522637572077184775,
      "inState": 2507524292527222213,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2578169245525112306,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 976647507208964729,
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
