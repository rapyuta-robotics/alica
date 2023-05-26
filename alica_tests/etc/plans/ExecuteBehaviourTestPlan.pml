{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4098998217189528063,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3848527741370035673,
      "positionWeb": {
        "x": 248,
        "y": 250.828125
      },
      "state": 3490852498214130330,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3848527741370035673,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "ExecuteBehaviourTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 43551935622382040,
      "inTransitions": [
        332255503511535936
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3848527741370035673,
      "positionWeb": {
        "x": 407,
        "y": 557.828125
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
          "abstractPlan": "ExecuteBehaviourInSubPlan.pml#3172561495666303184",
          "comment": "",
          "configuration": null,
          "id": 3046550977966565507,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 323918347171847765,
      "inTransitions": [
        4565965325116608332
      ],
      "name": "BehInSubPlanState",
      "outTransitions": [
        1489555807089946561
      ],
      "parentPlan": 3848527741370035673,
      "positionWeb": {
        "x": 783,
        "y": 295.828125
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestBehaviour.beh#55178365253414982",
          "comment": "",
          "configuration": null,
          "id": 1617910099941462436,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 4098998217189528063,
      "id": 3490852498214130330,
      "inTransitions": [
        1489555807089946561
      ],
      "name": "BehState",
      "outTransitions": [
        332255503511535936,
        4565965325116608332
      ],
      "parentPlan": 3848527741370035673,
      "positionWeb": {
        "x": 466,
        "y": 279.828125
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2062958764563158269",
      "id": 332255503511535936,
      "inState": 3490852498214130330,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 43551935622382040,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1582070240831454302,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 1489555807089946561,
      "inState": 323918347171847765,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3490852498214130330,
      "pointsWeb": [
        {
          "x": 674,
          "y": 241.828125
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2450557420251369157,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 4565965325116608332,
      "inState": 3490852498214130330,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 323918347171847765,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3320803912990003693,
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
