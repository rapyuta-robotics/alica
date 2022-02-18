{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 58084702421574748,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "ParameterPassingSubplanEP",
      "plan": 1692837668719979457,
      "positionWeb": {
        "x": 309,
        "y": 218
      },
      "state": 1092447442809556626,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1692837668719979457,
  "masterPlan": false,
  "name": "TestParameterPassing",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": true,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/TestParameterPassingBehaviour.beh#831400441334251602",
          "comment": "",
          "configuration": null,
          "id": 445396005944825225,
          "name": "",
          "keyMapping" : 
          {
            "input" : 
            [
              {
                "parentKey" : "planOutputKey",
                "childKey" : "behaviorInputKey"
              }
            ],
            "output" :
            [
              {
                "parentKey" : "planInputKey",
                "childKey" : "behaviorOutputKey"
              }
            ]
          }
        }
      ],
      "entryPoint": 58084702421574748,
      "id": 1092447442809556626,
      "inTransitions": [2229456609900],
      "name": "FirstCall",
      "outTransitions": [1129456609900],
      "parentPlan": 1692837668719979457,
      "positionWeb": {
        "x": 735,
        "y": 243
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/TestParameterPassingBehaviour.beh#831400441334251602",
          "comment": "",
          "configuration": null,
          "id": 445396005944825226,
          "name": "",
          "keyMapping" : 
          {
            "input" : 
            [
              {
                "parentKey" : "planSecondOutputKey",
                "childKey" : "behaviorInputKey"
              }
            ]
          }
        }
      ],
      "entryPoint": null,
      "id": 1529456591400,
      "inTransitions": [1129456609900],
      "name": "SecondCall",
      "outTransitions": [2229456609900],
      "parentPlan": 1692837668719979457,
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
      "comment": "Forth",
      "id": 1129456609900,
      "inState": 1092447442809556626,
      "name": "MISSING_NAME",
      "outState": 1529456591400,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1529456610600,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "Back",
      "id": 2229456609900,
      "inState": 1529456591400,
      "name": "MISSING_NAME",
      "outState":1092447442809556626 ,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2529456610600,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": [],
  "inheritBlackboard" : false,
  "blackboard" : [
    {
      "key" : "planOutputKey",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is a blackboard entry for testing"
    },
    {
      "key" : "planInputKey",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is a blackboard entry for testing"
    },
    {
      "key" : "planSecondOutputKey",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is a blackboard entry for testing"
    },
    {
      "key" : "planKey",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is a blackboard entry for testing"
    },
    {
      "key" : "planInputFromMaster",
      "type" : "std::any",
      "access" : "input",
      "defaultValue" : null,
      "comment" : "This is a blackboard entry for testing"
    }
  ]
}