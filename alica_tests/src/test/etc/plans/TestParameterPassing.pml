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
          keyMapping : 
          {
            "input" : 
            [
              {
                "parent" : "planOutputKey",
                "child" : "behaviorInputKey"
              }
            ],
            "output" :
            [
              {
                "parent" : "behaviorOutputKey",
                "child" : "planInputKey"
              }
            ]
          }
        }
      ],
      "entryPoint": 58084702421574748,
      "id": 1092447442809556626,
      "inTransitions": [],
      "name": "ParameterPassingRunBehaviour",
      "outTransitions": [],
      "parentPlan": 1692837668719979457,
      "positionWeb": {
        "x": 735,
        "y": 243
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
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
      "key" : "planKey",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is a blackboard entry for testing"
    },
  ]
}