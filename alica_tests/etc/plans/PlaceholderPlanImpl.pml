{
  "blackboard": [
    {
      "access": "input",
      "comment": "",
      "id": 1515319670549888806,
      "key": "plan_key",
      "type": "double"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2401521081953341622,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 769233687233101247,
      "positionWeb": {
        "x": 364,
        "y": 305.3125
      },
      "state": 1658772054709491059,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1111111111111111112,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "PlaceholderPlanImpl",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlaceholderBehImpl.beh#1209933261934111920",
          "comment": "",
          "configuration": null,
          "id": 3637760735574134240,
          "keyMapping": {
            "input": [
              {
                "childKey": "beh_key",
                "parentKey": "plan_key",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2401521081953341622,
      "id": 1658772054709491059,
      "inTransitions": [],
      "name": "PlaceholderBehState",
      "outTransitions": [],
      "parentPlan": 769233687233101247,
      "positionWeb": {
        "x": 540.796335154078,
        "y": 294.88229797117026
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.0,
  "variables": [],
  "interface": true
}
