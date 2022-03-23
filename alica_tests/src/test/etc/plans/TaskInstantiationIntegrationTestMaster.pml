{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1269233481168214500,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4603312216886200747,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 824745231464311542,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 4603312216886200747,
  "masterPlan": true,
  "name": "TaskInstantiationIntegrationTestMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MovePayload.pml#725594143882346503",
          "comment": "",
          "configuration": null,
          "id": 1975970727016849361,
          "name": ""
        }
      ],
      "entryPoint": 1269233481168214500,
      "id": 824745231464311542,
      "inTransitions": [],
      "name": "RunTest",
      "outTransitions": [],
      "parentPlan": 4603312216886200747,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.0,
  "variables": []
}