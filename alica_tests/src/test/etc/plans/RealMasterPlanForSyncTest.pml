{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1418902217841,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1418902217839,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1418902217840,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1418902217839,
  "masterPlan": true,
  "name": "RealMasterPlanForSyncTest",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MasterSyncTransition.pml#1418825395939",
          "comment": "",
          "configuration": null,
          "id": 1587718662652,
          "name": "1587718662652"
        }
      ],
      "entryPoint": 1418902217841,
      "id": 1418902217840,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1418902217839,
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
  "utilityThreshold": 0.1,
  "variables": []
}