{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2572645828071056843,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "SerializationCEP",
      "plan": 2359124678252958039,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2604890859274745239,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2359124678252958039,
  "masterPlan": false,
  "name": "SerializationSubPlanC",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "EmptyPlan.pml#984284423749038756",
          "comment": "",
          "configuration": null,
          "id": 1693312522770780390,
          "name": ""
        },
        {
          "abstractPlan": "SerializationSubPlanB.pml#230205985761632608",
          "comment": "",
          "configuration": null,
          "id": 2626133623807380846,
          "name": ""
        },
        {
          "abstractPlan": "SerializationSubPlanA.pml#1433931143598606082",
          "comment": "",
          "configuration": null,
          "id": 4188959909879260613,
          "name": ""
        }
      ],
      "entryPoint": 2572645828071056843,
      "id": 2604890859274745239,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [],
      "parentPlan": 2359124678252958039,
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