{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": "123",
      "id": 1202182562814773905,
      "key": "masterKey",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3343320852743350914,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 450195508701585436,
      "positionWeb": {
        "x": 274,
        "y": 255.30540466308594
      },
      "state": 2931682442813202891,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 30,
  "id": 450195508701585436,
  "implementationName": "",
  "inheritBlackboard": true,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "TestGlobalBlackboardMaster",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestInheritBlackboard.pml#1692837668719979400",
          "comment": "",
          "configuration": null,
          "id": 948772846071364571,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 3343320852743350914,
      "id": 2931682442813202891,
      "inTransitions": [],
      "name": "InheritBlackboardRunSubPlan",
      "outTransitions": [],
      "parentPlan": 450195508701585436,
      "positionWeb": {
        "x": 483,
        "y": 242.30540466308594
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
