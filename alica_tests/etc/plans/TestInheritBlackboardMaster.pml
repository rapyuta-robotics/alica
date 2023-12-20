{
  "blackboard": [
    {
      "access": "protected",
      "comment": "This is a blackboard entry for testing",
      "id": 1944554894288661764,
      "key": "masterKey",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4058387577648167303,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "InheritBlackboardMasterEP",
      "plan": 1179066429431332056,
      "positionWeb": {
        "x": 265,
        "y": 263
      },
      "state": 2069338196796962571,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1179066429431332056,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "TestInheritBlackboardMaster",
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
          "id": 105160539449888469,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 4058387577648167303,
      "id": 2069338196796962571,
      "inTransitions": [],
      "name": "InheritBlackboardRunSubPlan",
      "outTransitions": [],
      "parentPlan": 1179066429431332056,
      "positionWeb": {
        "x": 627,
        "y": 277
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
