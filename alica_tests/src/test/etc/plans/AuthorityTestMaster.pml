{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1414403396331,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1414403396331",
      "plan": 1414403396328,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1414403820806,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1414403396328,
  "masterPlan": false,
  "name": "AuthorityTestMaster",
  "preCondition": null,
  "relativeDirectory": "Authority",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AuthorityTest.pml#1414403413451",
          "comment": "",
          "configuration": null,
          "id": 1587718663037,
          "name": "1587718663037"
        }
      ],
      "entryPoint": null,
      "id": 1414403396329,
      "inTransitions": [
        1414403840950
      ],
      "name": "testState",
      "outTransitions": [],
      "parentPlan": 1414403396328,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1414403396331,
      "id": 1414403820806,
      "inTransitions": [],
      "name": "Init",
      "outTransitions": [
        1414403840950
      ],
      "parentPlan": 1414403396328,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 1414403840950,
      "inState": 1414403820806,
      "name": "1414403840950",
      "outState": 1414403396329,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1414403842622,
        "name": "1414403842622",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}