{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 699381937789438517,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1602078208698393838,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 4467904887554008050,
      "successRequired": false,
      "task": "taskrepository.tsk#3903894018484081749"
    }
  ],
  "frequency": 0,
  "id": 1602078208698393838,
  "masterPlan": true,
  "name": "DynamicTaskAssignmentTestMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 699381937789438517,
      "id": 4467904887554008050,
      "inTransitions": [],
      "name": "Init",
      "outTransitions": [
        4266666033623620026
      ],
      "parentPlan": 1602078208698393838,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1317277234576050904,
      "inTransitions": [
        3712615202042019043
      ],
      "name": "Finish",
      "outTransitions": [],
      "parentPlan": 1602078208698393838,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "DynamicTaskAssignmentTest.pml#2252865124432942907",
          "comment": "",
          "configuration": null,
          "id": 3876183027210268413,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 751302000461175045,
      "inTransitions": [
        4266666033623620026
      ],
      "name": "Start",
      "outTransitions": [
        3712615202042019043
      ],
      "parentPlan": 1602078208698393838,
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
      "comment": "",
      "id": 4266666033623620026,
      "inState": 4467904887554008050,
      "name": "",
      "outState": 751302000461175045,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4496654201854254411,
        "name": "InitDone",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 3712615202042019043,
      "inState": 751302000461175045,
      "name": "",
      "outState": 1317277234576050904,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4344644064496100420,
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