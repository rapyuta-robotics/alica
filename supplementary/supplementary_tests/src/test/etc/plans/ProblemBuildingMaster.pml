{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1479556022228,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1479556022226,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1479556022227,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1479556022226,
  "masterPlan": false,
  "name": "ProblemBuildingMaster",
  "preCondition": null,
  "relativeDirectory": "ProblemModule",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "ProbBuildingLevel1.pml#1479557378264",
          "comment": "",
          "configuration": null,
          "id": 1597658636476,
          "name": "1597658636476"
        }
      ],
      "entryPoint": 1479556022228,
      "id": 1479556022227,
      "inTransitions": [],
      "name": "State1",
      "outTransitions": [
        1479557591331
      ],
      "parentPlan": 1479556022226,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1479557551007,
          "name": "MISSING_NAME",
          "subPlan": "ProbBuildingLevel1.pml#1479557378264",
          "subVariable": "ProbBuildingLevel1.pml#1479557444388",
          "variable": 1479557345903
        },
        {
          "comment": "",
          "id": 1479557555606,
          "name": "MISSING_NAME",
          "subPlan": "ProbBuildingLevel1.pml#1479557378264",
          "subVariable": "ProbBuildingLevel1.pml#1479557432793",
          "variable": 1479557337956
        }
      ]
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "ProbBuildingLevel1_1.pml#1479557664989",
          "comment": "",
          "configuration": null,
          "id": 1597658636487,
          "name": "1597658636487"
        }
      ],
      "entryPoint": null,
      "id": 1479557585252,
      "inTransitions": [
        1479557591331
      ],
      "name": "State2",
      "outTransitions": [],
      "parentPlan": 1479556022226,
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
      "id": 1479557591331,
      "inState": 1479556022227,
      "name": "MISSING_NAME",
      "outState": 1479557585252,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1479557592662,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [
          {
            "comment": "",
            "id": 1479557619214,
            "name": "MISSING_NAME",
            "quantifierType": "all",
            "scope": 1479556022227,
            "sorts": [
              "X",
              "Y"
            ]
          }
        ],
        "variables": [
          1479557337956,
          1479557345903
        ]
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": [
    {
      "comment": "",
      "id": 1479557337956,
      "name": "PBMX",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1479557345903,
      "name": "PBMY",
      "variableType": ""
    }
  ]
}