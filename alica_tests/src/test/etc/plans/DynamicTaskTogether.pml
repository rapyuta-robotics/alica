{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2633712961224790694,
      "isDynamic": true,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1338298120374694644,
      "positionWeb": {
        "x": 362,
        "y": 245
      },
      "state": 2362235348110947949,
      "successRequired": false,
      "task": "taskrepository.tsk#4026821563126910189"
    },
    {
      "comment": "",
      "id": 2665027307523422046,
      "isDynamic": true,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1338298120374694644,
      "positionWeb": {
        "x": 361,
        "y": 393
      },
      "state": 2564904534754645793,
      "successRequired": false,
      "task": "taskrepository.tsk#4028411332434222682"
    }
  ],
  "frequency": 0,
  "id": 1338298120374694644,
  "masterPlan": false,
  "name": "DynamicTaskTogether",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "DynamicTaskAssignmentTest.pml#2252865124432942907",
          "comment": "",
          "configuration": null,
          "id": 1974764245616019971,
          "name": ""
        }
      ],
      "entryPoint": 2633712961224790694,
      "id": 2362235348110947949,
      "inTransitions": [],
      "name": "",
      "outTransitions": [],
      "parentPlan": 1338298120374694644,
      "positionWeb": {
        "x": 590,
        "y": 231
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "DynamicTaskLA.pml#3337489358878214836",
          "comment": "",
          "configuration": null,
          "id": 840075399563888264,
          "name": ""
        }
      ],
      "entryPoint": 2665027307523422046,
      "id": 2564904534754645793,
      "inTransitions": [],
      "name": "",
      "outTransitions": [],
      "parentPlan": 1338298120374694644,
      "positionWeb": {
        "x": 591,
        "y": 379
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