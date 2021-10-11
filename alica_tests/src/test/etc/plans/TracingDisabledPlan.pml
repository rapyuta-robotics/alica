{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2870795844818415325,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "TracingDisabledEntry",
      "plan": 3148641312534759067,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 4222160850529504715,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3148641312534759067,
  "masterPlan": false,
  "name": "TracingDisabledPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/EnabledTracing.beh#3606794787493300754",
          "comment": "",
          "configuration": null,
          "id": 1038584704312324156,
          "name": ""
        },
        {
          "abstractPlan": "behaviours/DisabledTracing.beh#863651328966767832",
          "comment": "",
          "configuration": null,
          "id": 4202745231514801924,
          "name": ""
        }
      ],
      "entryPoint": 2870795844818415325,
      "id": 4222160850529504715,
      "inTransitions": [],
      "name": "RunBehaviour",
      "outTransitions": [],
      "parentPlan": 3148641312534759067,
      "positionWeb": {
        "x": 427.984375,
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