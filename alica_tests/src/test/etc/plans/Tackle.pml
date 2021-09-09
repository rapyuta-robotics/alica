{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402489329142,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402489318663,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1402489329141,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402489318663,
  "masterPlan": false,
  "name": "Tackle",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/AttackOpp.beh#1402489351885",
          "comment": "",
          "configuration": null,
          "id": 1587718662873,
          "name": "1587718662873"
        }
      ],
      "entryPoint": 1402489329142,
      "id": 1402489329141,
      "inTransitions": [],
      "name": "AttackOpp",
      "outTransitions": [],
      "parentPlan": 1402489318663,
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