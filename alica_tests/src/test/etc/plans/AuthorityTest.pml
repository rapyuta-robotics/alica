{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1414403429951,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "1414403429951",
      "plan": 1414403413451,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1414403429950,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1414403522424,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "AttackTask",
      "plan": 1414403413451,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1414403553717,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    }
  ],
  "frequency": 0,
  "id": 1414403413451,
  "masterPlan": false,
  "name": "AuthorityTest",
  "preCondition": null,
  "relativeDirectory": "Authority",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/EmptyBehaviour.beh#1625610857563",
          "comment": "",
          "configuration": null,
          "id": 1626437211225,
          "name": "1626437211225"
        }
      ],
      "entryPoint": 1414403429951,
      "id": 1414403429950,
      "inTransitions": [],
      "name": "UpperState",
      "outTransitions": [],
      "parentPlan": 1414403413451,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/EmptyBehaviour.beh#1625610857563",
          "comment": "",
          "configuration": null,
          "id": 1626437213779,
          "name": "1626437213779"
        }
      ],
      "entryPoint": 1414403522424,
      "id": 1414403553717,
      "inTransitions": [],
      "name": "LowerState",
      "outTransitions": [],
      "parentPlan": 1414403413451,
      "positionWeb": {
        "x": 428,
        "y": 400
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