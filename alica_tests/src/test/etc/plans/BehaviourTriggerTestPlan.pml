{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1428508768574,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1428508768572,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1428508768573,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1428508768572,
  "masterPlan": true,
  "name": "BehaviourTriggerTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/TriggerA.beh#1428508297492",
          "comment": "",
          "configuration": null,
          "id": 1587718662751,
          "name": "1587718662751"
        },
        {
          "abstractPlan": "behaviours/TriggerB.beh#1428508316905",
          "comment": "",
          "configuration": null,
          "id": 1587718662753,
          "name": "1587718662753"
        },
        {
          "abstractPlan": "behaviours/TriggerC.beh#1428508355209",
          "comment": "",
          "configuration": null,
          "id": 1587718662755,
          "name": "1587718662755"
        }
      ],
      "entryPoint": 1428508768574,
      "id": 1428508768573,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [
        1429017235181
      ],
      "parentPlan": 1428508768572,
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
          "abstractPlan": "behaviours/NotToTrigger.beh#1429017274116",
          "comment": "",
          "configuration": null,
          "id": 1587718662758,
          "name": "1587718662758"
        }
      ],
      "entryPoint": null,
      "id": 1429017227839,
      "inTransitions": [
        1429017235181
      ],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1428508768572,
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
      "id": 1429017235181,
      "inState": 1428508768573,
      "name": "MISSING_NAME",
      "outState": 1429017227839,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1429017236633,
        "name": "MISSING_NAME",
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