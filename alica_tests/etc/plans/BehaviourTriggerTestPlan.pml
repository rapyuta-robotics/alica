{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1428508768574,
      "isDynamic": false,
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
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "BehaviourTriggerTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TriggerA.beh#1428508297492",
          "comment": "",
          "configuration": null,
          "id": 1587718662751,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662751"
        },
        {
          "abstractPlan": "TriggerB.beh#1428508316905",
          "comment": "",
          "configuration": null,
          "id": 1587718662753,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662753"
        },
        {
          "abstractPlan": "TriggerC.beh#1428508355209",
          "comment": "",
          "configuration": null,
          "id": 1587718662755,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662755"
        },
        {
          "abstractPlan": "NotToTrigger.beh#1429017274116",
          "comment": "",
          "configuration": null,
          "id": 1587718662758,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662758"
        }
      ],
      "entryPoint": 1428508768574,
      "id": 1428508768573,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1428508768572,
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
