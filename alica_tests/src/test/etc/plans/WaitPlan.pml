{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 13426738844110157,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 2773486839180285027,
      "positionWeb": {
        "x": 200,
        "y": 211.48134443681317
      },
      "state": 1909100645626369899,
      "successRequired": false,
      "task": "taskrepository.tsk#1222613952469"
    }
  ],
  "frequency": 0,
  "id": 2773486839180285027,
  "masterPlan": false,
  "name": "WaitPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 699161635959867032,
      "inTransitions": [
        1509957282302076977
      ],
      "name": "Suc",
      "outTransitions": [],
      "parentPlan": 2773486839180285027,
      "positionWeb": {
        "x": 685.96875,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 13426738844110157,
      "id": 1909100645626369899,
      "inTransitions": [],
      "name": "Wait",
      "outTransitions": [
        1509957282302076977
      ],
      "parentPlan": 2773486839180285027,
      "positionWeb": {
        "x": 427.984375,
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
      "id": 1509957282302076977,
      "inState": 1909100645626369899,
      "name": "",
      "outState": 699161635959867032,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3266818544279107129,
        "name": "ToSuccess",
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