{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 560547937773733569,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 3172561495666303184,
      "positionWeb": {
        "x": 278,
        "y": 261
      },
      "state": 4459593820134418510,
      "successRequired": false,
      "task": "taskrepository.tsk#1222613952469"
    }
  ],
  "frequency": 0,
  "id": 3172561495666303184,
  "masterPlan": false,
  "name": "ExecuteBehaviourPlan2",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3575867719445223184,
      "inTransitions": [
        2330492839242485043
      ],
      "name": "Suc",
      "outTransitions": [],
      "parentPlan": 3172561495666303184,
      "positionWeb": {
        "x": 1306,
        "y": 313
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "ExecuteBehaviourPlan.pml#2773486839180285027",
          "comment": "",
          "configuration": null,
          "id": 1933475273272858624,
          "name": ""
        }
      ],
      "entryPoint": 560547937773733569,
      "id": 4459593820134418510,
      "inTransitions": [],
      "name": "Start",
      "outTransitions": [
        2330492839242485043
      ],
      "parentPlan": 3172561495666303184,
      "positionWeb": {
        "x": 800,
        "y": 277
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 2330492839242485043,
      "inState": 4459593820134418510,
      "name": "",
      "outState": 3575867719445223184,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1943478533524176732,
        "name": "ToSuc",
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