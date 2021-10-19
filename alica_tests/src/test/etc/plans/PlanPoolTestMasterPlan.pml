{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4019498150183138248,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 1964838032551226161,
      "positionWeb": {
        "x": 324.99476896251065,
        "y": 222.08544027898864
      },
      "state": 508968687272454527,
      "successRequired": false,
      "task": "taskrepository.tsk#1222613952469"
    }
  ],
  "frequency": 0,
  "id": 1964838032551226161,
  "masterPlan": true,
  "name": "PlanPoolTestMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanPoolTestSubPlan.pml#432995127772554364",
          "comment": "",
          "configuration": "configurations/ConfigA.cfg#1588061188681",
          "id": 1853629593917045647,
          "name": ""
        }
      ],
      "entryPoint": 4019498150183138248,
      "id": 508968687272454527,
      "inTransitions": [
        3610919168422994279
      ],
      "name": "",
      "outTransitions": [
        4186311028071767502
      ],
      "parentPlan": 1964838032551226161,
      "positionWeb": {
        "x": 625.2563208369658,
        "y": 366.462074978204
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanPoolTestSubPlan.pml#432995127772554364",
          "comment": "",
          "configuration": "configurations/ConfigB.cfg#1588061200689",
          "id": 796804573315698853,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 807253925929611286,
      "inTransitions": [
        4186311028071767502
      ],
      "name": "",
      "outTransitions": [
        3610919168422994279
      ],
      "parentPlan": 1964838032551226161,
      "positionWeb": {
        "x": 1087.6800348735833,
        "y": 419.81865736704447
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 3610919168422994279,
      "inState": 807253925929611286,
      "name": "",
      "outState": 508968687272454527,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4115970455290610262,
        "name": "4115970455290610262",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 4186311028071767502,
      "inState": 508968687272454527,
      "name": "",
      "outState": 807253925929611286,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4238964946542987247,
        "name": "4238964946542987247",
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
