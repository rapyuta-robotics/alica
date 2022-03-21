{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2603044554417791500,
      "isDynamic": true,
      "maxCardinality": 1,
      "minCardinality": 0,
      "name": "MovePayload",
      "plan": 725594143882346503,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 3785266111914580157,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1615605791478451403,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "Idle",
      "plan": 725594143882346503,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 3766678166599855988,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 725594143882346503,
  "masterPlan": false,
  "name": "PickDrop",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Pick.beh#2580816776008671737",
          "comment": "",
          "configuration": null,
          "id": 4062831504515318713,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1965586362363306757,
      "inTransitions": [
        3201336270766679779
      ],
      "name": "Pick",
      "outTransitions": [
        1425389364493331507
      ],
      "parentPlan": 725594143882346503,
      "positionWeb": {
        "x": 944,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1615605791478451403,
      "id": 3766678166599855988,
      "inTransitions": [],
      "name": "Idle",
      "outTransitions": [],
      "parentPlan": 725594143882346503,
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
          "abstractPlan": "behaviours/AssignPayload.beh#3826644292150922713",
          "comment": "",
          "configuration": null,
          "id": 2590285740552773678,
          "name": ""
        }
      ],
      "entryPoint": 2603044554417791500,
      "id": 3785266111914580157,
      "inTransitions": [],
      "name": "AssignPayload",
      "outTransitions": [
        3103663386312740882
      ],
      "parentPlan": 725594143882346503,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/FreePayload.beh#422054015709952219",
          "comment": "",
          "configuration": null,
          "id": 4191136452969136787,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3464891834530950837,
      "inTransitions": [
        2630758425967053453
      ],
      "name": "Moved",
      "outTransitions": [],
      "parentPlan": 725594143882346503,
      "positionWeb": {
        "x": 1718,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/NavigateToPick.beh#4505472195947429717",
          "comment": "",
          "configuration": null,
          "id": 1463506596775213702,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2867928428650937962,
      "inTransitions": [
        3103663386312740882
      ],
      "name": "TPToPickSpot",
      "outTransitions": [
        3201336270766679779
      ],
      "parentPlan": 725594143882346503,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Drop.beh#3009473645416620380",
          "comment": "",
          "configuration": null,
          "id": 3177891931432741196,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 624744054901478287,
      "inTransitions": [
        1537785163841820841
      ],
      "name": "Drop",
      "outTransitions": [
        2630758425967053453
      ],
      "parentPlan": 725594143882346503,
      "positionWeb": {
        "x": 1460,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/NavigateToDrop.beh#4459885370764933844",
          "comment": "",
          "configuration": null,
          "id": 2630222039531635439,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 726015981546724416,
      "inTransitions": [
        1425389364493331507
      ],
      "name": "TPToDropSpot",
      "outTransitions": [
        1537785163841820841
      ],
      "parentPlan": 725594143882346503,
      "positionWeb": {
        "x": 1202,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 3103663386312740882,
      "inState": 3785266111914580157,
      "name": "",
      "outState": 2867928428650937962,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1971173312201839855,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1537785163841820841,
      "inState": 726015981546724416,
      "name": "",
      "outState": 624744054901478287,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3691801807787093963,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 2630758425967053453,
      "inState": 624744054901478287,
      "name": "",
      "outState": 3464891834530950837,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2187308102082241829,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1425389364493331507,
      "inState": 1965586362363306757,
      "name": "",
      "outState": 726015981546724416,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3953991713597643491,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 3201336270766679779,
      "inState": 2867928428650937962,
      "name": "",
      "outState": 1965586362363306757,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 32970225314063392,
        "name": "",
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