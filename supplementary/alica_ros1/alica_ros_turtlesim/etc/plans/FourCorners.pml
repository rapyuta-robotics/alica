{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 599427197773116147,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 0,
      "name": "",
      "plan": 1225570798912217901,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 3162295592305824185,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    },
    {
      "comment": "",
      "id": 919409193351805481,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 0,
      "name": "",
      "plan": 1225570798912217901,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 3319164559788507935,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    },
    {
      "comment": "",
      "id": 1002169119911528732,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 0,
      "name": "",
      "plan": 1225570798912217901,
      "positionWeb": {
        "x": 200,
        "y": 812
      },
      "state": 3102339349066193934,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    },
    {
      "comment": "",
      "id": 3975697190021470017,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 0,
      "name": "",
      "plan": 1225570798912217901,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 3797889270889861969,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 1225570798912217901,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "FourCorners",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 331991001677741800,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 1.5
              },
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 8.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1002169119911528732,
      "id": 3102339349066193934,
      "inTransitions": [],
      "name": "BottomRight",
      "outTransitions": [],
      "parentPlan": 1225570798912217901,
      "positionWeb": {
        "x": 428,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 3010029469400064215,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 1.5
              },
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 1.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 599427197773116147,
      "id": 3162295592305824185,
      "inTransitions": [],
      "name": "BottomLeft",
      "outTransitions": [],
      "parentPlan": 1225570798912217901,
      "positionWeb": {
        "x": 428,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 1462104461789462222,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 1.5
              },
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 8.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 919409193351805481,
      "id": 3319164559788507935,
      "inTransitions": [],
      "name": "TopLeft",
      "outTransitions": [],
      "parentPlan": 1225570798912217901,
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
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 259482488220618618,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 8.5
              },
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 8.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 3975697190021470017,
      "id": 3797889270889861969,
      "inTransitions": [],
      "name": "TopRight",
      "outTransitions": [],
      "parentPlan": 1225570798912217901,
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
