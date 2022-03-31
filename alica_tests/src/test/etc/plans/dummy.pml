{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402488437263,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402488437260,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1402488437261,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402488437260,
  "masterPlan": true,
  "name": "MasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662710,
          "name": "1587718662710"
        }
      ],
      "entryPoint": 1402488437263,
      "id": 1402488437261,
      "inTransitions": [],
      "name": "Attack",
      "outTransitions": [
        1402488517667,
        1409218318661
      ],
      "parentPlan": 1402488437260,
      "positionWeb": {
        "x": 428,
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
      "id": 1402488517667,
      "inState": 1402488437261,
      "name": "AttackToGoal",
      "outState": 1402488477650,
      "pointsWeb": [],
	  "condition": "conditions/onSuccess.cond#1532424188199",
      "conditionRelation": {
            "input" : [
              {
                "parentKey" : "masterKey",
                "childKey" : "planInputFromMaster"
              },
              {
                "parentKey" : "nonExistingKey",
                "childKey" : "planInputFromMaster"
              },
              {
                "parentKey" : "masterKey",
                "childKey" : "nonExistingKey"
              }
            ],
            "output": []
          },
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402488519140,
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
