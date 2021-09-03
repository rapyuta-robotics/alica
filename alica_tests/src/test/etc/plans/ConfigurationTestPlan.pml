{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1588061024407,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1588061024407",
      "plan": 1588060981661,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1588060991102,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1588060981661,
  "masterPlan": true,
  "name": "ConfigurationTestPlan",
  "preCondition": null,
  "relativeDirectory": "Configurations",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/ReadConfigurationBehaviour.beh#1588061129360",
          "comment": "",
          "configuration": "configurations/ConfigA.cfg#1588061188681",
          "id": 1588061141112,
          "name": "1588061141112"
        },
        {
          "abstractPlan": "behaviours/ReadConfigurationBehaviour.beh#1588061129360",
          "comment": "",
          "configuration": "configurations/ConfigB.cfg#1588061200689",
          "id": 1588061144175,
          "name": "1588061144175"
        },
        {
          "abstractPlan": "ReadConfigurationPlantype.pty#1588061351007",
          "comment": "",
          "configuration": "configurations/ConfigA.cfg#1588061188681",
          "id": 1588246105794,
          "name": "1588246105794"
        },
        {
          "abstractPlan": "ReadConfigurationPlan.pml#1588061334567",
          "comment": "",
          "configuration": "configurations/ConfigA.cfg#1588061188681",
          "id": 1588253325052,
          "name": "1588253325052"
        }
      ],
      "entryPoint": 1588061024407,
      "id": 1588060991102,
      "inTransitions": [],
      "name": "Default Name",
      "outTransitions": [
        1588253347211
      ],
      "parentPlan": 1588060981661,
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
          "abstractPlan": "ReadConfigurationPlantype.pty#1588061351007",
          "comment": "",
          "configuration": "configurations/ConfigB.cfg#1588061200689",
          "id": 1588253360557,
          "name": "1588253360557"
        },
        {
          "abstractPlan": "ReadConfigurationPlan.pml#1588061334567",
          "comment": "",
          "configuration": "configurations/ConfigB.cfg#1588061200689",
          "id": 1588253370222,
          "name": "1588253370222"
        }
      ],
      "entryPoint": null,
      "id": 1588253341545,
      "inTransitions": [
        1588253347211
      ],
      "name": "Default Name",
      "outTransitions": [],
      "parentPlan": 1588060981661,
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
      "comment": "MISSING_COMMENT",
      "id": 1588253347211,
      "inState": 1588060991102,
      "name": "FromDefault NameTo Default Name",
      "outState": 1588253341545,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1588253347213,
        "name": "1588253347213",
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