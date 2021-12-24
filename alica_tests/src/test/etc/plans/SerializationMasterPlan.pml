{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 125037907796569874,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "SerializationEP",
      "plan": 373109241446504968,
      "positionWeb": {
        "x": 200,
        "y": 812
      },
      "state": 1886795261620096590,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 373109241446504968,
  "masterPlan": true,
  "name": "SerializationMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SerializationSubPlanB.pml#230205985761632608",
          "comment": "",
          "configuration": null,
          "id": 4037048436490152345,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 174185769149002104,
      "inTransitions": [
        2606234571502403331
      ],
      "name": "PlanB",
      "outTransitions": [],
      "parentPlan": 373109241446504968,
      "positionWeb": {
        "x": 736.8072353921411,
        "y": 632.5646401118099
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SerializationSubPlanC.pml#2359124678252958039",
          "comment": "",
          "configuration": null,
          "id": 977691147032446781,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 458399185905834888,
      "inTransitions": [
        3214980101932259560
      ],
      "name": "PlanC",
      "outTransitions": [],
      "parentPlan": 373109241446504968,
      "positionWeb": {
        "x": 713.585174434231,
        "y": 835.7576734935225
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SerializationSubPlanD.pml#1781630225028158279",
          "comment": "",
          "configuration": null,
          "id": 162647391651889891,
          "name": ""
        },
        {
          "abstractPlan": "EmptyPlan.pml#984284423749038756",
          "comment": "",
          "configuration": null,
          "id": 1984835195334359468,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 837657643540052235,
      "inTransitions": [
        1491726255888784762
      ],
      "name": "PlanD",
      "outTransitions": [],
      "parentPlan": 373109241446504968,
      "positionWeb": {
        "x": 686,
        "y": 1000
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 125037907796569874,
      "id": 1886795261620096590,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1491726255888784762,
        2057783493960201520,
        2606234571502403331,
        3214980101932259560
      ],
      "parentPlan": 373109241446504968,
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
          "abstractPlan": "SerializationSubPlanA.pml#1433931143598606082",
          "comment": "",
          "configuration": null,
          "id": 747074121811963922,
          "name": ""
        },
        {
          "abstractPlan": "EmptyPlan.pml#984284423749038756",
          "comment": "",
          "configuration": null,
          "id": 2012988928861934803,
          "name": ""
        },
        {
          "abstractPlan": "EmptyPlan.pml#984284423749038756",
          "comment": "",
          "configuration": null,
          "id": 3779615517995020859,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4556827380180239242,
      "inTransitions": [
        2057783493960201520
      ],
      "name": "PlanA",
      "outTransitions": [],
      "parentPlan": 373109241446504968,
      "positionWeb": {
        "x": 686,
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
      "id": 1491726255888784762,
      "inState": 1886795261620096590,
      "name": "",
      "outState": 837657643540052235,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1693256954385338259,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 2057783493960201520,
      "inState": 1886795261620096590,
      "name": "",
      "outState": 4556827380180239242,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3932287302905544988,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 2606234571502403331,
      "inState": 1886795261620096590,
      "name": "",
      "outState": 174185769149002104,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3461968191733792853,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 3214980101932259560,
      "inState": 1886795261620096590,
      "name": "",
      "outState": 458399185905834888,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2915681556800498724,
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