{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1579432417749536498,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3497513845787611552,
      "positionWeb": {
        "x": 300,
        "y": 286.5625
      },
      "state": 3948898364502694199,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3497513845787611552,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "OrderedRunTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 585671864125157712,
      "inTransitions": [
        2051663941196992083,
        3287673972734121286
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3497513845787611552,
      "positionWeb": {
        "x": 913,
        "y": 633.5625
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
          "abstractPlan": "PlanB.pml#1629895853508",
          "comment": "",
          "configuration": null,
          "id": 3277687245726366174,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1281215768152897620,
      "inTransitions": [
        3387122205432919824
      ],
      "name": "PlanBState",
      "outTransitions": [
        1326164621067032711,
        2051663941196992083,
        4354493579586106089
      ],
      "parentPlan": 3497513845787611552,
      "positionWeb": {
        "x": 887,
        "y": 308.5625
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2916901529830054921,
      "inTransitions": [
        2669653169162188065,
        4354493579586106089
      ],
      "name": "FailureState",
      "outTransitions": [],
      "parentPlan": 3497513845787611552,
      "positionWeb": {
        "x": 470,
        "y": 614.5625
      },
      "postCondition": null,
      "success": false,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 3909269512640028193,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1579432417749536498,
      "id": 3948898364502694199,
      "inTransitions": [
        1326164621067032711
      ],
      "name": "PlanAState",
      "outTransitions": [
        2669653169162188065,
        3287673972734121286,
        3387122205432919824
      ],
      "parentPlan": 3497513845787611552,
      "positionWeb": {
        "x": 531,
        "y": 297.5625
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": null,
      "id": 1326164621067032711,
      "inState": 1281215768152897620,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3948898364502694199,
      "pointsWeb": [
        {
          "x": 734,
          "y": 224.5625
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1277118090350391970,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 2051663941196992083,
      "inState": 1281215768152897620,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 585671864125157712,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 321396616973492295,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 2669653169162188065,
      "inState": 3948898364502694199,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2916901529830054921,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2665752143146828541,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 3287673972734121286,
      "inState": 3948898364502694199,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 585671864125157712,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2594642312535232960,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 3387122205432919824,
      "inState": 3948898364502694199,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1281215768152897620,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3707942615412201464,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 4354493579586106089,
      "inState": 1281215768152897620,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2916901529830054921,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 757341098304194972,
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
