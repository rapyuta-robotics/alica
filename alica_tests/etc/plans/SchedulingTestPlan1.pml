{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 683514589549348538,
      "key": "Plan2Sub",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1571743223454861605,
      "key": "Init2Term",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1613378541158,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1613378541158",
      "plan": 1613378406860,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1613378543512,
      "successRequired": false,
      "task": "taskrepository.tsk#1613372009777"
    }
  ],
  "frequency": 0,
  "id": 1613378406860,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SchedulingTestPlan1",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1613378541158,
      "id": 1613378543512,
      "inTransitions": [],
      "name": "InitPlan1",
      "outTransitions": [
        1614960055819
      ],
      "parentPlan": 1613378406860,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1613977406218,
      "inTransitions": [
        1614960063842
      ],
      "name": "TerminateSubPlans",
      "outTransitions": [],
      "parentPlan": 1613378406860,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SchedulingTestPlan2.pml#1613378423610",
          "comment": "",
          "configuration": null,
          "id": 1614960077329,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1614960077329"
        },
        {
          "abstractPlan": "SchedulingTestPlan3.pml#1613378433623",
          "comment": "",
          "configuration": null,
          "id": 1614960079819,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1614960079819"
        }
      ],
      "entryPoint": null,
      "id": 1614960038398,
      "inTransitions": [
        1614960055819
      ],
      "name": "InitSubPlans",
      "outTransitions": [
        1614960063842
      ],
      "parentPlan": 1613378406860,
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
      "condition": "ConditionRepository.cnd#2901825906319407673",
      "id": 1614960055819,
      "inState": 1613378543512,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": "Plan2Sub",
            "value": null
          }
        ],
        "output": []
      },
      "name": "FromDefault NameTo Default Name",
      "outState": 1614960038398,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1614960055821,
        "name": "1614960055821",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#2901825906319407673",
      "id": 1614960063842,
      "inState": 1614960038398,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": "Init2Term",
            "value": null
          }
        ],
        "output": []
      },
      "name": "FromDefault NameTo Default Name",
      "outState": 1613977406218,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1614960063843,
        "name": "1614960063843",
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
