{
  "blackboard": [
    {
      "access": "input",
      "comment": "",
      "id": 226020182788822510,
      "key": "inputInt",
      "type": "int64"
    },
    {
      "access": "input",
      "comment": "",
      "id": 2251781496563135384,
      "key": "inputUint",
      "type": "uint64"
    },
    {
      "access": "input",
      "comment": "",
      "id": 2479552827579794359,
      "key": "inputBool",
      "type": "bool"
    },
    {
      "access": "input",
      "comment": "",
      "id": 3215626887588980818,
      "key": "inputString",
      "type": "std::string"
    },
    {
      "access": "input",
      "comment": "",
      "id": 3742096257643612656,
      "key": "inputDouble",
      "type": "double"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1207515621856042037,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4191036869656162042,
      "positionWeb": {
        "x": 304,
        "y": 352
      },
      "state": 2198736029264610559,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 4191036869656162042,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "ValueMappingPlansPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1412629153498867728,
      "inTransitions": [
        593692066986245834
      ],
      "name": "ValueMappingPlansSuccessState",
      "outTransitions": [],
      "parentPlan": 4191036869656162042,
      "positionWeb": {
        "x": 1121,
        "y": 382
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1207515621856042037,
      "id": 2198736029264610559,
      "inTransitions": [],
      "name": "ValueMappingPlansState",
      "outTransitions": [
        593692066986245834
      ],
      "parentPlan": 4191036869656162042,
      "positionWeb": {
        "x": 669,
        "y": 345
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2756315630654213874",
      "id": 593692066986245834,
      "inState": 2198736029264610559,
      "keyMapping": {
        "input": [
          {
            "childKey": "inputString",
            "parentKey": "inputString",
            "value": null
          },
          {
            "childKey": "inputBool",
            "parentKey": "inputBool",
            "value": null
          },
          {
            "childKey": "inputInt",
            "parentKey": "inputInt",
            "value": null
          },
          {
            "childKey": "inputDouble",
            "parentKey": "inputDouble",
            "value": null
          },
          {
            "childKey": "inputUint",
            "parentKey": "inputUint",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1412629153498867728,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2713623733097135933,
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
