{
  "blackboard": [
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 54989083355939699,
      "key": "mappedBoolValue",
      "type": "bool"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 1234313196608957375,
      "key": "mappedDoubleValue",
      "type": "double"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 1535353035896902577,
      "key": "mappedIntValue",
      "type": "int64"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 2933912270549461002,
      "key": "mappedUintValue",
      "type": "uint64"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 4434332895838233306,
      "key": "mappedStringValue",
      "type": "std::string"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 953921531708941324,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3938799313883693450,
      "positionWeb": {
        "x": 213,
        "y": 353
      },
      "state": 2157045678929644274,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3938799313883693450,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "ValueMappingPlanTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1483412031843812599,
      "inTransitions": [
        3424784027493766968
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3938799313883693450,
      "positionWeb": {
        "x": 710.9755368364055,
        "y": 385.3326214187521
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 953921531708941324,
      "id": 2157045678929644274,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        3424784027493766968
      ],
      "parentPlan": 3938799313883693450,
      "positionWeb": {
        "x": 446.152015679996,
        "y": 368.9127210608825
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2872265442510628524",
      "id": 3424784027493766968,
      "inState": 2157045678929644274,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1483412031843812599,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2746271972957018702,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
