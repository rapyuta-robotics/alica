{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": "0",
      "id": 3717282134545268689,
      "key": "runTimeConditionCounter",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3387018301065137266,
      "key": "targetChildStatus",
      "type": "std::any"
    }
  ],
  "comment": "Run behaviour and move from states",
  "entryPoints": [
    {
      "comment": "",
      "id": 2768515147950934595,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2504351804499332310,
      "positionWeb": {
        "x": 200,
        "y": 205.505792074091
      },
      "state": 1006522403402265538,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2504351804499332310,
  "inheritBlackboard": false,
  "masterPlan": false,
  "name": "RunBehaviourInSimplePlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 4404788800584486714,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/State2Behaviour.beh#2219945377054027027",
          "comment": "",
          "configuration": null,
          "id": 3644364485282695789,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 681938134974956178,
      "inTransitions": [
        1055311237496664394
      ],
      "name": "State2",
      "outTransitions": [],
      "parentPlan": 2504351804499332310,
      "positionWeb": {
        "x": 713.2522669792288,
        "y": 209.79196244397045
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/State1Behaviour.beh#3563417394101512880",
          "comment": "",
          "configuration": null,
          "id": 2280007875848684191,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2768515147950934595,
      "id": 1006522403402265538,
      "inTransitions": [],
      "name": "State1",
      "outTransitions": [
        1055311237496664394
      ],
      "parentPlan": 2504351804499332310,
      "positionWeb": {
        "x": 370.6651523868165,
        "y": -4.1643653748850795
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#843443485857038179",
      "id": 1055311237496664394,
      "inState": 1006522403402265538,
      "keyMapping": {
        "input": [
          {
            "childKey": "childStatus",
            "parentKey": "targetChildStatus"
          }
        ],
        "output": []
      },
      "name": "1055311237496664394",
      "outState": 681938134974956178,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 122747038863060590,
        "name": "122747038863060590",
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
