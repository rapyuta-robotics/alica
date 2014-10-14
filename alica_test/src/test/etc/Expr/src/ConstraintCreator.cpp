using namespace std;

#include "ConstraintCreator.h"
#include <iostream>

#include  "PlanThree1407153663917Constraints.h"

#include  "GoalPlan1402488870347Constraints.h"

#include  "SimpleTestPlan1412252439925Constraints.h"

#include  "PlanOne1407153611768Constraints.h"

#include  "AttackPlan1402488634525Constraints.h"

#include  "PlanTwo1407153645238Constraints.h"

#include  "MasterPlanTaskAssignment1407152758497Constraints.h"

#include  "PlanFive1407153703092Constraints.h"

#include  "Tackle1402489318663Constraints.h"

#include  "PlanFour1407153683051Constraints.h"

#include  "MidFieldPlayPlan1402488770050Constraints.h"

#include  "Defend1402488893641Constraints.h"

#include  "MasterPlan1402488437260Constraints.h"

namespace alica {

ConstraintCreator::ConstraintCreator() {
}

ConstraintCreator::ConstraintCreator() {
}

shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(
    long constraintConfId) {
  switch (constraintConfId) {

    case 1402489131988
    return make_shared<Constraint1402489131988>();
    break;

    case 1403773741874
    return make_shared<Constraint1403773741874>();
    break;

    case 1402489174338
    return make_shared<Constraint1402489174338>();
    break;

    case 1402489206278
    return make_shared<Constraint1402489206278>();
    break;

    case 1402489218027
    return make_shared<Constraint1402489218027>();
    break;

    case 1412781693884
    return make_shared<Constraint1412781693884>();
    break;

    case 1412781707952
    return make_shared<Constraint1412781707952>();
    break;

    case 1412761926856
    return make_shared<Constraint1412761926856>();
    break;

    case 1402489460549
    return make_shared<Constraint1402489460549>();
    break;

    case 1402489462088
    return make_shared<Constraint1402489462088>();
    break;

    case 1402489260911
    return make_shared<Constraint1402489260911>();
    break;

    case 1402489258509
    return make_shared<Constraint1402489258509>();
    break;

    case 1402489278408
    return make_shared<Constraint1402489278408>();
    break;

    case 1402500844446
    return make_shared<Constraint1402500844446>();
    break;

    case 1402488991641
    return make_shared<Constraint1402488991641>();
    break;

    case 1402488993122
    return make_shared<Constraint1402488993122>();
    break;

    case 1402489065962
    return make_shared<Constraint1402489065962>();
    break;

    case 1402489073613
    return make_shared<Constraint1402489073613>();
    break;

    case 1402488519140
    return make_shared<Constraint1402488519140>();
    break;

    case 1402488520968
    return make_shared<Constraint1402488520968>();
    break;

    case 1402488558741
    return make_shared<Constraint1402488558741>();
    break;

    case 1409218319990
    return make_shared<Constraint1409218319990>();
    break;

  default:
    cerr << "ConstraintCreator: Unknown constraint requested: "
        << constraintConfId << endl;
    throw new exception();
    break;
  }
}

}
