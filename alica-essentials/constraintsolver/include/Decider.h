/*
 * Decider.h
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#ifndef DECIDER_H_
#define DECIDER_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica {
namespace reasoner {
namespace cnsat {
class CNSat;
class Lit;
class Var;

class Decider {
public:
    static shared_ptr<Var> decideRangeBased(shared_ptr<vector<shared_ptr<Var>>> variables, shared_ptr<CNSat> solver);
    static shared_ptr<Var> decideActivityBased(shared_ptr<vector<shared_ptr<Var>>> variables, shared_ptr<CNSat> solver);
    static shared_ptr<Var> decideVariableCountBased(
            shared_ptr<vector<shared_ptr<Var>>> variables, shared_ptr<CNSat> solver);
    static bool litRangeCompare(shared_ptr<Lit> a, shared_ptr<Lit> b);
};

}  // namespace cnsat
/* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */

#endif /* DECIDER_H_ */
