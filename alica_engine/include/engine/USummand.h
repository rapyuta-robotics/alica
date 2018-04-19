/*
 * USummand.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#ifndef USUMMAND_H_
#define USUMMAND_H_

#include <vector>
#include <string>
#include <sstream>
#include <map>

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/UtilityInterval.h"
#include "engine/model/EntryPoint.h"
#include "engine/Types.h"

namespace alica {

class EntryPoint;
class IAssignment;

/**
 * Abstract super class for domain dependent utility summands.
 */
class USummand {
public:
    USummand()
            : ui(0.0, 0.0) {
        this->id = 0;
        this->weight = 0;
    }
    virtual ~USummand() {}
    /**
     * Searches every needed entrypoint in the hashtable of the xmlparser
     * and stores it in the relevant entrypoint vector. This will increase the
     * performance of the evaluation of this utility summand.
     */
    virtual void init(AlicaEngine* ae) {
        // init relevant entrypoint vector
        this->relevantEntryPoints.resize(this->relevantEntryPointIds.size());
        // find the right entrypoint for each id in relevant entrypoint id
        for (int i = 0; i < static_cast<int>(this->relevantEntryPoints.size()); ++i) {
            const EntryPoint* curEp = ae->getPlanRepository()->getEntryPoints().find(this->relevantEntryPointIds[i]);
            if (curEp != nullptr) {
                this->relevantEntryPoints[i] = curEp;
            } else {
                cerr << "Could not find Entrypoint " << this->relevantEntryPointIds[i] << " Hint is: " << this->name
                     << endl;
                throw new std::exception();
            }
        }
    }
    std::string toString() const {
        std::stringstream ss;
        ss << this->name << ": Weight " << this->weight << "EntryPoints: ";
        for (int i = 0; i < static_cast<int>(this->relevantEntryPointIds.size()); ++i) {
            ss << this->relevantEntryPointIds[i] << " ";
        }
        ss << std::endl;
        return ss.str();
    }
    double getWeight() const { return weight; }
    /**
     * Evaluates the utilityfunction summand
     * @return The result of the evaluation
     */
    virtual UtilityInterval eval(IAssignment* ass) = 0;
    /**
     * Cache every data for the current evaluation, to
     * assure consistency over the complete current evaluation.
     */
    virtual void cacheEvalData(){};
    virtual pair<vector<double>, double>* differentiate(IAssignment* newAss) { return nullptr; }
    void setWeight(double weight) { this->weight = weight; }

protected:
    UtilityInterval ui;
    double weight;
    int64_t id;
    std::vector<int64_t> relevantEntryPointIds;
    EntryPointSet relevantEntryPoints;

    std::string name;
    std::string info;
};

} /* namespace alica */

#endif /* USUMMAND_H_ */
