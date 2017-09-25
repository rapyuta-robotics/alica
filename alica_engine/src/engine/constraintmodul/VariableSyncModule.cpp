//#define RS_DEBUG
#include "engine/constraintmodul/VariableSyncModule.h"

#include "SystemConfig.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/ITeamManager.h"
#include "engine/ITeamObserver.h"
#include "engine/constraintmodul/ResultEntry.h"
#include "engine/containers/SolverResult.h"
#include "engine/model/Variable.h"
#include <algorithm>
#include <cmath>
#include <math.h>

namespace alica
{
VariableSyncModule::VariableSyncModule(AlicaEngine *ae)
    : ae(ae)
    , running(false)
    , timer(nullptr)
	, distThreshold(0)
	, communicator(nullptr)
	, ttl4Communication(0)
	, ttl4Usage(0)
	, communicationEnabled(false)
{
}

VariableSyncModule::~VariableSyncModule()
{
    delete timer;
}

void VariableSyncModule::init()
{
    if (running)
    {
        return;
    }
    running = true;
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    communicationEnabled = (*sc)["Alica"]->get<bool>("Alica", "CSPSolving", "EnableCommunication", NULL);
    ttl4Communication = 1000000 * (*sc)["Alica"]->get<long>("Alica", "CSPSolving", "SeedTTL4Communication", NULL);
    ttl4Usage = 1000000 * (*sc)["Alica"]->get<long>("Alica", "CSPSolving", "SeedTTL4Usage", NULL);
    ownId = ae->getTeamManager()->getLocalAgentID();
    ownResults = make_shared<ResultEntry>(ownId, ae);
    store.push_back(ownResults);
    distThreshold = (*sc)["Alica"]->get<double>("Alica", "CSPSolving", "SeedMergingThreshold", NULL);
    if (communicationEnabled)
    {
        communicator = ae->getCommunicator();
        int interval =
            (int)round(1000.0 / (*sc)["Alica"]->get<double>("Alica", "CSPSolving", "CommunicationFrequency", NULL));
        timer = new supplementary::NotifyTimer<VariableSyncModule>(interval, &VariableSyncModule::publishContent, this);
        timer->start();
    }
}

void VariableSyncModule::close()
{
    this->running = false;
    if (timer)
    {
        timer->stop();
    }
    timer = nullptr;
}

void VariableSyncModule::clear()
{
    for (auto r : store)
    {
        r->clear();
    }
}

void VariableSyncModule::onSolverResult(shared_ptr<SolverResult> msg)
{
    if (msg->senderID == ownId)
    {
        return;
    }
    if (ae->getTeamManager()->isAgentIgnored(msg->senderID))
    {
        return;
    }
    bool found = false;
    shared_ptr<ResultEntry> re = nullptr;
    for (int i = 0; i < this->store.size(); ++i)
    {
        re = store[i];
        if (re->getId() == msg->senderID)
        {
            found = true;
            break;
        }
    }
    // Lockguard here!
    if (!found)
    {
        re = make_shared<ResultEntry>(msg->senderID, ae);
        this->store.push_back(re);
    }
    for (auto sv : msg->vars)
    {
        shared_ptr<vector<uint8_t>> tmp = make_shared<vector<uint8_t>>(sv->value);
        re->addValue(sv->id, tmp);
    }
}

void VariableSyncModule::publishContent()
{
    if (!this->running)
        return;
    if (!ae->isMaySendMessages())
        return;
    shared_ptr<vector<SolverVar *>> lv = ownResults->getCommunicatableResults(ttl4Communication);
    if (lv->size() == 0)
        return;
    SolverResult sr;
    sr.senderID = ownId;
    sr.vars = *lv;
    communicator->sendSolverResult(sr);
}

void VariableSyncModule::postResult(long vid, shared_ptr<vector<uint8_t>> &result)
{
    this->ownResults->addValue(vid, result);
}

shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>>
VariableSyncModule::getSeeds(shared_ptr<vector<Variable *>> query,
                             shared_ptr<vector<shared_ptr<vector<double>>>> limits)
{
    // Lockguard here!

    int dim = query->size();

    /*cout << "VSyncMod:";
    for(auto& avar : *query) {
            cout << " " << avar->getId();
    }
    cout << endl;*/

    list<shared_ptr<VotedSeed>> seeds;
    vector<double> scaling(dim);
    for (int i = 0; i < dim; i++)
    {
        scaling[i] = (limits->at(i)->at(1) - limits->at(i)->at(0));
        scaling[i] *= scaling[i]; // Sqr it for dist calculation speed up
    }
    //		cout << "VSM: Number of Seeds in Store: " << this->store.size() << endl;
    for (int i = 0; i < this->store.size(); i++)
    {
        shared_ptr<ResultEntry> re = this->store.at(i); // allow for lock free iteration (no value is deleted from
                                                        // store)
        shared_ptr<vector<shared_ptr<vector<uint8_t>>>> vec = re->getValues(query, this->ttl4Usage);
        if (vec == nullptr)
        {
            continue;
        }
        bool found = false;
        for (auto s : seeds)
        {
            if (s->takeVector(vec, scaling, distThreshold, true))
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            seeds.push_back(make_shared<VotedSeed>(dim, vec));
        }
    }
#ifdef RS_DEBUG
    cout << "RS: Generated " << seeds.size() << "seeds" << endl;
    for (int i = 0; i < seeds.size(); i++)
    {
        cout << "Seed " << i; // (sup:{1}): ",i);
        int i = 0;
        for (auto j = seeds.begin(); j != seeds.end(); j++, i++)
        {
            cout << (*j)->values.at(i) << "\t";
        }
        cout << endl;
    }

#endif

    int maxNum = min((int)seeds.size(), dim);
    auto ret = make_shared<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>>(maxNum);

    seeds.sort([](shared_ptr<VotedSeed> &a, shared_ptr<VotedSeed> &b) {
        if (a->totalSupCount != b->totalSupCount)
        {
            return a->totalSupCount > b->totalSupCount;
        }
        else
        {
            if (a->values == nullptr || a->values->size() == 0)
                return true;
            if (b->values == nullptr || b->values->size() == 0)
                return false;

            return a->hash > b->hash;
            // return &*a > &*b;
        }
    });

    auto iter = seeds.begin();
    for (int i = 0; i < maxNum; i++)
    {
        ret->at(i) = (*iter)->values;
        iter++;
    }
    //		cout << "VSM: Number of present seeds: " << ret->size() << " dim: "<< dim << " seedcount: "<< seeds.size() <<
    //endl;

    return ret;
}

VariableSyncModule::VotedSeed::VotedSeed(int dim, shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v)
{
    this->values = v;
    this->supporterCount = vector<int>(dim);
    this->dim = dim;
    hash = 0;
    if (v != nullptr)
    {
        for (int i = 0; i < dim; ++i)
        {
            if (v->at(i) == nullptr)
            {
                continue;
            }
            if (v->at(i)->size() > 0)
            {
                this->totalSupCount++;
            }
        }
    }
}

shared_ptr<vector<double>>
VariableSyncModule::VotedSeed::deserializeToDoubleVec(shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v)
{
    shared_ptr<vector<double>> singleseed = make_shared<vector<double>>();
    singleseed->reserve(v->size());
    for (auto &serialvalue : *v)
    {
        if (serialvalue != nullptr)
        {
            double v;
            uint8_t *pointer = (uint8_t *)&v;
            if (serialvalue->size() == sizeof(double))
            {
                for (int k = 0; k < sizeof(double); k++)
                {
                    *pointer = serialvalue->at(k);
                    pointer++;
                }
                singleseed->push_back(v);
            }
            else
            {
                cerr << "VSM: Received Seed that is not size of double" << endl;
                cout << "VSM: Received Seed that is not size of double" << endl;
                break;
            }
        }
        else
        {
            singleseed->push_back(std::numeric_limits<double>::max());
        }
    }

    return singleseed;
}
shared_ptr<vector<shared_ptr<vector<uint8_t>>>>
VariableSyncModule::VotedSeed::serializeFromDoubleVec(shared_ptr<vector<double>> d)
{

    shared_ptr<vector<shared_ptr<vector<uint8_t>>>> res = make_shared<vector<shared_ptr<vector<uint8_t>>>>();

    for (int i = 0; i < d->size(); i++)
    {

        uint8_t *tmp = ((uint8_t *)&d->at(i));
        shared_ptr<vector<uint8_t>> result = make_shared<vector<uint8_t>>(sizeof(double));
        for (int s = 0; s < sizeof(double); s++)
        {
            result->at(s) = *tmp;
            tmp++;
        }
        res->push_back(result);
    }

    return res;
}
bool VariableSyncModule::VotedSeed::takeVector(shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v,
                                               vector<double> &scaling, double distThreshold, bool isDouble)
{
    //		if(!isDouble) {
    //
    //			int nans = 0;
    //			double distSqr = 0;
    //
    //			bool nan = true;
    //			bool sameRes = true;
    //			bool nptrs = true;
    //
    //			if(v == nullptr || values == nullptr) {
    //				return true;
    //			}
    //			if(values->size() != v->size()) {
    //				return false;
    //			}
    //
    //			for(int i = 0; i  < dim; ++i) {
    //				if(v->at(i) != nullptr) {
    //					nptrs = false;
    //				}
    //				else {
    //					continue;
    //				}
    //				if(values->at(i) == nullptr) continue;
    //
    //				if(values->at(i)->size() == v->at(i)->size()) {
    //					for(int j = 0; j < v->at(i)->size(); j++) {
    //						nan = false;
    //						if(values->at(i)->at(j) != v->at(i)->at(j)) {
    //							sameRes = false;
    //						}
    //					}
    //				} else {
    //					return true;
    //				}
    //			}
    //
    //			if(nan || sameRes || nptrs) {
    //	//			cout << "VSM: takeVector true."  << nan << sameRes << nptrs << endl;
    //				if(sameRes) this->totalSupCount++;
    //				return true;
    //			} else {
    //	//			cout << "VSM: takeVector false." << endl;
    //				return false;
    //			}
    //		} else {
    shared_ptr<vector<double>> v1 = deserializeToDoubleVec(v);
    shared_ptr<vector<double>> values1 = deserializeToDoubleVec(values);
    int nans = 0;
    double distSqr = 0;
    for (int i = 0; i < dim; ++i)
    {

        if (!std::isnan(v1->at(i)) && v1->at(i) != std::numeric_limits<double>::max())
        {
            if (!std::isnan(values1->at(i)))
            {
                if (scaling[i] != 0)
                {
                    distSqr += ((values1->at(i) - v1->at(i)) * (values1->at(i) - v1->at(i))) / scaling[i];
                }
                else
                {
                    distSqr += (values1->at(i) - v1->at(i)) * (values1->at(i) - v1->at(i));
                }
            }
        }
        else
        {
            nans++;
        }
    }
    if (dim == nans)
    {
        hash = -1;
        return true; // silently absorb a complete NaN vector
    }
    if (distSqr / (dim - nans) < distThreshold)
    {
        for (int i = 0; i < dim; ++i)
        {
            if (!std::isnan(v1->at(i)))
            {
                if (std::isnan(values1->at(i)))
                {
                    this->supporterCount[i] = 1;
                    this->totalSupCount++;

                    values1->at(i) = v1->at(i);
                }
                else
                {
                    values1->at(i) = values1->at(i) * this->supporterCount[i] + v1->at(i);
                    this->supporterCount[i]++;
                    this->totalSupCount++;
                    values1->at(i) /= this->supporterCount[i];
                }
            }
        }

        this->values = serializeFromDoubleVec(values1);

        hash = 0;
        for (double d : *values1)
        {
            if (!std::isnan(d) && d != std::numeric_limits<double>::max())
                hash += d;
        }
        return true;
    }
    hash = 0;
    for (double d : *values1)
    {
        if (!std::isnan(d) && d != std::numeric_limits<double>::max())
            hash += d;
    }
    return false;

    //		}

    //		for (int i = 0; i < dim; ++i)
    //		{
    //
    //			if (!std::isnan(v->at(i)))
    //			{
    //				if (!std::isnan(values->at(i)))
    //				{
    //					if (scaling[i] != 0)
    //					{
    //						distSqr += ((values->at(i) - v->at(i)) * (values->at(i) - v->at(i))) /
    //scaling[i];
    //					}
    //					else
    //					{
    //						distSqr += (values->at(i) - v->at(i)) * (values->at(i) - v->at(i));
    //					}
    //				}
    //			}
    //			else
    //			{
    //				nans++;
    //			}
    //		}
    //		if (dim == nans)
    //		{
    //			return true; //silently absorb a complete NaN vector
    //		}
    //		if (distSqr / (dim - nans) < distThreshold)
    //		{
    //			for (int i = 0; i < dim; ++i)
    //			{
    //				if (!std::isnan(v->at(i)))
    //				{
    //					if (std::isnan(values->at(i)))
    //					{
    //						this->supporterCount[i] = 1;
    //						this->totalSupCount++;
    //						this->values->at(i) = v->at(i);
    //					}
    //					else
    //					{
    //						this->values->at(i) = values->at(i) * this->supporterCount[i] +
    //v->at(i);
    //						this->supporterCount[i]++;
    //						this->totalSupCount++;
    //						this->values->at(i) /= this->supporterCount[i];
    //					}
    //				}
    //			}
    //			return true;
    //		}
    //		return false;
}
} /* namespace alica */
