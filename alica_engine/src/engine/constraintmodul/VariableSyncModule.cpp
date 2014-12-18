/*
 * ResultStore.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

//#define RS_DEBUG
#include "engine/constraintmodul/VariableSyncModule.h"

#include "SystemConfig.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/ITeamObserver.h"
#include "engine/constraintmodul/ResultEntry.h"
#include "engine/containers/SolverResult.h"
#include <math.h>
#include <algorithm>

namespace alica
{
	VariableSyncModule::VariableSyncModule(AlicaEngine* ae)
	{
		running = false;
		this->ae = ae;
		this->timer = nullptr;
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
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		communicationEnabled = (*sc)["Alica"]->get<bool>("Alica", "CSPSolving", "EnableCommunication", NULL);
		ttl4Communication = 1000000 * (*sc)["Alica"]->get<long>("Alica", "CSPSolving", "SeedTTL4Communication", NULL);
		ttl4Usage = 1000000 * (*sc)["Alica"]->get<long>("Alica", "CSPSolving", "SeedTTL4Usage", NULL);
		ownId = ae->getTeamObserver()->getOwnId();
		ownResults = make_shared<ResultEntry>(ownId, ae);
		store.push_back(ownResults);
		distThreshold = (*sc)["Alica"]->get<double>("Alica", "CSPSolving", "SeedMergingThreshold", NULL);
		if (communicationEnabled)
		{
			communicator = ae->getCommunicator();
			int interval = (int)round(
					1000.0 / (*sc)["Alica"]->get<double>("Alica", "CSPSolving", "CommunicationFrequency", NULL));
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
		if (ae->getTeamObserver()->isRobotIgnored(msg->senderID))
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
		shared_ptr<vector<SolverVar*>> lv = ownResults->getCommunicatableResults(ttl4Communication);
		if (lv->size() == 0)
			return;
		SolverResult sr;
		sr.senderID = ownId;
		sr.vars = *lv;
		communicator->sendSolverResult(sr);
	}

	void VariableSyncModule::postResult(long vid, shared_ptr<vector<uint8_t>>& result)
	{
		this->ownResults->addValue(vid, result);
	}

	shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>> VariableSyncModule::getSeeds(shared_ptr<vector<Variable*>> query, shared_ptr<vector<shared_ptr<vector<double>>>> limits)
	{
		int dim = query->size();
		list<shared_ptr<VotedSeed>> seeds;
		vector<double> scaling(dim);
		for(int i=0; i<dim; i++)
		{
			scaling[i] = (limits->at(i)->at(1)-limits->at(i)->at(0));
			scaling[i] *= scaling[i]; //Sqr it for dist calculation speed up
		}
		for(int i=0; i<this->store.size(); i++)
		{
			shared_ptr<ResultEntry> re = this->store.at(i); //allow for lock free iteration (no value is deleted from store)
			shared_ptr<vector<shared_ptr<vector<uint8_t>>>> vec = re->getValues(query, this->ttl4Usage);
			bool found = false;
			for(auto s : seeds)
			{
				if(s->takeVector(vec,scaling,distThreshold))
				{
					found = true;
					break;
				}
			}
			if(!found)
			{
				seeds.push_back(make_shared<VotedSeed>(dim,vec));
			}
		}
#ifdef RS_DEBUG
		cout << "RS: Generated "<< seeds.size() << "seeds" << endl;
		for(int i=0; i<seeds.size(); i++)
		{
			cout << "Seed " << i; // (sup:{1}): ",i);
			int i=0;
			for(auto j=seeds.begin(); j!=seeds.end(); j++, i++)
			{
				cout << (*j)->values.at(i) << "\t";
			}
			cout << endl;
		}

#endif

		int maxNum = min((int)seeds.size(),dim);
		auto ret = make_shared<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>>(maxNum);
		seeds.sort([](shared_ptr<VotedSeed>& a, shared_ptr<VotedSeed>& b)
				{
					if(a->totalSupCount != b->totalSupCount)
					{
						return a->totalSupCount < b->totalSupCount;
					}
					else
					{
						return a<b;
					}
				});

		auto iter = seeds.begin();
		for(int i=0; i<maxNum;i++)
		{
			ret->at(i) = (*iter)->values;
			iter++;
		}

		return ret;
	}

	VariableSyncModule::VotedSeed::VotedSeed(int dim, shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v)
	{
		this->values = v;
		this->supporterCount = vector<int>(dim);
		this->dim = dim;
		for (int i = 0; i < dim; ++i)
		{
			//TODO
			//if (!std::isnan(v->at(i)))
			{
				this->totalSupCount++;
			}
		}
	}

	bool VariableSyncModule::VotedSeed::takeVector(shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v, vector<double>& scaling, double distThreshold)
	{
/*		int nans = 0;
		double distSqr = 0;
		for (int i = 0; i < dim; ++i)
		{
			if (!std::isnan(v->at(i)))
			{
				if (!std::isnan(values->at(i)))
				{
					if (scaling[i] != 0)
					{
						distSqr += ((values->at(i) - v->at(i)) * (values->at(i) - v->at(i))) / scaling[i];
					}
					else
					{
						distSqr += (values->at(i) - v->at(i)) * (values->at(i) - v->at(i));
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
			return true; //silently absorb a complete NaN vector
		}
		if (distSqr / (dim - nans) < distThreshold)
		{
			for (int i = 0; i < dim; ++i)
			{
				if (!std::isnan(v->at(i)))
				{
					if (std::isnan(values->at(i)))
					{
						this->supporterCount[i] = 1;
						this->totalSupCount++;
						this->values->at(i) = v->at(i);
					}
					else
					{
						this->values->at(i) = values->at(i) * this->supporterCount[i] + v->at(i);
						this->supporterCount[i]++;
						this->totalSupCount++;
						this->values->at(i) /= this->supporterCount[i];
					}
				}
			}
			return true;
		}
		return false;
		*/
		return false;
	}
} /* namespace alica */
