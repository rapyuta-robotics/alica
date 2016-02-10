/*
 * RobotProperties.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#include <engine/collections/RobotProperties.h>
#include <engine/model/Characteristic.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>
#include <engine/model/CapValue.h>

namespace alica
{

	RobotProperties::RobotProperties()
	{
	}

	RobotProperties::RobotProperties(AlicaEngine* ae, string name)
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->id = (*sc)["Globals"]->tryGet<int>(-1, "Globals", "Team", name.c_str(), "ID", NULL);
		this->name = name;
		this->characteristics = map<string, Characteristic*>();
		this->capabilities = ae->getPlanRepository()->getCapabilities();
		string key = "";
		string kvalue = "";
		shared_ptr<vector<string> > caps = (*sc)["Globals"]->getNames("Globals", "Team", this->name.c_str(), NULL);
		for (string s : *caps)
		{
			if (s.compare("ID") == 0 || s.compare("DefaultRole") == 0)
			{
				continue;
			}
			key = s;
			kvalue = (*sc)["Globals"]->get<string>("Globals", "Team", this->name.c_str(), s.c_str(), NULL);
			for (auto p : this->capabilities)
			{
				if (p.second->getName().compare(key) == 0)
				{
					for (CapValue* val : p.second->getCapValues())
					{
						//transform(kvalue.begin(), kvalue.end(), kvalue.begin(), ::tolower);
						if (val->getName().compare(kvalue) == 0)
						{
							Characteristic* cha = new Characteristic();
							cha->setCapability(p.second);
							cha->setCapValue(val);
							this->characteristics.insert(pair<string, Characteristic*>(key, cha));
						}
					}
				}

			}
		}
		this->defaultRole = (*sc)["Globals"]->tryGet<string>("NOROLESPECIFIED", "Globals", "Team", name.c_str(), "DefaultRole",
																NULL);
	}

	RobotProperties::~RobotProperties()
	{
		for(auto x : this->characteristics)
		{
			delete x.second;
		}
	}

	int RobotProperties::getId() const
	{
		return id;
	}

	void RobotProperties::setId(int id)
	{
		this->id = id;
	}

	const string& RobotProperties::getName() const
	{
		return name;
	}

	void RobotProperties::setName(const string& name)
	{
		this->name = name;
	}

	map<string, Characteristic*>& RobotProperties::getCharacteristics()
	{
		return this->characteristics;
	}

	const string& RobotProperties::getDefaultRole() const
	{
		return defaultRole;
	}

	void RobotProperties::setDefaultRole(const string& defaultRole)
	{
		this->defaultRole = defaultRole;
	}

	string RobotProperties::toString()
	{
		stringstream ss;
		ss << "[RobotProperties: Id=" << this->getId() << " Default Role: " << this->getDefaultRole() << endl;
		for (pair<string, Characteristic*> p : this->getCharacteristics())
		{
			ss << p.first << " = " << p.second->getCapValue()->getName() << endl;
		}
		return ss.str();
	}

} /* namespace alica */
