/*
 * BasicBehaviour.h
 *
 *  Created on: Jun 4, 2014
 *      Author: stefan
 */

#ifndef BASICBEHAVIOUR_H_
#define BASICBEHAVIOUR_H_

using namespace std;

#include <string>
#include <iostream>
#include <map>

namespace alica
{

	class BasicBehaviour
	{
	public:
		BasicBehaviour();
		BasicBehaviour(string name);
		virtual ~BasicBehaviour();
		virtual void run(void* msg) = 0;
		const string getName() const;
		void setName(string name);

		typedef  BasicBehaviour * (*creatorFunc)(void);
		static BasicBehaviour * createInstance(string const& s)
		{
			cout << "DEBUG: createInstance(name) called!" << endl;
			map<string, BasicBehaviour*(*)()>::iterator it = getCreatorMap()->find(s);

			auto* myMap = getCreatorMap();
			for(auto it = myMap->cbegin(); it != myMap->cend(); ++it)
			{
				cout << "DEBUG: Pair: " << it->first << " " << it->second << endl;
			}
			if (it == getCreatorMap()->end())
			{
				cout << "DEBUG: " << s << " not found in map!" << endl;
				return nullptr;
			}
			return it->second(); // instantiates and returns the corresponding Behaviour
		}

		static bool reg(string name, creatorFunc f)
		{
			creators->insert(make_pair(name, f));
			return true;
		}

	protected:
		string name;
		map<string, string>* parameters;
		int getOwnId();

		/**
		 * Creates if not initialised the map for the Behaviour Construction Function Pointer.
		 * @return A map from behaviour name to construction function pointer.
		 */
		static map<string, BasicBehaviour*(*)()> * getCreatorMap();


	private:
		/**
		 * This map matches behaviour names to their construction methods.
		 */
		static map<string, BasicBehaviour*(*)()>* creators;

	};

/**
 * Creates a new instance of a BasicBehaviour. Function pointer to specialised versions of this
 * method are stored in the creators map.
 * @return An instance of a BasicBehaviour
 */
//	template<typename T = BasicBehaviour>
//	BasicBehaviour * createBasicBehaviour()
//	{
//		return new T;
//	}
/**
 * Artificial Type. Each Behaviour, which is inheriting BasicBehaviour must have a static
 * variable of this type, in order to register itself in the creators map.
 */
//	template<typename T>
//	class DerivedRegister : BasicBehaviour
//	{
//	public:
//		DerivedRegister(std::string const& s, creatorFunc f)
//		{
//			getCreatorMap()->insert(make_pair(s, f));
//		}
//
//		/**
//		 * Can be ignored, because it must be implemented, because of the inheritance structure.
//		 * @param msg
//		 */
//		void run(void* msg)
//		{
//		}
//		;
//	};
} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
