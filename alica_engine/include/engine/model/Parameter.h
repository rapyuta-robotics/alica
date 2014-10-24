/*
 * Parameters.h
 *
 *  Created on: Oct 24, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_SRC_ENGINE_MODEL_PARAMETERS_H_
#define ALICA_ALICA_ENGINE_SRC_ENGINE_MODEL_PARAMETERS_H_

#include <string>

#include <engine/model/AlicaElement.h>

namespace alica
{

	class Parameter : public AlicaElement
	{
	public:
		Parameter();
		virtual ~Parameter();
		string getKey();
		void setKey(string key);
		string getValue();
		void setValue(string value);

	protected:
		string key;
		string value;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_SRC_ENGINE_MODEL_PARAMETERS_H_ */
