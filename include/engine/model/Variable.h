/*
 * Variable.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef VARIABLE_H_
#define VARIABLE_H_

using namespace std;

#include <string>
#include <sstream>

#include "AlicaElement.h"

namespace alica
{

	class Variable : public AlicaElement
	{
	public:
		Variable();
		Variable(long id, string name, string type);
		virtual ~Variable();

		string toString();

		const string& getType() const;
		void setType(const string& type);


	private:
		string type;
	};

} /* namespace Alica */

#endif /* VARIABLE_H_ */
