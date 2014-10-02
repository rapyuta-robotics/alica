/*
 * PlanElement.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ALICAELEMENT_H_
#define ALICAELEMENT_H_
using namespace std;

#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
namespace alica
{

	/**
	 * Base class of all model elements
	 */
	class AlicaElement
	{
	public:
		AlicaElement();
		virtual ~AlicaElement();

		void setName(string name);
		string getName() const;
		void setComment(string comment);
		string getComment();
		long getId() const;
		void setId(long id);

		virtual string toString() const;

	protected:
		/**
		 * This element's unique id
		 */
		long id;
		/**
		 * This element's descriptive name.
		 */
		string name;
		string comment;
	};

} /* namespace Alica */

#endif /* ALICAELEMENT_H_ */
