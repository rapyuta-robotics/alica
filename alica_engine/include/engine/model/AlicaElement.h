#pragma once

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

		void setName(std::string name);
		std::string getName() const;
		void setComment(std::string comment);
		std::string getComment();
		long getId() const;
		void setId(long id);

		virtual std::string toString() const;

	protected:
		/**
		 * This element's unique id
		 */
		long id;
		/**
		 * This element's descriptive name.
		 */
		std::string name;
		std::string comment;
	};

} /* namespace Alica */
