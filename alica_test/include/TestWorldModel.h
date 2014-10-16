/*
 * TestWorldModel.h
 *
 *  Created on: Oct 14, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_TEST_SRC_TESTWORLDMODEL_H_
#define ALICA_ALICA_TEST_SRC_TESTWORLDMODEL_H_

namespace alicaTests
{

	class TestWorldModel
	{
	public:

		virtual ~TestWorldModel();
		static TestWorldModel* getOne();
		static TestWorldModel* getTwo();
		bool isTransitionCondition1413201227586();
		void setTransitionCondition1413201227586(bool transitionCondition1413201227586);
		bool isTransitionCondition1413201389955();
		void setTransitionCondition1413201389955(bool transitionCondition1413201389955);
		bool isTransitionCondition1413201052549();
		void setTransitionCondition1413201052549(bool transitionCondition1413201052549);
		bool isTransitionCondition1413201367990();
		void setTransitionCondition1413201367990(bool transitionCondition1413201367990);
		bool isTransitionCondition1413201370590();
		void setTransitionCondition1413201370590(bool transitionCondition1413201370590);

	private:
		TestWorldModel();
		bool transitionCondition1413201227586;
		bool transitionCondition1413201389955;
		bool transitionCondition1413201052549;
		bool transitionCondition1413201367990;
		bool transitionCondition1413201370590;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_SRC_TESTWORLDMODEL_H_ */
