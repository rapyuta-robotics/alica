/*
 * DistXContour.h
 *
 *  Created on: Oct 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_TEST_INCLUDE_DISTXCONTOUR_H_
#define ALICA_ALICA_TEST_INCLUDE_DISTXCONTOUR_H_

#include <engine/USummand.h>
#include <vector>
#include <string>

using namespace std;

namespace alica
{

	class UtilityInterval;
	class IAssignment;

	class DistXContour : public USummand
	{
	public:
		DistXContour(double weight, string name, long id, vector<long>& relevantEntryPointIds, vector<pair<double, double>>& ContourPoints, double xMaxVal, double xMinVal, int ownId);
		virtual ~DistXContour();
		void cacheEvalData();
		double interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint);
		virtual UtilityInterval* eval(IAssignment* ass);


	protected:
		int ownId;
		double weight;
		string name;
		long id;
		vector<long> relevantEntryPointIds;
		vector<pair<double, double>> contourPoints;
		double xAlloBall;
		double xMaxVal;
		double xMinVal;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_INCLUDE_DISTXCONTOUR_H_ */
