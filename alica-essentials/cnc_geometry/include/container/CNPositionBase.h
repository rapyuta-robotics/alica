#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONBASE_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONBASE_H_

#include "geometry_msgs/Pose2D.h"

namespace geometry {

	using namespace std;

	class CNPositionBase : public geometry_msgs::Pose2D {
	public:
	    CNPositionBase() : CNPositionBase(0, 0, 0) { }
		CNPositionBase(double x, double y, double theta);
		virtual ~CNPositionBase();
		virtual string toString();
	};

}

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONBASE_H_ */
