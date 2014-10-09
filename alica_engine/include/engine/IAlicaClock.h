/*
 * IAlicaTime.h
 *
 *  Created on: Jul 16, 2014
 *      Author: snook
 */

#ifndef IALICATIME_H_
#define IALICATIME_H_
namespace alica
{
	typedef signed long long alicaTime;

	class IAlicaClock
	{
	public:
		virtual ~IAlicaClock() {}
		virtual alicaTime now() = 0;
		virtual void sleep(long us) = 0;
	};

}
#endif /* IALICATIME_H_ */
