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
	typedef long long alicaTime;

	class IAlicaClock
	{
	public:
		IAlicaClock();
		virtual ~IAlicaClock();
		virtual alicaTime now() = 0;
		virtual void sleep(int availTime) = 0;
	};

}
#endif /* IALICATIME_H_ */
