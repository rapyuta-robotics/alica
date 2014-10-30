#ifndef SUPPLEMENTARY_DATETIME_H
#define SUPPLEMENTARY_DATETIME_H 1

#include <sys/time.h>
#include <time.h>

#define EPOCH_ADJUST (62135596800LL)

namespace supplementary {
	
	struct DateTime {

		protected:

			long long ticks;

		public:

			/**
			 * Initialize structure using a time value in 100 ns resolution
			 * @param ticks Tocks (100 ns resolution
			 */
			DateTime(long long t) : ticks(t) {
			}

			DateTime(const DateTime &other) :
				ticks(other.ticks)
			{
			}

			static inline DateTime getUtcNow() {
				struct timeval tv;
				memset(&tv, 0, sizeof(tv));
				gettimeofday(&tv, NULL);
				return DateTime(((static_cast<unsigned long long>(tv.tv_sec) + EPOCH_ADJUST) * 1000000 + tv.tv_usec) * 10);
			}

			static inline unsigned long long getUtcNowC() {
				struct timeval tv;
				memset(&tv, 0, sizeof(tv));
				gettimeofday(&tv, NULL);
				return (((static_cast<unsigned long long>(tv.tv_sec) + EPOCH_ADJUST) * 1000000 + tv.tv_usec) * 10);
			}

			inline long long getTicks() {
				return this->ticks;
			}
	};
}

#endif /* SUPPLEMENTARY_DATETIME_H */

