#include <gtest/gtest.h>
#include "engine/containers/ConcurrentQueue.h"

TEST(SchedulingTest, scheduling)
{
    /*
     * Test if moodycamel ConcurrentQueue can be used as intended.
     * TODO: Replace with an actual test of the new scheduling.
     */
    moodycamel::ConcurrentQueue<int> queue;
    queue.enqueue(25);

    int item;
    bool found = queue.try_dequeue(item);

    EXPECT_TRUE(found);
    EXPECT_EQ(item, 25);
}

