#include <tracing/TraceFactory.h>
#include <tracing/Trace.h>

#include <gtest/gtest.h>

TEST(TestAmrTracing, TestAmrTracingInit)
{
    auto tf = alicaTracing::TraceFactory("amr_routing", "any_path");
    // Report init
    auto trace = tf.create("init", std::nullopt);
    trace->setTag("Init", "initialized");
}

/* Main
 * -------------------------------------------------------------------------------------
 */
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}