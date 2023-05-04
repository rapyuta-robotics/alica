#include <tracing/Trace.h>
#include <tracing/TraceFactory.h>

#include <gtest/gtest.h>

TEST(TestAmrTracing, TestAmrTracingInit)
{
    auto tf = alicaTracing::TraceFactory(
            "amr_routing", "/home/rr/catkin_ws/src/rr_io_amr/rr_di_core/robot/external/alica/supplementary/alica_tracing/config/tracer_default.config");
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
