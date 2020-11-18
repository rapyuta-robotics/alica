#include <gtest/gtest.h>
#include "engine/util/PathParser.h"

TEST(PathParserTest, parsePath)
{
    PathParser pathParser;
    std::vector<std::string> params = pathParser.getParams('.', "Alica.Test.Path.To.Value");

    EXPECT_EQ("Alica", params[0]);
    EXPECT_EQ("Test", params[1]);
    EXPECT_EQ("Path", params[2]);
    EXPECT_EQ("To", params[3]);
    EXPECT_EQ("Value", params[4]);
    EXPECT_EQ(5, params.size());

    params = pathParser.getParams(';', "Alica.Test.Path.To.Value");
    EXPECT_EQ("Alica.Test.Path.To.Value", params[0]);
    EXPECT_EQ(1, params.size());

    params = pathParser.getParams(';', "Alica.Test;Path.To.Value");
    EXPECT_EQ("Alica.Test", params[0]);
    EXPECT_EQ("Path.To.Value", params[1]);
    EXPECT_EQ(2, params.size());

    params = pathParser.getParams(';', nullptr);
    EXPECT_TRUE(params.empty());
}