#include <gtest/gtest.h>
#include "engine/util/ConfigPathParser.h"

TEST(PathParserTest, parsePath)
{
    ConfigPathParser pathParser;
    std::vector<std::string> params = pathParser.getParams('.', "Alica.Test.Path.To.Value");

    EXPECT_EQ("Alica", params[0]);
    EXPECT_EQ("Test", params[1]);
    EXPECT_EQ("Path", params[2]);
    EXPECT_EQ("To", params[3]);
    EXPECT_EQ("Value", params[4]);
    EXPECT_EQ(5u, params.size());

    params = pathParser.getParams(';', "Alica.Test.Path.To.Value");
    EXPECT_EQ("Alica.Test.Path.To.Value", params[0]);
    EXPECT_EQ(1u, params.size());

    params = pathParser.getParams(';', "Alica.Test;Path.To.Value");
    EXPECT_EQ("Alica.Test", params[0]);
    EXPECT_EQ("Path.To.Value", params[1]);
    EXPECT_EQ(2u, params.size());

    params = pathParser.getParams(';', "");
    EXPECT_TRUE(params.empty());
}