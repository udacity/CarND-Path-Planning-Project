#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/tools.h"

using namespace std;

TEST_CASE("test that 2.0 + 2.0 equals 4.0")
{
  REQUIRE(2.0 + 2.0 == Approx(4.0));
}
