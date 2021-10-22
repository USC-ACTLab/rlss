#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/SVMSequence/SVMSequence.hpp>

TEST_CASE("SVMSequence basic_operation test", "SVMSequence"){
    using SS = rlss::SVMSequence<double, 2>;
    using Hyperplane = rlss::internal::Hyperplane<double, 2>;
    using Pair = std::pair<double, Hyperplane>;
    SS testSequence = SS();

    SECTION("add()"){
        REQUIRE(testSequence.size() == 0);

        REQUIRE(testSequence.add(1.0, Hyperplane()));
        REQUIRE(testSequence.size() == 1);

        REQUIRE(testSequence.add(2.0, Hyperplane()));
        REQUIRE(testSequence.size() == 2);

        // add a hyperplane of earlier time, which should fail
        REQUIRE(testSequence.add(0.1, Hyperplane()) == false);
        REQUIRE(testSequence.size() == 2);

        // clear all elements
        testSequence.forget(3.0);
    }

    SECTION("forget()"){
        REQUIRE(testSequence.size() == 0);
        REQUIRE(testSequence.forget(0.5) == 0);
        testSequence.add(1.0, Hyperplane());
        testSequence.add(2.0, Hyperplane());
        REQUIRE(testSequence.size() == 2);

        REQUIRE(testSequence.forget(0.5) == 0);
        REQUIRE(testSequence.size() == 2);

        REQUIRE(testSequence.forget(1.5) == 1);
        REQUIRE(testSequence.size() == 1);

        testSequence.add(3.0, Hyperplane());
        REQUIRE(testSequence.size() == 2);
        REQUIRE(testSequence.forget(4.0) == 2);
        REQUIRE(testSequence.size() == 0);
    }
}