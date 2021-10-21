#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/SVMSequence/SVMSequence.hpp>

TEST_CASE("SVMSequence basic_operation test", "SVMSequence"){
    using SS = rlss::SVMSequence<double, 2>;
    using Hyperplane = rlss::internal::Hyperplane<double, DIM>;
    using Pair = std::pair<double, Hyperplane>;
    SS testSequence;

    SECTION("add() ascending time"){
        testSequence = new SS();
        REQUIRE(testSequence.size() == 0);

        testSequence.add(double(1.0), new Hyperplane());
        REQUIRE(testSequence.size() == 1);
    }
}