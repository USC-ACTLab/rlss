#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/TimeSequence/TimeSequence.hpp>

TEST_CASE("TimeSequence basic_operation test", "SVMSequence"){
    using Hyperplane = rlss::internal::Hyperplane<double, 2U>;
    using SS = rlss::TimeSequence<double, Hyperplane>;
    using Pair = std::pair<double, Hyperplane>;
    SS testSequence = SS();

    SECTION("add()"){
        REQUIRE(testSequence.size() == 0);

        // try negative time
        REQUIRE(testSequence.add((float)-1.0, Hyperplane()));
        REQUIRE(testSequence.size() == 1);

        REQUIRE(testSequence.add(-1.5, Hyperplane()) == false);
        REQUIRE(testSequence.size() == 1);

        REQUIRE(testSequence.add(2.0, Hyperplane()));
        REQUIRE(testSequence.size() == 2);

        // add a hyperplane of earlier time, which should fail
        REQUIRE(testSequence.add(0.1, Hyperplane()) == false);
        REQUIRE(testSequence.size() == 2);

        // try double, int..... NaN
        testSequence.add(sqrt(-1.0), Hyperplane());
        REQUIRE(testSequence.size() == 2);
    }

    SECTION("forget()"){
        // clear all elements
        testSequence.forget(99999);
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

        // try negative time
        testSequence.add(-1.0, Hyperplane());
        REQUIRE(testSequence.size() == 1);
        testSequence.add(-1.5, Hyperplane());
        REQUIRE(testSequence.size() == 1);

        // try double, int..... NaN
        testSequence.add(sqrt(-1.0), Hyperplane());
        REQUIRE(testSequence.size() == 1);
    }

    SECTION("operator[]"){
        // clear all elements
        testSequence.forget(99999);
        testSequence.add(1.0, Hyperplane());
        testSequence.add(2.0, Hyperplane());
        REQUIRE(testSequence.size() == 2);
        auto element1 = testSequence[0];
        REQUIRE(element1.ok());
        auto element2 = testSequence[2];
        REQUIRE(!element2.ok());
    }
}

TEST_CASE("TimeSequence const_iterator test", "SVMSequence"){
    using Hyperplane = rlss::internal::Hyperplane<double, 2U>;
    using VectorDIM = rlss::internal::VectorDIM<double, 2U>;
    using SS = rlss::TimeSequence<double, Hyperplane>;
    using Pair = std::pair<double, Hyperplane>;

    Hyperplane shp {VectorDIM {-0.461538461538462, -0.307692307692308}, 0.230769230769231};

    SS testSequence = SS();

    SECTION("cbegin() & cend()"){
        auto it = testSequence.cbegin();
        REQUIRE(it == testSequence.cend());
        testSequence.add(1.0, shp);
        it = testSequence.cbegin();
        REQUIRE(it != testSequence.cend());
        REQUIRE((*it).isApprox(shp, 1e-9));
    }

    SECTION("operator*() & operator->()"){
        testSequence.forget(99999);
        testSequence.add(1.0, shp);
        auto it = testSequence.cbegin();
        REQUIRE((*it).isApprox(shp, 1e-9));
        REQUIRE(it->isApprox(shp, 1e-9));
    }

    SECTION("operator++() & operator++(int)"){
        testSequence.forget(99999);
        testSequence.add(1.0, Hyperplane());
        testSequence.add(2.0, Hyperplane());
        testSequence.add(3.0, shp);
        auto it = testSequence.cbegin();
        REQUIRE(!(*it).isApprox(shp, 1e-9));
        ++it;
        REQUIRE(!(*it).isApprox(shp, 1e-9));
        ++it;
        REQUIRE((*it).isApprox(shp, 1e-9));
    }

    SECTION("operator==() & operator!=()"){
        testSequence.forget(99999);
        testSequence.add(1.0, Hyperplane());
        testSequence.add(2.0, Hyperplane());
        testSequence.add(3.0, shp);
        auto it = testSequence.cbegin();
        auto it2 = testSequence.cbegin();
        REQUIRE(it == it2);
        ++it;
        REQUIRE(it != it2);
    }
}