#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this once per test-runner!

#include "catch.hpp"
#include "PrimeChecker.hpp"

//Test case 1
TEST_CASE("Test PrimeChecker 1.") {
    PrimeChecker pc;
    REQUIRE(pc.isPrime(5));
}

//Test case 2
TEST_CASE("Test PrimeChecker 2.") {
    PrimeChecker pc;
    REQUIRE(pc.isPrime(7));
}


//Test case 3
TEST_CASE("Test PrimeChecker 3.") {
    PrimeChecker pc;
    REQUIRE(pc.isPrime(11));
}


//Test case 4
TEST_CASE("Testing non prime numbers 1") {
	PrimeChecker pc;
    REQUIRE(pc.isPrime(10) == false);
}

//Test case 5
TEST_CASE("Testing non prime numbers  2") {
	PrimeChecker pc;
    REQUIRE(pc.isPrime(9) == false);
}

/*Experimenting with failing tests
TEST_CASE("Failing tests case.") {
    PrimeChecker pc;
    REQUIRE(pc.isPrime(20));
} */