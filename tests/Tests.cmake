add_executable(
    internal_Util_test
    tests/internal_Util_test.cpp
)
target_link_libraries(
    internal_Util_test
    rlss
)
add_test(NAME internal_Util_test COMMAND internal_Util_test)