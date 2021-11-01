add_custom_target(build_rlss_tests)

function(generate_test testnamebase)
add_executable(
    ${testnamebase}
    tests/${testnamebase}.cpp
)

target_link_libraries(
    ${testnamebase} PUBLIC
    rlss
)

add_test(NAME ${testnamebase} COMMAND ${testnamebase})
add_dependencies(build_rlss_tests ${testnamebase})
endfunction(generate_test)


generate_test(internal_Util_test)
generate_test(internal_SVM_test)
generate_test(OccupancyGrid_test)
generate_test(internal_DiscreteSearch_test)
generate_test(internal_BFS_test)
generate_test(internal_Statistics_test)
generate_test(TimeSequence_test)