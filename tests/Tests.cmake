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

endfunction(generate_test)

generate_test(internal_Util_test)
generate_test(internal_SVM_test)
generate_test(OccupancyGrid_test)
generate_test(internal_DiscreteSearch_test)