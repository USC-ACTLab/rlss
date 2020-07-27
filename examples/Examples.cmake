add_custom_target(build_rlss_examples)

function(generate_example examplenamebase)
    add_executable(
            ${examplenamebase}
            examples/${examplenamebase}.cpp
    )

    target_link_libraries(
            ${examplenamebase} PUBLIC
            rlss
    )

    add_dependencies(build_rlss_examples ${examplenamebase})
endfunction(generate_example)


add_executable(
    2d_sim
    examples/simulation.cpp
)

target_link_libraries(
    2d_sim PUBLIC
    rlss
)

target_compile_definitions(
    2d_sim PUBLIC
    SIMULATION_DIMENSION=2
)

add_dependencies(build_rlss_examples 2d_sim)

add_executable(
    3d_sim
    examples/simulation.cpp
)

target_link_libraries(
    3d_sim PUBLIC
    rlss
)


target_compile_definitions(
    3d_sim PUBLIC
    SIMULATION_DIMENSION=3
)


add_dependencies(build_rlss_examples 3d_sim)
