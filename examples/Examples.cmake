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


generate_example(2d_planning_simulation)
generate_example(3d_planning_simulation)