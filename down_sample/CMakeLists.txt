get_filename_component(example ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(dependencies easy3d::viewer easy3d::algo)

set(${example}_files
        main.cpp
        )

add_executable(${example} main.cpp)
set_target_properties(${example} PROPERTIES FOLDER "tutorials")
target_include_directories(${example} PRIVATE ${Easy3D_INCLUDE_DIR})
target_link_libraries(${example} PRIVATE ${dependencies})
target_compile_definitions(${example} PRIVATE "EXAMPLE_TITLE=\"${example}\"")
