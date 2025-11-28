cmake_minimum_required(VERSION 3.11)

add_subdirectory(nxt_com)
add_subdirectory(nxt_remote)

# Export all targets
export(EXPORT lib_targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/nxt_libsTargets.cmake"
    NAMESPACE nxt::
)

install(EXPORT lib_targets
    FILE nxt_libsTargets.cmake
    NAMESPACE nxt::
    DESTINATION lib/cmake/nxt_libs
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/nxt_libsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/nxt_libsConfig.cmake"
    INSTALL_DESTINATION lib/cmake/nxt_libs
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/nxt_libsConfig.cmake"
    DESTINATION lib/cmake/nxt_libs
)
