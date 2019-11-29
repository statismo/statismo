# common options used by project CMakeLists.txt
# and superbuild

option(BUILD_DOCUMENTATION "Build doxygen documentation" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_CLI_TOOLS "Build command-line tools" ON)
option(BUILD_WITH_TIDY "Build with clang-tidy for more sanity" OFF)
option(ITK_SUPPORT "Build with ITK Support" ON)
option(VTK_SUPPORT "Build with VTK Support" ON)
option(BUILD_WRAPPING "Build Python wrappers (experimental)" OFF)
mark_as_advanced(BUILD_WRAPPING)
include(${CMAKE_ROOT}/Modules/Documentation.cmake)
mark_as_advanced(BUILD_DOCUMENTATION)

include(CMakeDependentOption)
cmake_dependent_option(BUILD_CLI_TOOLS_DOC "Build documentation for the command-line tools" OFF
  "BUILD_CLI_TOOLS" OFF
)

cmake_dependent_option(BUILD_LONG_RUNNING_CLI_TESTS "Run the cli examples (the execution of these tests can take some time)" ON
  "BUILD_CLI_TOOLS;BUILD_TESTS" OFF
)