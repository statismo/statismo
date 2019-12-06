# Create target
add_library(statismo_ITK INTERFACE)
target_include_directories(
    statismo_ITK INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)
target_link_libraries(statismo_ITK INTERFACE statismo_core ${ITK_LIBRARIES})
target_compile_features(statismo_ITK INTERFACE cxx_std_17)

set(TARGET_INCLUDE_DIR_SUFFIX /statismo/ITK)
set(TARGET_INCLUDE_DIR include/${TARGET_INCLUDE_DIR_SUFFIX})

# Test

if(${BUILD_TESTS})
  add_subdirectory(tests)
endif()

# Example

if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

# Tools

if(${BUILD_CLI_TOOLS})
  add_subdirectory(cli)
endif()

# Wrapping

if(${BUILD_WRAPPING})
  add_subdirectory(wrapping)
endif()

# Install

file(GLOB TARGET_HEADERS
  ${TARGET_INCLUDE_DIR}/*.h ${TARGET_INCLUDE_DIR}/*.hxx ${TARGET_INCLUDE_DIR}/*.hpp ${TARGET_INCLUDE_DIR}/*.txx ${TARGET_INCLUDE_DIR}/*.tpp
)
install(FILES ${TARGET_HEADERS}
  DESTINATION ${INSTALL_INCLUDE_DIR}/${TARGET_INCLUDE_DIR_SUFFIX}
)
