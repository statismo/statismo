# Create target
add_library(statismo_ITK INTERFACE)
target_include_directories(
    statismo_ITK INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/statismo/ITK>
)
target_link_libraries(statismo_ITK INTERFACE statismo_core ${ITK_LIBRARIES})
target_compile_features(statismo_ITK INTERFACE cxx_std_17)

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
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
)
install(FILES ${TARGET_HEADERS}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/ITK
)
