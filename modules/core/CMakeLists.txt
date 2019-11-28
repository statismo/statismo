# Create target
add_library(statismo_core ${statismo_LIB_TYPE}
  src/HDF5Utils.cxx 
  src/ModelInfo.cxx
  src/Representer.cxx
  src/ThreadPool.cxx 
)

set(deps
  ${HDF5_LIBRARIES}
)

if(UNIX)
  set(deps
    ${deps}
    pthread
  )
endif()

target_link_libraries(statismo_core PUBLIC ${deps})
target_include_directories(statismo_core PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/statismo/core>
)
target_compile_features(statismo_core PUBLIC cxx_std_17)
set_target_properties(statismo_core PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)

# Test
if(${BUILD_TESTS})
  add_subdirectory(tests)
endif()

# Install

install(TARGETS statismo_core
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)

file(GLOB _target_headers
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
)

install(FILES ${_target_headers}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/core
)
