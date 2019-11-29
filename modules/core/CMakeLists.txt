# Create target
add_library(statismo_core ${statismo_LIB_TYPE}
  src/HDF5Utils.cxx 
  src/ModelInfo.cxx
  src/RandUtils.cxx
  src/Representer.cxx
  src/StatismoUtils.cxx
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/statismo/core>
)
target_compile_features(statismo_core PUBLIC cxx_std_17)
target_compile_options(statismo_core PRIVATE "${STATISMO_COMPILE_OPTIONS}")
set_target_properties(statismo_core PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)
if(BUILD_WITH_TIDY)
   set_target_properties(
   ${statismo_core} PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
generate_export_header(
  statismo_core
  BASE_NAME STATISMO_CORE
  EXPORT_FILE_NAME StatismoCoreExport.h
  INCLUDE_GUARD_NAME __STATISMO_CORE_EXPORT_H_
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

install(FILES ${_target_headers} ${CMAKE_CURRENT_BINARY_DIR}/StatismoCoreExport.h
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/core
)
