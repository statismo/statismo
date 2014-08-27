add_library( statismo-core
  ModelInfo.cxx
)
target_link_libraries( statismo-core ${HDF5_LIBRARIES} )

set_target_properties( statismo-core PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)

install( TARGETS statismo-core
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
