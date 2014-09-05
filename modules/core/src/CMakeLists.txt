add_library( statismo-core ${statismo_LIB_TYPE}
  ModelInfo.cxx
)
target_link_libraries( statismo-core ${HDF5_LIBRARIES} )

set_target_properties( statismo-core PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)

install( TARGETS statismo-core
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
