add_library( statismo_core ${statismo_LIB_TYPE}
  ModelInfo.cxx
)
target_link_libraries( statismo_core ${HDF5_LIBRARIES} ${Boost_LIBRARIES} )

set_target_properties( statismo_core PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)

install( TARGETS statismo_core
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
