add_library( statismo_VTK ${statismo_LIB_TYPE}
  vtkStandardMeshRepresenter.cxx
  vtkUnstructuredGridRepresenter.cxx
)
target_link_libraries( statismo_VTK
  statismo_core
  ${VTK_LIBRARIES}
)

set_target_properties( statismo_VTK PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)

install( TARGETS statismo_VTK
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
