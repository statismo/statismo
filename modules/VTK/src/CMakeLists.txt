add_library( statismo-VTK
  vtkStandardMeshRepresenter.cxx
  vtkUnstructuredGridRepresenter.cxx
)
target_link_libraries( statismo-VTK
  statismo-core
  ${VTK_LIBRARIES}
)

install( TARGETS statismo-VTK
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
