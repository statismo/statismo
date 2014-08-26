add_library( statismo-VTK
  vtkStandardMeshRepresenter.cxx
  vtkUnstructuredGridRepresenter.cxx
)
target_link_libraries( statismo-VTK
  statismo-core
  ${VTK_LIBRARIES}
)
