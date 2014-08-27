add_library( statismo-core
  ModelInfo.cxx
)
target_link_libraries( statismo-core ${HDF5_LIBRARIES} )


install( TARGETS statismo-core
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
