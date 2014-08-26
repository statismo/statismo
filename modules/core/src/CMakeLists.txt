add_library( statismo-core
  ModelInfo.cxx
)
target_link_libraries( statismo-core ${HDF5_LIBRARIES} )
