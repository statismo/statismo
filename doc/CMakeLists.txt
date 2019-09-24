if(BUILD_DOCUMENTATION)

  find_package(LATEX)
  if(NOT LATEX_COMPILER)
    message(WARNING "LaTeX not found. Formulas will not be generated in documentation.")
  endif()

  set(DOXYGEN_PROJECT_NAME ${PROJECT_NAME})
  set(DOXYGEN_PROJECT_NUMBER ${PROJECT_VERSION})
  set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set(DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR})
  set(DOXYGEN_PROJECT_BRIEF "Statismo doxygen documentation")
  set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/classdoc)
  set(DOXYGEN_GENERATE_HTML YES)

  doxygen_add_docs(
      statismoDox
      ${CMAKE_CURRENT_SOURCE_DIR}/classdoc/intro.dox ${PROJECT_SOURCE_DIR}/modules
      COMMENT "Generate doxygen doc"
      ALL
  )

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/
    DESTINATION ${INSTALL_DOC_DIR}/html
  )
endif()