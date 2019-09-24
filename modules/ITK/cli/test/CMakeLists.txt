# Test some of the commands provided in the man pages
# Not all commands for 3D deformation models will be tested because the example image data is in 2d

set(TEST_DEFORMATION_DATA_DIR "${CMAKE_BINARY_DIR}/tests-cli/deformation/")
set(TEST_SHAPE_DATA_DIR "${CMAKE_BINARY_DIR}/tests-cli/shape/")
set(STATISMO_DATA_DIR "${statismo_SOURCE_DIR}/data/")

# write a file containing the paths to the deformation fields/meshes to build the models.
# in this test the absolute paths are used so that it's independent from the current working directory. Paths relative to the current working directory can be used as well.
function(write_filelist fileName filePattern)
  file(GLOB file_list "${filePattern}")
  if(EXISTS ${fileName})
    FILE(REMOVE ${fileName})
  endif()
  foreach(line ${file_list})
    file(APPEND ${fileName} "${line}\n")
  endforeach()
  foreach(line ${ARGN})
    file(APPEND ${fileName} "${line}\n")
  endforeach()
endfunction()

set(FILENAME_DEFORMATION_FIELDS "${TEST_DEFORMATION_DATA_DIR}data-list.txt")
write_filelist(${FILENAME_DEFORMATION_FIELDS} "${STATISMO_DATA_DIR}/hand_dfs/df-hand-*.vtk")

# List of 2D tests

set(FILENAME_DEFORMATION_MODEL_2D "${TEST_DEFORMATION_DATA_DIR}2d deformationmodel.h5")
add_test(NAME statismo-build-deformation-model-test-2d
  COMMAND statismo-build-deformation-model -d 2 -l "${FILENAME_DEFORMATION_FIELDS}" "${FILENAME_DEFORMATION_MODEL_2D}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

set(FILENAME_GAUSSDEFORMATION_MODEL_2D "${TEST_DEFORMATION_DATA_DIR}2d gaussian deformationmodel.h5")
add_test(NAME statismo-build-gp-model-test-2d-image
  COMMAND statismo-build-gp-model -d 2 -t deformation -k gaussian -p 75 -s 100 -n 100 -r "${STATISMO_DATA_DIR}hand_images/hand-1.vtk" "${FILENAME_GAUSSDEFORMATION_MODEL_2D}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

add_test(NAME statismo-build-gp-model-test-2d-image-extend-existing-model
    COMMAND statismo-build-gp-model -d 2 -t deformation -k gaussian -p 75 -s 100 -n 100 -m "${FILENAME_GAUSSDEFORMATION_MODEL_2D}" "${TEST_DEFORMATION_DATA_DIR}2d extended gaussian deformationmodel.h5"
    WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-gp-model-test-2d-image-extend-existing-model APPEND PROPERTY DEPENDS statismo-build-gp-model-test-2d-image)

add_test(NAME statismo-sample-test-2d-image
  COMMAND statismo-sample -d 2 -t deformation -i "${FILENAME_DEFORMATION_MODEL_2D}" "${TEST_DEFORMATION_DATA_DIR}sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-2d-image APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-2d)

set(FILENAME_DEFORMATION_FIELD "${TEST_DEFORMATION_DATA_DIR}model-deform-field-no-landmarks.vtk")
add_test(NAME statismo-fit-image-test-2d-no-landmarks
  COMMAND statismo-fit-image -d 2 -i "${FILENAME_GAUSSDEFORMATION_MODEL_2D}" -m "${STATISMO_DATA_DIR}/hand_images/hand-2.vtk" -w 0.25 -f "${STATISMO_DATA_DIR}/hand_images/hand-1.vtk" -e "${FILENAME_DEFORMATION_FIELD}" -a "${TEST_DEFORMATION_DATA_DIR}entire-deform-field-no-landmarks.vtk" -p "${TEST_DEFORMATION_DATA_DIR}projection-hand2-no-landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-image-test-2d-no-landmarks APPEND PROPERTY DEPENDS statismo-build-gp-model-test-2d-image)

add_test(NAME statismo-fit-image-test-2d-with-landmarks
  COMMAND statismo-fit-image -d 2 -i "${FILENAME_GAUSSDEFORMATION_MODEL_2D}" -m "${STATISMO_DATA_DIR}/hand_images/hand-2.vtk" -w 0.25 -f "${STATISMO_DATA_DIR}/hand_images/hand-1.vtk" --fixed-landmarks "${STATISMO_DATA_DIR}/hand_landmarks/hand-2.csv" --moving-landmarks "${STATISMO_DATA_DIR}/hand_landmarks/hand-1.csv" -v 0.1 -p "${TEST_DEFORMATION_DATA_DIR}projection-hand2-with-landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-image-test-2d-with-landmarks APPEND PROPERTY DEPENDS statismo-build-gp-model-test-2d-image)

add_test(NAME statismo-reduce-model-test-2d
  COMMAND statismo-reduce-model -d 2 -t deformation -i "${FILENAME_DEFORMATION_MODEL_2D}" -v 91 "${TEST_DEFORMATION_DATA_DIR}reduced 2D model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-2d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-2d)

add_test(NAME statismo-posterior-test-2d
  COMMAND statismo-posterior -d 2 -i "${FILENAME_DEFORMATION_MODEL_2D}" -t deformation -f "${STATISMO_DATA_DIR}/hand_landmarks/hand-2.csv" -m "${STATISMO_DATA_DIR}/hand_landmarks/hand-1.csv" -v 0.5 "${TEST_DEFORMATION_DATA_DIR}2D posterior model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-2d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-2d)

add_test(NAME statismo-warp-image-test-2d
  COMMAND statismo-warp-image -d 2 -i "${STATISMO_DATA_DIR}/hand_images/hand-1.vtk" -f "${FILENAME_DEFORMATION_FIELD}" "${TEST_DEFORMATION_DATA_DIR}hand-1 image warped to hand-2.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-warp-image-test-2d APPEND PROPERTY DEPENDS statismo-fit-image-test-2d-no-landmarks)

# List of 3D tests

set(FILENAME_DEFORMATION_MODEL_3D "${TEST_DEFORMATION_DATA_DIR}3d deformationmodel.h5")
add_test(NAME statismo-build-deformation-model-test-3d
  COMMAND statismo-build-deformation-model -l "${FILENAME_DEFORMATION_FIELDS}" "${FILENAME_DEFORMATION_MODEL_3D}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

set(FILENAME_GAUSSDEFORMATION_MODEL_3D "${TEST_DEFORMATION_DATA_DIR}3d gaussian deformationmodel.h5")
add_test(NAME statismo-build-gp-model-test-3d-image
  COMMAND statismo-build-gp-model -t deformation -k gaussian -p 75 -s 100 -n 25 -r "${STATISMO_DATA_DIR}hand_images/hand-0.vtk" "${FILENAME_GAUSSDEFORMATION_MODEL_3D}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

add_test(NAME statismo-reduce-model-test-3d
  COMMAND statismo-reduce-model -t deformation -i "${FILENAME_DEFORMATION_MODEL_3D}" -v 91 "${TEST_DEFORMATION_DATA_DIR}reduced 3D model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-3d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-3d)

add_test(NAME statismo-posterior-test-3d
  COMMAND statismo-posterior -i "${FILENAME_DEFORMATION_MODEL_3D}" -t deformation -f "${STATISMO_DATA_DIR}/hand_landmarks/hand-2.csv" -m "${STATISMO_DATA_DIR}/hand_landmarks/hand-1.csv" -v 0.5 "${TEST_DEFORMATION_DATA_DIR}3D posterior model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-3d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-3d)

set(FILENAME_3D_DEFORM_SAMPLE "${TEST_DEFORMATION_DATA_DIR}3d deformation field sample.vtk")
add_test(NAME statismo-sample-test-3d-image
  COMMAND statismo-sample -t deformation -i "${FILENAME_GAUSSDEFORMATION_MODEL_3D}" "${FILENAME_3D_DEFORM_SAMPLE}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-3d-image APPEND PROPERTY DEPENDS statismo-build-gp-model-test-3d-image)

add_test(NAME statismo-warp-image-test-3d
  COMMAND statismo-warp-image -i "${STATISMO_DATA_DIR}hand_images/hand-0.vtk" -f "${FILENAME_3D_DEFORM_SAMPLE}" "${TEST_DEFORMATION_DATA_DIR}hand-0 warp.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-warp-image-test-3d APPEND PROPERTY DEPENDS statismo-sample-test-3d-image)

set(FILENAME_GP_SHAPE_MODEL "${TEST_SHAPE_DATA_DIR}gp shapemodel.h5")
add_test(NAME statismo-build-gp-model-test
  COMMAND statismo-build-gp-model -t shape -k gaussian -p 95.5 -s 100.42 -n 150 -r "${STATISMO_DATA_DIR}/femur_meshes/VSD001_femur.vtk" "${FILENAME_GP_SHAPE_MODEL}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

add_test(NAME statismo-reduce-model-test-5-components
  COMMAND statismo-reduce-model -i "${FILENAME_GP_SHAPE_MODEL}" -n 5 "${TEST_SHAPE_DATA_DIR}reduced_model 5 components.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-5-components APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-reduce-model-test-42.1-percent
  COMMAND statismo-reduce-model -i "${FILENAME_GP_SHAPE_MODEL}" -v 42.1 "${TEST_SHAPE_DATA_DIR}reduced_model 42.1 percent.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-42.1-percent APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-random
  COMMAND statismo-sample -i "${FILENAME_GP_SHAPE_MODEL}" "${TEST_SHAPE_DATA_DIR}random sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-random APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-random-2
  COMMAND statismo-sample -i "${FILENAME_GP_SHAPE_MODEL}" "${TEST_SHAPE_DATA_DIR}random sample 2.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-random-2 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-mean
  COMMAND statismo-sample -m -i "${FILENAME_GP_SHAPE_MODEL}" "${TEST_SHAPE_DATA_DIR}mean sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-mean APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-reference
  COMMAND statismo-sample -r -i "${FILENAME_GP_SHAPE_MODEL}" "${TEST_SHAPE_DATA_DIR}reference sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-reference APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-parameters
  COMMAND statismo-sample -p 1:1,4:0.1,9:2.5 -i "${FILENAME_GP_SHAPE_MODEL}" "${TEST_SHAPE_DATA_DIR}reference sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-parameters APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-fit-surface-test-no-landmarks
  COMMAND statismo-fit-surface  -i "${FILENAME_GP_SHAPE_MODEL}" -t "${STATISMO_DATA_DIR}/femur_meshes/VSD002_femur.vtk" -w 0.01 -j "${TEST_SHAPE_DATA_DIR}projected mesh no landmarks.vtk" -o "${TEST_SHAPE_DATA_DIR}fitted mesh no landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-surface-test-no-landmarks APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

set(FILENAME_PROJECTED_MESH "${TEST_SHAPE_DATA_DIR}projected mesh with landmarks.vtk")
add_test(NAME statismo-fit-surface-test-with-landmarks-1
  COMMAND statismo-fit-surface  -i "${FILENAME_GP_SHAPE_MODEL}" -t "${STATISMO_DATA_DIR}/femur_meshes/VSD002_femur.vtk" -w 0.1 -j "${FILENAME_PROJECTED_MESH}" -f "${STATISMO_DATA_DIR}/femur_meshes/VSD001-lm.csv"  -m "${STATISMO_DATA_DIR}/femur_meshes/VSD002-lm.csv" -v 0.1 
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-surface-test-with-landmarks-1 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-fit-surface-test-with-landmarks-2
  COMMAND statismo-fit-surface  -i "${FILENAME_GP_SHAPE_MODEL}" -t "${STATISMO_DATA_DIR}/femur_meshes/VSD002_femur.vtk" -w 0.05 -f "${STATISMO_DATA_DIR}/femur_meshes/VSD001-lm.csv"  -m "${STATISMO_DATA_DIR}/femur_meshes/VSD002-lm.csv" -v 0.1 -p -o "${TEST_SHAPE_DATA_DIR}fitted mesh with landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-surface-test-with-landmarks-2 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-posterior-test-1
  COMMAND statismo-posterior  -i "${FILENAME_GP_SHAPE_MODEL}" -f "${STATISMO_DATA_DIR}/femur_meshes/VSD001-lm.csv"  -m "${STATISMO_DATA_DIR}/femur_meshes/VSD002-lm.csv" -v 0.1 "${TEST_SHAPE_DATA_DIR}posterior gp model 1.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-1 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-posterior-test-2
  COMMAND statismo-posterior  -i "${FILENAME_GP_SHAPE_MODEL}" -t shape -f "${STATISMO_DATA_DIR}/femur_meshes/VSD001-lm.csv"  -m "${STATISMO_DATA_DIR}/femur_meshes/VSD002-lm.csv" -v 0.1 "${TEST_SHAPE_DATA_DIR}posterior gp model 2.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-2 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-posterior-test-with-corresponding-mesh
  COMMAND statismo-posterior  -i "${FILENAME_GP_SHAPE_MODEL}" -c "${FILENAME_PROJECTED_MESH}" "${TEST_SHAPE_DATA_DIR}posterior gp model with corresponding mesh.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-with-corresponding-mesh APPEND PROPERTY DEPENDS statismo-build-gp-model-test)
set_property(TEST statismo-posterior-test-with-corresponding-mesh APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)



set(FILENAME_MESHES "${TEST_SHAPE_DATA_DIR}data-list.txt")
write_filelist(${FILENAME_MESHES} "${STATISMO_DATA_DIR}/femur_meshes/VSD001_femur.vtk" "${FILENAME_PROJECTED_MESH}")

add_test(NAME statismo-build-shape-model-test-1
  COMMAND statismo-build-shape-model --data-list "${FILENAME_MESHES}" "${TEST_SHAPE_DATA_DIR}shape model 1.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-shape-model-test-1 APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)

add_test(NAME statismo-build-shape-model-test-2
  COMMAND statismo-build-shape-model -p GPA -l "${FILENAME_MESHES}" "${TEST_SHAPE_DATA_DIR}shape model 2.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-shape-model-test-2 APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)

add_test(NAME statismo-build-shape-model-test-3
  COMMAND statismo-build-shape-model -p reference -r "${FILENAME_PROJECTED_MESH}" -l "${FILENAME_MESHES}" "${TEST_SHAPE_DATA_DIR}shape model 3.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-shape-model-test-3 APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)
