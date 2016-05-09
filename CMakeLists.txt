cmake_minimum_required(VERSION 2.8.0)
project(Denoising)

find_package(OpenCV REQUIRED)

#
## Find Eigen
#
if (WIN32)
  set(Eigen_ROOT "Eigen_NOT-FOUND" CACHE PATH "")
  if (NOT EXISTS "${Eigen_ROOT}/Eigen/Core")
  else()
    set(EIGEN3_INCLUDE_DIR "${Eigen_ROOT}")
  endif()
else()
  find_package(Eigen3 REQUIRED)
endif()

#
## Find freeglut
#
if (WIN32)
  set(freeglut_ROOT "freeglut_NOT-FOUND" CACHE PATH "")
  if (NOT EXISTS "${freeglut_ROOT}/include/GL/freeglut.h")
    message(FATAL_ERROR "[ERROR] freeglut path seems to be wrong!!")
  else()
    set(freeglut_INCLUDE_DIR "${freeglut_ROOT}/include")
    set(freeglut_LIBRARIES "$<$<CONFIG:Release>:${freeglut_ROOT}/lib/freeglut.lib>"
                           "$<$<CONFIG:Debug>:${freeglut_ROOT}/lib/freeglutd.lib>")
  endif()
else()
  find_package(FreeGLUT REQUIRED)
endif()

#
## Find OpenMesh
#
if (WIN32)
 set(OpenMesh_ROOT "OpenMesh_NOT-FOUND" CACHE PATH "")
 if (NOT EXISTS "${OpenMesh_ROOT}/include/OpenMesh/Core")
   message(FATAL_ERROR "[ERROR] OpenMesh path seems to be wrong!!")
 else()
   set(OPENMESH_INCLUDE_DIR "${OpenMesh_ROOT}/include")
   set(OPENMESH_LIBRARY_DIR "${OpenMesh_ROOT}/lib")
   set(OPENMESH_LIBRARIES "$<$<CONFIG:Release>:OpenMeshCore.lib>"
                          "$<$<CONFIG:Debug>:OpenMeshCored.lib>"
                          "$<$<CONFIG:Release>:OpenMeshTools.lib>"
                          "$<$<CONFIG:Debug>:OpenMeshToolsd.lib>")
   include_directories(${OPENMESH_INCLUDE_DIR})
   link_directories(${OPENMESH_LIBRARY_DIR})
 endif()
else()
 find_package(OpenMesh REQUIRED)
endif()


include_directories(${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${OPENMESH_INCLUDE_DIR}
                    ${freeglut_INCLUDE_DIR})
add_subdirectory(sources)
