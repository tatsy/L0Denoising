set(BUILD_TARGET "denoise")

set(SOURCES ${SOURCES}
    denoising.cpp denoising.h
    main.cpp
    mt19937ar.c mt19937ar.h)

add_executable(${BUILD_TARGET} ${SOURCES})

if (MSVC)
  source_group("Source Files" FILES ${SOURCES})
endif()

target_link_libraries(${BUILD_TARGET} ${OpenCV_LIBS}
                                      ${OPENMESH_LIBRARIES}
                                      ${freeglut_LIBRARIES})
