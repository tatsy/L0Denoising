set(BUILD_TARGET "denoise")

set(SOURCES ${SOURCES}
    denoising.cpp
    main.cpp
    mt19937ar.c)

set(HEADERS ${HEADERS}
    denoising.h
    mt19937ar.h)

add_executable(${BUILD_TARGET} ${SOURCES} ${HEADERS})
target_link_libraries(${BUILD_TARGET} ${OpenCV_LIBS}
                                      ${OPENMESH_LIBRARIES}
                                      ${freeglut_LIBRARIES})
