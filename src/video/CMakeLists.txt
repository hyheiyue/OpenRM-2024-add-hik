add_library(
    openrm_video
        SHARED
)

# 查找 gxiapi 动态链接库
find_library(GXIAPI_LIB
    NAMES gxiapi
)
target_link_libraries(openrm_video
    PUBLIC
    ${OpenCV_LIBS}
    /opt/MVS/lib/64/libMvCameraControl.so
)


# 检查库是否找到
if(GXIAPI_LIB)
    message(STATUS "Found DaHeng: ${GXIAPI_LIB}")
    set(HAVE_GXIAPI TRUE)
else()
    message(STATUS "Could NOT found DaHeng libgxiapi.so, disable DaHeng camera support.")
    set(HAVE_GXIAPI FALSE)
endif()

target_sources(
    openrm_video
        PRIVATE
        ${CMAKE_SOURCE_DIR}/src/video/uvc.cpp
        ${CMAKE_SOURCE_DIR}/src/video/tools.cpp
        ${CMAKE_SOURCE_DIR}/src/video/hik.cpp
        $<IF:$<BOOL:${HAVE_GXIAPI}>,${CMAKE_SOURCE_DIR}/src/video/daheng.cpp,>
        $<IF:$<BOOL:${HAVE_GXIAPI}>,${CMAKE_SOURCE_DIR}/src/video/hik.cpp,>

)

target_include_directories(
    openrm_video
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/openrm>
)

target_link_libraries(
    openrm_video
        PRIVATE
        $<IF:$<BOOL:${HAVE_GXIAPI}>,gxiapi,>
)
