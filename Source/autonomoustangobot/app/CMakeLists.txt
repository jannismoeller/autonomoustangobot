cmake_minimum_required(VERSION 3.4.1)

set(SOURCE_FILES
    src/main/cpp/octomapjni/NativeObjectHelper.h
    src/main/cpp/octomapjni/NativeObjectHelper.cpp
    src/main/cpp/octomapjni/OcTreeJNI.cpp
    src/main/cpp/octomapjni/PointCloudHelperJNI.cpp
    src/main/cpp/octomapjni/GeometryHelper.h

    src/main/cpp/ringbuffer3DExperimental/RingBuffer3D.h
    src/main/cpp/ringbuffer3DExperimental/RingBuffer3D.cpp
    src/main/cpp/ringbuffer3DExperimental/RingBuffer3DJNI.cpp
    
	src/main/cpp/octomapcustomization/NavigationNode.h
    src/main/cpp/octomapcustomization/NavigationNode.cpp
    src/main/cpp/octomapcustomization/NavigationOcTree.h
    src/main/cpp/octomapcustomization/NavigationOcTree.cpp
    )

# OCTOMAP_OMP = enable OpenMP parallelization (experimental, defaults to OFF)
set(OCTOMAP_OMP TRUE CACHE BOOL "Enable/disable OpenMP parallelization")
if(DEFINED ENV{OCTOMAP_OMP})
    set(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
endif(DEFINED ENV{OCTOMAP_OMP})
if(OCTOMAP_OMP)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
	find_package( OpenMP REQUIRED)
endif(OCTOMAP_OMP)

include_directories(src/main/cpp/octomap/include/math)
add_subdirectory(src/main/cpp/octomap/src/math)

include_directories(src/main/cpp/octomap/include)
add_subdirectory(src/main/cpp/octomap/src)

add_library(octomapjni SHARED ${SOURCE_FILES})

add_dependencies(octomapjni octomap octomath)
target_link_libraries(octomapjni octomath octomap android log)