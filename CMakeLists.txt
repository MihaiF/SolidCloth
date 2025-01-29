cmake_minimum_required(VERSION 3.25)
project(SolidCloth)

find_package(freeimage REQUIRED)
find_package(imgui REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)
find_package(assimp CONFIG REQUIRED)

set(SOURCES_CLOTH 
	Physics/ClothPatch.cpp
	Physics/ClothModel.cpp
	Physics/ClothCollisionHandler.cpp
	Physics/ClothCollision.cpp
	Physics/ClothPBD.cpp
)

set(SOURCES_GEOM
	Geometry/Mesh.cpp
	Geometry/Collision3D.cpp
	Geometry/SDF.cpp
	Geometry/MarchingCubes.cpp
	Geometry/AabbTree.cpp
	Geometry/ClosestPointOnMesh.cpp
	Geometry/ClosestPointsBetweenMeshes.cpp
	Geometry/VoronoiRegions.cpp
	Math/Quaternion.cpp
)

set(SOURCES
	Graphics2D/TextureGL.cpp
	Graphics2D/Graphics2DGLS.cpp
	Graphics3D/Graphics3DGLS.cpp
	Graphics3D/OpenGLSData.cpp
	Engine/Engine.cpp
	Engine/Profiler.cpp
)

add_executable(ClothDemo WIN32 ${SOURCES} ${SOURCES_GEOM} ${SOURCES_CLOTH} Geometry/Assets.cpp Demo/ClothDemo.cpp Demo/DemoHost.cpp)
add_executable(ClothDemoAPI WIN32 ${SOURCES} ${SOURCES_GEOM} Geometry/Assets.cpp Demo/ClothDemoAPI.cpp Demo/DemoHostAPI.cpp)
add_library(ClothLib SHARED ${SOURCES_CLOTH} ${SOURCES_GEOM} Demo/ClothAPI.cpp)

if(MSVC)
 target_compile_options(ClothLib PRIVATE "/MP")
endif()

target_include_directories(ClothDemo PRIVATE .)
target_include_directories(ClothDemoAPI PRIVATE .)
target_include_directories(ClothLib PRIVATE .)

add_compile_definitions(USE_IMGUI)
target_compile_definitions(ClothLib PRIVATE DISABLE_PROFILER)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(ClothDemo PUBLIC OpenMP::OpenMP_CXX)
endif()

add_dependencies(ClothDemoAPI ClothLib)
target_link_libraries(ClothDemoAPI PRIVATE ClothLib freeimage::FreeImage imgui::imgui OpenGL::GL GLEW::GLEW Freetype::Freetype assimp::assimp)
target_link_libraries(ClothLib PRIVATE imgui::imgui)
target_link_libraries(ClothDemo PRIVATE freeimage::FreeImage imgui::imgui OpenGL::GL GLEW::GLEW Freetype::Freetype assimp::assimp)
