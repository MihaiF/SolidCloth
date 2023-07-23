cmake_minimum_required(VERSION 3.25)
project(SolidCloth)

#find_package(freeimage CONFIG REQUIRED)
#find_package(imgui CONFIG REQUIRED)
#find_package(GLEW REQUIRED)

set(SOURCES 
	Demo/ClothDemo.cpp
	Demo/DemoHost.cpp
	Physics/ClothPatch.cpp
	Physics/ClothModel.cpp
	Physics/ClothCollisionHandler.cpp
	Physics/ClothCollision.cpp
	Physics/ClothPBD.cpp
	Geometry/Collision3D.cpp
	Geometry/SDF.cpp
	Geometry/Mesh.cpp
	Geometry/Assets.cpp
	Geometry/MarchingCubes.cpp
	Geometry/AabbTree.cpp
	Geometry/ClosestPointOnMesh.cpp
	Math/Quaternion.cpp
	Graphics2D/TextureGL.cpp
	Graphics2D/Graphics2DGLS.cpp
	Graphics3D/Graphics3DGLS.cpp
	Graphics3D/OpenGLSData.cpp
	Engine/Engine.cpp
	Engine/Profiler.cpp
)

add_executable(ClothDemo WIN32 ${SOURCES})

target_include_directories(ClothDemo PRIVATE .)

add_compile_definitions(USE_IMGUI)

#target_link_libraries(ClothDemo PRIVATE freeimage::FreeImage freeimage::FreeImagePlus)
