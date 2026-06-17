# =============================================================================
# GameEngine — CMake build
# =============================================================================
# Fetches GLFW (window) and GLM (math) via CMake FetchContent.
# Finds the Vulkan SDK and compiles GLSL shaders to SPIR-V at build time.
# =============================================================================

cmake_minimum_required(VERSION 3.20)
project(GameEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# Dependencies (downloaded on first configure)
# -----------------------------------------------------------------------------

include(FetchContent)

# GLFW — creates a window and a Vulkan-compatible surface (VkSurfaceKHR).
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# GLM — header-only math (matrices for 3D transforms).
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.2
)
FetchContent_MakeAvailable(glm)

# cgltf — small non-deprecated glTF 2.0 loader header.
FetchContent_Declare(
    cgltf
    GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
    GIT_TAG        v1.15
)
FetchContent_MakeAvailable(cgltf)

# stb — image loading required by TextureLoader.
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(stb)

find_package(Vulkan REQUIRED)

# Vulkan is linked to the engine library later.

# -----------------------------------------------------------------------------
# Shader compilation (GLSL -> SPIR-V via Vulkan SDK's glslc)
# -----------------------------------------------------------------------------

set(SHADER_SOURCES
    ${CMAKE_SOURCE_DIR}/assets/shaders/basic.vert
    ${CMAKE_SOURCE_DIR}/assets/shaders/basic.frag
    ${CMAKE_SOURCE_DIR}/assets/shaders/ui.vert
    ${CMAKE_SOURCE_DIR}/assets/shaders/ui.frag
)

set(SHADER_SPV "")
foreach(SHADER ${SHADER_SOURCES})
    get_filename_component(SHADER_NAME ${SHADER} NAME)
    set(SPV ${CMAKE_BINARY_DIR}/shaders/${SHADER_NAME}.spv)
    add_custom_command(
        OUTPUT ${SPV}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/shaders
        COMMAND glslc ${SHADER} -o ${SPV}
        DEPENDS ${SHADER}
        COMMENT "Compiling ${SHADER_NAME} -> SPIR-V"
    )
    list(APPEND SHADER_SPV ${SPV})
endforeach()

add_custom_target(CompileShaders ALL DEPENDS ${SHADER_SPV})

# -----------------------------------------------------------------------------
# Engine library
# -----------------------------------------------------------------------------

set(ENGINE_SOURCES
    src/engine/lib/mesh/MeshData.cpp
    src/engine/lib/mesh/Material.cpp
    src/engine/lib/mesh/MtlLoader.cpp
    src/engine/lib/mesh/ObjMeshLoader.cpp
    src/engine/lib/mesh/GltfMeshLoader.cpp
    src/engine/lib/asset/TextureLoader.cpp
    src/engine/lib/asset/AssetLoader.cpp
    src/engine/lib/asset/AssetManager.cpp
    src/engine/lib/asset/AssetManifest.cpp
    src/engine/lib/scene/SceneGraph.cpp
    src/engine/lib/Engine.cpp
    src/engine/lib/Window.cpp
    src/engine/lib/vulkan/VulkanContext.cpp
    src/engine/lib/vulkan/VulkanDevice.cpp
    src/engine/lib/vulkan/VulkanSwapchain.cpp
    src/engine/lib/vulkan/VulkanPipeline.cpp
    src/engine/lib/vulkan/VulkanBuffer.cpp
    src/engine/lib/vulkan/VulkanRenderer.cpp
    src/engine/lib/physics/PhysicsEngine.cpp
)

add_library(game_engine_lib STATIC ${ENGINE_SOURCES})
add_dependencies(game_engine_lib CompileShaders)

if (NOT DEFINED Vulkan_INCLUDE_DIRS)
    set(Vulkan_INCLUDE_DIRS "$ENV{VULKAN_SDK}/Include")
endif()

target_include_directories(game_engine_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${Vulkan_INCLUDE_DIRS}
    ${cgltf_SOURCE_DIR}
    ${stb_SOURCE_DIR}
    ${glfw_SOURCE_DIR}/include
    ${glm_SOURCE_DIR}
)

target_link_libraries(game_engine_lib PUBLIC)


target_compile_definitions(game_engine_lib PUBLIC
    SHADER_DIR="${CMAKE_BINARY_DIR}/shaders"
    $<$<CONFIG:Debug>:GE_DEBUG=1>
)

if(MSVC)
    target_compile_options(game_engine_lib PUBLIC /W4 /permissive-)
else()
    target_compile_options(game_engine_lib PUBLIC -Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Engine applications
# -----------------------------------------------------------------------------

set(GAME_SOURCES
    src/game/main.cpp
    src/game/Game.cpp
    src/game/Level.cpp
    src/game/LevelManager.cpp
)

add_executable(Game ${GAME_SOURCES})
add_dependencies(Game CompileShaders)
target_link_libraries(Game PRIVATE game_engine_lib glfw glm::glm Vulkan::Vulkan)

if(MSVC)
    target_compile_options(Game PRIVATE /W4 /permissive-)
else()
    target_compile_options(Game PRIVATE -Wall -Wextra -Wpedantic)
endif()
