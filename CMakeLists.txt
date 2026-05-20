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
    GIT_TAG        1.0.1
)
FetchContent_MakeAvailable(glm)

find_package(Vulkan REQUIRED)

# -----------------------------------------------------------------------------
# Shader compilation (GLSL -> SPIR-V via Vulkan SDK's glslc)
# -----------------------------------------------------------------------------

set(SHADER_SOURCES
    ${CMAKE_SOURCE_DIR}/shaders/basic.vert
    ${CMAKE_SOURCE_DIR}/shaders/basic.frag
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
# Engine executable
# -----------------------------------------------------------------------------

set(ENGINE_SOURCES
    src/main.cpp
    src/engine/mesh/MeshData.cpp
    src/engine/mesh/Material.cpp
    src/engine/mesh/MtlLoader.cpp
    src/engine/mesh/ObjMeshLoader.cpp
    src/engine/Engine.cpp
    src/engine/Window.cpp
    src/engine/vulkan/VulkanContext.cpp
    src/engine/vulkan/VulkanDevice.cpp
    src/engine/vulkan/VulkanSwapchain.cpp
    src/engine/vulkan/VulkanPipeline.cpp
    src/engine/vulkan/VulkanBuffer.cpp
    src/engine/vulkan/VulkanRenderer.cpp
)

add_executable(GameEngine ${ENGINE_SOURCES})
add_dependencies(GameEngine CompileShaders)

target_include_directories(GameEngine PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${Vulkan_INCLUDE_DIRS}
)

target_link_libraries(GameEngine PRIVATE
    glfw
    glm::glm
    Vulkan::Vulkan
)

# Shaders are loaded from CMAKE_BINARY_DIR/shaders at runtime.
target_compile_definitions(GameEngine PRIVATE
    SHADER_DIR="${CMAKE_BINARY_DIR}/shaders"
)

if(MSVC)
    target_compile_options(GameEngine PRIVATE /W4 /permissive-)
else()
    target_compile_options(GameEngine PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Debug builds enable Vulkan validation layers in code (see VulkanContext.cpp).
target_compile_definitions(GameEngine PRIVATE
    $<$<CONFIG:Debug>:GE_DEBUG=1>
)
