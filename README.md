# GameEngine

A **lightweight 3D game engine** built with **C++20** and **Vulkan**. The codebase is written like a hands-on tutorial: almost every file has verbose comments explaining *what* each piece does and *why* it exists in a Vulkan renderer.

## What you get

- GLFW window + Vulkan surface
- Instance, validation layers (debug builds), physical/logical device, swapchain
- Graphics pipeline with SPIR-V shaders (vertex + fragment)
- Vertex/index buffers, uniform buffer (MVP matrix)
- **Wavefront `.obj` loading** (positions + optional vertex normals for pseudo-color)
- A **rotating colored cube** as the default when no file is passed

## Loading your own 3D models

Pass an **OBJ** file path as the first argument (absolute or relative to your current working directory):

```bash
./build/GameEngine path/to/mesh.obj
```

If you run with **no arguments** from the project root and `models/` contains `.obj` files (e.g. `models/SuomiKP.obj`), that mesh is loaded automatically.

Quick check without opening the window:

```bash
./build/GameEngine --check models/SuomiKP.obj
```

The loader triangulates face lines (`f`) with a fan, reads **`mtllib` / `usemtl`**, and loads companion **`.mtl`** files (`Kd`, `Ks`, `Ns`, …). Shading uses a simple **Blinn-Phong** model per material. Models are **centered** (not squashed), oriented from Blender **Y-up** to **Z-up**, and the camera is fitted to the mesh bounding sphere. A **depth buffer** sorts front/back faces correctly. Texture maps (`map_Kd`) are not loaded yet.

## Project layout

```
src/
  main.cpp                 # Entry point (+ optional OBJ path argv[1])
  engine/
    Engine.*               # Top-level loop: window + renderer
    Window.*               # GLFW wrapper
    mesh/
      Material.*            # Materials
      MeshData.*            # CPU mesh + unit cube + normalize-to-fit
      MtlLoader.*           # Material loader
      ObjMeshLoader.*       # Minimal Wavefront OBJ parser
    math/Types.h           # Vertex layout (position + color)
    vulkan/
      VulkanContext.*      # VkInstance, debug messenger
      VulkanDevice.*         # GPU selection, VkDevice, queues
      VulkanSwapchain.*      # Images, views, render pass, framebuffers
      VulkanPipeline.*       # Pipeline layout, graphics pipeline
      VulkanBuffer.*         # Buffers + memory allocation
      VulkanRenderer.*       # Per-frame recording, draw mesh
shaders/
  basic.vert / basic.frag  # GLSL (compiled to .spv at build time)
```

## Prerequisites

1. **Vulkan SDK** — [https://vulkan.lunarg.com/](https://vulkan.lunarg.com/) (provides `glslc` for shaders)
2. **CMake** 3.20+
3. **C++20** compiler (MSVC, Clang, or GCC)
4. **Git** (CMake fetches GLFW and GLM automatically)

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j 8
```

Run from the project root (so shader paths resolve if you run without installing):

```bash
./build/GameEngine        # Linux / macOS
build\Debug\GameEngine.exe   # Windows (Visual Studio generator)
```

Press **Escape** or close the window to quit.

## Next steps (ideas)

- Depth buffer (`VK_FORMAT_D32_SFLOAT`) to sort front/back faces correctly
- Texture sampling (use `vt` from OBJ + `.mtl`)
- Descriptor sets for materials
- Entity / scene graph
- Input system and camera controller
