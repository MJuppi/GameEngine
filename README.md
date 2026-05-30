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

If you run with **no arguments** from the project root and `assets/models/SuomiKP.obj` exists, that model is loaded automatically.

Camera controls in the viewer:
- `W` / `S`: move forward / backward
- `A` / `D`: strafe left / right
- `E` / `Q`: move up / down
- Right mouse button + drag: look around

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

This project uses an out-of-source CMake build and compiles GLSL shaders from `assets/shaders/` into SPIR-V under `build/shaders`.

First time building on Windows with MinGW:
```powershell
$env:VULKAN_SDK = "C:\VulkanSDK\1.4.350.0"
$env:PATH = "$env:VULKAN_SDK\Bin;C:\msys64\ucrt64\bin;$env:PATH"
cmake -B build -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_CXX_COMPILER="C:/msys64/ucrt64/bin/g++.exe" `
  -DCMAKE_MAKE_PROGRAM="C:/msys64/ucrt64/bin/mingw32-make.exe"
```

After the first build:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j 8
```

Run from the project root so the asset-relative paths resolve correctly:

```bash
./build/GameEngine        # Linux / macOS
build\GameEngine.exe      # Windows (MinGW generator)
```

If you are using the `Game` target instead of `GameEngine`:

```bash
./build/Game              # Linux / macOS
build\Game.exe           # Windows (MinGW generator)
```

Press **Escape** or close the window to quit.

## Next steps (ideas)

- Depth buffer (`VK_FORMAT_D32_SFLOAT`) to sort front/back faces correctly
- Texture sampling (use `vt` from OBJ + `.mtl`)
- Descriptor sets for materials
- Entity / scene graph
- Input system and camera controller

## Recommended project structure

Suggested layout to turn this renderer into a small game project:

```
assets/                  # Runtime assets (bundled with release)
  models/                # .obj, .gltf, or other model sources
  textures/              # PNG/JPEG/etc. (maps used by materials)
  materials/             # .mtl files or material JSONs
  shaders/               # GLSL source (also keep compiled .spv here or in build/)
  audio/                 # sound effects / music
  fonts/                 # bitmap/TTF fonts
src/                     # application & engine source
  engine/                # renderer + window + device + low-level systems
  game/                  # game-specific code: entities, levels, gameplay
  tools/                 # editor/tools (exporters, asset processors)
  common/                # shared utilities, math, types
include/                 # public headers (optional)
third_party/             # pinned external sources (if not using CMake FetchContent)
build/                   # out-of-source CMake build (already present)
tests/                   # unit/integration tests
docs/                    # design notes, asset conventions
```

Guidelines:
- Put all runtime assets under `assets/` and load by relative paths at runtime.
- Keep engine code in `src/engine` and game logic in `src/game` so the engine can be reused.
- Store shader sources in `assets/shaders/` and compile them during the build into SPIR-V in `build/shaders` (or check them into `assets/shaders/bin` for releases).
- Use `third_party/` only if you need to patch upstream; prefer CMake FetchContent for reproducible builds.

## Asset loading conventions

- Search `assets/` first when resolving runtime files. For command-line tools/tests allow absolute paths too.
- Normalized model unit: export meshes so that 1 unit == 1 meter (or document chosen convention) and center meshes on origin when appropriate.
- Texture units: use sRGB for color maps and linear for normal/roughness/metallic maps; name textures alongside material entries (e.g. `mesh_albedo.png`, `mesh_normal.png`).

## Project TODOs

Short actionable items to get started building a small game on top of this engine:

- [ ] Draft folder structure and repository layout (move assets into `assets/`)  
- [ ] Organize assets into `assets/models`, `assets/textures`, `assets/shaders`, etc.  
- [ ] Separate engine (`src/engine`) and game (`src/game`) code; add a small `game` target in CMake  
- [ ] Add CMake build targets for runtime asset packaging and shader compilation  
- [ ] Create a minimal game skeleton in `src/game` showing an entry-level scene and input handling  
- [ ] Update README with structure, conventions, and next development steps  

If you want, I can: create the directories, add a minimal `src/game` skeleton, and update `CMakeLists.txt` to build a second target for game code.
