# GameEngine

A **lightweight 3D game engine** built with **C++20** and **Vulkan**. The codebase is written like a hands-on tutorial: almost every file has verbose comments explaining *what* each piece does and *why* it exists in a Vulkan renderer.

## What you get

- **Vulkan Renderer:** GLFW window, validation layers, swapchain, and graphics pipeline.
- **Physics Engine:** Simple collision detection and rigid body simulation (static, dynamic, kinematic).
- **Game Loop:** Decoupled update and render loops with fixed-timestep physics.
- **Level System:** Structured level loading and scene configuration via `LevelManager` and `SceneFactory`.
- **Asset Loading:** Wavefront `.obj` (with `.mtl`) and glTF 2.0 support.
- **Player Controller:** First-person style movement and interaction.


```bash
./build/Game.exe
```

Camera controls in the viewer:
- `W` / `S`: move forward / backward
- `A` / `D`: strafe left / right
- `E` / `Q`: move up / down
- Right mouse button + drag: look around

Quick check without opening the window:

```bash
./build/Game
```

The loader triangulates face lines (`f`) with a fan, reads **`mtllib` / `usemtl`**, and loads companion **`.mtl`** files (`Kd`, `Ks`, `Ns`, …). Shading uses a simple **Blinn-Phong** model per material. Models are **centered** (not squashed), oriented from Blender **Y-up** to **Z-up**, and the camera is fitted to the mesh bounding sphere. A **depth buffer** sorts front/back faces correctly. OBJ texture maps (`map_Kd`) are not loaded yet; the glTF loader reads base color factors but does not sample external image textures yet.

## Project layout

```
assets/                  # runtime assets loaded by the engine
  audio/
  fonts/
  models/
  shaders/
    basic.vert / .frag   # default mesh vertex + fragment shaders
    ui.vert / .frag      # overlay/debug UI shaders
  textures/
build/                   # out-of-source CMake build output (.spv, binaries)
include/
  engine/
    Engine.h             # top-level game loop owner
    Window.h             # GLFW window wrapper
    asset/
      AssetLoader.h      # unified loader interface for all asset types
      AssetManager.h     # caches and manages loaded assets (meshes, textures, materials)
      AssetManifest.h    # describes available assets in the project
      TextureData.h      # raw pixel data + format metadata
      TextureLoader.h    # loads PNG/JPEG via stb_image
    math/
      Types.h            # Vec3, Mat4, and other math type aliases
    mesh/
      GltfMeshLoader.h   # glTF 2.0 mesh loader (cgltf)
      Material.h         # Blinn-Phong material definition (Kd, Ks, Ns)
      MeshData.h         # vertex/index buffers + draw info
      MtlLoader.h        # .mtl file parser (Kd/Ks/Ns/ambient/diffuse/specular)
      ObjMeshLoader.h    # Wavefront OBJ loader with triangulation
    physics/
      PhysicsEngine.h    # simple collision/simulation engine
    scene/
      SceneGraph.h       # hierarchical node graph for transforms + meshes
    vulkan/
src/
  game/
    main.cpp             # entry point (Game executable)
    Game.cpp / .h        # high-level game state machine + loop
    Level.cpp / .h       # a playable scene / stage
    LevelManager.cpp / .h # loads and switches between levels
  engine/
    Engine.cpp         # wires Window + VulkanRenderer + PhysicsEngine
    Window.cpp         # GLFW window creation, resize, input polling
    FrameTimer.cpp     # delta time and fixed timestep tracking
    asset/
        AssetLoader.cpp  # dispatches to the right loader by file type
        AssetManager.cpp # in-memory caching of loaded assets
        AssetManifest.cpp # parses manifest.json for available content
        TextureLoader.cpp # stb_image wrapper + VkImage upload path
      mesh/
        Material.cpp     # converts Material → Vulkan push constant / UBO
        MeshData.cpp     # builds vertex/index buffer layouts
        MtlLoader.cpp    # .mtl parser implementation
        ObjMeshLoader.cpp # OBJ file reader with fan triangulation
        GltfMeshLoader.cpp # glTF 2.0 mesh importer (cgltf)
      physics/
        PhysicsEngine.cpp # collision detection / simulation loop
      scene/
        SceneGraph.cpp   # node hierarchy transform + cull logic
      vulkan/
        VulkanContext.cpp    # instance, validation layers, surface
        VulkanDevice.cpp     # physical/logical device selection
        VulkanSwapchain.cpp  # swapchain creation + image views
        VulkanPipeline.cpp   # graphics pipeline + descriptor sets
        VulkanBuffer.cpp     # vertex/index/uniform buffer management
        VulkanRenderer.cpp   # frame rendering, command buffers
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
./build/Game         # Linux / macOS
.\build\Game.exe    # Windows (MinGW generator)
```

Press **Escape** or close the window to quit.

## Next steps (ideas)

- Texture sampling (use `vt` from OBJ + `.mtl`)
- Descriptor sets for materials
- Advanced lighting (multiple lights, shadows)
- UI system (ImGui integration)

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

## Roadmap to a Playable Game

This project has evolved from a simple renderer into a basic engine. Below is the progress checklist.

- [x] **Build & Run Baseline:** Project builds and runs with Vulkan.
- [x] **Input & Game Loop:** Fixed-timestep update loop and centralized input.
- [x] **Physics & Collision:** Integrated physics engine with collision events.
- [x] **Scene / Entity System:** Lightweight scene management and level loading.
- [x] **Sample Level:** Test levels with physics interactions.
- [ ] **Renderer Polish:** Material/descriptor-system, batching, and shader hot-reload.
- [ ] **Stabilize Asset Pipeline:** Robust path resolution and asset manifest.
- [ ] **UI & HUD:** Minimal overlay for gameplay information.
- [ ] **Gameplay Loop:** Objectives, scoring, and state transitions.

## License

MIT License

Copyright (c) 2024-2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

