# rl_host - Visual Studio 2022 Build

This project is configured for a CMake workflow using:

- **Generator:** `Visual Studio 17 2022`
- **Configurations:** `Debug`, `Release` (and the standard Visual Studio configs)

## Generate the build tree

Open a **Developer Command Prompt for VS 2022** and run:

```bat
cmake -S . -B build -G "Visual Studio 17 2022"
```

## Build Debug

```bat
cmake --build build --config Debug
```

Output:

```text
build\Debug\rl_host.exe
```

## Build Release

```bat
cmake --build build --config Release
```

Output:

```text
build\Release\rl_host.exe
```

## Open in Visual Studio

You can also open the generated solution directly:

```text
build\rl_host.sln
```

## Notes

- This is a **single build tree** with multiple configurations.
- You do **not** need separate `build/debug` and `build/release` directories when using a multi-config generator.
- `ws2_32` is linked automatically on Windows for Rigol TCP socket support.
- The source files are grouped in Solution Explorer for easier navigation.
