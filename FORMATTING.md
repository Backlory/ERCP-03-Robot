# C/C++ formatting

This repository formats project-owned C/C++ sources with clang-format 22.1.3.

```powershell
.\format-cpp.ps1 check
.\format-cpp.ps1 apply
```

The script uses `clang-format` from `PATH`, the Visual Studio bundled copy, or the executable
specified by `ERCP_CLANG_FORMAT_EXE`. It fails if the version is not exactly 22.1.3.

`.clang-format-ignore` is the formatting boundary. It excludes third-party code and every
`[SHARED-WIRE]` distributed copy. Shared-wire files must be changed in the canonical
`shared-wire` repository and distributed with `sync.bat`; never format a project copy directly.

Keep formatting-only changes separate from functional changes. Run the complete build and wire
golden tests after a repository-wide formatting update.
