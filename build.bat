@echo off
setlocal enabledelayedexpansion

:: Check if MSYS2 is installed
if not exist "C:\msys64\usr\bin" (
    echo MSYS2 is not installed. Please install it from https://www.msys2.org
    exit /b 1
)

:: Add MSYS2 to PATH
set "PATH=C:\msys64\usr\bin;%PATH%"

:: Update MSYS2 packages
bash -lc "pacman -Syu --noconfirm"

:: Install required build tools if not present
bash -lc "pacman -S --needed --noconfirm base-devel mingw-w64-x86_64-toolchain"

:: Create build directory
if not exist "build" mkdir build
cd build

:: Configure and build using CMake
bash -lc "cmake -G 'MSYS Makefiles' .."
if errorlevel 1 (
    echo CMake configuration failed
    exit /b 1
)

bash -lc "make -j4"
if errorlevel 1 (
    echo Build failed
    exit /b 1
)

echo Build completed successfully
cd ..
exit /b 0
exit /b 0
