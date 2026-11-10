@echo off
setlocal EnableDelayedExpansion
pushd "%~dp0"

REM Locate MSYS2 UCRT64 toolchain (checking %MSYS2_ROOT%, standard paths, then PATH)
set "UCRT_BIN="
if defined MSYS2_ROOT (
    if exist "%MSYS2_ROOT%\ucrt64\bin\g++.exe" set "UCRT_BIN=%MSYS2_ROOT%\ucrt64\bin"
)
if not defined UCRT_BIN (
    if exist "C:\msys64\ucrt64\bin\g++.exe" set "UCRT_BIN=C:\msys64\ucrt64\bin"
)
if not defined UCRT_BIN (
    if exist "D:\msys64\ucrt64\bin\g++.exe" set "UCRT_BIN=D:\msys64\ucrt64\bin"
)
if not defined UCRT_BIN (
    for /f "delims=" %%I in ('where g++ 2^>nul') do (
        if not defined UCRT_BIN set "UCRT_BIN=%%~dpI"
    )
)
if defined UCRT_BIN set "PATH=%UCRT_BIN%;%PATH%"

if /I "%~1"=="test" (
    echo Building and running offline verification suite ^(tests.cpp^)...
    g++ -std=c++17 -O3 -Wall -Wextra tests.cpp -o subhz_tests.exe ^
      -static-libgcc -static-libstdc++ ^
      -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive,-Bdynamic ^
      -lwinmm
    if !ERRORLEVEL! neq 0 (
        echo Test build failed with error !ERRORLEVEL!
        popd
        exit /b !ERRORLEVEL!
    )
    .\subhz_tests.exe
    set "TEST_EXIT=!ERRORLEVEL!"
    del /Q subhz_tests.exe 2>nul
    popd
    exit /b !TEST_EXIT!
)

echo Building SubHz...
g++ -std=c++17 -O3 -Wall -Wextra main.cpp -o SubHz.exe ^
  -static-libgcc -static-libstdc++ ^
  -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive,-Bdynamic ^
  -lraylib -lopengl32 -lgdi32 -lwinmm

if !ERRORLEVEL! equ 0 (
    if defined UCRT_BIN (
        if not exist "libraylib.dll" if exist "%UCRT_BIN%\libraylib.dll" copy /Y "%UCRT_BIN%\libraylib.dll" . >nul
        if not exist "glfw3.dll"     if exist "%UCRT_BIN%\glfw3.dll"     copy /Y "%UCRT_BIN%\glfw3.dll" . >nul
    )
    echo Build succeeded: SubHz.exe
) else (
    echo Build failed with error !ERRORLEVEL!
    popd
    exit /b !ERRORLEVEL!
)
popd
endlocal
