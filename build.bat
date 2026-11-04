@echo off
setlocal
pushd "%~dp0"
echo Building SubHz...

set "UCRT_BIN=C:\msys64\ucrt64\bin"
set "PATH=%UCRT_BIN%;%PATH%"

g++ -std=c++17 -O3 -Wall -Wextra main.cpp -o SubHz.exe ^
  -static-libgcc -static-libstdc++ ^
  -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive,-Bdynamic ^
  -lraylib -lopengl32 -lgdi32 -lwinmm

if %ERRORLEVEL% equ 0 (
    if not exist "libraylib.dll" if exist "%UCRT_BIN%\libraylib.dll" copy /Y "%UCRT_BIN%\libraylib.dll" . >nul
    if not exist "glfw3.dll"     if exist "%UCRT_BIN%\glfw3.dll"     copy /Y "%UCRT_BIN%\glfw3.dll" . >nul
    echo Build succeeded: SubHz.exe
) else (
    echo Build failed with error %ERRORLEVEL%
    popd
    exit /b %ERRORLEVEL%
)
popd
endlocal
