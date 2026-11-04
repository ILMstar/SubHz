@echo off
setlocal
echo Building SubHz...

set "PATH=C:\msys64\ucrt64\bin;%PATH%"

g++ -std=c++20 -O2 -Wall -Wextra main.cpp -o SubHz.exe ^
  -static-libgcc -static-libstdc++ -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic ^
  -lraylib -lopengl32 -lgdi32 -lwinmm

if %ERRORLEVEL% equ 0 (
    echo Build succeeded: SubHz.exe
) else (
    echo Build failed with error %ERRORLEVEL%
)
endlocal
