@echo off

setlocal enabledelayedexpansion

if "%1"=="" (
  echo ERROR: Too few arguments.
  echo USAGE: %~0 [csv path] [--output dir]
  exit /b 1
)

set CSV=%~n1%~x1
set CSV_P=%~d1%~p1


if "%2"=="" (
    set DIR=%~dp0
) else (
    set DIR=%~d2%~p2
)
@echo %CSV%
@echo %CSV_P%
@echo %DIR%

docker run -it --rm ^
-v "%CSV_P%:/tmp_in/" ^
-v "%DIR%:/tmp/" ^
-e DISPLAY=192.168.80.1:0.0 ^
labansuite:latest ^
/bin/bash -c "source activate labansuite && python main.py --alg parallel --inputfile /tmp_in/%CSV% --outputfolder /tmp/ --nogui"