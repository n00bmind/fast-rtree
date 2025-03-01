@echo off

if not exist "bin" mkdir bin
pushd bin

set DBG_FLAGS=-DCONFIG_DEBUG=1 -Z7 -MTd -Od
set REL_FLAGS=-DCONFIG_RELEASE=1 -Z7 -MT -O2 -GL

set CFG_FLAGS=%DBG_FLAGS%
if "%1" == "release" (
    set CFG_FLAGS=%REL_FLAGS%
)

cl /nologo ../src/test.cpp /D_HAS_EXCEPTIONS=0 %CFG_FLAGS%
cl /nologo ../src/bench.cpp /D_HAS_EXCEPTIONS=0 %CFG_FLAGS%

popd
