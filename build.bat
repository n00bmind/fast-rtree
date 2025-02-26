@echo off

if not exist "bin" mkdir bin
pushd bin

set DBG_FLAGS=-DCONFIG_DEBUG=1 -Z7 -MTd -Od
set CFG_FLAGS=DBG_FLAGS

cl /nologo ../src/test.cpp /D_HAS_EXCEPTIONS=0 %DBG_FLAGS%

popd
