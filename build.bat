@echo off

if not exist "bin" mkdir bin
pushd bin

set COMMON_FLAGS=-D_CRT_SECURE_NO_WARNINGS -D_HAS_EXCEPTIONS=0
set DBG_FLAGS=-DCONFIG_DEBUG=1 -Z7 -MTd -Od
set REL_FLAGS=-DCONFIG_RELEASE=1 -Z7 -MT -O2 -GL

set CC=cl
set CFG_FLAGS=%DBG_FLAGS%
if "%1" == "release" (
    set CC=clang-cl
    set CFG_FLAGS=%REL_FLAGS%
)

%CC% /nologo ../src/test.cpp %COMMON_FLAGS% %CFG_FLAGS% /link /incremental:no /debug:full /LTCG
%CC% /nologo ../src/bench.cpp %COMMON_FLAGS% %CFG_FLAGS% /link /incremental:no /debug:full /LTCG

popd
