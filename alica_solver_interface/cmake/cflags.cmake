set(WARNING "-Wall -Wuninitialized -Wmissing-field-initializers -Wno-reorder")
set(C_ERROR "-Werror=implicit-function-declaration -Werror=return-type -Werror=pointer-arith")
set(CPP_ERROR "-Werror=return-type -Werror=pointer-arith")
set(C_WARNING "-Wbad-function-cast -Wstrict-prototypes")
set(CPP_WARNING "")
set(C_STD "-std=c11")
set(CPP_STD "-std=c++14")
set(OPTIMIZATION "-O2 -finline-functions -fomit-frame-pointer -funroll-loops -ffast-math -ftree-vectorize")

if(${SANITATION})
    set(OPTIMIZATION "-O0") # For memory sanitation we turn off optimization.
    set(SANITIZE "-fsanitize=memory -fPIE -fno-omit-frame-pointer -fsanitize-memory-track-origins=2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SANITIZE}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZE}")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_STD} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPP_STD} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG ${C_STD} ${OPTIMIZATION} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG ${CPP_STD} ${OPTIMIZATION} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -g -DNDEBUG ${C_STD} ${OPTIMIZATION} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g -DNDEBUG ${CPP_STD} ${OPTIMIZATION} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -fnon-call-exceptions -ggdb ${C_STD} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -fnon-call-exceptions -ggdb ${CPP_STD} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")
