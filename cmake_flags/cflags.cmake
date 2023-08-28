macro(alica_compile_flags)
set(WARNING "-Wall -Wuninitialized -Wmissing-field-initializers -Wno-reorder -Wpedantic -Wextra -Wno-format -Wno-unused-parameter")
set(C_ERROR "-Werror=implicit-function-declaration -Werror=return-type -Werror=pointer-arith")
set(CPP_ERROR "-Werror -Wno-error=deprecated-declarations")
set(C_WARNING "-Wbad-function-cast -Wstrict-prototypes")
set(CPP_WARNING "")
set(OPTIMIZATION "-O2 -finline-functions -fomit-frame-pointer -ffast-math -ftree-vectorize")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

if(${SANITATION})
    set(OPTIMIZATION "-O0") # For memory sanitation we turn off optimization.
    set(SANITIZE "-fsanitize=memory -fPIE -fno-omit-frame-pointer -fsanitize-memory-track-origins=2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SANITIZE}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZE}")
endif()


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG ${OPTIMIZATION} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG ${OPTIMIZATION} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -g -DNDEBUG ${OPTIMIZATION} ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g -DNDEBUG ${OPTIMIZATION} ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -fnon-call-exceptions -ggdb ${WARNING} ${C_ERROR} ${C_WARNING}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -fnon-call-exceptions -ggdb ${WARNING} ${CPP_ERROR} ${CPP_WARNING}")

endmacro(alica_compile_flags)
