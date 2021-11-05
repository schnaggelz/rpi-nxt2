include (CMakeForceCompiler)

set(CMAKE_CROSSCOMPILING TRUE)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX   arm-none-eabi)

set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_EXE_LINKER   ${TOOLCHAIN_PREFIX}-ld)
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}-objcopy)

string(CONCAT C_CXX_FLAGS
    " -Wall"
    " -Werror"
    " -Wno-unused-variable"
    " -Wno-unused-but-set-variable" # HW register read
    " -ffreestanding"
    " -fsigned-char"
    " -ffunction-sections"
    " -fdata-sections"
    " -O0"
)

string(CONCAT C_FLAGS
    ${C_CXX_FLAGS}
    " -Werror=implicit-function-declaration"
)

string(CONCAT CXX_FLAGS
    ${C_CXX_FLAGS}
    " -Wno-write-strings"
    " -fno-rtti"
    " -fno-common"
    " -fno-exceptions"
    " --std=c++14"
)

string(CONCAT LINKER_FLAGS
    " -nostartfiles"
    " -lc"
    " -lgcc"
    " -lstdc++"
    " -lm"
    " -Wl,--cref"
    " -Wl,--gc-sections"
)

string(CONCAT TARGET_FLAGS
    " -mcpu=arm7tdmi"
)

set(CMAKE_ASM_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${TARGET_FLAGS} ${C_FLAGS}")
set(CMAKE_CXX_FLAGS "${TARGET_FLAGS} ${CXX_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}")

set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_ASM_OUTPUT_EXTENSION ".o")
set(CMAKE_C_OUTPUT_EXTENSION ".o")
set(CMAKE_CXX_OUTPUT_EXTENSION ".o")

# Add a command to generate firmare in binary format
function(generate_object target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Obinary
        ${CMAKE_CURRENT_BINARY_DIR}/${target}
        ${CMAKE_CURRENT_BINARY_DIR}/${target}.bin
    )
endfunction()
