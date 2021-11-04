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

string(CONCAT C_FLAGS
    " -Wall"
    " -Werror"
    " -Wno-unused-variable"
    " -Wno-unused-but-set-variable" # HW register read
    " -Werror=implicit-function-declaration"
)

string(CONCAT CXX_FLAGS
    " -Wall"
    " -Werror"
    " -Wno-unused-variable"
    " -Wno-unused-but-set-variable" # HW register read
)

set(CMAKE_TARGET_FLAGS "-mcpu=arm7tdmi")
set(CMAKE_ASM_FLAGS "${CMAKE_TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_TARGET_FLAGS} ${C_FLAGS} -O0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffreestanding -fsigned-char -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_TARGET_FLAGS} ${CXX_FLAGS} -fno-rtti -fno-common -fno-exceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14")

set(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nosys.specs -nostartfiles -lc -lgcc -lstdc++ -lm")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--cref -Wl,--gc-sections")

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
