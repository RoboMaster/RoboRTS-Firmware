# the name of the target operating system
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_VERSION   1)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# which compilers to use for C and C++
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# core flags
set(CORE_FLAGS "-mthumb -mcpu=cortex-m4 -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork -fno-strict-aliasing --specs=nano.specs --specs=nosys.specs ${ADDITIONAL_CORE_FLAGS}")

# compiler: language specific flags
set(CMAKE_C_FLAGS "${CORE_FLAGS} -fno-builtin -Wall -std=gnu99 -fdata-sections -ffunction-sections -g -o -gdwarf-2" CACHE INTERNAL "c compiler flags")
set(CMAKE_C_FLAGS_DEBUG "" CACHE INTERNAL "c compiler flags: Debug")
set(CMAKE_C_FLAGS_RELEASE "" CACHE INTERNAL "c compiler flags: Release")

set(CMAKE_CXX_FLAGS "${CORE_FLAGS} -fno-rtti -fno-exceptions -fno-builtin -Wall -std=gnu++11 -fdata-sections -ffunction-sections -g -ggdb3" CACHE INTERNAL "cxx compiler flags")
set(CMAKE_CXX_FLAGS_DEBUG "" CACHE INTERNAL "cxx compiler flags: Debug")
set(CMAKE_CXX_FLAGS_RELEASE "" CACHE INTERNAL "cxx compiler flags: Release")

set(CMAKE_ASM_FLAGS "${CORE_FLAGS} -g -o -ggdb3 -x assembler-with-cpp -D__USES_CXX" CACHE INTERNAL "asm compiler flags")
set(CMAKE_ASM_FLAGS_DEBUG "" CACHE INTERNAL "asm compiler flags: Debug")
set(CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm compiler flags: Release")

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# find additional toolchain executables
find_program(ARM_SIZE_EXECUTABLE arm-none-eabi-size)
find_program(ARM_GDB_EXECUTABLE arm-none-eabi-gdb)
find_program(ARM_OBJCOPY_EXECUTABLE arm-none-eabi-objcopy)
find_program(ARM_OBJDUMP_EXECUTABLE arm-none-eabi-objdump)
