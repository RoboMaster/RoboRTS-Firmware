cmake_minimum_required (VERSION 2.8)
project (###PROJECT_NAME###)

ENABLE_LANGUAGE(ASM)


set(DIR_SRCS
###SRC_FILE###
)

INCLUDE_DIRECTORIES(
###INC_DIR###
)


set(CMAKE_EXE_LINKER_FLAGS
"--cpu Cortex-M4.fp --library_type=microlib --strict --scatter \"infantry.sct\" --summary_stderr --info summarysizes --map --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers"
)
set(CMAKE_C_FLAGS
"--c99 -c --cpu Cortex-M4.fp -D__MICROLIB -g -O3 --apcs=interwork --split_sections -D__UVISION_VERSION=\"524\" -D_RTE_ -DSTM32F427xx -DUSE_HAL_DRIVER -DSTM32F427xx -DARM_MATH_CM4"
)
set(CMAKE_ASM_FLAGS
"--cpu Cortex-M4.fp -g --apcs=interwork --pd \"__MICROLIB SETA 1\" --pd \"__UVISION_VERSION SETA 524\" --pd \"_RTE_ SETA 1\" --pd \"STM32F427xx SETA 1\"")


add_executable(###PROJECT_NAME### ${DIR_SRCS})

set_target_properties(
    ###PROJECT_NAME### 
    PROPERTIES 
        OUTPUT_NAME "###PROJECT_NAME###"
        SUFFIX ".axf"
)

add_custom_command(TARGET ###PROJECT_NAME### POST_BUILD
    COMMENT "Generationg .bin ....."
    COMMAND C:/Keil_v5/ARM/ARMCC/bin/fromelf.exe  --bin -o ###PROJECT_NAME###.bin ###PROJECT_NAME###.axf
)