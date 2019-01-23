if(_ARMCC_CMAKE_LOADED)
  return()
endif()
set(_ARMCC_CMAKE_LOADED TRUE)

# See ARM Compiler documentation at:
# http://infocenter.arm.com/help/topic/com.arm.doc.set.swdev/index.html

set(CMAKE_C_COMPILER "C:/Keil_v5/ARM/ARMCC/bin/armcc.exe")
set(CMAKE_CXX_COMPILER "C:/Keil_v5/ARM/ARMCC/bin/armcc.exe")
set(CMAKE_ASM_COMPILER "C:/Keil_v5/ARM/ARMCC/bin/armasm.exe")

get_filename_component(_CMAKE_C_TOOLCHAIN_LOCATION "${CMAKE_C_COMPILER}" PATH)
get_filename_component(_CMAKE_CXX_TOOLCHAIN_LOCATION "${CMAKE_CXX_COMPILER}" PATH)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

find_program(CMAKE_ARMCC_LINKER armlink HINTS "${_CMAKE_C_TOOLCHAIN_LOCATION}" "${_CMAKE_CXX_TOOLCHAIN_LOCATION}" )
find_program(CMAKE_ARMCC_AR     armar   HINTS "${_CMAKE_C_TOOLCHAIN_LOCATION}" "${_CMAKE_CXX_TOOLCHAIN_LOCATION}" )
find_program(CMAKE_ARMCC_ASM     armasm   HINTS "${_CMAKE_C_TOOLCHAIN_LOCATION}" "${_CMAKE_CXX_TOOLCHAIN_LOCATION}" )
set(CMAKE_LINKER "${CMAKE_ARMCC_LINKER}" CACHE FILEPATH "The ARMCC linker" FORCE)

mark_as_advanced(CMAKE_ARMCC_LINKER)
set(CMAKE_AR "${CMAKE_ARMCC_AR}" CACHE FILEPATH "The ARMCC archiver" FORCE)
mark_as_advanced(CMAKE_ARMCC_AR)
set(CMAKE_ASM "${CMAKE_ARMCC_ASM}" CACHE FILEPATH "The ARMCC asm" FORCE)
mark_as_advanced(CMAKE_ARMCC_ASM)

macro(__compiler_armcc lang)  
# 上面是一个宏，可以用宏里面定义的变量替换下面引用的地方
# 但是这个宏的调用时机还不清楚
  string(APPEND CMAKE_${lang}_FLAGS_INIT " ")
  string(APPEND CMAKE_${lang}_FLAGS_DEBUG_INIT " -g")
  string(APPEND CMAKE_${lang}_FLAGS_MINSIZEREL_INIT " -Ospace -DNDEBUG")
  string(APPEND CMAKE_${lang}_FLAGS_RELEASE_INIT " -Otime -DNDEBUG")
  string(APPEND CMAKE_${lang}_FLAGS_RELWITHDEBINFO_INIT " -O2 -g")

  set(CMAKE_${lang}_OUTPUT_EXTENSION ".o")
  set(CMAKE_${lang}_OUTPUT_EXTENSION_REPLACE 1)
  set(CMAKE_${lang}_RESPONSE_FILE_LINK_FLAG "--via=")

  set(CMAKE_${lang}_LINK_EXECUTABLE      "<CMAKE_LINKER> <CMAKE_${lang}_LINK_FLAGS> <LINK_FLAGS> <LINK_LIBRARIES> <OBJECTS> -o <TARGET> --list <TARGET_BASE>.map")
  set(CMAKE_${lang}_CREATE_STATIC_LIBRARY  "<CMAKE_AR> --create -cr <TARGET> <LINK_FLAGS> <OBJECTS>")

  set(CMAKE_DEPFILE_FLAGS_${lang} "--depend=<DEPFILE> --depend_single_line --no_depend_system_headers")

  set(CMAKE_${lang}_LINKER_WRAPPER_FLAG "-Xlinker" " ")
endmacro()
