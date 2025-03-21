# CMSIS Build CMakeLists generated on 2024-04-03T15:22:27

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET MC_Praktikum)
set(CPU Cortex-M4)
set(PRJ_DIR "/Users/oledonnermeyer/Documents/coding/mc-praktikum")
set(OUT_DIR "/Users/oledonnermeyer/Documents/coding/mc-praktikum/out/MC_Praktikum/Discocery_Board__Flash")
set(INT_DIR "/Users/oledonnermeyer/Documents/coding/mc-praktikum/tmp/MC_Praktikum/Discocery_Board__Flash")
set(FPU SP_FPU)
set(BYTE_ORDER Little-endian)
set(OPTIMIZE none)
set(DEBUG on)
set(AS_FLAGS_GLOBAL "-masm=auto")
set(CC_FLAGS_GLOBAL "-std=c99 -fno-rtti -funsigned-char -ffunction-sections -Weverything -Wno-packed -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(CXX_FLAGS_GLOBAL "-xc++ -std=c++11 -fno-rtti -funsigned-char -ffunction-sections -Weverything -Wno-packed -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(LD_FLAGS_GLOBAL "--map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --strict --summary_stderr --info summarysizes")
set(LD_SCRIPT "/Users/oledonnermeyer/Documents/coding/mc-praktikum/MC_Praktikum_Discocery_Board__Flash.sct")

set(DEFINES
  STM32F407xx
  _RTE_
)

set(INC_PATHS
  "/Users/oledonnermeyer/Documents/coding/mc-praktikum/RTE/_Discocery_Board__Flash"
  "/Users/oledonnermeyer/.cache/arm/packs/ARM/CMSIS/5.9.0/CMSIS/Core/Include"
  "/Users/oledonnermeyer/.cache/arm/packs/Keil/STM32F4xx_DFP/2.17.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
)

set(AS_ARM_SRC_FILES
  "/Users/oledonnermeyer/Documents/coding/mc-praktikum/RTE/Device/STM32F407VGTx/startup_stm32f407xx.s"
)

set(CC_SRC_FILES
  "/Users/oledonnermeyer/Documents/coding/mc-praktikum/RTE/Device/STM32F407VGTx/system_stm32f4xx.c"
  "/Users/oledonnermeyer/Documents/coding/mc-praktikum/_mcpr_stm32f407.c"
  "/Users/oledonnermeyer/Documents/coding/mc-praktikum/main.c"
)

# Toolchain config map

set(REGISTERED_TOOLCHAIN_ROOT "/Users/oledonnermeyer/.vcpkg/artifacts/2139c4c6/compilers.arm.armclang/6.22.0/bin")
set(REGISTERED_TOOLCHAIN_VERSION "6.22.0")
set(TOOLCHAIN_VERSION_MIN "6.22.0")
include ("/Users/oledonnermeyer/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.2.1/etc/AC6.6.18.0.cmake")
include ("/Users/oledonnermeyer/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.2.1/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES AS_ARM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_AS_ARM_FLAGS "${AS_ARM_CPU} ${AS_ARM_BYTE_ORDER} ${AS_ARM_DEFINES} ${AS_ARM_OPTIONS_FLAGS} ${AS_ARM_FLAGS} ${AS_FLAGS_GLOBAL}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_BYTE_ORDER} ${CC_DEFINES} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Local Flags

foreach(SRC ${AS_ARM_SRC_FILES})
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE AS_ARM)
endforeach()

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${AS_ARM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "MC_Praktikum")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
