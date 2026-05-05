set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Prefer the STM32Cube bundled GNU Arm toolchain so we do not accidentally
# pick up an incomplete Homebrew installation from PATH.
if(NOT DEFINED STM32CUBE_BUNDLE_PATH)
    if(DEFINED ENV{CUBE_BUNDLE_PATH} AND NOT "$ENV{CUBE_BUNDLE_PATH}" STREQUAL "")
        set(STM32CUBE_BUNDLE_PATH "$ENV{CUBE_BUNDLE_PATH}")
    else()
        set(STM32CUBE_BUNDLE_PATH "$ENV{HOME}/Library/Application Support/stm32cube/bundles")
    endif()
endif()

file(GLOB STM32_GNU_TOOLCHAIN_DIRS
    LIST_DIRECTORIES true
    "${STM32CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/*"
)
list(LENGTH STM32_GNU_TOOLCHAIN_DIRS STM32_GNU_TOOLCHAIN_COUNT)
if(STM32_GNU_TOOLCHAIN_COUNT EQUAL 0)
    message(FATAL_ERROR
        "Could not find STM32Cube GNU Arm toolchain under "
        "${STM32CUBE_BUNDLE_PATH}/gnu-tools-for-stm32"
    )
endif()
list(SORT STM32_GNU_TOOLCHAIN_DIRS COMPARE NATURAL ORDER DESCENDING)
list(GET STM32_GNU_TOOLCHAIN_DIRS 0 STM32_GNU_TOOLCHAIN_DIR)

if(NOT STM32_GNU_TOOLCHAIN_DIR OR
   NOT EXISTS "${STM32_GNU_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc")
    message(FATAL_ERROR
        "Could not find STM32Cube GNU Arm toolchain under "
        "${STM32CUBE_BUNDLE_PATH}/gnu-tools-for-stm32"
    )
endif()

set(TOOLCHAIN_PREFIX                "${STM32_GNU_TOOLCHAIN_DIR}/bin/arm-none-eabi-")

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc CACHE FILEPATH "" FORCE)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++ CACHE FILEPATH "" FORCE)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++ CACHE FILEPATH "" FORCE)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy CACHE FILEPATH "" FORCE)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size CACHE FILEPATH "" FORCE)
set(CMAKE_AR                        ${TOOLCHAIN_PREFIX}ar CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB                    ${TOOLCHAIN_PREFIX}ranlib CACHE FILEPATH "" FORCE)
set(CMAKE_NM                        ${TOOLCHAIN_PREFIX}nm CACHE FILEPATH "" FORCE)
set(CMAKE_OBJDUMP                   ${TOOLCHAIN_PREFIX}objdump CACHE FILEPATH "" FORCE)
set(CMAKE_READELF                   ${TOOLCHAIN_PREFIX}readelf CACHE FILEPATH "" FORCE)
set(CMAKE_ADDR2LINE                 ${TOOLCHAIN_PREFIX}addr2line CACHE FILEPATH "" FORCE)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H743XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
