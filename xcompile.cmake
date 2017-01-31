INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER arm-none-eabi-gcc )
#CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
#CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/LinkerScript.ld)


set(ARM_FLAGS "-mcpu=cortex-m0 -mthumb -ffunction-sections -fmessage-length=0 -fsigned-char -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARM_FLAGS} -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -ffunction-sections -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 ${ARM_FLAGS} -ffreestanding -fno-move-loop-invariants -Wall -Wextra ")
# add -g above to add debug info

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")