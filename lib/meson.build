project('start', 'c')

# Define FreeRTOS source files
freertos_sources = files(
  'Source/croutine.c',
  'Source/event_groups.c',
  'Source/list.c',
  'Source/queue.c',
  'Source/tasks.c',
  'Source/timers.c',
  'Source/portable/GCC/ATMega328/port.c',
  'Source/portable/MemMang/heap_1.c',

  'Drivers/led.c',
)

# Define FreeRTOS include directories
freertos_include_dirs = include_directories(
  'Drivers',
  'Source/include',
  'Source/portable/GCC/ATMega328',  # Port-specific headers
  '.'  # To include FreeRTOSConfig.h from lib/freertos
)

# Create FreeRTOS static library
freertos_lib = static_library(
  'freertos',
  freertos_sources,
  include_directories: freertos_include_dirs
)

# Declare a dependency for FreeRTOS
freertos_dep = declare_dependency(
  link_with: freertos_lib,
  include_directories: freertos_include_dirs
)
