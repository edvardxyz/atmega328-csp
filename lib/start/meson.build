project('start', 'c')

# Define Atmel Start source files, including main and driver files from lib/start/src
atmel_start_sources = files(
  'src/atmel_start.c',
  'src/driver_isr.c',
  #'src/main.c',  # Include other relevant .c files in src as needed
)

# Additional source files in deeper directories
atmel_start_sources += files(
  'src/src/driver_init.c',
  'src/src/i2c_simple_master.c',
  'src/src/usart_basic.c',
  'src/src/i2c_master.c',
  'src/src/i2c_types.c',
  'src/src/protected_io.S',
  'src/src/wdt.c',
  'src/src/spi_basic.c',
)

# Define include directories
atmel_start_include_dirs = include_directories(
  'src',
  'src/include',
  'src/utils',
  'src/config',
)

# Create Atmel Start static library
atmel_start_lib = static_library(
  'atmel_start',
  atmel_start_sources,
  include_directories: atmel_start_include_dirs
)

# Declare a dependency to expose the library and include directories
atmel_start_dep = declare_dependency(
  link_with: atmel_start_lib,
  include_directories: atmel_start_include_dirs
)
