project('csp-avr', 'c', subproject_dir: 'lib', default_options: [
	'buildtype=minsize', 
	'c_std=gnu11', 
	'b_staticpic=false', 
	'default_library=static',
	'csp:have_stdio=false',
	'csp:use_hmac=false',
	'csp:use_promisc=false',
])

mcu = 'atmega328p'
compiler = 'avr-gcc'

# Set compiler and linker arguments specific to AVR
#c_args = ['-mmcu=' + mcu, '-Os']
#link_args = ['-mmcu=' + mcu]

# Retrieve the Atmel Start dependency
atmel_start_dep = subproject('start').get_variable('atmel_start_dep')
freertos_dep = subproject('freertos').get_variable('freertos_dep')

# Main project source files
sources = files('src/main.c')

# Define the executable and link with Atmel Start and FreeRTOS libraries
main_elf = executable('main',
  sources,
  #c_args: c_args,
  link_args: [
    '-Wl,-Map,output.map'
  ],
  dependencies: [atmel_start_dep, freertos_dep],  # Link with dependencies
  install: false
)

hex_target = custom_target(
  'main_hex',
  input: main_elf,
  build_by_default: true,
  output: 'main.hex',
  command: ['avr-objcopy', '-O', 'ihex', main_elf, '@OUTPUT@']
)
