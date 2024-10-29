project('csp-avr', 'c', subproject_dir: 'lib', default_options: [
	'buildtype=minsize', 
	'c_std=gnu11', 
	'b_staticpic=false', 
	'default_library=static',
	'csp:have_stdio=false',
	'csp:use_hmac=false',
	'csp:use_promisc=false',
])

atmel_start_dep = subproject('start').get_variable('atmel_start_dep')
freertos_dep = subproject('freertos').get_variable('freertos_dep')

sources = files(
  'src/main.c',
)

#include_dirs = include_directories(
#  'include',
#)

# Define the executable and link with Atmel Start and FreeRTOS libraries
main_elf = executable('main',
  sources,
  #  include_directories: include_dirs,
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
