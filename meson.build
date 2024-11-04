project('csp-avr', 'c', subproject_dir: 'lib', default_options: [
	'buildtype=minsize', 
	'c_std=gnu11', 
	'b_staticpic=false', 
	'default_library=static',
	'csp:have_stdio=false',
	'csp:use_hmac=false',
	'csp:use_promisc=false',
        'csp:enable_pic=false',
        'csp:enable_csp_print=false',
        'csp:use_rdp=false',
        'csp:use_dedup=false',
        'csp:use_crc32=false',
        'csp:qfifo_len=2',
        'csp:buffer_count=4',
        'csp:buffer_size=32',
        'csp:conn_rxqueue_len=2',
        'csp:conn_max=2',
        'csp:port_max_bind=4',
        'csp:version=2',
        
])

add_project_arguments('-save-temps', language: 'c')


atmel_start_dep = subproject('start').get_variable('atmel_start_dep')
freertos_dep = subproject('freertos').get_variable('freertos_dep')
csp_dep = dependency('csp', fallback: ['csp', 'csp_dep'])

sources = files(
  'src/main.c',
)


# Define the executable and link with Atmel Start and FreeRTOS libraries
main_elf = executable('main',
  sources,
  #c_args: c_args,
  link_args: [
    '-Wl,-Map,output.map'
  ],
  dependencies: [atmel_start_dep, freertos_dep, csp_dep],  # Link with dependencies
  install: false
)

hex_target = custom_target(
  'main_hex',
  input: main_elf,
  build_by_default: true,
  output: 'main.hex',
  command: ['avr-objcopy', '-O', 'ihex', main_elf, '@OUTPUT@']
)
