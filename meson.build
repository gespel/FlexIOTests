project('FlexIOTests', 'c',
	version : '1.0.0'
)

# Host-side dependencies
ibverbs_dep = dependency('libibverbs', required: true)
mlx5_dep    = dependency('libmlx5',   required: true)
flexio_dep  = dependency('libflexio', required: true)

# Native C compiler for host side
c = meson.get_compiler('c', native: true)
add_project_arguments('-Wmissing-prototypes', language: 'c', native: true)
add_project_arguments('-Wstrict-prototypes',  language: 'c', native: true)

# DPACC build script for device side
build_dpacc = find_program('doca_build_dpacc.sh', required: true)

# Device-side compiler flags
dpacc_devicecc_array = [
	'-DE_MODE_LE',
	'-Wall',
	'-Wextra',
	'-Wpedantic',
	'-Wdouble-promotion',
	'-Wno-empty-translation-unit',
	'-Wmissing-prototypes',
	'-Wstrict-prototypes',
	'-ffreestanding',
	'-mcmodel=medany',
]

meson_build_type = get_option('buildtype')
dpa_cpus     = get_option('cpu')
dpa_cpus_str = '-mcpu=nv-dpa-' + ',nv-dpa-'.join(dpa_cpus)
dpacc_options = [dpa_cpus_str]

if meson_build_type == 'debug'
	dpacc_devicecc_array += ['-g', '-O0', '-gdwarf-4']
elif meson_build_type == 'debugoptimized' or meson_build_type == ''
	dpacc_devicecc_array += ['-g', '-O2', '-gdwarf-4']
	dpacc_options += ['-flto']
elif meson_build_type == 'release'
	dpacc_devicecc_array += ['-O3']
	dpacc_options += ['-flto']
else
	error('Unsupported build type: ', meson_build_type)
endif

dynamic_link = get_option('dynamic') ? 'yes' : 'no'

dpacc_devicecc_array += ['-Wno-error']

# Build shared common libraries, then the sample
subdir('common')
subdir('packet_printer')
