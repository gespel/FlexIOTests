common_host_sources = ['flow_steering_utils.c']

common_host_library = library('common_host',
			      common_host_sources,
			      native: true,
			      include_directories: ['.'],
			      dependencies: [ibverbs_dep, mlx5_dep, flexio_dep])

common_host_dep = declare_dependency(include_directories: '.',
				     link_with: common_host_library)
