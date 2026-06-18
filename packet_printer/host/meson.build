# Host-side build for packet_printer

executable('packet_printer',
	   ['packet_printer_host.c'],
	   c_args: '-DDEV_APP_NAME=' + dev_app_name,
	   native: true,
	   dependencies: [ibverbs_dep, mlx5_dep, flexio_dep, common_host_dep, sample_device_dep],
	   link_depends: sample_dev_target)
