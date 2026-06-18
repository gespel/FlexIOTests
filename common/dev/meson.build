common_dev_sources = ['com_dev.c']

common_dev_name      = 'commondev'
common_dev_full_name = 'lib' + common_dev_name + '.a'
common_dev_include_dir = meson.current_source_dir()
common_dev_lib_path    = meson.current_build_dir()

common_dev_includes = [join_paths(common_dev_include_dir, 'com_dev.h')]

common_dev_target = custom_target('archive_' + common_dev_name,
				  command: [build_dpacc, '--library',
					    '--archive_name', '@OUTPUT@',
					    '--external_cc_options', ','.join(dpacc_devicecc_array),
					    '--additional_dpacc_options', dpacc_options,
					    '--srcs', '@INPUT@'],
				  output: common_dev_full_name,
				  input: common_dev_sources,
				  depend_files: common_dev_includes)
