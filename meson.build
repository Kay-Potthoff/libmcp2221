# SPDX-License-Identifier:    GPL-3.0
# Copyright (C) 2020 MicroSys Electronics GmbH
# Author: Kay Potthoff <kay.potthoff@microsys.de>

# For Fedora/CentOS install: libusb, hidapi-devel, libudev-devel

# build with:
# > meson build
# > ninja -C build/

project('mcp2221', 'c',
        version: '1.0.4')

debug_info_hid = get_option('debug-info-hid')
with_examples = get_option('with-examples')

if debug_info_hid
    libmcp_c_args = '-DDEBUG_INFO_HID'
else
    libmcp_c_args = '-UDEBUG_INFO_HID'
endif

libmcp_inc = include_directories('libmcp2221')

udev_dep = dependency('libudev')
usb_dep = dependency('libusb')
hidapi_hidraw_dep = dependency('hidapi-hidraw')

libmcp = shared_library('mcp2221',
                        join_paths('libmcp2221', 'libmcp2221.c'),
                        dependencies: [udev_dep, usb_dep, hidapi_hidraw_dep],
                        c_args: libmcp_c_args,
                        version: meson.project_version(),
                        install: true)

libmcp_a = static_library('mcp2221',
                          join_paths('libmcp2221', 'libmcp2221.c'),
                          dependencies: [udev_dep, usb_dep, hidapi_hidraw_dep],
                          c_args: libmcp_c_args,
                          install: true)

libmcp_dep = declare_dependency(link_with: libmcp)

install_headers(join_paths('libmcp2221', 'libmcp2221.h'),
                subdir: meson.project_name())

rcwtool = executable('rcwtool',
                     join_paths('utils', 'rcwtool.c'),
                     include_directories: libmcp_inc,
                     dependencies: libmcp_dep,
                     install: true)

if with_examples

i2c_exe = executable('i2c',
                     join_paths('examples', 'i2c', 'main.c'),
                     include_directories: libmcp_inc,
                     dependencies: libmcp_dep,
                     install: false)

i2c_static_exe = executable('i2c-static',
                     join_paths('examples', 'i2c', 'main.c'),
                     include_directories: libmcp_inc,
                     dependencies: libmcp_dep,
                     install: false,
                     link_with: libmcp_a)

endif
