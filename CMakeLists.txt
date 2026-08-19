# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# TODO(IDF6): the line below was removed during the 6.0 migration.
#
#   set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
#
# protocol_examples_common is leftover boilerplate from the IDF example this
# project was started from. Nothing under main/ references it - no
# example_connect(), no esp_eth, no ethernet_init - but pulling it in drags in
# the espressif/ethernet_init managed component, whose pinned version in
# dependencies.lock predates 6.0 and fails to find esp_eth_driver.h.
#
# Dropping the line removes that entire dependency chain. If you ever do want
# example_connect(), put the line back and delete dependencies.lock so the
# component manager re-resolves ethernet_init against 6.0.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trace)
