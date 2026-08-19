# ---------------------------------------------------------------------------
# Trace - main component
#
# Updated for ESP-IDF 6.0.x
#
# What changed and why:
#
#  * The source glob was  ${CMAKE_SOURCE_DIR}/main/*.*  which swept every file
#    with a dot in its name into SRCS - .h, .html, .png, .json, .pem, .js and
#    the stray main/build/CMakeLists.txt included. Narrowed to *.c.
#
#  * INCLUDE_DIRS listed "." twice and "./tcpip" in lower case; the directory
#    on disk is "TCPIP". Windows does not care, anything case-sensitive does.
#
#  * REQUIRES was missing most of what the code actually uses. In 6.0 the old
#    monolithic "driver" and "hal" components were split into per-peripheral
#    components (esp_driver_*, esp_hal_*), so the missing entries now bite.
# ---------------------------------------------------------------------------

file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.c)

# TODO(IDF6): main/drivers/sockets.c is a verbatim copy of lwIP's own
# sockets.c (4613 lines, "This file is part of the lwIP TCP/IP stack").
# It #includes api_lib.c / api_msg.c / netbuf.c, which are lwIP-internal
# sources not on any include path, and every symbol in it collides with the
# real lwIP already linked in via the lwip component. Excluded from the build.
# It looks like a reference copy rather than something meant to compile - if
# you actually intended to override lwIP's sockets layer, that needs doing a
# different way and I'd want to talk it through with you first.
list(FILTER app_sources EXCLUDE REGEX "/sockets\\.c$")

# TODO(IDF6): "main/drivers/server .c" has been RENAMED to "server.c" - it had a
# space in the filename. It carries a stale WiFi.c file header, which is why it
# looks like a duplicate, but it is not: server_send(), server_receive_poll(),
# server_socket_poll_0..3(), server_accept_poll() and WiFi_show_connections()
# are defined here and nowhere else. serial_io.c calls server_send(), so
# leaving this file out of the build breaks the link.

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS
                            "."
                            "./TCPIP"
                            "./drivers"
                       REQUIRES
                            # was already here
                            log
                            esp_timer
                            esp_eth
                            esp_http_server
                            esp_adc
                            freertos
                            hal
                            lwip
                            # added - used by the code but never declared
                            nvs_flash
                            esp_wifi
                            esp_netif
                            esp_event
                            esp_http_client
                            esp-tls
                            esp_hw_support
                            app_update      # esp_ota_ops.h, used by TCPIP/http_services.c
                            # added - 6.0 peripheral component split
                            driver
                            esp_driver_gpio
                            esp_driver_spi
                            esp_driver_ledc
                            esp_driver_rmt
                            esp_driver_pcnt
                            esp_driver_uart # driver/uart.h, used by drivers/serial_io.c
                            esp_hal_gpio
                       )
