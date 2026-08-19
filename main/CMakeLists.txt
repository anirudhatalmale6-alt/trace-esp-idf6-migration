file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)
idf_component_register(SRCS ${app_sources} 
                    INCLUDE_DIRS "."
                    "." 
                    "./tcpip" 
                    "./drivers"
                    REQUIRES log esp_timer esp_eth esp_http_server esp_adc freertos hal lwip

                    EMBED_FILES 
                    )

