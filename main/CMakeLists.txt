idf_component_register(SRCS "main.c" "ei_inference_wrapper.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES edge_impulse esp_driver_gpio esp_driver_i2c esp_timer esp_wifi esp_netif esp_event nvs_flash esp_http_client esp-tls mbedtls)

target_compile_options(${COMPONENT_LIB} PRIVATE
                    -Wno-error=format)
