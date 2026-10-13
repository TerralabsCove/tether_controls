idf_component_register(
  SRCS "tether_link.cpp"
  INCLUDE_DIRS "include"
  REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_driver_gpio
)
