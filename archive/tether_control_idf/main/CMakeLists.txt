set(TETHER_ROLE "0" CACHE STRING "0=coordinator, 1=motor node")
set(TETHER_NODE_ID "0" CACHE STRING "Module id (master=0, remote nodes=1 or 2)")
set(TETHER_MOTOR_SLAVE_ID "0x000" CACHE STRING "Local Damiao slave CAN id")
set(TETHER_MOTOR_MASTER_ID "0x000" CACHE STRING "Local Damiao feedback CAN id")
set(TETHER_MASTER_MOTOR_CONFIGURED "0" CACHE STRING "Set to 1 after provisioning id-0 CAN IDs")

if(NOT TETHER_ROLE MATCHES "^[01]$")
  message(FATAL_ERROR "TETHER_ROLE must be 0 or 1")
endif()
if(TETHER_ROLE STREQUAL "1" AND NOT TETHER_NODE_ID MATCHES "^[12]$")
  message(FATAL_ERROR "Motor nodes require TETHER_NODE_ID=1 or 2")
endif()

idf_component_register(
  SRCS "app_main.cpp"
  INCLUDE_DIRS "."
  REQUIRES damiao tether_link esp_timer esp_driver_uart
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  TETHER_ROLE=${TETHER_ROLE}
  TETHER_NODE_ID=${TETHER_NODE_ID}
  TETHER_MOTOR_SLAVE_ID=${TETHER_MOTOR_SLAVE_ID}
  TETHER_MOTOR_MASTER_ID=${TETHER_MOTOR_MASTER_ID}
  TETHER_MASTER_MOTOR_CONFIGURED=${TETHER_MASTER_MOTOR_CONFIGURED}
)
