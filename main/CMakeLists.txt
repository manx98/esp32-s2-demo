file(GLOB_RECURSE APP_SRC *.c)
idf_component_register(SRCS ${APP_SRC}
                    INCLUDE_DIRS ".")

message("WIFI_SSID ==> $ENV{WIFI_SSID}")
message("WIFI_PASSWORD ==> $ENV{WIFI_PASSWORD}")
add_compile_definitions(WIFI_SSID=$ENV{WIFI_SSID} WIFI_PASSWORD=$ENV{WIFI_PASSWORD})