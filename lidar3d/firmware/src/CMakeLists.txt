# Komponentenbeschreibung fuer die Umgebung `usb`.
idf_component_register(
    SRCS
        "main.cpp"
        "rplidar_s2.cpp"
        "yaw_axis.cpp"
        "usb_ncm.cpp"
    INCLUDE_DIRS
        "."
        "../include"
    REQUIRES
        arduino
        driver
        esp_netif
        esp_timer
        esp_tinyusb
)
