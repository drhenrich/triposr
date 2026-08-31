# Komponentenbeschreibung fuer die Umgebung `usb`.
idf_component_register(
    SRCS
        "main.cpp"
        "rplidar.cpp"
        "yaw_axis.cpp"
        "feetech_servo.cpp"
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
