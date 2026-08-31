# Uebersetzungstest der Firmware ohne ESP-Toolchain.
#
#     make -C firmware/test/compile
#
# Baut main.cpp, rplidar.cpp, feetech_servo.cpp und yaw_axis.cpp gegen
# Attrappen der Arduino- und IDF-APIs und bindet sie. Das findet keine
# Laufzeitfehler - dafuer aber alles, was Compiler und Binder finden:
# Tippfehler, falsche Signaturen, fehlende Deklarationen, nicht definierte
# Symbole. Genau die Fehlerklasse, die sonst erst beim Flashen auffaellt,
# und die hier lange unentdeckt bleiben konnte, weil es keine ESP-Toolchain
# gibt.
#
# usb_ncm.cpp ist mit dabei, aber nur in seiner ausgeschalteten Fassung
# (ENABLE_USB_NCM 0, der Standard aus config.h): dann besteht es aus drei
# Rumpffunktionen, und genau die muessen binden - sonst scheitert jeder Build
# ohne ESP-IDF, also `pio run -e wifi` und die Arduino-IDE. Der eingeschaltete
# Zweig haengt an esp_tinyusb, dessen API sich zwischen IDF-Fassungen mehrfach
# geaendert hat; eine Attrappe davon wuerde eine Vertrautheit vortaeuschen,
# die nicht da ist.

# Beide Anschlusswege werden geprueft. Der USB-Weg ist der Normalfall, aber
# der UART-Weg ist die Rueckfallebene und darf nicht unbemerkt verrotten.
SRC := ../../src
CXX ?= g++
BASE := -std=gnu++17 -Wall -Wextra -Wshadow -g -Istubs -DARDUINO=200
SOURCES := $(SRC)/main.cpp $(SRC)/rplidar.cpp $(SRC)/feetech_servo.cpp \
           $(SRC)/yaw_axis.cpp $(SRC)/usb_ncm.cpp stubs.cpp

.PHONY: all usb uart clean
all: usb uart
	@echo
	@echo "Uebersetzung und Bindung fehlerfrei"

usb:
	@echo "== LiDAR am USB-Host (LIDAR_LINK_USB=1) =="
	@$(CXX) $(BASE) -DLIDAR_LINK_USB=1 -o check_usb $(SOURCES)

uart:
	@echo "== LiDAR an der UART (LIDAR_LINK_USB=0) =="
	@$(CXX) $(BASE) -DLIDAR_LINK_USB=0 -o check_uart $(SOURCES)

clean:
	@rm -f *.o check_usb check_uart compile_check
