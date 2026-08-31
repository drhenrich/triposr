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

# Vier Durchlaeufe: beide Anschlusswege mal beide ESP-IDF-Fassungen.
#
# Letzteres, weil die Arduino-Cores 2.x und 3.x unterschiedliche IDF-Versionen
# mitbringen und der UART-Treiber sich dazwischen geaendert hat. Genau daran
# ist ein Build beim Benutzer gescheitert - siehe src/uart_compat.h.
SRC := ../../src
CXX ?= g++
BASE := -std=gnu++17 -Wall -Wextra -Wshadow -g -Istubs -DARDUINO=200
SOURCES := $(SRC)/main.cpp $(SRC)/rplidar.cpp $(SRC)/feetech_servo.cpp \
           $(SRC)/yaw_axis.cpp $(SRC)/usb_ncm.cpp $(SRC)/web_stream.cpp stubs.cpp

.PHONY: all clean
all:
	@for link in 0 1; do \
	  for idf in 4 5; do \
	    printf '%-42s' "LIDAR_LINK_USB=$$link, ESP-IDF $$idf.x"; \
	    $(CXX) $(BASE) -DLIDAR_LINK_USB=$$link -DSTUB_IDF_MAJOR=$$idf \
	      -o check_$${link}_$${idf} $(SOURCES) && echo "ok" || exit 1; \
	  done; \
	done
	@echo
	@echo "Uebersetzung und Bindung fehlerfrei"

clean:
	@rm -f *.o check_* compile_check
