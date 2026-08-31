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

SRC := ../../src
CXX ?= g++
CXXFLAGS := -std=gnu++17 -Wall -Wextra -Wshadow -g -Istubs -DARDUINO=200

OBJS := main.o rplidar.o feetech_servo.o yaw_axis.o usb_ncm.o stubs.o

.PHONY: all clean
all: compile_check
	@echo
	@echo "Uebersetzung und Bindung fehlerfrei"

compile_check: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: $(SRC)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

stubs.o: stubs.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	@rm -f *.o compile_check
