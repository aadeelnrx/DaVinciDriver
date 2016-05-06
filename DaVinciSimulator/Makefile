CC=g++
INC=-I.\
		-I/Users/alex/Documents/Arduino/libraries/SdFat\
		-I/Users/alex/Documents/Arduino/libraries/Adafruit_Sensor-master\
		-I/Users/alex/Documents/Arduino/libraries/Adafruit_BNO055-master\
		-I/Users/alex/Documents/Arduino/libraries/br3ttb-Arduino-PID-Library-fb095d8\
		-I/Applications/Arduino.app/Contents/Resources/Java/hardware/teensy/avr/libraries/SPI\
		-I/Applications/Arduino.app/Contents/Resources/Java/hardware/teensy/avr/libraries/Wire\
		-I/Applications/Arduino.app/Contents/Resources/Java/hardware/teensy/avr/libraries/Metro
CFLAGS=-c -Wall $(INC) -g -DARDUINO=200 -DSIMULATION -DAdafruit_BNO055=BnoSim
LDFLAGS= -g
SOURCES=main.cpp Arduino.cpp Logging.cpp Logging_helper.cpp Motor.cpp PID_v1.cpp BnoSim.cpp measuring.cpp Metro.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)
clean:
	@rm -f $(OBJECTS) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
