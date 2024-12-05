INCLUDE += -I /home/ssk/ethercat/_install/include
INCLUDE += -I ./ethercat 
INCLUDE += -I ./term 
INCLUDE += -I ./dcsync 
CFLAGS  += -g -Wall -O2 $(DEFINES) $(INCLUDE)
LIBS    += -lethercat
LIBS    += -lpthread
LDFLAGS := -L /home/ssk/ethercat/_install/lib
CXXFLAGS:= $(CFLAGS)
SOURCE  := $(wildcard *.c) $(wildcard ethercat/*.c) $(wildcard term/*.c) $(wildcard dcsync/*.c)
OBJS    := $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCE)))
TARGET  := igh_ethercat_dc_motor
CC=aarch64-linux-gnu-gcc
.PHONY : everything objs clean distclean rebuild

all : $(TARGET)

objs : $(OBJS)

rebuild: distclean all

clean :
	rm -rf *~
	rm -rf *.o ethercat/*.o  term/*.o  canopen/*.o dcsync/*.o


distclean : clean
	rm -rf $(TARGET)

$(TARGET) : $(OBJS)
	$(CC) $(CXXFLAGS) -g -o $@ $(OBJS) $(LDFLAGS) $(LIBS)
