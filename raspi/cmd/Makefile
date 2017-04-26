DEBUGFLAGS	 = -g
CFLAGS		+= -I../include $(DEBUGFLAGS)
LFLAGS		+= -L../lib -ltrackcam -Wl,-rpath=/usr/local/lib

BINPATH		 = /usr/local/bin

OBJS		 = main.o led.o led-user-sequence.o servo-pos.o
TARGETS		 = trackcam

all: $(TARGETS)

trackcam: $(OBJS)
	$(CC) $(LFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) $(TARGETS)

install:
	cp $(TARGETS) $(BINPATH)

uninstall:
	for f in $(TARGETS);do rm -f $(BINPATH)/$$f;done
