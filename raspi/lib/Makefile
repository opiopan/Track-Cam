DEBUGFLAGS	 = -g
CFLAGS		+= -fPIC -I../include $(DEBUGFLAGS)
LFLAGS		+= -shared

LIBPATH		= /usr/local/lib

OBJS		= foundation.o led.o
TARGETS		= libtrackcam.so

all: $(TARGETS)

libtrackcam.so: $(OBJS)
	$(CC) $(LFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) $(TARGETS)

install:
	cp $(TARGETS) $(LIBPATH)

uninstall:
	for f in $(TARGETS);do rm -f $(LIBPATH)/$$f;done
