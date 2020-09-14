CC=gcc
CFLAGS=-Wall
INCLUDES=-L/usr/local/lib/ -lpiface-1.0
DEST=/usr/local/bin
PROGRAM=pepopiface

pepopiface: pepopiface.c
	$(CC) $(CFLAGS) $(INCLUDES) -o pepopiface pepopiface.c

clean:
	rm -f *.o $(PROGRAM) 

install:
	install -s $(PROGRAM) $(DEST)

uninstall:
	rm -f $(DEST)/$(PROGRAM)
