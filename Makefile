NAME            = serialT
OBJECTS         = main.o
ASFLAGS         =
CFLAGS          = -pg -O2 -Wall -g

#Switch the compiler (for the internal make rules)
CC              = gcc
AS              = gcc

.PHONY: all FORCE clean download download-jtag

all: ${NAME}

${NAME}: ${OBJECTS}
	${CC} -o $@ ${OBJECTS}

clean:
	rm -f ${NAME} $(OBJECTS)


