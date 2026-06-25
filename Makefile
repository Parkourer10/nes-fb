
.PHONY: all clean
all: main

WARN=-std=c99 \
		 -Wall \
     -Wno-parentheses \
		 -Wno-misleading-indentation \
		 -Wno-bool-operation \
		 -Wno-discarded-qualifiers \
		 -Wno-incompatible-pointer-types-discards-qualifiers \
		 -Wno-unknown-warning-option \
		 -Wno-switch-outside-range \
		 -Wno-unused-value \
		 -Wno-char-subscripts \
		 -Wno-switch
SDLFLAGS=$(shell pkg-config --cflags --libs --static sdl2) -static

main: main.c
	$(CC) -O3 -o $@ $< ${SDLFLAGS} -g ${WARN}

clean:
	rm -f main
