CC=mos-c64-clang

all: c64.prg
clean:
	rm *.prg

c64.prg: c64.c roguelike.h
	$(CC) -Os -flto -lm -DLLVM_MOS_PLATFORM -o $@ $<
