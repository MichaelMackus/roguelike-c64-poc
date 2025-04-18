CC=cc
CLANG=~/llvm-mos/bin/mos-c64-clang
CL65=cl65
MINGWCC=x86_64-w64-mingw32-gcc
PDCURSES_DIR=$(HOME)/PDCurses
all: pc llvm.prg cc65.prg
clean:
	rm *.prg *.o pc *.exe

pc: main.c roguelike.h common.c
	$(CC) -lcurses -o $@ $<
llvm.prg: main.c c64.c roguelike.h c64_getch.o common.c
	$(CLANG) -Os -flto -DLLVM_MOS_PLATFORM -o $@ $< c64_getch.o
cc65.prg: main.c c64.c roguelike.h c64_getch.o common.c
	$(CL65) -t c64 -o $@ $< c64_getch.o

c64_getch.o: c64_getch.s
	ca65 -t c64 $<

# cross compile for windows
pc.exe: main.c roguelike.h common.c
	$(MINGWCC) -I $(PDCURSES_DIR) -o $@ -L$(PDCURSES_DIR)/wincon $< -l:pdcurses.a
