LLVM=~/llvm-mos/bin/mos-c64-clang
all: c64.prg cc65.prg
clean:
	rm *.prg

c64.prg: c64.c roguelike.h
	$(LLVM) -Os -flto -lm -DLLVM_MOS_PLATFORM -o $@ $<

cc65.prg: cc65.c roguelike.c89.h
	cl65 --standard c99 -lm -o $@ $<
