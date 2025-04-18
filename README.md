# Commodore 64 C Roguelike POC

This is a simple proof of concept of dungeon generation with my roguelike
library [libroguelike](https://github.com/MichaelMackus/libroguelike). Check
the releases for a .prg file if you want to try to run this in VICE (emulator)
or on a real C64.

## Dependencies

There are two demos here - one requires
[LLVM-MOS](https://github.com/llvm-mos/llvm-mos) and
[LLVM-MOS-SDK](https://github.com/llvm-mos/llvm-mos-sdk); the other requires
the cc65 compiler which is much simpler. You might have to compile these from
scratch if they do not provide binaries for your system. Once you have the
compiler, you can build the .prg files with `make`. `make c64.prg` makes the
LLVM version and `make cc65.prg` makes the cc65 version.
