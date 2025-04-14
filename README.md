# LLVM-MOS C Roguelike POC

This is a simple proof of concept of dungeon generation with my roguelike
library [libroguelike](https://github.com/MichaelMackus/libroguelike). Check
the releases for a .prg file if you want to try to run this in VICE (emulator)
or on a real C64.

## Dependencies

This requires [LLVM-MOS](https://github.com/llvm-mos/llvm-mos) and
[LLVM-MOS-SDK](https://github.com/llvm-mos/llvm-mos-sdk). You might have to
compile these from scratch if they do not provide binaries for your system.
Once you have the compiler, you can build the .prg file with `make`.
