#include <stdio.h>
#include <assert.h>
#include <string.h>

/**
 * Commodore 64 hardware-specific routines
 */
int c64_rand()
{
    /* query SID chip for RNG */
    return *((volatile char*)0xD41B);
}

void c64_seed()
{
    *((char*) 0xd40e) = 0xff;
    *((char*) 0xd40f) = 0xff;
    *((char*) 0xd412) = 0x80;
    /* asm("LDA #0xFF"); */
    /* asm("STA $D40E"); */
    /* asm("STA $D40F"); */
    /* asm("LDA #0x80"); */
    /* asm("STA $D412"); */
    c64_rand();
}

void c64_screen_set(int idx, char screencode)
{
    char *screen = (char*) 0x0400;
    screen[idx] = screencode;
}

char c64_screen_get(int idx)
{
    char *screen = (char*) 0x0400;
    return screen[idx];
}

void c64_kb_init()
{
    *((char*) 0x0289) = 0x01; /* set keyboard buffer length to 1 */
    *((char*) 0x028a) = 0b10000000; /* enable all keys to repeat */
    /* asm("lda #1"); */
    /* asm("sta $0289");  // set keyboard buffer length to 1 */
    /* enable all keys to repeat */
    /* asm("lda #0b10000000"); */
    /* asm("sta $028a"); */
    /* disable key repeat */
    /* asm("lda #0"); */
    /* asm("sta $028a"); */
}

extern char c64_getch();
/* linked with assembly, see c64_getch.s */
/* { */
/*     char r; */
/*     asm("jsr 0xff9f"); // scan keyboard */
/*     asm("jsr 0xffe4"); // read kb buffer */
/*     asm volatile("sta %0" : "=r"(r)); */
/*     return r; */
/* } */

char c64_getch_blocking()
{
    char ch = 0;
    while (ch == 0) {
        ch = c64_getch();
    }

    return ch;
}
