#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "common.c"

#define WIDTH 40
#define HEIGHT 25

/* LLVM-MOS doesn't convert petscii to ascii chars, cc65 *does* */
/* see: https://llvm-mos.org/wiki/Character_set */
#if LLVM_MOS_PLATFORM
#define LEFT  72
#define DOWN  74
#define UP    75
#define RIGHT 76
#define QUIT  81
#else
#define LEFT  'h'
#define DOWN  'j'
#define UP    'k'
#define RIGHT 'l'
#define QUIT  'q'
#endif

#if LLVM_MOS_PLATFORM || __C64__ /* C64 platform */
#include "c64.c"

#define RL_RNG_CUSTOM
unsigned int rl_rng_generate(unsigned int min, unsigned int max)
{
    assert(max >= min);
    assert(max <= 255);
    if (min == max) return min;

    return min + c64_rand() % (max - min + 1);
}

void platform_init()
{
    c64_seed();
    c64_kb_init();
}

void platform_deinit()
{
}

#define C64_JIFFIES (*((volatile char*)0xA2))
static char start;
int abs(int n)
{
    return n < 0 ? n *= -1 : n;
}
char platform_getch()
{
    char ch;
    // wait for a few jiffies, otherwise input happens too fast
    while (abs(C64_JIFFIES - start) < 5) {
        c64_getch(); // clear keyboard buffer for input responsiveness
    }
    ch = c64_getch_blocking();
    start = C64_JIFFIES;
    return ch;
}

extern char platform_screen_code(char ch)
{
    switch (ch) {
        case '#':
            return 102;
            break;
        case '.':
            return 123;
            break;
        case '+':
            return 43;
            break;
        case '>':
            return 62;
            break;
        case ' ':
            return 32;
            break;
        case '@':
            return 0;
            break;
        default:
            assert("Unhandled tile" && 0);
            break;
    }
}

void platform_screen_set(int idx, char ch)
{
    c64_screen_set(idx, ch);
}

char platform_screen_get(int idx)
{
    return c64_screen_get(idx);
}

extern void platform_screen_clear()
{
    memset((char*) 0x0400, 32, SCREEN_SIZE);
}
#else /* PC platform */
#include <curses.h>
#include <time.h>
#include <stdlib.h>

void platform_init()
{
    srand(time(0));
    initscr();
    curs_set(0);
    noecho();
}

void platform_deinit()
{
    endwin();
}

char platform_getch()
{
    return getch();
}

char platform_screen_code(char ch)
{
    return ch;
}

void platform_screen_set(int idx, char ch)
{
    int x, y;
    x = idx % SCREEN_WIDTH;
    y = idx / SCREEN_WIDTH;
    assert(x < SCREEN_WIDTH);
    assert(y < SCREEN_HEIGHT);
    mvaddch(y, x, ch);
    refresh();
}

char platform_screen_get(int idx)
{
    int x, y;
    chtype str[2];
    x = idx % SCREEN_WIDTH;
    y = idx / SCREEN_WIDTH;
    assert(mvinchnstr(y, x, str, 1) != ERR);
    return str[0] & A_CHARTEXT;
}

extern void platform_screen_clear()
{
    clear();
}
#endif /* platform */

#define RL_MAPGEN_BSP_RANDOMISE_ROOM_LOC 0 // don't randomise room loc within the BSP leaves so we can easily find the room point to connect corridors
#define RL_ENABLE_PATHFINDING 0 /* required for RL_Point */
#define RL_ENABLE_FOV 0
#define RL_IMPLEMENTATION
#include "roguelike.h"

RL_Byte tiles[SCREEN_SIZE];
RL_Map map;
RL_BSP bsp;
MapPoint player;

int main(void)
{
    bool quit = false;
    map.width = WIDTH;
    map.height = HEIGHT;
    map.tiles = tiles;
    platform_init();
    map_generate(&map, &bsp, &player);

    // game loop
    while (!quit) {
        MapPoint new_loc;
        char ch;
        // simplistic FOV - we should also be able to easily light up rooms, à la rogue
        int player_idx = player.x + player.y*SCREEN_WIDTH;
        int new_idx;
        draw_fov(&map, player_idx, player_idx);
        // input
        ch = platform_getch();
        new_loc = player;
        switch (ch) {
            case LEFT:
                new_loc.x -= 1;
                break;
            case DOWN:
                new_loc.y += 1;
                break;
            case UP:
                new_loc.y -= 1;
                break;
            case RIGHT:
                new_loc.x += 1;
                break;
            case QUIT:
                quit = true;
            default:
                break;
        }
        new_idx = new_loc.x + new_loc.y*SCREEN_WIDTH;
        if (!rl_map_in_bounds(&map, new_loc.x, new_loc.y)) {
            // no-op
        } else if (map.tiles[new_idx] == '>') {
            map_generate(&map, &bsp, &player);
        } else if (rl_map_is_passable(&map, new_loc.x, new_loc.y)) {
            player = new_loc;
            platform_screen_set(player_idx, platform_screen_code(' '));
            platform_screen_set(new_idx, platform_screen_code('@'));
        }
    }

    platform_deinit();

    return 0;
}
