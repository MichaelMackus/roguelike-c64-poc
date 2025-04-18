#define RL_RNG_CUSTOM
int c64_rand();
unsigned int rl_rng_generate(unsigned int min, unsigned int max)
{
    /* assert(max >= min); */
    /* assert(max <= 255); */
    if (min == max) return min;

    return min + c64_rand() % (max - min + 1);
}

#define RL_IMPLEMENTATION
#define RL_ENABLE_PATHFINDING 0
#define RL_ENABLE_FOV 0
#define RL_MAPGEN_BSP_RANDOMISE_ROOM_LOC 0
#include "roguelike.h"

#define WIDTH 40
#define HEIGHT 25
#define SCREEN_SIZE 1000

int c64_rand()
{
    /* query SID chip for RNG */
    // test comment
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

int main()
{
    int x, y, idx;
    char ch, t;
    RL_Map *map;
    RL_BSP *bsp;
    RL_MapgenConfigBSP config = RL_MAPGEN_BSP_DEFAULTS;
    config.draw_doors = false;
    config.draw_corridors = RL_ConnectSimple;
    config.max_splits = 3;

    c64_seed();
    map = rl_map_create(WIDTH, HEIGHT);
    memset(map->tiles, RL_TileRock, WIDTH * HEIGHT);
    bsp = rl_bsp_create(WIDTH, HEIGHT);
    rl_mapgen_bsp_ex(map, bsp, &config);
    rl_bsp_destroy(bsp);

    memset((char*) 0x0400, 32, SCREEN_SIZE);
    for (y=0; y<HEIGHT; ++y) {
        for (x=0; x<WIDTH; ++x) {
            idx = x + y*WIDTH;
            ch = 32;
            t = map->tiles[idx];
            switch (t) {
                case RL_TileRock:
                    ch = 102;
                    break;
                case RL_TileRoom:
                    ch = 123;
                    break;
                case RL_TileCorridor:
                    break;
                case RL_TileDoor:
                    ch = 43;
                    break;
                case '>':
                    ch = 62;
                    break;
                default:
                    assert("Unhandled tile" && 0);
                    break;
            }
            c64_screen_set(idx, ch);
        }
    }

    return 0;
}
