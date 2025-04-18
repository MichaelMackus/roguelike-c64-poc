#include <stdio.h>
#include <assert.h>
#include <string.h>

/**
 * Some setup is needed since LLVM-MOS doesn't provide very good RNG or math functions.
 */

#define fabs(a) ((a) < 0 ? (a) * -1 : (a))
#define sqrt(a) a
#define floor(a) (int)(a)

#define RL_RNG_CUSTOM
int c64_rand();
unsigned int rl_rng_generate(unsigned int min, unsigned int max)
{
    assert(max >= min);
    assert(max <= 255);
    if (min == max) return min;

    return min + c64_rand() % (max - min + 1);
}

// arena allocator
#define ARENA_SIZE 20000
typedef struct {
    size_t count;
    char data[ARENA_SIZE];
} Arena;
Arena arena;
void *c64_arena_malloc(size_t size)
{
    assert(arena.count + size < ARENA_SIZE);
    char *ptr = &arena.data[arena.count];
    arena.count += size;
    return ptr;
}
void *c64_arena_calloc(size_t n, size_t size)
{
    void *data = c64_arena_malloc(n*size);
    memset(data, 0, n*size);

    return data;
}
void c64_free(void *ptr) {} // no-op
void *c64_realloc(void *ptr, size_t size)
{
    // unable to use heap
    assert("Realloc is not implemented" && 0);
}
void c64_arena_reset()
{
    arena.count = 0;
}

#define C64_JIFFIES (*((volatile char*)0xA2))

/* #define rl_malloc  c64_arena_malloc */
/* #define rl_calloc  c64_arena_calloc */
/* #define rl_realloc c64_realloc */
/* #define rl_free    c64_free */
#define RL_MAX_NEIGHBOR_COUNT 4
#define RL_MAPGEN_BSP_RANDOMISE_ROOM_LOC 0 // don't randomise room loc within the BSP leaves so we can easily find the room point to connect corridors
#define RL_IMPLEMENTATION
#define RL_ENABLE_PATHFINDING 1 /* required for RL_Point */
#define RL_ENABLE_FOV 0
#include "roguelike.h"

/**
 * Assembly functions to interact with Commodore 64 hardware directly
 */

void c64_kb_init()
{
    asm("lda #1");
    asm("sta $0289");  // set keyboard buffer length to 1
    // enable all keys to repeat
    asm("lda #0b10000000");
    asm("sta $028a");
    // disable key repeat
    /* asm("lda #0"); */
    /* asm("sta $028a"); */
}

char c64_getch()
{
    char r;
    asm("jsr 0xff9f"); // scan keyboard
    asm("jsr 0xffe4"); // read kb buffer
    asm volatile("sta %0" : "=r"(r));

    return r;
}

char c64_getch_blocking()
{
    char ch = 0;
    while (ch == 0) {
        ch = c64_getch();
    }

    return ch;
}

int c64_rand()
{
    // query SID chip for RNG
    return *((volatile char*)0xD41B);
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

void c64_seed()
{
    // setup the SID chip to generate random noise
    asm("LDA #0xFF");  // ; maximum frequency value
    asm("STA $D40E"); // ; voice 3 frequency low byte
    asm("STA $D40F"); // ; voice 3 frequency high byte
    asm("LDA #0x80");  // ; noise waveform, gate bit off
    asm("STA $D412"); // ; voice 3 control register
    c64_rand();
}

#define WIDTH 40
#define HEIGHT 25
#define SCREEN_WIDTH 40
#define SCREEN_SIZE  40*25

RL_BSP* bsp_random_leaf(RL_BSP *root)
{
    if (root == NULL)
        return NULL;

    RL_BSP *node = root;
    while (!rl_bsp_is_leaf(node)) {
        if (rl_rng_generate(0, 1)) {
            node = node->left;
        } else {
            node = node->right;
        }
    }

    return node;
}

// draw tiles in simplistic FOV around player
void draw_fov(RL_Map *map, int origin_idx)
{
    int neighbors[8] = {
        origin_idx + 1,
        origin_idx - 1,
        origin_idx + SCREEN_WIDTH,
        origin_idx - SCREEN_WIDTH,
        origin_idx + 1 + SCREEN_WIDTH,
        origin_idx - 1 + SCREEN_WIDTH,
        origin_idx + 1 - SCREEN_WIDTH,
        origin_idx - 1 - SCREEN_WIDTH,
    };
    for (int i=0; i<8; ++i) {
        int idx = neighbors[i];
        if (idx >= 0 && idx < SCREEN_SIZE) {
            RL_Byte t = map->tiles[idx];
            char ch = 32;
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
            char cur_ch = c64_screen_get(idx);
            if (cur_ch != ch && cur_ch != 0) { // if chars are different and not equal to @ draw
                c64_screen_set(idx, ch);
                if (t == RL_TileRoom) {
                    draw_fov(map, idx); // keep drawing FOV if we're in a room
                }
            }
        }
    }
}

RL_Map map = { .width = WIDTH, .height = HEIGHT, .tiles = (RL_Byte[SCREEN_SIZE]) {0} };
RL_Point player;

void map_generate()
{
    RL_MapgenConfigBSP config = {
        .room_min_width = 4,
        .room_max_width = 6,
        .room_min_height = 4,
        .room_max_height = 6,
        .room_padding = 0,
        .draw_doors = false,
        .draw_corridors = RL_ConnectSimple,
        .max_splits = 5,
    };
    printf("Generating bsp...\n");
    RL_BSP *bsp = rl_bsp_create(WIDTH, HEIGHT);
    memset(map.tiles, RL_TileRock, sizeof(*map.tiles)*map.width*map.height);
    rl_mapgen_bsp_ex(&map, bsp, &config);
    printf("Done\n");

    // pick random room for player start
    printf("Random start node...\n");
    RL_BSP *start_bsp = rl_bsp_random_leaf(bsp);
    assert(start_bsp);
    player.x = start_bsp->x + start_bsp->width/2;
    player.y = start_bsp->y + start_bsp->height/2;
    // pick random room for downstairs
    RL_BSP *downstair_bsp;
    while ((downstair_bsp = rl_bsp_random_leaf(bsp)) && downstair_bsp == start_bsp) {}
    assert(downstair_bsp);
    int downstair_x = downstair_bsp->x + downstair_bsp->width/2;
    int downstair_y = downstair_bsp->y + downstair_bsp->height/2;
    map.tiles[downstair_x + downstair_y*WIDTH] = '>';
    rl_bsp_destroy(bsp);
    printf("Done\n");

    // clear screen & draw player
    memset((char*) 0x0400, 32, SCREEN_SIZE);
    c64_screen_set(player.x + player.y*SCREEN_WIDTH, 0);
}

int main(void)
{
    c64_seed();
    c64_kb_init();
    map_generate();

    // game loop
    for (;;) {
        char start = C64_JIFFIES;
        // simplistic FOV - we should also be able to easily light up rooms, à la rogue
        int player_idx = player.x + player.y*SCREEN_WIDTH;
        draw_fov(&map, player_idx);
        // wait for a few jiffies
        while (abs(C64_JIFFIES - start) < 5) {
            c64_getch(); // clear keyboard buffer for input responsiveness
        }
        // input
        char ch = c64_getch_blocking();
        RL_Point new_loc = player;
        switch (ch) {
            case 72: // h
                new_loc.x -= 1;
                break;
            case 74: // j
                new_loc.y += 1;
                break;
            case 75: // k
                new_loc.y -= 1;
                break;
            case 76: // l
                new_loc.x += 1;
                break;
            default:
                break;
        }
        int new_idx = new_loc.x + new_loc.y*SCREEN_WIDTH;
        if (!rl_map_in_bounds(&map, new_loc.x, new_loc.y)) {
            // no-op
        } else if (map.tiles[new_idx] == '>') {
            map_generate();
        } else if (rl_map_is_passable(&map, new_loc.x, new_loc.y)) {
            player = new_loc;
            c64_screen_set(player_idx, 32);
            c64_screen_set(new_idx, 0);
        }
    }

    return 0;
}
