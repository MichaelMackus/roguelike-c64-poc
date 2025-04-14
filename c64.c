#include <stdio.h>
#include <assert.h>

/**
 * Some setup is needed since LLVM-MOS doesn't provide very good RNG or math functions.
 */

#define fabs(a) ((a) < 0 ? (a) * -1 : (a))
#define sqrt(a) a

#define RL_RNG_CUSTOM
int c64_rand();
unsigned int rl_rng_generate(unsigned int min, unsigned int max)
{
    assert(max >= min);
    assert(max <= 255);
    if (min == max) return min;

    return min + c64_rand() % (max - min + 1);
}

#define C64_JIFFIES (*((volatile char*)0xA2))

#define RL_MAPGEN_BSP_RANDOMISE_ROOM_LOC 0 // don't randomise room loc within the BSP leaves so we can easily find the room point to connect corridors
#define RL_IMPLEMENTATION
#include "roguelike.h"


/**
 * Assembly functions to interact with Commodore 64 hardware directly
 */

void c64_kb_init()
{
    asm("lda #1");
    asm("sta $0289");  // set keyboard buffer length to 1
    // enable all keys to repeat
    /* asm("lda #0b10000000"); */
    /* asm("sta $028a"); */
    // disable key repeat
    asm("lda #0");
    asm("sta $028a");
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
    return *((char*)0xD41B);
}

void c64_screen(int idx, char screencode)
{
    char *screen = (char*) 0x0400;
    screen[idx] = screencode;
}

void c64_seed()
{
    // setup the SID chip to generate random noise
    asm("LDA #0xFF");  // ; maximum frequency value
    asm("STA 0xD40E"); // ; voice 3 frequency low byte
    asm("STA 0xD40F"); // ; voice 3 frequency high byte
    asm("LDA #0x80");  // ; noise waveform, gate bit off
    asm("STA 0xD412"); // ; voice 3 control register
    c64_rand();
}

#define WIDTH 40
#define HEIGHT 25

// custom corridor connection to most efficiently connect leaves of the BSP tree
void connect_corridors(RL_Map *map, RL_BSP *root)
{
    rl_assert(map && root);
    if (!map || !root) return;

    // connect siblings
    RL_BSP *node = root->left;
    RL_BSP *sibling = root->right;
    if (node == NULL || sibling == NULL) return;

    // find rooms in BSP
    RL_BSP *left = node, *right = sibling;
    while (!rl_bsp_is_leaf(left)) {
        if (rl_rng_generate(0, 1)) {
            left = left->left;
        } else {
            left = left->right;
        }
    }
    while (!rl_bsp_is_leaf(right)) {
        if (rl_rng_generate(0, 1)) {
            right = right->left;
        } else {
            right = right->right;
        }
    }
    RL_Point dig_start = RL_XY(left->x + left->width / 2, left->y + left->height / 2);
    RL_Point dig_end = RL_XY(right->x + right->width / 2, right->y + right->height / 2);
    rl_assert(rl_map_is_passable(map, dig_end));
    rl_assert(!(dig_start.x == dig_end.x && dig_start.y == dig_end.y));

    // carve out corridors
    RL_Point cur = dig_start;
    int direction = 0;
    if (fabs(cur.y - dig_end.y) > fabs(cur.x - dig_end.x)) {
        direction = 1;
    }
    while (cur.x != dig_end.x || cur.y != dig_end.y) {
        // prevent digging float wide corridors
        RL_Point next = cur;
        if (direction == 0) { // digging left<->right
            if (cur.x == dig_end.x) {
                direction = !direction;
            } else {
                next.x += dig_end.x < cur.x ? -1 : 1;
            }
        }
        if (direction == 1) { // digging up<->down
            if (cur.y == dig_end.y) {
                direction = !direction;
            } else {
                next.y += dig_end.y < cur.y ? -1 : 1;
            }
        }
        // dig
        if (map->tiles[(int)cur.x + (int)cur.y*map->width] == RL_TileRock) {
            map->tiles[(int)cur.x + (int)cur.y*map->width] = RL_TileCorridor;
        }
        cur = next;
    }

    // connect siblings' children
    connect_corridors(map, node);
    connect_corridors(map, sibling);
}

int main(void)
{
    c64_seed();
    c64_kb_init();

    RL_Map *map = rl_map_create(WIDTH, HEIGHT);
    RL_MapgenConfigBSP config = {
        .room_min_width = 4,
        .room_max_width = 6,
        .room_min_height = 4,
        .room_max_height = 6,
        .room_padding = 0,
        .draw_doors = false,
        .draw_corridors = RL_ConnectNone,
    };
    printf("Generating bsp...\n");
    RL_BSP *bsp = rl_mapgen_bsp_ex(map, config);
    printf("Done\n");

    printf("Connecting corridors...\n");
    connect_corridors(map, bsp);
    printf("Done\n");

    // pick random room
    RL_Point player;
    for (int y=0; y<HEIGHT; y++) {
        int x;
        for (x=0; x<WIDTH; x++) {
            if (map->tiles[map->width*y + x] == RL_TileRoom) {
                player.x = x;
                player.y = y;
                break;
            }
        }
        if (map->tiles[map->width*y + x] == RL_TileRoom) break;
    }
    memset((char*) 0x0400, 32, WIDTH*HEIGHT);

    for (;;) {
        // simplistic FOV - we should also be able to easily light up rooms, à la rogue
        int player_idx = player.x + player.y*WIDTH;
        /* char start = C64_JIFFIES; */
        int neighbors[8] = {
            player_idx + 1,
            player_idx - 1,
            player_idx + WIDTH,
            player_idx - WIDTH,
            player_idx + 1 + WIDTH,
            player_idx - 1 + WIDTH,
            player_idx + 1 - WIDTH,
            player_idx - 1 - WIDTH,
        };
        c64_screen(player_idx, 0);
        for (int i=0; i<8; ++i) {
            int idx = neighbors[i];
            if (idx >= 0 && idx < 1000) {
                /* fov->visibility[idx] = RL_TileVisible; */
                RL_Tile t = map->tiles[idx];
                switch (t) {
                    case RL_TileRock:
                        c64_screen(idx, 102);
                        break;
                    case RL_TileRoom:
                        c64_screen(idx, 123);
                        break;
                    case RL_TileCorridor:
                        /* draw_buffer[x + y*WIDTH] = 32; */
                        break;
                    case RL_TileDoor:
                        c64_screen(idx, 42);
                        break;
                }
            }
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
        if (rl_map_is_passable(map, new_loc)) {
            int old_idx = player.x + player.y*WIDTH;
            player = new_loc;
            int new_idx = player.x + player.y*WIDTH;
            c64_screen(old_idx, 32);
            c64_screen(new_idx, 0);
        }
        /* while (abs(C64_JIFFIES - start) < 20) {} */
    }

    rl_map_destroy(map);

    return 0;
}
