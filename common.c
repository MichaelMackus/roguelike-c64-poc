#include <assert.h>

void platform_init();
void platform_deinit();
char platform_getch();
char platform_screen_code(char ch);
void platform_screen_set(int idx, char ch);
char platform_screen_get(int idx);
void platform_screen_clear();

typedef struct {
    int x;
    int y;
} MapPoint;

#include "roguelike.h"
#include <stdio.h>
#include <string.h>

#define SCREEN_WIDTH  40
#define SCREEN_HEIGHT 25
#define SCREEN_SIZE   SCREEN_WIDTH * SCREEN_HEIGHT

// draw tiles in simplistic FOV around player
void draw_fov(RL_Map *map, int origin_idx, int player_idx)
{
    int i;
    int neighbors[8];
    neighbors[0] = origin_idx + 1;
    neighbors[1] = origin_idx - 1;
    neighbors[2] = origin_idx + SCREEN_WIDTH;
    neighbors[3] = origin_idx - SCREEN_WIDTH;
    neighbors[4] = origin_idx + 1 + SCREEN_WIDTH;
    neighbors[5] = origin_idx - 1 + SCREEN_WIDTH;
    neighbors[6] = origin_idx + 1 - SCREEN_WIDTH;
    neighbors[7] = origin_idx - 1 - SCREEN_WIDTH;
    for (i=0; i<8; ++i) {
        int idx = neighbors[i];
        if (idx >= 0 && idx < SCREEN_SIZE && idx != player_idx) {
            RL_Byte t = map->tiles[idx];
            char ch = ' ';
            char code;
            char cur_code;
            switch (t) {
                case RL_TileRock:
                    ch = '#';
                    break;
                case RL_TileRoom:
                    ch = '.';
                    break;
                case RL_TileCorridor:
                    break;
                case RL_TileDoor:
                    ch = '+';
                    break;
                case '>':
                    ch = '>';
                    break;
                default:
                    assert("Unhandled tile" && 0);
                    break;
            }
            code = platform_screen_code(ch);
            cur_code = platform_screen_get(idx);
            /* if (cur_code != code && cur_code != 0) { // if chars are different and not equal to @ draw */
            if (cur_code != code) { // if chars are different and not equal to @ draw
                platform_screen_set(idx, code);
                if (t == RL_TileRoom) {
                    draw_fov(map, idx, player_idx); // keep drawing FOV if we're in a room
                }
            }
        }
    }
}

void map_generate(RL_Map *map, RL_BSP *bsp, MapPoint *player)
{
    RL_BSP *start_bsp, *downstair_bsp;
    int downstair_x, downstair_y;
    RL_MapgenConfigBSP config = RL_MAPGEN_BSP_DEFAULTS;
    config.room_padding = 0;
    config.draw_corridors = RL_ConnectSimple;
    config.draw_doors = false;
    config.max_splits = 5;
    printf("Generating bsp...\n");
    if (bsp->left)  rl_bsp_destroy(bsp->left); /* free children */
    if (bsp->right) rl_bsp_destroy(bsp->right);
    bsp->left = bsp->right = NULL;
    bsp->width = map->width;
    bsp->height = map->height;
    memset(map->tiles, RL_TileRock, sizeof(*map->tiles)*map->width*map->height);
    rl_mapgen_bsp_ex(map, bsp, &config);
    printf("Done\n");

    // pick random room for player start
    printf("Random start node...\n");
    start_bsp = rl_bsp_random_leaf(bsp);
    assert(start_bsp);
    player->x = start_bsp->x + start_bsp->width/2;
    player->y = start_bsp->y + start_bsp->height/2;
    // pick random room for downstairs
    while ((downstair_bsp = rl_bsp_random_leaf(bsp)) && downstair_bsp == start_bsp) {}
    assert(downstair_bsp);
    downstair_x = downstair_bsp->x + downstair_bsp->width/2;
    downstair_y = downstair_bsp->y + downstair_bsp->height/2;
    map->tiles[downstair_x + downstair_y*map->width] = '>';
    printf("Done\n");

    // clear screen & draw player
    platform_screen_clear();
    platform_screen_set(player->x + player->y*map->width, platform_screen_code('@'));
}
