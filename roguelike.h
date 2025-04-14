#ifndef RL_ROGUELIKE_H
#define RL_ROGUELIKE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>

/**
 * MIT License
 *
 * Copyright (c) 2024 Michael H. Mackus
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Generic structs for library.
 */

// each tile is the size of 1 byte, so it can be casted back & forth from char <-> RL_Tile
typedef unsigned char RL_Byte;

// Generic dungeon map structure, supporting hex & square 2d maps, along with the associated tile enum.
typedef enum {
    RL_TileRock = ' ',
    RL_TileRoom = '.',
    RL_TileCorridor = '#',
    RL_TileDoor = '+',
} RL_Tile;
typedef struct RL_Map {
    unsigned int width;
    unsigned int height;
    RL_Byte *tiles; // a sequential array of RL_Tiles, stride for each row equals the map width.
} RL_Map;

// Type of wall on the map - idea is they can be bitmasked together (e.g. for corners). See rl_map_wall and other
// related functions.
typedef enum {
    RL_WallToWest  = 1,
    RL_WallToEast  = 1 << 1,
    RL_WallToNorth = 1 << 2,
    RL_WallToSouth = 1 << 3,
    RL_WallOther   = 1 << 7, // e.g. a wall that has no connecting walls
} RL_Wall;

// Structure containing information for the FOV algorithm, along with the associated visibility enum.
typedef enum {
    RL_TileCannotSee = 0,
    RL_TileVisible,
    RL_TileSeen,
} RL_TileVisibility;
typedef struct {
    unsigned int width;
    unsigned int height;
    RL_Byte *visibility; // a sequential array of RL_Visibility, stride for each row = the map width
} RL_FOV;

// A point on the map. The points are a float type for flexibility, but for most roguelikes they will probably be casted
// from an integer type.
typedef struct RL_Point {
    float x, y;
} RL_Point;

// Macro to easily create a RL_Point.
#define RL_XY(x, y) (RL_Point) { (float)(x), (float)(y) }

// BSP tree
typedef struct RL_BSP {
    unsigned int width;
    unsigned int height;
    unsigned int x;
    unsigned int y;
    struct RL_BSP *parent;
    struct RL_BSP *left;  // left child
    struct RL_BSP *right; // right child
} RL_BSP;

// BSP split direction
typedef enum {
    RL_SplitHorizontally, // split the BSP node on the x axis (splits width)
    RL_SplitVertically,   // split the BSP node on the y axis (splits height)
} RL_SplitDirection;

// Max neighbors for a pathfinding node.
#ifndef RL_MAX_NEIGHBOR_COUNT
#define RL_MAX_NEIGHBOR_COUNT 8
#endif

// Represents a graph of pathfinding nodes that has been scored for pathfinding (e.g. with the Dijkstra algorithm).
typedef struct RL_GraphNode {
    float score; // will be FLT_MAX for an unreachable/unscored node in the Dijkstra algorithm
    RL_Point point;
    size_t neighbors_length;
    struct RL_GraphNode *neighbors[RL_MAX_NEIGHBOR_COUNT];
} RL_GraphNode;
typedef struct RL_Graph {
    size_t length; // length of nodes
    RL_GraphNode *nodes; // array of nodes - length will be the size of the map.width * map.height
} RL_Graph;

// A path is a linked list of paths. You can "walk" a path using rl_path_walk which will simultaneously free the
// previous path.
typedef struct RL_Path {
    RL_Point point;
    struct RL_Path *next;
} RL_Path;

/**
 * Random map generation
 */

// Creates empty map and fills it with impassable tiles (width & height
// must be positive). Make sure to call rl_map_destroy to clear memory.
RL_Map *rl_map_create(unsigned int width, unsigned int height);

// Frees map tile memory.
void rl_map_destroy(RL_Map *map);

// Enum representing the type of corridor connection algorithm. RL_ConnectRandomly is the default and results in the
// most interesting & aesthetic maps.
typedef enum {
    RL_ConnectNone = 0,       // don't connect corridors
    RL_ConnectRandomly,       // connect corridors to random rooms using Dijkstra algorithm for smarter digging
    RL_ConnectBSP,            // connect corridors via the BSP graph (faster than above but less circular/interesting maps) using Dijkstra algorithm
    RL_ConnectBSPSimple,      // connect corridors via the BSP just drawing straight lines (fastest)
} RL_MapgenCorridorConnection;

// The config for BSP map generation - note that the dimensions *include* the walls on both sides, so the min room width
// & height the library accepts is 3.
typedef struct {
    unsigned int room_min_width;
    unsigned int room_max_width;
    unsigned int room_min_height;
    unsigned int room_max_height;
    unsigned int room_padding;
    RL_MapgenCorridorConnection draw_corridors; // type of corridor connection algorithm to use
    bool draw_doors; // whether to draw doors while connecting corridors
} RL_MapgenConfigBSP;

// Provide some defaults for mapgen.
#ifndef RL_MAPGEN_BSP_DEFAULTS
#define RL_MAPGEN_BSP_DEFAULTS ((RL_MapgenConfigBSP) { \
    .room_min_width = 4, \
    .room_max_width = 6, \
    .room_min_height = 4, \
    .room_max_height = 6, \
    .room_padding = 1, \
    .draw_corridors = RL_ConnectRandomly, \
    .draw_doors = true, \
})
#endif

// Generate map rooms with BSP split & discard the BSP.
void rl_mapgen_bsp(RL_Map *map, RL_MapgenConfigBSP config);

// Same as the above function but returns the generated BSP. Make sure to free the BSP with rl_bsp_destroy.
RL_BSP *rl_mapgen_bsp_ex(RL_Map *map, RL_MapgenConfigBSP config);

// Called by rl_mapgen_bsp when specifying RL_ConnectBSP and RL_ConnectBSPSimple.
//
// The BSP graph is used to find the "rooms" to connect corridors to. With rl_mapgen_connect_corridors_bsp the algorithm
// traverses the BSP graph downward, recursively connecting siblings at each level. This ensures that the entire BSP
// graph is connected via corridors. If you pass NULL for the dijkstra graph it will fallback to drawing straight lines.
void rl_mapgen_connect_corridors_bsp(RL_Map *map, RL_BSP *root, bool draw_doors, RL_Graph *graph);

// Called by rl_mapgen_bsp when specifying RL_ConnectRandomly.
//
// The BSP graph is used to find the "rooms" to connect corridors to. This algorithm algorithm traverses the BSP graph's
// leaf nodes from left to right, connecting each to another random leaf node. This does result in some cases where the
// entire BSP graph is not fully connected to corridors - rl_mapgen_bsp automatically culls these unconnected rooms
// using rl_map_largest_connected_area.
void rl_mapgen_connect_corridors_randomly(RL_Map *map, RL_BSP *root, bool draw_doors);

/**
 * Generic map helper functions.
 */

// Verifies a coordinates is within bounds of map.
bool rl_map_in_bounds(const RL_Map *map, RL_Point point);

// Checks if a tile is passable.
bool rl_map_is_passable(const RL_Map *map, RL_Point point);

// Get tile at point
RL_Byte *rl_map_tile(const RL_Map *map, RL_Point point);

// Returns 1 if tile at point matches given parameter.
bool rl_map_tile_is(const RL_Map *map, RL_Point point, RL_Tile tile);

// A tile is considered a wall if it is touching a passable tile.
//
// Returns a bitmask of the RL_Wall enum. For example, a wall with a wall tile to the south, west, and east would have a
// bitmask of 0b1011.
RL_Byte rl_map_wall(const RL_Map *map, RL_Point point);

// Is the tile a wall tile?
bool rl_map_is_wall(const RL_Map *map, RL_Point point);

// Is the wall a corner?
bool rl_map_is_corner_wall(const RL_Map *map, RL_Point point);

// Is this a wall that is touching a room tile?
bool rl_map_is_room_wall(const RL_Map *map, RL_Point point);

// A wall that is touching a room tile (e.g. to display it lit).
RL_Byte rl_map_room_wall(const RL_Map *map, RL_Point point);

// Returns a the largest connected area (of passable tiles) on the map. Make sure to destroy the graph with
// rl_graph_destroy after you are done.
RL_Graph *rl_map_largest_connected_area(const RL_Map *map);

/**
 * Simple priority queue implementation
 */

typedef struct {
    void **heap;
    int cap;
    int len;
    int (*comparison_f)(const void *heap_item_a, const void *heap_item_b);
} RL_Heap;

// Allocates memory for the heap. Make sure to call rl_heap_destroy after you are done.
//
// capacity - initial capacity for the heap
// comparison_f - A comparison function that returns 1 if heap_item_a should be
//  popped from the queue before heap_item_b. If NULL the heap will still work
//  but order will be undefined.
RL_Heap *rl_heap_create(int capacity, int (*comparison_f)(const void *heap_item_a, const void *heap_item_b));

// Free up the allocated heap memory.
void rl_heap_destroy(RL_Heap *h);

// Return the length of the heap items
int rl_heap_length(RL_Heap *h);

// Insert item into the heap. This will resize the heap if necessary.
bool rl_heap_insert(RL_Heap *h, void *item);

// Returns & removes an item from the queue.
void *rl_heap_pop(RL_Heap *h);

// Peek at the first item in the queue. This does not remove the item from the queue.
void *rl_heap_peek(RL_Heap *h);

/**
 * BSP Manipulation
 */

// Params width & height must be positive.
RL_BSP *rl_bsp_create(unsigned int width, unsigned int height);
void rl_bsp_destroy(RL_BSP *root);

// Split the BSP by direction - this creates the left & right leaf and
// populates them in the BSP node. Position must be positive and within
// the BSP root node. Also node->left & node->right must be NULL
void rl_bsp_split(RL_BSP *node, unsigned int position, RL_SplitDirection direction);

// Recursively split the BSP. Used for map generation.
//
// Returns true if the BSP was able to split at least once
bool rl_bsp_recursive_split(RL_BSP *root, unsigned int min_width, unsigned int min_height, unsigned int max_recursion);

// Returns 1 if the node is a leaf node.
bool rl_bsp_is_leaf(RL_BSP *node);

// Return sibling node. Returns NULL if there is no parent (i.e. for the root
// node).
RL_BSP *rl_bsp_sibling(RL_BSP *node);

// Returns amount of leaves in tree.
size_t rl_bsp_leaf_count(RL_BSP *root);

// Return the next leaf node to the right if it exists.
RL_BSP *rl_bsp_next_leaf(RL_BSP *node);

/**
 * Pathfinding
 */

// Useful distance functions for pathfinding.
float rl_distance_manhattan(RL_Point node, RL_Point end);
float rl_distance_euclidian(RL_Point node, RL_Point end);
float rl_distance_chebyshev(RL_Point node, RL_Point end);

// Custom distance function for pathfinding - calculates distance between map nodes
typedef float (*RL_DistanceFun)(RL_Point from, RL_Point to);

// Custom passable function for pathfinding. Return 0 to prevent neighbor from being included in graph.
typedef bool (*RL_PassableFun)(const RL_Map *map, RL_Point point);

// Custom score function for pathfinding - most users won't need this, but it gives flexibility in weighting the
// Dijkstra graph. Note that Dijkstra expects you to add the current node's score to the newly calculated score.
typedef float (*RL_ScoreFun)(RL_GraphNode *current, RL_GraphNode *neighbor, void *context);

// Generates a line starting at from ending at to. Each path in the line will be incremented by step.
RL_Path *rl_line_create(RL_Point from, RL_Point to, float step);

// Find a path between start and end via Dijkstra algorithm. Make sure to call rl_path_destroy when done with path.
// Pass NULL to distance_f to use rough approximation for euclidian.
RL_Path *rl_path_create(const RL_Map *map, RL_Point start, RL_Point end, RL_DistanceFun distance_f, RL_PassableFun passable_f);

// Find a path between start and end via the scored Dijkstra graph. Make sure to call rl_path_destroy when done with path (or
// use rl_path_walk).
RL_Path *rl_path_create_from_graph(const RL_Graph *graph, RL_Point start);

// Convenience function to "walk" the path. This will return the next path, freeing the current path. You do not need to
// call rl_path_destroy if you walk the full path.
RL_Path *rl_path_walk(RL_Path *path);

// Destroy & clean up all nodes from path onward.
void rl_path_destroy(RL_Path *path);

// Dijkstra pathfinding algorithm. Pass NULL to distance_f to use rough approximation for euclidian.  Pass NULL to
// passable_f to pass through impassable tiles, otherwise pass rl_map_is_passable for the default.
//
// You can use Dijkstra maps for pathfinding, simple AI, and much more. For example, by setting the player point to
// "start" then you can pick the highest scored tile in the map and set that as the new "start" point. As with all
// Dijkstra maps, you just walk the map by picking the lowest scored neighbor. This is a simplistic AI resembling a
// wounded NPC fleeing from the player.
//
// Make sure to destroy the resulting RL_Graph with rl_graph_destroy.
RL_Graph *rl_dijkstra_create(const RL_Map *map,
                            RL_Point start,
                            RL_DistanceFun distance_f,
                            RL_PassableFun passable_f);

// Dijkstra pathfinding algorithm. Uses RL_Graph so that your code doesn't need to rely on RL_Map. Each node's
// distance should equal FLT_MAX in the resulting graph if it is impassable.
void rl_dijkstra_score(RL_Graph *graph, RL_Point start, RL_DistanceFun distance_f);

// Dijkstra pathfinding algorithm for advanced use cases such as weighting certain tiles higher than others. Uses
// RL_Graph so that your code doesn't need to rely on RL_Map. Each node's distance should equal FLT_MAX in the resulting
// graph if it is impassable. Most users should just use rl_dijkstra_score - only use this if you have a specific need.
void rl_dijkstra_score_ex(RL_Graph *graph, RL_Point start, RL_ScoreFun score_f, void *score_context);

// Create an unscored graph based on the 2d map. Make sure to call rl_graph_destroy when finished.
RL_Graph *rl_graph_create(const RL_Map *map, RL_PassableFun passable_f, bool allow_diagonal_neighbors);

// Free graph memory.
void rl_graph_destroy(RL_Graph *graph);

/**
 * FOV
 */

// Creates empty FOV and fills it with opaque tiles. Make sure to call rl_fov_destroy to clear memory.
RL_FOV *rl_fov_create(unsigned int width, unsigned int height);

// Frees map tile memory.
void rl_fov_destroy(RL_FOV *fov);

// Function to determine if a tile is considered Opaque for FOV calculation. Make sure you do bounds checking that the point is within your map.
typedef bool (*RL_IsOpaqueFun)(RL_Point point, void *context);
// Function to mark a tile as visible within the FOV. Make sure you do bounds checking that the point is within your map.
typedef void (*RL_MarkAsVisibleFun)(RL_Point point, void *context);

// Calculate FOV using simple shadowcasting algorithm. Set fov_radius to a negative value to have unlimited FOV (note
// this is limited by RL_MAX_RECURSION).
//
// Note that this sets previously visible tiles to RL_TileSeen.
void rl_fov_calculate(RL_FOV *fov, const RL_Map *map, RL_Point start, int fov_radius, RL_DistanceFun distance_f);

// Calculate FOV using simple shadowcasting algorithm. Set fov_radius to a negative value to have unlimited FOV (note
// this is limited by RL_MAX_RECURSION).
//
// Generic version of above function.
void rl_fov_calculate_ex(void *context, RL_Point start, int fov_radius, RL_DistanceFun distance_f, RL_IsOpaqueFun opaque_f, RL_MarkAsVisibleFun mark_visible_f);

// Checks if a point is visible within FOV. Make sure to call rl_fov_calculate_for_map first.
bool rl_fov_is_visible(const RL_FOV *map, RL_Point point);

// Checks if a point has been seen within FOV. Make sure to call rl_fov_calculate_for_map first.
bool rl_fov_is_seen(const RL_FOV *map, RL_Point point);

/**
 * Random number generation
 */

// Define RL_RNG_CUSTOM to provide your own function body for rl_rng_generate.
unsigned int rl_rng_generate(unsigned int min, unsigned int max);
#endif

#ifdef RL_IMPLEMENTATION

#include <time.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>

#ifndef RL_FOV_SYMMETRIC
#define RL_FOV_SYMMETRIC 1
#endif

#ifndef RL_MAX_RECURSION
#define RL_MAX_RECURSION 100
#endif

#ifndef RL_MAPGEN_BSP_DEVIATION
#define RL_MAPGEN_BSP_DEVIATION 0
#endif

#define RL_UNUSED(x) (void)x

#ifndef rl_assert
#include <assert.h>
#define rl_assert(expr)		(assert(expr));
#endif

#ifndef rl_malloc
#define rl_malloc malloc
#endif

#ifndef rl_calloc
#define rl_calloc calloc
#endif

#ifndef rl_realloc
#define rl_realloc realloc
#endif

#ifndef rl_free
#define rl_free free
#endif

RL_Map *rl_map_create(unsigned int width, unsigned int height)
{
    rl_assert(width*height < UINT_MAX);
    rl_assert(width > 0 && height > 0);
    RL_Map *map = NULL;
    // allocate all the memory we need at once
    char *memory = (char*) rl_calloc(sizeof(*map) + sizeof(*map->tiles)*width*height, 1);
    rl_assert(memory);
    if (memory == NULL) return NULL;
    map = (RL_Map*) memory;
    map->width = width;
    map->height = height;
    map->tiles = (RL_Byte*) (memory + sizeof(*map));
    if (memset(map->tiles, RL_TileRock, sizeof(*map->tiles)*width*height) == NULL) {
        rl_assert("Error initializing RL_Map tiles to RL_TileRock!" && false);
    }
    rl_assert(map);
    rl_assert(map->tiles);

    return map;
}

void rl_map_destroy(RL_Map *map)
{
    if (map) {
        rl_free(map);
    }
}

bool rl_map_in_bounds(const RL_Map *map, RL_Point point)
{
    return point.x >= 0 && point.y >= 0 && point.x < map->width && point.y < map->height;
}

bool rl_map_is_passable(const RL_Map *map, RL_Point point)
{
    if (rl_map_in_bounds(map, point)) {
        return map->tiles[(size_t)point.y * map->width + (size_t)point.x] == RL_TileRoom ||
               map->tiles[(size_t)point.y * map->width + (size_t)point.x] == RL_TileCorridor ||
               map->tiles[(size_t)point.y * map->width + (size_t)point.x] == RL_TileDoor;
    }

    return 0;
}

RL_Byte *rl_map_tile(const RL_Map *map, RL_Point point)
{
    if (rl_map_in_bounds(map, point)) {
        return &map->tiles[(size_t)point.x + (size_t)point.y*map->width];
    }

    return NULL;
}

bool rl_map_is_wall(const RL_Map *map, RL_Point point)
{
    int y = point.y;
    int x = point.x;
    if (!rl_map_in_bounds(map, point))
        return 0;
    if (!rl_map_is_passable(map, point) || rl_map_tile_is(map, point, RL_TileDoor)) {
        return rl_map_is_passable(map, (RL_Point){ x, y + 1 }) ||
               rl_map_is_passable(map, (RL_Point){ x, y - 1 }) ||
               rl_map_is_passable(map, (RL_Point){ x + 1, y }) ||
               rl_map_is_passable(map, (RL_Point){ x - 1, y }) ||
               rl_map_is_passable(map, (RL_Point){ x + 1, y - 1 }) ||
               rl_map_is_passable(map, (RL_Point){ x - 1, y - 1 }) ||
               rl_map_is_passable(map, (RL_Point){ x + 1, y + 1 }) ||
               rl_map_is_passable(map, (RL_Point){ x - 1, y + 1 });
    }

    return 0;
}

// checks if target tile is connecting from source (e.g. they can reach it)
static float rl_distance_simple(RL_Point node, RL_Point end);
bool rl_map_is_connecting(const RL_Map *map, RL_Point from, RL_Point target)
{
    // check that from passable neighbors can connect to target
    for (int x = from.x - 1; x <= from.x + 1; ++x) {
        for (int y = from.y - 1; y <= from.y + 1; ++y) {
            if (!rl_map_in_bounds(map, RL_XY(x, y)) || !rl_map_is_passable(map, RL_XY(x, y)))
                continue;
            if (rl_map_tile_is(map, RL_XY(x, y), RL_TileDoor))
                continue;
            // this is a passable neighbor - check its neighbors to see if it can reach target
            for (int x2 = x - 1; x2 <= x + 1; ++x2) {
                for (int y2 = y - 1; y2 <= y + 1; ++y2) {
                    if (!rl_map_in_bounds(map, RL_XY(x2, y2))) continue;
                    if (x2 == target.x && y2 == target.y)
                        return true;
                }
            }
        }
    }

    return false;
}

RL_Byte rl_map_wall(const RL_Map *map, RL_Point point)
{
    RL_Byte mask = 0;
    if (!rl_map_is_wall(map, point))
        return mask;
    if (rl_map_is_wall(map, RL_XY(point.x + 1, point.y)) && rl_map_is_connecting(map, point, RL_XY(point.x + 1, point.y)))
        mask |= RL_WallToEast;
    if (rl_map_is_wall(map, RL_XY(point.x - 1, point.y)) && rl_map_is_connecting(map, point, RL_XY(point.x - 1, point.y)))
        mask |= RL_WallToWest;
    if (rl_map_is_wall(map, RL_XY(point.x,     point.y - 1)) && rl_map_is_connecting(map, point, RL_XY(point.x,     point.y - 1)))
        mask |= RL_WallToNorth;
    if (rl_map_is_wall(map, RL_XY(point.x,     point.y + 1)) && rl_map_is_connecting(map, point, RL_XY(point.x,     point.y + 1)))
        mask |= RL_WallToSouth;
    return mask ? mask : RL_WallOther;
}

bool rl_map_is_corner_wall(const RL_Map *map, RL_Point point)
{
    int wall = rl_map_wall(map, point);
    if (!wall) return 0;
    return (wall & RL_WallToWest && wall & RL_WallToNorth) ||
           (wall & RL_WallToWest && wall & RL_WallToSouth) ||
           (wall & RL_WallToEast && wall & RL_WallToNorth) ||
           (wall & RL_WallToEast && wall & RL_WallToSouth);
}

bool rl_map_tile_is(const RL_Map *map, RL_Point point, RL_Tile tile)
{
    if (!rl_map_in_bounds(map, point)) return 0;
    return map->tiles[(size_t)point.x + (size_t)point.y*map->width] == tile;
}

bool rl_map_is_room_wall(const RL_Map *map, RL_Point point)
{
    int y = point.y;
    int x = point.x;
    if (!rl_map_is_wall(map, point))
        return 0;

    return rl_map_tile_is(map, (RL_Point){ x, y + 1 }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x, y - 1 }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x + 1, y }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x - 1, y }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x + 1, y - 1 }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x - 1, y - 1 }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x + 1, y + 1 }, RL_TileRoom) ||
           rl_map_tile_is(map, (RL_Point){ x - 1, y + 1 }, RL_TileRoom);
}

RL_Byte rl_map_room_wall(const RL_Map *map, RL_Point point)
{
    RL_Byte mask = 0;
    if (!rl_map_is_room_wall(map, point))
        return mask;
    if (rl_map_is_room_wall(map, RL_XY(point.x + 1, point.y)))
        mask |= RL_WallToEast;
    if (rl_map_is_room_wall(map, RL_XY(point.x - 1, point.y)))
        mask |= RL_WallToWest;
    if (rl_map_is_room_wall(map, RL_XY(point.x,     point.y - 1)))
        mask |= RL_WallToNorth;
    if (rl_map_is_room_wall(map, RL_XY(point.x,     point.y + 1)))
        mask |= RL_WallToSouth;
    return mask ? mask : RL_WallOther;
}

#ifndef RL_RNG_CUSTOM
unsigned int rl_rng_generate(unsigned int min, unsigned int max)
{
    rl_assert(max >= min);
    rl_assert(max < RAND_MAX);
    rl_assert(max < UINT_MAX);

    if (max < min || max >= RAND_MAX || max >= UINT_MAX)
        return min;
    if (min == max)
        return min;

    int rnd = rand();
    if (rnd < 0) rnd = abs(rnd); // fixes issue on LLVM MOS

    // produces more uniformity than using mod
    return min + rnd / (RAND_MAX / (max - min + 1) + 1);
}
#endif

RL_BSP *rl_bsp_create(unsigned int width, unsigned int height)
{
    rl_assert(width > 0 && height > 0);
    RL_BSP *bsp = (RL_BSP*) rl_calloc(sizeof(*bsp), 1);
    if (bsp == NULL) return NULL;
    bsp->width = width;
    bsp->height = height;

    return bsp;
}
void rl_bsp_destroy(RL_BSP* root)
{
    if (root) {
        if (root->left) {
            rl_bsp_destroy(root->left);
            root->left = NULL;
        }
        if (root->right) {
            rl_bsp_destroy(root->right);
            root->right = NULL;
        }
        rl_free(root);
    }
}

void rl_bsp_split(RL_BSP *node, unsigned int position, RL_SplitDirection direction)
{
    // can't split something already split
    rl_assert(node->left == NULL && node->right == NULL);

    if (node->left || node->right)
        return;

    if (direction == RL_SplitVertically && position >= node->height)
        return;
    if (direction == RL_SplitHorizontally && position >= node->width)
        return;

    RL_BSP *left = (RL_BSP*) rl_calloc(1, sizeof(RL_BSP));
    if (left == NULL)
        return;
    RL_BSP *right = (RL_BSP*) rl_calloc(1, sizeof(RL_BSP));
    if (right == NULL) {
        rl_free(left);
        return;
    }

    if (direction == RL_SplitVertically) {
        left->width = node->width;
        left->height = position;
        left->x = node->x;
        left->y = node->y;
        right->width = node->width;
        right->height = node->height - position;
        right->x = node->x;
        right->y = node->y;
        right->y += position;
    } else {
        left->width = position;
        left->height = node->height;
        left->x = node->x;
        left->y = node->y;
        right->width = node->width - position;
        right->height = node->height;
        right->x = node->x;
        right->y = node->y;
        right->x += position;
    }

    left->parent = right->parent = node;
    node->left = left;
    node->right = right;
}

bool rl_bsp_recursive_split(RL_BSP *root, unsigned int min_width, unsigned int min_height, unsigned int max_recursion)
{
    rl_assert(root);
    rl_assert(min_width > 0 && min_height > 0 && root != NULL);

    if (min_width > root->width)
        return false;
    if (min_height > root->height)
        return false;
    if (root == NULL)
        return false;
    if (max_recursion <= 0)
        return false;

    rl_assert(RL_MAPGEN_BSP_DEVIATION >= 0.0 && RL_MAPGEN_BSP_DEVIATION <= 1.0);

    unsigned int width = root->width;
    unsigned int height = root->height;
    unsigned int max_width = width - min_width;
    unsigned int max_height = height - min_height;

    // determine split dir & split
    RL_SplitDirection dir;
    if (rl_rng_generate(0, 1)) {
        if (width < min_width*2)
            dir = RL_SplitVertically;
        else
            dir = RL_SplitHorizontally;
    } else {
        if (height < min_height*2)
            dir = RL_SplitHorizontally;
        else
            dir = RL_SplitVertically;
    }

    unsigned int split_position;
    if (dir == RL_SplitHorizontally) {
        // cannot split if current node size is too small
        if (width < min_width*2)
            return false;

        unsigned int center = width / 2;
        unsigned int from = center - (center * RL_MAPGEN_BSP_DEVIATION);
        unsigned int to = center + (center * RL_MAPGEN_BSP_DEVIATION);
        if (from < min_width)
            from = min_width;
        if (to > max_width)
            to = max_width;

        split_position = rl_rng_generate(from, to);
    } else {
        // cannot split if current node size is too small
        if (height < min_height*2)
            return false;

        unsigned int center = height / 2;
        unsigned int from = center - (center * RL_MAPGEN_BSP_DEVIATION);
        unsigned int to = center + (center * RL_MAPGEN_BSP_DEVIATION);
        if (from < min_height)
            from = min_height;
        if (to > max_height)
            to = max_height;

        split_position = rl_rng_generate(from, to);
    }

    rl_bsp_split(root, split_position, dir);

    // continue recursion
    RL_BSP *left = root->left;
    RL_BSP *right = root->right;

    if (left == NULL || right == NULL)
        return false;

    if (!rl_bsp_recursive_split(left, min_width, min_height, max_recursion - 1) ||
        !rl_bsp_recursive_split(right, min_width, min_height, max_recursion - 1)
    ) {
        rl_free(left);
        rl_free(right);
        root->left = root->right = NULL;

        return true;
    }

    return true;
}

bool rl_bsp_is_leaf(RL_BSP *node)
{
    if (node == NULL) return 0;
    return (node->left == NULL && node->right == NULL);
}

RL_BSP *rl_bsp_sibling(RL_BSP *node)
{
    if (node && node->parent) {
        if (node->parent->left == node)
            return node->parent->right;
        if (node->parent->right == node)
            return node->parent->left;

        rl_assert(1 != 1); // BSP structure is invalid
    }

    return NULL;
}

RL_BSP *rl_bsp_next_node_recursive_down(RL_BSP *node, int depth)
{
    if (node == NULL)
        return NULL;
    if (depth == 0) // found the node
        return node;
    if (node->left == NULL)
        return NULL;
    return rl_bsp_next_node_recursive_down(node->left, depth + 1);
}
RL_BSP *rl_bsp_next_node_recursive(RL_BSP *node, int depth)
{
    if (node == NULL || node->parent == NULL)
        return NULL;
    if (node->parent->left == node) // traverse back down
        return rl_bsp_next_node_recursive_down(node->parent->right, depth);
    return rl_bsp_next_node_recursive(node->parent, depth - 1);
}
RL_BSP *rl_bsp_next_node(RL_BSP *node)
{
    if (node == NULL || node->parent == NULL)
        return NULL;

    // LOOP up until we are on the left, then go back down
    return rl_bsp_next_node_recursive(node, 0);
}

RL_BSP *rl_bsp_next_leaf_recursive_down(RL_BSP *node)
{
    if (node == NULL)
        return NULL;
    if (rl_bsp_is_leaf(node)) // found the node
        return node;
    if (node->left == NULL)
        return NULL;
    return rl_bsp_next_leaf_recursive_down(node->left);
}
RL_BSP *rl_bsp_next_leaf_recursive(RL_BSP *node)
{
    if (node == NULL || node->parent == NULL)
        return NULL;
    if (node->parent->left == node) // traverse back down
        return rl_bsp_next_leaf_recursive_down(node->parent->right);
    return rl_bsp_next_leaf_recursive(node->parent);
}
RL_BSP *rl_bsp_next_leaf(RL_BSP *node)
{
    if (node == NULL || node->parent == NULL)
        return NULL;
    rl_assert(rl_bsp_is_leaf(node));

    // LOOP up until we are on the left, then go back down
    return rl_bsp_next_leaf_recursive(node);
}

size_t rl_bsp_leaf_count(RL_BSP *root)
{
    if (root == NULL) return 0;
    rl_assert(root->parent == NULL);
    // find first leaf
    RL_BSP *node = root;
    while (node->left != NULL) {
        node = node->left;
    }
    // count leaves
    int count = 1;
    while ((node = rl_bsp_next_leaf(node)) != NULL) {
        count++;
    }
    return count;
}

static void rl_map_bsp_generate_room(RL_Map *map, unsigned int room_width, unsigned int room_height, RL_Point room_loc)
{
    rl_assert(map && room_width + room_loc.x <= map->width);
    rl_assert(map && room_height + room_loc.y <= map->height);
    if (map == NULL) return;
    for (unsigned int x = room_loc.x; x < room_loc.x + room_width; ++x) {
        for (unsigned int y = room_loc.y; y < room_loc.y + room_height; ++y) {
            if (x == room_loc.x || x == room_loc.x + room_width - 1 ||
                    y == room_loc.y || y == room_loc.y + room_height - 1
               ) {
                // set sides of room to walls
                map->tiles[y*map->width + x] = RL_TileRock;
            } else {
                map->tiles[y*map->width + x] = RL_TileRoom;
            }
        }
    }
}
static void rl_map_bsp_generate_rooms(RL_BSP *node, RL_Map *map, unsigned int room_min_width, unsigned int room_max_width, unsigned int room_min_height, unsigned int room_max_height, unsigned int room_padding)
{
    rl_assert(map);
    rl_assert(room_min_width < room_max_width);
    rl_assert(room_min_height < room_max_height);
    rl_assert(room_max_width + room_padding*2 < UINT_MAX);
    rl_assert(room_max_height + room_padding*2 < UINT_MAX);
    rl_assert(room_min_width > 2 && room_min_height > 2); // width of 2 can end up having rooms made of nothing but walls
    rl_assert(node && room_min_width < node->width);
    rl_assert(node && room_min_height < node->height);
    rl_assert(node && room_max_width <= node->width);
    rl_assert(node && room_max_height <= node->height);
    if (map == NULL) return;
    if (node && node->left) {
        if (rl_bsp_is_leaf(node->left)) {
            unsigned int room_width, room_height;
            RL_Point room_loc;
            RL_BSP *leaf = node->left;
            room_width = rl_rng_generate(room_min_width, room_max_width);
            if (room_width + room_padding*2 > leaf->width)
                room_width = leaf->width - room_padding*2;
            room_height = rl_rng_generate(room_min_height, room_max_height);
            if (room_height + room_padding*2 > leaf->height)
                room_height = leaf->height - room_padding*2;
            // room_loc.x = rl_rng_generate(leaf->x + room_padding, leaf->x + leaf->width - room_width - room_padding);
            // room_loc.y = rl_rng_generate(leaf->y + room_padding, leaf->y + leaf->height - room_height - room_padding);
            room_loc.x = leaf->x + leaf->width/2 - room_width/2 - room_padding/2;
            room_loc.y = leaf->y + leaf->height/2 - room_height/2 - room_padding/2;

            rl_map_bsp_generate_room(map, room_width, room_height, room_loc);
        } else {
            rl_map_bsp_generate_rooms(node->left, map, room_min_width, room_max_width, room_min_height, room_max_height, room_padding);
        }
    }
    if (node && node->right) {
        if (rl_bsp_is_leaf(node->left)) {
            unsigned int room_width, room_height;
            RL_Point room_loc;
            RL_BSP *leaf = node->right;
            room_width = rl_rng_generate(room_min_width, room_max_width);
            if (room_width + room_padding*2 > leaf->width)
                room_width = leaf->width - room_padding*2;
            room_height = rl_rng_generate(room_min_height, room_max_height);
            if (room_height + room_padding*2 > leaf->height)
                room_height = leaf->height - room_padding*2;
            // room_loc.x = rl_rng_generate(leaf->x + room_padding, leaf->x + leaf->width - room_width - room_padding);
            // room_loc.y = rl_rng_generate(leaf->y + room_padding, leaf->y + leaf->height - room_height - room_padding);
            room_loc.x = leaf->x + leaf->width/2 - room_width/2 - room_padding/2;
            room_loc.y = leaf->y + leaf->height/2 - room_height/2 - room_padding/2;

            rl_map_bsp_generate_room(map, room_width, room_height, room_loc);
        } else {
            rl_map_bsp_generate_rooms(node->right, map, room_min_width, room_max_width, room_min_height, room_max_height, room_padding);
        }
    }
}

void rl_mapgen_bsp(RL_Map *map, RL_MapgenConfigBSP config)
{
    RL_BSP *bsp = rl_mapgen_bsp_ex(map, config);
    rl_bsp_destroy(bsp);
}

RL_BSP *rl_mapgen_bsp_ex(RL_Map *map, RL_MapgenConfigBSP config)
{
    rl_assert(map);
    rl_assert(config.room_min_width > 0 && config.room_max_width >= config.room_min_width && config.room_min_height > 0 && config.room_max_height >= config.room_min_height);
    rl_assert(config.room_max_width <= map->width && config.room_max_height <= map->height);
    memset(map->tiles, (RL_Byte) RL_TileRock, map->width*map->height*sizeof(*map->tiles));

    if (map == NULL) {
        return NULL;
    }

    RL_BSP *root = rl_bsp_create(map->width, map->height);
    rl_assert(root);
    if (root == NULL) {
        return NULL;
    }

    rl_bsp_recursive_split(root, config.room_max_width + config.room_padding, config.room_max_height + config.room_padding, RL_MAX_RECURSION);
    rl_map_bsp_generate_rooms(root, map, config.room_min_width, config.room_max_width, config.room_min_height, config.room_max_height, config.room_padding);

    switch (config.draw_corridors) {
        case RL_ConnectNone:
            break;
        case RL_ConnectRandomly:
            rl_mapgen_connect_corridors_randomly(map, root, config.draw_doors);
            {
                // cull non-connected tiles
                RL_Graph *floodfill = rl_map_largest_connected_area(map);
                rl_assert(floodfill);
                if (floodfill) {
                    for (size_t x=0; x < map->width; ++x) {
                        for (size_t y=0; y < map->height; ++y) {
                            if (floodfill->nodes[x + y*map->width].score == FLT_MAX) {
                                // set unreachable tiles to rock
                                map->tiles[x + y*map->width] = RL_TileRock;
                            }
                        }
                    }
                    rl_graph_destroy(floodfill);
                }
            }
            break;
        case RL_ConnectBSP:
            {
                RL_Graph *graph = rl_graph_create(map, NULL, 0);
                rl_assert(graph);
                rl_mapgen_connect_corridors_bsp(map, root, config.draw_doors, graph);
                if (graph) {
                    rl_graph_destroy(graph);
                }
            }
            break;
        case RL_ConnectBSPSimple:
            rl_mapgen_connect_corridors_bsp(map, root, config.draw_doors, NULL);
            break;
        default:
            rl_assert("Invalid corridor connection argument passed to rl_mapgen_bsp" && 0);
            break;
    }

    // if (config.use_secret_passages) {
        // TODO connect secret passages
    // }

    return root;
}

// custom Dijkstra scorer function to prevent carving double wide doors when carving corridors
static inline float rl_mapgen_corridor_scorer(RL_GraphNode *current, RL_GraphNode *neighbor, void *context)
{
    RL_Map *map = (RL_Map*) context;
    RL_Point start = current->point;
    RL_Point end = neighbor->point;
    float r = current->score + rl_distance_manhattan(start, end);

    if (rl_map_tile_is(map, end, RL_TileDoor)) {
        return r; // doors are passable but count as "walls" - encourage passing through them
    }
    if (rl_map_is_corner_wall(map, end)) {
        return r + 99; // discourage double wide corridors & double carving into walls
    }
    if (rl_map_is_wall(map, end)) {
        return r + 9; // discourage double wide corridors & double carving into walls
    }

    return r;
}

void rl_mapgen_connect_corridors_bsp(RL_Map *map, RL_BSP *root, bool draw_doors, RL_Graph *graph)
{
    rl_assert(map && root);
    if (!map || !root) return;

    // connect siblings
    RL_BSP *node = root->left;
    RL_BSP *sibling = root->right;
    if (node == NULL || sibling == NULL) return;

    // find rooms in BSP
    // TODO find a point closest to node & sibling
    RL_Point dig_start, dig_end;
    // TODO only connect if leaf or parent of leaf?
    if (rl_bsp_is_leaf(node) && rl_bsp_is_leaf(sibling)) {
        dig_start = RL_XY(node->x + node->width / 2, node->y + node->height / 2);
        // TODO need way to find point faster
        // for (unsigned int x = node->x; x < node->width + node->x; ++x) {
        //     for (unsigned int y = node->y; y < node->height + node->y; ++y) {
        //         if (rl_map_tile_is(map, RL_XY(x, y), RL_TileRoom)) {
        //             dig_start = RL_XY(x, y);
        //         }
        //     }
        // }
        // rl_assert(rl_map_is_passable(map, dig_start));
        dig_end = RL_XY(sibling->x + sibling->width / 2, sibling->y + sibling->height / 2);
        // for (unsigned int x = sibling->x; x < sibling->width + sibling->x; ++x) {
        //     for (unsigned int y = sibling->y; y < sibling->height + sibling->y; ++y) {
        //         if (rl_map_tile_is(map, RL_XY(x, y), RL_TileRoom)) {
        //             dig_end = RL_XY(x, y);
        //         }
        //     }
        // }
        // rl_assert(rl_map_is_passable(map, dig_end));
        rl_assert(!(dig_start.x == dig_end.x && dig_start.y == dig_end.y));

        // carve out corridors
        if (graph) {
            rl_dijkstra_score_ex(graph, dig_end, rl_mapgen_corridor_scorer, map);
            RL_Path *path = rl_path_create_from_graph(graph, dig_start);
            rl_assert(path);
            while ((path = rl_path_walk(path))) {
                if (rl_map_tile_is(map, path->point, RL_TileRock)) {
                    if (rl_map_is_room_wall(map, path->point) && draw_doors) {
                        map->tiles[(size_t)path->point.x + (size_t)path->point.y * map->width] = RL_TileDoor;
                    } else {
                        map->tiles[(size_t)path->point.x + (size_t)path->point.y * map->width] = RL_TileCorridor;
                    }
                }
            }
        } else {
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
                    if (draw_doors && rl_map_is_room_wall(map, cur)) {
                        map->tiles[(int)cur.x + (int)cur.y*map->width] = RL_TileDoor;
                    } else {
                        map->tiles[(int)cur.x + (int)cur.y*map->width] = RL_TileCorridor;
                    }
                }
                cur = next;
            }
        }
    }

    // connect siblings' children
    rl_mapgen_connect_corridors_bsp(map, node, draw_doors, graph);
    rl_mapgen_connect_corridors_bsp(map, sibling, draw_doors, graph);
}

void rl_mapgen_connect_corridors_randomly(RL_Map *map, RL_BSP *root, bool draw_doors)
{
    rl_assert(map && root);
    if (!map || !root) return;

    // find deepest left-most node
    RL_BSP *leftmost_node = root;
    while (leftmost_node->left != NULL) {
        leftmost_node = leftmost_node->left;
    }
    rl_assert(leftmost_node && rl_bsp_is_leaf(leftmost_node));
    RL_BSP *node = leftmost_node;
    RL_Graph *graph = rl_graph_create(map, NULL, 0);
    rl_assert(graph);
    if (graph == NULL) return;
    int leaf_count = rl_bsp_leaf_count(root);
    while (node) {
        RL_BSP *sibling;

        // find random sibling
        rl_assert(leaf_count > 1);
        do {
            int target = rl_rng_generate(0, leaf_count - 1);
            sibling = leftmost_node;
            for (int i = 0; i < target; i++) {
                sibling = rl_bsp_next_leaf(sibling);
            }
        } while (sibling == node);

        rl_assert(sibling);

        // find rooms in BSP
        // TODO find a random point on a wall or center of the room?
        RL_Point dig_start, dig_end;
        for (unsigned int x = node->x; x < node->width + node->x; ++x) {
            for (unsigned int y = node->y; y < node->height + node->y; ++y) {
                if (rl_map_tile_is(map, RL_XY(x, y), RL_TileRoom)) {
                    dig_start = RL_XY(x, y);
                }
            }
        }
        rl_assert(rl_map_is_passable(map, dig_start));
        for (unsigned int x = sibling->x; x < sibling->width + sibling->x; ++x) {
            for (unsigned int y = sibling->y; y < sibling->height + sibling->y; ++y) {
                if (rl_map_tile_is(map, RL_XY(x, y), RL_TileRoom)) {
                    dig_end = RL_XY(x, y);
                }
            }
        }
        rl_assert(rl_map_is_passable(map, dig_end));
        rl_assert(!(dig_start.x == dig_end.x && dig_start.y == dig_end.y));

        // carve out corridors
        rl_dijkstra_score_ex(graph, dig_end, rl_mapgen_corridor_scorer, map);
        RL_Path *path = rl_path_create_from_graph(graph, dig_start);
        rl_assert(path);
        while ((path = rl_path_walk(path))) {
            if (rl_map_tile_is(map, path->point, RL_TileRock)) {
                if (rl_map_is_room_wall(map, path->point) && draw_doors) {
                    map->tiles[(size_t)path->point.x + (size_t)path->point.y * map->width] = RL_TileDoor;
                } else {
                    map->tiles[(size_t)path->point.x + (size_t)path->point.y * map->width] = RL_TileCorridor;
                }
            }
        }

        // find start node for next loop iteration
        node = rl_bsp_next_leaf(node);
    }

    rl_graph_destroy(graph);
}

RL_Graph *rl_map_largest_connected_area(const RL_Map *map)
{
    rl_assert(map);
    if (map == NULL) return NULL;
    int *visited = (int*) rl_calloc(sizeof(*visited), map->width * map->height);
    rl_assert(visited);
    if (visited == NULL) return NULL;
    RL_Graph *floodfill = NULL; // largest floodfill
    int floodfill_scored = 0;
    for (unsigned int x = 0; x < map->width; ++x) {
        for (unsigned int y = 0; y < map->height; ++y) {
            if (rl_map_is_passable(map, RL_XY(x, y)) && !visited[x + y*map->width]) {
                RL_Graph *test = rl_dijkstra_create(map, RL_XY(x, y), NULL, rl_map_is_passable);
                rl_assert(test);
                if (test == NULL) {
                    rl_free(visited);
                    if (floodfill) {
                        rl_graph_destroy(floodfill);
                    }
                    return NULL;
                }
                int test_scored = 0;
                for (size_t i = 0; i < test->length; i++) {
                    if (test->nodes[i].score != FLT_MAX) {
                        visited[i] = 1;
                        test_scored ++;
                    }
                }
                if (test_scored > floodfill_scored) {
                    floodfill_scored = test_scored;
                    if (floodfill) {
                        rl_graph_destroy(floodfill);
                    }
                    floodfill = test;
                } else {
                    rl_graph_destroy(test);
                }
            }
        }
    }

    rl_free(visited);

    return floodfill;
}

/**
 * Heap functions for pathfinding
 *
 * Ref: https://gist.github.com/skeeto/f012a207aff1753662b679917f706de6
 */

static int rl_heap_noop_comparison_f(const void *_a, const void *_b)
{
    RL_UNUSED(_a);
    RL_UNUSED(_b);
    return 1;
}

RL_Heap *rl_heap_create(int capacity, int (*comparison_f)(const void *heap_item_a, const void *heap_item_b))
{
    RL_Heap *heap = (RL_Heap*) rl_malloc(sizeof(*heap));
    rl_assert(heap);
    if (heap == NULL) {
        return NULL;
    }
    heap->heap = (void**) rl_malloc(sizeof(*heap->heap) * capacity);
    rl_assert(heap->heap);
    if (heap->heap == NULL) {
        rl_free(heap);
        return NULL;
    }

    if (comparison_f == NULL) {
        comparison_f = rl_heap_noop_comparison_f;
    }

    heap->cap = capacity;
    heap->comparison_f = comparison_f;
    heap->len = 0;

    return heap;
}

void rl_heap_destroy(RL_Heap *h)
{
    if (h) {
        if (h->heap) {
            rl_free(h->heap);
        }
        rl_free(h);
    }
}

int rl_heap_length(RL_Heap *h)
{
    if (h == NULL) return 0;
    return h->len;
}

bool rl_heap_insert(RL_Heap *h, void *item)
{
    rl_assert(h != NULL);
    if (h == NULL) return false;

    if (h->len == h->cap) {
        // resize the heap
        void **heap_items = (void**) rl_realloc(h->heap, sizeof(void*) * h->cap * 2);
        rl_assert(heap_items);
        if (heap_items == NULL) {
            rl_heap_destroy(h);
            return false;
        }
        h->heap = heap_items;
        h->cap *= 2;
    }

    h->heap[h->len] = item;
    for (int i = h->len++; i;) {
        int p = (i - 1) / 2;
        if (h->comparison_f(h->heap[p], h->heap[i])) {
            break;
        }
        void *tmp = h->heap[p];
        h->heap[p] = h->heap[i];
        h->heap[i] = tmp;
        i = p;
    }
    return true;
}

static void rl_heap_remove(RL_Heap *h, int index)
{
    rl_assert(h);
    if (h == NULL) {
        return;
    }

    h->heap[index] = h->heap[--h->len];
    for (int i = index;;) {
        int a = 2*i + 1;
        int b = 2*i + 2;
        int j = i;
        if (a < h->len && h->comparison_f(h->heap[a], h->heap[j])) j = a;
        if (b < h->len && h->comparison_f(h->heap[b], h->heap[j])) j = b;
        if (i == j) break;
        void *tmp = h->heap[j];
        h->heap[j] = h->heap[i];
        h->heap[i] = tmp;
        i = j;
    }
}

void *rl_heap_pop(RL_Heap *h)
{
    if (h == NULL) {
        return NULL;
    }

    void *r = NULL;
    if (h->len) {
        rl_assert(h->heap);
        r = h->heap[0];
        rl_heap_remove(h, 0);
    }
    return r;
}

void *rl_heap_peek(RL_Heap *h)
{
    if (h == NULL) {
        return NULL;
    }

    rl_assert(h->heap);
    if (h->len) {
        return h->heap[0];
    } else {
        return NULL;
    }
}

// simplified distance for side by side nodes
static float rl_distance_simple(RL_Point node, RL_Point end)
{
    if (node.x == end.x && node.y == end.y) return 0;
    if (node.x == end.x || node.y == end.y) return 1;
    return 1.4;
}

static int rl_scored_graph_heap_comparison(const void *heap_item_a, const void *heap_item_b)
{
    RL_GraphNode *node_a = (RL_GraphNode*) heap_item_a;
    RL_GraphNode *node_b = (RL_GraphNode*) heap_item_b;

    return node_a->score < node_b->score;
}

RL_Path *rl_path(RL_Point p)
{
    RL_Path *path = (RL_Path*) rl_malloc(sizeof(*path));
    rl_assert(path);
    if (path == NULL) return NULL;
    path->next = NULL;
    path->point = p;

    return path;
}

float rl_distance_manhattan(RL_Point node, RL_Point end)
{
    return fabs(node.x - end.x) + fabs(node.y - end.y);
}

float rl_distance_euclidian(RL_Point node, RL_Point end)
{
    float distance_x = node.x - end.x;
    float distance_y = node.y - end.y;

    return sqrt(distance_x * distance_x + distance_y * distance_y);
}

float rl_distance_chebyshev(RL_Point node, RL_Point end)
{
    float distance_x = fabs(node.x - end.x);
    float distance_y = fabs(node.y - end.y);

    return distance_x > distance_y ? distance_x : distance_y;
}

RL_Path *rl_line_create(RL_Point a, RL_Point b, float step)
{
    float delta_x = fabs(a.x - b.x);
    float x_increment = b.x > a.x ? step : -step;
    float delta_y = fabs(a.y - b.y);
    float y_increment = b.y > a.y ? step : -step;
    float error = 0.0;
    float slope = delta_x ? delta_y / delta_x : 0.0;

    RL_Path *head = rl_path(a);
    RL_Path *path = head;
    while (path->point.x != b.x || path->point.y != b.y) {
        RL_Point point = path->point;

        if (delta_x > delta_y) {
            error += slope;
            if (error > 0.5 && point.y != b.y) {
                error -= 1.0;
                point.y += y_increment;
            }

            point.x += x_increment;
        } else {
            error += 1/slope;
            if (error > 0.5 && point.x != b.x) {
                error -= 1.0;
                point.x += x_increment;
            }

            point.y += y_increment;
        }

        // add new member to linked list & advance
        path->next = rl_path(point);
        path = path->next;
    }

    return head;
}

RL_Path *rl_path_create(const RL_Map *map, RL_Point start, RL_Point end, RL_DistanceFun distance_f, RL_PassableFun passable_f)
{
    RL_Graph *graph = rl_dijkstra_create(map, end, distance_f, passable_f);
    rl_assert(graph);
    if (graph == NULL) return NULL;
    RL_Path *path = rl_path_create_from_graph(graph, start);
    rl_assert(path);
    rl_graph_destroy(graph);

    return path;
}

RL_Path *rl_path_create_from_graph(const RL_Graph *graph, RL_Point start)
{
    RL_Path *path = rl_path(start);
    RL_Path *path_start = path;
    RL_GraphNode *node = NULL;
    rl_assert(path);
    rl_assert(graph && graph->nodes);
    if (path == NULL || graph == NULL || graph->nodes == NULL) return NULL;
    for (size_t i=0; i<graph->length; i++) {
        if (graph->nodes[i].point.x == start.x && graph->nodes[i].point.y == start.y) {
            node = &graph->nodes[i];
        }
    }
    if (node == NULL) {
        return path;
    }
    while (node->score > 0) {
        RL_GraphNode *lowest_neighbor = NULL;
        for (size_t i=0; i<node->neighbors_length; i++) {
            RL_GraphNode *neighbor = node->neighbors[i];
            if (!lowest_neighbor || neighbor->score < lowest_neighbor->score) {
                lowest_neighbor = neighbor;
            }
        }
        if (!lowest_neighbor || lowest_neighbor->score == FLT_MAX || node == lowest_neighbor) {
            break; // no path found
        }
        node = lowest_neighbor;
        path->next = rl_path(node->point);
        rl_assert(path->next);
        if (path->next == NULL) {
            rl_path_destroy(path);
            return NULL;
        }
        path = path->next;
    }

    return path_start;
}

RL_Path *rl_path_walk(RL_Path *path)
{
    if (!path) return NULL;
    RL_Path *next = path->next;
    path->next = NULL;
    rl_free(path);

    return next;
}

void rl_path_destroy(RL_Path *path)
{
    if (path) {
        while ((path = rl_path_walk(path))) {}
    }
}

RL_Graph *rl_graph_create(const RL_Map *map, RL_PassableFun passable_f, bool allow_diagonal_neighbors)
{
    RL_Graph *graph = (RL_Graph*) rl_malloc(sizeof(*graph));
    rl_assert(graph);
    if (graph == NULL) return NULL;
    size_t length = map->width * map->height;
    RL_GraphNode *nodes = (RL_GraphNode*) rl_calloc(sizeof(*nodes), length);
    rl_assert(nodes != NULL);
    if (nodes == NULL) {
        rl_free(graph);
        return NULL;
    }
    for (unsigned int x=0; x<map->width; x++) {
        for (unsigned int y=0; y<map->height; y++) {
            size_t idx = x + y*map->width;
            RL_GraphNode *node = &nodes[idx];
            node->point.x = (float) x;
            node->point.y = (float) y;
            node->neighbors_length = 0;
            node->score = FLT_MAX;
            // calculate neighbors
            RL_Point neighbor_coords[8] = {
                (RL_Point) { x + 1, y },
                (RL_Point) { x - 1, y },
                (RL_Point) { x,     y + 1 },
                (RL_Point) { x,     y - 1 },
                (RL_Point) { x + 1, y + 1 },
                (RL_Point) { x + 1, y - 1 },
                (RL_Point) { x - 1, y + 1 },
                (RL_Point) { x - 1, y - 1 },
            };
            for (int i=0; i<8; i++) {
                if (passable_f && !passable_f(map, RL_XY(neighbor_coords[i].x, neighbor_coords[i].y)))
                    continue;
                if (!rl_map_in_bounds(map, neighbor_coords[i]))
                    continue;
                if (!allow_diagonal_neighbors && i >= 4)
                    continue;

                size_t idx = neighbor_coords[i].x + neighbor_coords[i].y*map->width;
                node->neighbors[node->neighbors_length] = &nodes[idx];
                node->neighbors_length++;
            }
        }
    }

    graph->length = length;
    graph->nodes = nodes;

    return graph;
}

RL_Graph *rl_dijkstra_create(const RL_Map *map,
                            RL_Point start,
                            RL_DistanceFun distance_f,
                            RL_PassableFun passable_f)
{
    RL_Graph *graph = rl_graph_create(map, passable_f, 1);
    rl_dijkstra_score(graph, start, distance_f);

    return graph;
}

// default scorer function for Dijkstra - this simply accepts a RL_DistanceFun as context and adds the current nodes
// score to the result of the distance function
struct rl_score_context { RL_DistanceFun fun; };
float rl_dijkstra_default_score_f(RL_GraphNode *current, RL_GraphNode *neighbor, void *context)
{
    struct rl_score_context *distance_f = (struct rl_score_context*) context;

    return current->score + distance_f->fun(current->point, neighbor->point);
}

void rl_dijkstra_score(RL_Graph *graph, RL_Point start, RL_DistanceFun distance_f)
{
    struct { RL_DistanceFun fun; } scorer_context;
    scorer_context.fun = distance_f ? distance_f : rl_distance_simple; // default to rl_distance_simple
    rl_dijkstra_score_ex(graph, start, rl_dijkstra_default_score_f, &scorer_context);
}

void rl_dijkstra_score_ex(RL_Graph *graph, RL_Point start, RL_ScoreFun score_f, void *score_context)
{
    rl_assert(graph);
    rl_assert(score_f);
    if (graph == NULL) return;

    RL_GraphNode *current;
    RL_Heap *heap = rl_heap_create(graph->length, &rl_scored_graph_heap_comparison);

    // reset scores of dijkstra map, setting the start point to 0
    for (size_t i=0; i < graph->length; i++) {
        RL_GraphNode *node = &graph->nodes[i];
        if (node->point.x == start.x && node->point.y == start.y) {
            node->score = 0;
            current = node;
        } else {
            node->score = FLT_MAX;
        }
    }

    rl_heap_insert(heap, (void*) current);
    current = (RL_GraphNode*) rl_heap_pop(heap);
    while (current) {
        for (size_t i=0; i<current->neighbors_length; i++) {
            RL_GraphNode *neighbor = current->neighbors[i];
            float distance = score_f(current, neighbor, score_context);
            if (distance < neighbor->score) {
                if (neighbor->score == FLT_MAX) {
                    rl_heap_insert(heap, neighbor);
                }
                neighbor->score = distance;
            }
        }

        current = (RL_GraphNode *) rl_heap_pop(heap);
    }

    rl_heap_destroy(heap);
}

void rl_graph_destroy(RL_Graph *graph)
{
    if (graph) {
        if (graph->nodes) {
            rl_free(graph->nodes);
        }
        rl_free(graph);
    }
}

RL_FOV *rl_fov_create(unsigned int width, unsigned int height)
{
    rl_assert(width > 0 && height > 0);
    rl_assert(width != UINT_MAX && !(width > UINT_MAX / height)); // check for overflow
    RL_FOV *fov = NULL;
    // allocate all the memory we need at once
    char *memory = (char*) rl_calloc(sizeof(*fov) + sizeof(*fov->visibility)*width*height, 1);
    rl_assert(memory);
    if (memory == NULL) return NULL;
    fov = (RL_FOV*) memory;
    fov->width = width;
    fov->height = height;
    fov->visibility = (RL_Byte*) (memory + sizeof(*fov));
    rl_assert(fov);
    rl_assert(fov->visibility);

    return fov;
}

void rl_fov_destroy(RL_FOV *fov)
{
    if (fov) {
        rl_free(fov);
    }
}

typedef struct {
    int Y;
    int X;
} RL_Slope;

// adapted from: https://www.adammil.net/blog/v125_Roguelike_Vision_Algorithms.html#shadowcode (public domain)
// also see: https://www.roguebasin.com/index.php/FOV_using_recursive_shadowcasting
void rl_fov_calculate_recursive(void *map, RL_Point origin, int fov_radius, RL_DistanceFun distance_f, RL_IsOpaqueFun opaque_f, RL_MarkAsVisibleFun mark_visible_f, unsigned int octant, unsigned int x, RL_Slope top, RL_Slope bottom)
{
    rl_assert(distance_f);
    rl_assert(opaque_f);
    rl_assert(mark_visible_f);
    for(; x <= (unsigned int) fov_radius && x < RL_MAX_RECURSION; x++)
    {
        // compute the Y coordinates where the top vector leaves the column (on the right) and where the bottom vector
        // enters the column (on the left). this equals (x+0.5)*top+0.5 and (x-0.5)*bottom+0.5 respectively, which can
        // be computed like (x+0.5)*top+0.5 = (2(x+0.5)*top+1)/2 = ((2x+1)*top+1)/2 to avoid floating point math
        // the rounding is a bit tricky, though
        int topY = top.X == 1 ? x : ((x*2+1) * top.Y + top.X - 1) / (top.X*2); // the rounding is a bit tricky, though
        int bottomY = bottom.Y == 0 ? 0 : ((x*2-1) * bottom.Y + bottom.X) / (bottom.X*2);
        int wasOpaque = -1; // 0:false, 1:true, -1:not applicable
        for(int y=topY; y >= bottomY; y--)
        {
            int tx = origin.x, ty = origin.y;
            switch(octant) // translate local coordinates to map coordinates
            {
                case 0: tx += x; ty -= y; break;
                case 1: tx += y; ty -= x; break;
                case 2: tx -= y; ty -= x; break;
                case 3: tx -= x; ty -= y; break;
                case 4: tx -= x; ty += y; break;
                case 5: tx -= y; ty += x; break;
                case 6: tx += y; ty += x; break;
                case 7: tx += x; ty += y; break;
            }

            bool inRange = fov_radius < 0 || distance_f(origin, RL_XY(tx, ty)) <= (float)fov_radius;
            if(inRange) {
                if (RL_FOV_SYMMETRIC && (y != topY || top.Y*(int)x >= top.X*y) && (y != bottomY || bottom.Y*(int)x <= bottom.X*y)) {
                    mark_visible_f(RL_XY(tx, ty), map);
                } else {
                    mark_visible_f(RL_XY(tx, ty), map);
                }
            }

            bool isOpaque = !inRange || !opaque_f(RL_XY(tx, ty), map);
            if((int)x != fov_radius)
            {
                if(isOpaque)
                {
                    if(wasOpaque == 0) // if we found a transition from clear to opaque, this sector is done in this column, so
                    {                  // adjust the bottom vector upwards and continue processing it in the next column.
                        RL_Slope newBottom = { (y*2+1), (x*2-1) }; // (x*2-1, y*2+1) is a vector to the top-left of the opaque tile
                        if(!inRange || y == bottomY) { bottom = newBottom; break; } // don't recurse unless we have to
                        else if (x < (unsigned int) fov_radius) rl_fov_calculate_recursive(map, origin, fov_radius, distance_f, opaque_f, mark_visible_f, octant, x+1, top, newBottom);
                    }
                    wasOpaque = 1;
                }
                else // adjust top vector downwards and continue if we found a transition from opaque to clear
                {    // (x*2+1, y*2+1) is the top-right corner of the clear tile (i.e. the bottom-right of the opaque tile)
                    if(wasOpaque > 0) top = (RL_Slope) { (y*2+1), (x*2+1) };
                    wasOpaque = 0;
                }
            }
        }

        if(wasOpaque != 0) break; // if the column ended in a clear tile, continue processing the current sector
    }
}

struct RL_FOVMap {
    RL_FOV *fov;
    const RL_Map *map;
};

void rl_fovmap_mark_visible_f(RL_Point p, void *context)
{
    struct RL_FOVMap *map = (struct RL_FOVMap*) context;
    if (rl_map_in_bounds(map->map, p)) {
        map->fov->visibility[(int)p.x + (int)p.y*map->map->width] = RL_TileVisible;
    }
}

bool rl_fovmap_opaque_f(RL_Point p, void *context)
{
    struct RL_FOVMap *map = (struct RL_FOVMap*) context;
    if (!rl_map_in_bounds(map->map, p)) {
        return true;
    }
    return rl_map_is_passable(map->map, p);
}

void rl_fov_calculate(RL_FOV *fov, const RL_Map *map, RL_Point origin, int fov_radius, RL_DistanceFun distance_f)
{
    if (!rl_map_in_bounds(map, origin)) {
        return;
    }
    // set previously visible tiles to seen
    for (unsigned int x=0; x<map->width; ++x) {
        for (unsigned int y=0; y<map->height; ++y) {
            if (fov->visibility[x + y*map->width] == RL_TileVisible) {
                fov->visibility[x + y*map->width] = RL_TileSeen;
            }
        }
    }
    struct RL_FOVMap fovmap;
    fovmap.map = map;
    fovmap.fov = fov;
    rl_fov_calculate_ex(&fovmap, origin, fov_radius, distance_f, rl_fovmap_opaque_f, rl_fovmap_mark_visible_f);
}

void rl_fov_calculate_ex(void *context, RL_Point origin, int fov_radius, RL_DistanceFun distance_f, RL_IsOpaqueFun opaque_f, RL_MarkAsVisibleFun mark_visible_f)
{
    mark_visible_f(origin, context);
    for (int octant=0; octant<8; ++octant) {
        rl_fov_calculate_recursive(context, origin, fov_radius, distance_f, opaque_f, mark_visible_f, octant, 1, (RL_Slope) { 1, 1 }, (RL_Slope) { 0, 1 });
    }
}

bool rl_fov_is_visible(const RL_FOV *map, RL_Point point)
{
    rl_assert(map);
    if (map == NULL) return false;
    if (!rl_map_in_bounds((const RL_Map*) map, point)) {
        return false;
    }
    return map->visibility[(int)point.x + (int)point.y*map->width] == RL_TileVisible;
}

bool rl_fov_is_seen(const RL_FOV *map, RL_Point point)
{
    rl_assert(map);
    if (map == NULL) return false;
    if (!rl_map_in_bounds((const RL_Map*) map, point)) {
        return false;
    }
    return map->visibility[(int)point.x + (int)point.y*map->width] == RL_TileSeen;
}


#endif

#ifdef __cplusplus
}
#endif
