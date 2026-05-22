#ifndef MAP_PGM_H
#define MAP_PGM_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int      w;
    int      h;
    double   resolution;   /* meters per cell */
    double   origin_x;     /* meters */
    double   origin_y;
    double   occupied_thresh;
    double   free_thresh;
    int      negate;       /* 0 = white is free (default), 1 = black is free */
    uint8_t *pixels;       /* w*h bytes, 0..255 raw (post-negate) */
} pgm_map_t;

int  map_pgm_load(const char *yaml_path, pgm_map_t *out);
void map_pgm_free(pgm_map_t *m);

#endif
