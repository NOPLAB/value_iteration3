#include "map_pgm.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char* read_all(const char *path, size_t *out_sz) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0) { fclose(f); return NULL; }
    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) { free(buf); fclose(f); return NULL; }
    buf[sz] = 0;
    fclose(f);
    if (out_sz) *out_sz = (size_t)sz;
    return buf;
}

static int find_key(const char *text, const char *key, char *out, size_t out_sz) {
    const char *p = text;
    size_t klen = strlen(key);
    while ((p = strstr(p, key))) {
        if ((p == text || p[-1] == '\n') && p[klen] == ':' ) {
            p += klen + 1;
            while (*p == ' ' || *p == '\t') p++;
            size_t n = 0;
            while (*p && *p != '\n' && *p != '#' && n + 1 < out_sz) out[n++] = *p++;
            while (n > 0 && (out[n-1] == ' ' || out[n-1] == '\t' || out[n-1] == '\r')) n--;
            out[n] = 0;
            return 0;
        }
        p++;
    }
    return -1;
}

static int parse_yaml(const char *path, pgm_map_t *m, char *image_path, size_t image_sz) {
    size_t sz;
    char *yaml = read_all(path, &sz);
    if (!yaml) return -1;

    char buf[256];
    if (find_key(yaml, "image", buf, sizeof buf) < 0) goto bad;
    strncpy(image_path, buf, image_sz - 1); image_path[image_sz - 1] = 0;

    if (find_key(yaml, "resolution", buf, sizeof buf) < 0) goto bad;
    m->resolution = atof(buf);

    if (find_key(yaml, "origin", buf, sizeof buf) == 0) {
        /* "[x, y, yaw]" */
        char *p = buf;
        while (*p && (*p == '[' || *p == ' ')) p++;
        m->origin_x = atof(p);
        while (*p && *p != ',') p++;
        if (*p == ',') p++;
        m->origin_y = atof(p);
    }

    if (find_key(yaml, "occupied_thresh", buf, sizeof buf) == 0)
        m->occupied_thresh = atof(buf);
    else m->occupied_thresh = 0.65;

    if (find_key(yaml, "free_thresh", buf, sizeof buf) == 0)
        m->free_thresh = atof(buf);
    else m->free_thresh = 0.196;

    if (find_key(yaml, "negate", buf, sizeof buf) == 0)
        m->negate = atoi(buf);
    else m->negate = 0;

    free(yaml);
    return 0;
bad:
    free(yaml);
    return -1;
}

static int parse_pgm(const char *path, pgm_map_t *m) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    char magic[3] = {0};
    if (fread(magic, 1, 2, f) != 2) { fclose(f); return -1; }
    if (magic[0] != 'P' || magic[1] != '5') { fclose(f); return -1; }

    int w = 0, h = 0, maxv = 0;
    int got = 0;
    int c = fgetc(f);
    while (got < 3) {
        while (c == ' ' || c == '\t' || c == '\n' || c == '\r') c = fgetc(f);
        if (c == '#') { while (c != '\n' && c != EOF) c = fgetc(f); continue; }
        int val = 0;
        while (isdigit(c)) { val = val * 10 + (c - '0'); c = fgetc(f); }
        if (got == 0) w = val;
        else if (got == 1) h = val;
        else maxv = val;
        got++;
    }
    (void)maxv;

    uint8_t *pixels = malloc((size_t)w * h);
    if (!pixels) { fclose(f); return -1; }
    if (fread(pixels, 1, (size_t)w * h, f) != (size_t)w * h) {
        free(pixels); fclose(f); return -1;
    }
    fclose(f);

    m->w = w;
    m->h = h;
    m->pixels = pixels;
    return 0;
}

int map_pgm_load(const char *yaml_path, pgm_map_t *out) {
    memset(out, 0, sizeof *out);

    char image[512] = {0};
    if (parse_yaml(yaml_path, out, image, sizeof image) < 0) return -1;

    /* Resolve image path relative to yaml dir */
    char full[1024];
    const char *slash = strrchr(yaml_path, '/');
    if (!slash) slash = strrchr(yaml_path, '\\');
    if (slash && image[0] != '/' && !(image[0] && image[1] == ':')) {
        size_t dirlen = (size_t)(slash - yaml_path + 1);
        if (dirlen + strlen(image) + 1 > sizeof full) return -1;
        memcpy(full, yaml_path, dirlen);
        strcpy(full + dirlen, image);
    } else {
        strncpy(full, image, sizeof full - 1);
        full[sizeof full - 1] = 0;
    }

    if (parse_pgm(full, out) < 0) return -1;

    if (out->negate) {
        size_t n = (size_t)out->w * out->h;
        for (size_t i = 0; i < n; i++) out->pixels[i] = (uint8_t)(255 - out->pixels[i]);
    }
    return 0;
}

void map_pgm_free(pgm_map_t *m) {
    if (!m) return;
    free(m->pixels);
    m->pixels = NULL;
}
