/* vi_cli — command-line driver for libvi_sweep. */

#include "libvi_sweep.h"
#include "vi_device.h"
#include "map_pgm.h"
#include "penalty.h"
#include "transitions.h"
#include "vi_reference_c.h"

#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct {
    const char *map_path;
    int         gx, gy, gt;
    int         safety_radius;
    int         threshold;
    int         max_sweeps;
    const char *out_value;
    const char *out_action;
    int         verify;
    int         mock;
    int         verbose;
} cli_args_t;

static volatile sig_atomic_t g_interrupt = 0;
static void on_sigint(int sig) { (void)sig; g_interrupt = 1; }

static void usage(const char *prog) {
    fprintf(stderr,
        "usage: %s --map PATH.yaml --goal GX,GY[,GT] [options]\n"
        "  --safety-radius N   default 6 cells\n"
        "  --threshold N       default 0\n"
        "  --max-sweeps N      default 200\n"
        "  --out-value PATH    raw uint16 value table\n"
        "  --out-action PATH   raw uint8 action table\n"
        "  --verify            compare with CPU reference\n"
        "  --mock              use mock ops (no FPGA)\n"
        "  -v                  verbose\n", prog);
}

static int parse_goal(const char *arg, int *gx, int *gy, int *gt) {
    *gt = -1;
    return sscanf(arg, "%d,%d,%d", gx, gy, gt) >= 2 ? 0 : -1;
}

static int write_bin(const char *path, const void *data, size_t n) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    size_t w = fwrite(data, 1, n, f);
    fclose(f);
    return w == n ? 0 : -1;
}

int main(int argc, char **argv) {
    cli_args_t a = {0};
    a.safety_radius = 6;
    a.max_sweeps    = 200;

    static struct option opts[] = {
        {"map",           required_argument, 0, 'm'},
        {"goal",          required_argument, 0, 'g'},
        {"safety-radius", required_argument, 0, 's'},
        {"threshold",     required_argument, 0, 't'},
        {"max-sweeps",    required_argument, 0, 'n'},
        {"out-value",     required_argument, 0, 'V'},
        {"out-action",    required_argument, 0, 'A'},
        {"verify",        no_argument,       0, 'y'},
        {"mock",          no_argument,       0, 'M'},
        {"help",          no_argument,       0, 'h'},
        {0,0,0,0},
    };
    int c;
    while ((c = getopt_long(argc, argv, "v", opts, NULL)) != -1) {
        switch (c) {
        case 'm': a.map_path = optarg; break;
        case 'g':
            if (parse_goal(optarg, &a.gx, &a.gy, &a.gt) < 0) { usage(argv[0]); return 1; }
            break;
        case 's': a.safety_radius = atoi(optarg); break;
        case 't': a.threshold     = atoi(optarg); break;
        case 'n': a.max_sweeps    = atoi(optarg); break;
        case 'V': a.out_value     = optarg; break;
        case 'A': a.out_action    = optarg; break;
        case 'y': a.verify        = 1; break;
        case 'M': a.mock          = 1; break;
        case 'v': a.verbose       = 1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }
    if (!a.map_path) { usage(argv[0]); return 1; }

    signal(SIGINT, on_sigint);

    /* --- Load map --- */
    pgm_map_t map = {0};
    if (map_pgm_load(a.map_path, &map) < 0) {
        fprintf(stderr, "failed to load map %s\n", a.map_path);
        return 1;
    }
    if (a.verbose) printf("map: %dx%d res=%.3f\n", map.w, map.h, map.resolution);

    /* --- Open device --- */
    const vi_device_ops_t *ops;
    void *ctx;
    if (a.mock) {
        ops = &vi_mock_ops;
        ctx = vi_mock_ctx_new();
    } else {
#ifndef VI_MOCK_ONLY
        ops = &vi_linux_ops;
        ctx = NULL;  /* linux ops uses a singleton internal context */
#else
        fprintf(stderr, "built without linux ops; rerun with --mock\n");
        map_pgm_free(&map);
        return 2;
#endif
    }
    vi_device_t *dev = vi_open(ops, ctx);
    if (!dev) {
        fprintf(stderr, "vi_open failed\n");
        map_pgm_free(&map);
        if (a.mock) vi_mock_ctx_free(ctx);
        return 2;
    }

    /* --- Initialise tables --- */
    size_t nv, np, nt;
    uint16_t *val = vi_value_buffer(dev, &nv);
    uint16_t *pen = vi_penalty_buffer(dev, &np);
    uint32_t *tr  = vi_trans_buffer(dev, &nt);
    (void)nt;

    if ((size_t)map.w * map.h * VI_N_THETA > nv ||
        (size_t)map.w * map.h > np) {
        fprintf(stderr, "map too large for preallocated buffers\n");
        vi_close(dev);
        if (a.mock) vi_mock_ctx_free(ctx);
        map_pgm_free(&map);
        return 1;
    }

    for (size_t i = 0; i < (size_t)map.w * map.h * VI_N_THETA; i++) val[i] = 0xFFFF;
    penalty_build(&map, a.safety_radius, a.gx, a.gy, pen);

    /* Mark goal value = 0 */
    if (a.gx >= 0 && a.gx < map.w && a.gy >= 0 && a.gy < map.h) {
        if (a.gt >= 0 && a.gt < VI_N_THETA) {
            val[((size_t)a.gy * map.w + a.gx) * VI_N_THETA + a.gt] = 0;
        } else {
            for (int it = 0; it < VI_N_THETA; it++)
                val[((size_t)a.gy * map.w + a.gx) * VI_N_THETA + it] = 0;
        }
    }

    transitions_compute(map.resolution, tr);

    /* --- Run --- */
    vi_run_config_t cfg = { map.w, map.h, (uint16_t)a.threshold, a.max_sweeps };
    vi_run_stats_t stats = {0};
    int rc = vi_run_until_converged(dev, &cfg, &stats);

    printf("sweeps=%d final_delta=%u converged=%d elapsed=%.3fs\n",
           stats.sweeps, (unsigned)stats.final_delta, stats.converged, stats.elapsed_sec);

    int exit_code = (rc == VI_OK) ? 0 : 3;

    /* --- Verify against reference --- */
    if (a.verify) {
        size_t n = (size_t)map.w * map.h * VI_N_THETA;
        uint16_t *ref_val = malloc(n * sizeof(uint16_t));
        for (size_t i = 0; i < n; i++) ref_val[i] = 0xFFFF;
        if (a.gx >= 0 && a.gx < map.w && a.gy >= 0 && a.gy < map.h)
            for (int it = 0; it < VI_N_THETA; it++)
                ref_val[((size_t)a.gy * map.w + a.gx) * VI_N_THETA + it] = 0;
        vi_reference_run(ref_val, pen, tr, map.w, map.h, (uint16_t)a.threshold, a.max_sweeps);

        int mismatches = 0;
        for (size_t i = 0; i < n; i++)
            if (ref_val[i] != val[i]) mismatches++;
        printf("verify: %s (%d mismatches)\n",
               mismatches == 0 ? "PASS" : "FAIL", mismatches);
        if (mismatches) exit_code = 3;
        free(ref_val);
    }

    /* --- Outputs --- */
    if (a.out_value)
        write_bin(a.out_value, val, (size_t)map.w * map.h * VI_N_THETA * 2);

    if (a.out_action) {
        uint8_t *act = malloc((size_t)map.w * map.h * VI_N_THETA);
        if (vi_compute_action_table(dev, map.w, map.h, act) == VI_OK)
            write_bin(a.out_action, act, (size_t)map.w * map.h * VI_N_THETA);
        free(act);
    }

    /* g_interrupt is wired but currently unused beyond the handler; reference
       it so -Wunused-variable doesn't complain in non-verbose builds. */
    (void)g_interrupt;

    vi_close(dev);
    if (a.mock) vi_mock_ctx_free(ctx);
    map_pgm_free(&map);
    return exit_code;
}
