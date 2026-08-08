#include <assert.h>
#include "map_pgm.h"

int main(void) {
    pgm_map_t m = {0};
    int rc = map_pgm_load("test/data/tiny.yaml", &m);
    assert(rc == 0);
    assert(m.w == 4);
    assert(m.h == 4);
    assert(m.resolution > 0.049 && m.resolution < 0.051);
    assert(m.pixels[0] == 0);        /* top-left black */
    assert(m.pixels[4] == 255);      /* second row white */
    map_pgm_free(&m);
    return 0;
}
