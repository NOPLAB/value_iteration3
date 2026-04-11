#include "vi_assert.h"
#include "map_pgm.h"

int main(void) {
    pgm_map_t m = {0};
    int rc = map_pgm_load("test/data/tiny.yaml", &m);
    VI_ASSERT_EQ(rc, 0);
    VI_ASSERT_EQ(m.w, 4);
    VI_ASSERT_EQ(m.h, 4);
    VI_ASSERT(m.resolution > 0.049 && m.resolution < 0.051);
    VI_ASSERT_EQ(m.pixels[0], 0);        /* top-left black */
    VI_ASSERT_EQ(m.pixels[4], 255);      /* second row white */
    map_pgm_free(&m);
    VI_TEST_MAIN_END();
}
