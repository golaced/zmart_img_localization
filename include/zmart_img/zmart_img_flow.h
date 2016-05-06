#ifndef ZMART_IMG_FLOW_
#define ZMART_IMG_FLOW_

#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#define __INLINE inline
#define sign(x) ((x > 0) - (x < 0))

//////all parameters ///////
   extern int IMAGE_COLS; // The columns of image
    extern int IMAGE_ROWS;

    extern int CORNER_THRE;
    extern int SAD_THRE;  // the threshold of SAD

    extern int SEARCH_SIZE;

    extern int NUM_BLOCKS;

    extern int NUM_BLOCKS;



public:
    zmart
    double getTimeval();
    uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float *pixel_flow_x, float *pixel_flow_y);

    float com_rotation(int8_t sumx, int8_t sumy, uint16_t i, uint16_t j);
    uint32_t zrn_uhadd8(uint32_t f1, uint32_t f2);

    uint32_t zrm_usad8(uint32_t f1, uint32_t f2);

    uint32_t zrn_usada8(uint32_t f1, uint32_t f2, uint32_t f3);

    uint32_t zrn_uadd8(uint32_t f1, uint32_t f2);

#endif
