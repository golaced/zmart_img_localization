#ifndef OPENCV_flow
#define OPENCV_flow

#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#define __INLINE inline

/////////////////////////////////////// all parameters ////////////////////////////////////////
extern int IMAGE_COLS;
extern int IMAGE_ROWS;

extern int CORNER_THRE;
extern int SAD_THRE;

extern int SEARCH_SIZE;    // 8 maximum offset to search: 4 + 1/2 pixels  (better do not bigger than 8)
extern int NUM_BLOCKS;      // 5 do not bigger than 8 ,do not less than 4
////////////////////////////////////////////////////////////////////////////////////////////////

#define sign(x) (( x > 0 ) - ( x < 0 ))

double getTimeval();

uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float *pixel_flow_x, float *pixel_flow_y);

uint32_t zrn_uhadd8(uint32_t f1, uint32_t f2);

uint32_t zrn_usad8(uint32_t f1, uint32_t f2);

uint32_t zrn_usada8(uint32_t f1, uint32_t f2, uint32_t f3);

uint32_t zrn_uadd8(uint32_t f1, uint32_t f2);

#endif
