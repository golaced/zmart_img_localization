#include "zmart_img/zmart_img_flow.h"

bool first_flag_low_pass = true;
float flow_x_last = 0; flow_y_last = 0;


double getTimeval()
{
    struct timeval stTimeval;
    gettimeofday(&stTimeval, NULL); //stTimeval reserve the sec and usec
    return stTimeval.tv_sec + (double)stTimeval.tv_usec*1E-6;

}

uint32_t zrn_uadd8(uint32_t f1, uint32_t f2)
{
    uint8_t *p1 = (uint8_t*) &f1;
    uint8_t *p2 = (uint8_t*) &f2;
    uint32_t tmp = 0;
    uint8_t *p3 = (uint8_t*) &tmp;

    p3[0] = p1[0] + p2[0];
	p3[1] = p1[1] + p2[1];
	p3[2] = p1[2] + p2[2];
	p3[3] = p1[3] + p2[3];

	return tmp;
}

uint32_t zrn_usada8(uint32_t f1, uint32_t f2, uint32_t f3)
{
	uint8_t *p1 = (uint8_t*)&f1;
	uint8_t *p2 = (uint8_t*)&f2;
	uint32_t tmp = 0;
	tmp = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2]) + abs(p1[3] - p2[3]) + f3;
	return tmp;

}


uint32_t zrn_usad8(uint32_t f1, uint32_t f2)
{
	uint8_t *p1 = (uint8_t*)&f1;
	uint8_t *p2 = (uint8_t*)&f2;
	uint32_t tmp = 0;
	tmp = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2]) + abs(p1[3] - p2[3]);
	return tmp;
}

uint32_t zrn_uhadd8(uint32_t f1, uint32_t f2)
{
	uint8_t *p1 = (uint8_t*)&f1;
	uint8_t *p2 = (uint8_t*)&f2;
	uint32_t tmp = 0;
	uint8_t *p3 = (uint8_t*)&tmp;

	p3[0] = (uint8_t)((uint32_t)(p1[0] + p2[0])) >> 1;
	p3[1] = (uint8_t)((uint32_t)(p1[1] + p2[1])) >> 1;
	p3[2] = (uint8_t)((uint32_t)(p1[2] + p2[2])) >> 1;
	p3[3] = (uint8_t)((uint32_t)(p1[3] + p2[3])) >> 1;

	return tmp;
}

/******
**@brief compute the average pixel gradient of all horizontal and vertical steps
*
**TODO
*
**@param image ...
**@param offX x coordinate of upper left corner of 8x8 pattern in image
*
**@param offY y coordinate of upper left corner of 8X8 pattern in image
*
*******/

static inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
    /* calculate position in image buffer */
	uint16_t off = (offY - 2) * row_size + (offX - 2); // we calc only the 4x4 pat
	uint32_t acc;

    /* calc row diff */
	acc = zrn_usad8(*((uint32_t*)&image[off + 0 + 0 * row_size]), *((uint32_t*)&image[off + 0 + 1 * row_size]));
	acc = zrn_usada8(*((uint32_t*)&image[off + 0 + 1 * row_size]), *((uint32_t*)&image[off + 0 + 2 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image[off + 0 + 2 * row_size]), *((uint32_t*)&image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 * row_size] << 8 | image[off + 0 + 3 * row_size];
	uint32_t col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 * row_size] << 8 | image[off + 1 + 3 * row_size];
	uint32_t col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 2 + 3 * row_size];
	uint32_t col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 * row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = zrn_usada8(col1, col2, acc);
	acc = zrn_usada8(col2, col3, acc);
	acc = zrn_usada8(col3, col4, acc);
}

static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	* calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	* every iteration is one line of the 8x8 field.
	*
	*  + - + - + - + - + - + - + - + - +
	*  |   |   |   |   |   |   |   |   |
	*  + - + - + - + - + - + - + - + - +
	*
	*
	*/

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		* first column of 4 pixels:
		*
		*  + - + - + - + - + - + - + - + - +
		*  | x | x | x | x |   |   |   |   |
		*  + - + - + - + - + - + - + - + - +
		*
		* the 8 s values are from following positions for each pixel (X):
		*  + - + - + - +
		*  +   5   7   +
		*  + - + 6 + - +
		*  +   4 X 0   +
		*  + - + 2 + - +
		*  +   3   1   +
		*  + - + - + - +
		*
		*  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		*
		*/

		/* compute average of two pixel values */
		s0 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 1 + (i + 0) * row_size])));
		s1 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 + 1 + (i + 1) * row_size])));
		s2 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 0 + (i + 1) * row_size])));
		s3 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 - 1 + (i + 1) * row_size])));
		s4 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 - 1 + (i + 0) * row_size])));
		s5 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 - 1 + (i - 1) * row_size])));
		s6 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 0 + (i - 1) * row_size])));
		s7 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 + 1 + (i - 1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (zrn_uhadd8(s0, s1));
		t3 = (zrn_uhadd8(s3, s4));
		t5 = (zrn_uhadd8(s4, s5));
		t7 = (zrn_uhadd8(s7, s0));

		/*
		* finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		*  + - + - + - +
		*  |   |   |   |
		*  + - 5 6 7 - +
		*  |   4 X 0   |
		*  + - 3 2 1 - +
		*  |   |   |   |
		*  + - + - + - +
		*/

		/* fill accumulation vector */
		acc[0] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		* same for second column of 4 pixels:
		*
		*  + - + - + - + - + - + - + - + - +
		*  |   |   |   |   | x | x | x | x |
		*  + - + - + - + - + - + - + - + - +
		*
		*/

		s0 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 5 + (i + 0) * row_size])));
		s1 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 + 5 + (i + 1) * row_size])));
		s2 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 4 + (i + 1) * row_size])));
		s3 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 + 3 + (i + 1) * row_size])));
		s4 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 3 + (i + 0) * row_size])));
		s5 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 + 3 + (i - 1) * row_size])));
		s6 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 4 + (i - 1) * row_size])));
		s7 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 + 5 + (i - 1) * row_size])));

		t1 = (zrn_uhadd8(s0, s1));
		t3 = (zrn_uhadd8(s3, s4));
		t5 = (zrn_uhadd8(s4, s5));
		t7 = (zrn_uhadd8(s7, s0));

		acc[0] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

static inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = zrn_usad8(*((uint32_t*)&image1[off1 + 0 + 0 * row_size]), *((uint32_t*)&image2[off2 + 0 + 0 * row_size]));
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 0 * row_size]), *((uint32_t*)&image2[off2 + 4 + 0 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 1 * row_size]), *((uint32_t*)&image2[off2 + 0 + 1 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 1 * row_size]), *((uint32_t*)&image2[off2 + 4 + 1 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 2 * row_size]), *((uint32_t*)&image2[off2 + 0 + 2 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 2 * row_size]), *((uint32_t*)&image2[off2 + 4 + 2 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 3 * row_size]), *((uint32_t*)&image2[off2 + 0 + 3 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 3 * row_size]), *((uint32_t*)&image2[off2 + 4 + 3 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 4 * row_size]), *((uint32_t*)&image2[off2 + 0 + 4 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 4 * row_size]), *((uint32_t*)&image2[off2 + 4 + 4 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 5 * row_size]), *((uint32_t*)&image2[off2 + 0 + 5 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 5 * row_size]), *((uint32_t*)&image2[off2 + 4 + 5 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 6 * row_size]), *((uint32_t*)&image2[off2 + 0 + 6 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 6 * row_size]), *((uint32_t*)&image2[off2 + 4 + 6 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 7 * row_size]), *((uint32_t*)&image2[off2 + 0 + 7 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 7 * row_size]), *((uint32_t*)&image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

/****************************** compensate rotation ***********************************************/
float com_rotation(int8_t sumx, int8_t sumy, uint16_t i, uint16_t j)
{
	float dis = (i-32)*(i-32) + (j-32)*(j-32);
	float rotate = sumx*(j - 32) - sumy * (i - 32);
	return rotate / dis;
}


uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float *pixel_flow_x, float *pixel_flow_y)
{
	/* constants */
	const int16_t winmin = -SEARCH_SIZE;
	const int16_t winmax = SEARCH_SIZE;

	/* variables */
    	uint16_t pixLo = SEARCH_SIZE + 1;
    	uint16_t pixHi_x = IMAGE_COLS - (SEARCH_SIZE + 1) - 2*SEARCH_SIZE;
    	uint16_t pixHi_y = IMAGE_ROWS - (SEARCH_SIZE + 1) - 2*SEARCH_SIZE;
   	uint16_t pixStep_x = (pixHi_x - pixLo) / (NUM_BLOCKS-1);
   	uint16_t pixStep_y = (pixHi_y - pixLo) / (NUM_BLOCKS-1);
	uint16_t i, j;
	uint32_t acc[8]; // subpixels

	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;


	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi_y; j += pixStep_y)
	{
		for (i = pixLo; i < pixHi_x; i += pixStep_x)
		{
			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, IMAGE_COLS);
			if (diff < CORNER_THRE)
			{
				continue;
			}

			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;

			for (jj = winmin; jj <= winmax; jj++)
			{
				for (ii = winmin; ii <= winmax; ii++)
				{
					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, IMAGE_COLS);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < SAD_THRE)
			{
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, IMAGE_COLS);
				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction
				for(uint8_t k = 0; k < 8; k++)
				{
					if (acc[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}
				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				image1[(j+sumy) * IMAGE_COLS +sumx+ i] = 0;
				image1[(j+sumy) * IMAGE_COLS +sumx+ i+1] = 0;
				image1[(j+sumy+1) * IMAGE_COLS +sumx+ i] = 0;
				image1[(j+sumy+1) * IMAGE_COLS +sumx+ i+1] = 0;
			}
			image1[(j) * IMAGE_COLS + i] = 255;
			image1[(j) * IMAGE_COLS + i+1] = 255;
			image1[(j+1) * IMAGE_COLS + i] = 255;
			image1[(j+1) * IMAGE_COLS + i+1] = 255;
		}
	}

	//printf("meancount: %d\n",meancount );
	/* create flow image if needed (image1 is not needed anymore)
	 * -> can be used for debugging purpose
	 */
/*	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{

			uint32_t diff = compute_diff(image1, i, j, FRAME_SIZE);
			if (diff > CORNER_THRE)
			{
				image1[(j) * FRAME_SIZE + i] = 255;
				image1[(j) * FRAME_SIZE + i+1] = 255;
				image1[(j+1) * FRAME_SIZE + i] = 255;
				image1[(j+1) * FRAME_SIZE + i+1] = 255;
			}

		}
	}
*/

	/* evaluate flow calculation */
	if (meancount > 10)
	{
		meanflowx /= meancount;
		meanflowy /= meancount;

		/* use average of accepted flow values */
		uint32_t meancount_x = 0;
		uint32_t meancount_y = 0;

		for (uint8_t h = 0; h < meancount; h++)
		{
			float subdirx = 0.0f;
			if (subdirs[h] == 0 || subdirs[h] == 1 || subdirs[h] == 7) subdirx = 0.5f;
			if (subdirs[h] == 3 || subdirs[h] == 4 || subdirs[h] == 5) subdirx = -0.5f;
			histflowx += (float)dirsx[h] + subdirx;
			meancount_x++;

			float subdiry = 0.0f;
			if (subdirs[h] == 5 || subdirs[h] == 6 || subdirs[h] == 7) subdiry = -0.5f;
			if (subdirs[h] == 1 || subdirs[h] == 2 || subdirs[h] == 3) subdiry = 0.5f;
			histflowy += (float)dirsy[h] + subdiry;
			meancount_y++;
		}

		histflowx /= meancount_x;
		histflowy /= meancount_y;

		*pixel_flow_x = histflowx;
		*pixel_flow_y = histflowy;

		/******************************* compensate rotation here ***********************/



		/********************************************************************************/


        //  low_pass stuff //
		if(first_flag_low_pass == true)
		{
			flow_x_last = *pixel_flow_x;
			flow_y_last = *pixel_flow_y;
			first_flag_low_pass = false;
		}
		else
		{
			*pixel_flow_x = *pixel_flow_x * LOW_PASS / 100. + (1-LOW_PASS / 100.)* flow_x_last;
			*pixel_flow_y = *pixel_flow_y * LOW_PASS / 100. + (1-LOW_PASS / 100.)* flow_y_last;
			flow_x_last = *pixel_flow_x;
			flow_y_last = *pixel_flow_y;
		}
	}
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));

	return qual;
}















