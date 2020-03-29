/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"


#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)


#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif



// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  uint32_t color_count_1;
  uint32_t color_count_2;
  uint16_t greenp_Loc;

  bool updated;
};

// Additional variables

uint8_t treshold_loc = 60; // <-// ADJUST THIS TO CHANGE THE THRESHOLD WERE GREEN PATCHES ARE TOO LOW...


struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t treshold_loc,  uint8_t mark);

// ---------------------------- OPENCVSTUFF -----------------------------------------------



/*
// Function
struct image_t *opencv_func(struct image_t *img);
struct image_t *opencv_func(struct image_t *img)
{

    if (img->type == IMAGE_YUV422) {
        // Call OpenCV (C++ from paparazzi C function)
        opencv_example((char *) img->buf, img->w, img->h);
    }

// opencv_example(NULL, 10,10);

    return NULL;
}
*/








// ---------------------------- END OPENCVSTUFF -----------------------------------------------




/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */




static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  // opencv stuff

    if (img->type == IMAGE_YUV422) {
        // Call OpenCV (C++ from paparazzi C function)
        opencv_example((char *) img->buf, img->w, img->h);
    }


  // Filter and find centroid
  uint16_t greenp_Loc = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, treshold_loc, 3);
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, treshold_loc, 0);
  uint32_t count_1 = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, treshold_loc, 1);
  uint32_t count_2 = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, treshold_loc, 2);

  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].color_count_1 = count_1;
  global_filters[filter-1].color_count_2 = count_2;
  global_filters[filter-1].greenp_Loc = greenp_Loc;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img);
struct image_t *object_detector1(struct image_t *img)
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img);
struct image_t *object_detector2(struct image_t *img)
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t treshold_loc, uint8_t mark) {
    uint32_t cnt = 0;
    uint32_t tot_x = 0;
    uint32_t tot_y = 0;
    uint8_t *buffer = img->buf;

    // new added variables @Robbin

    uint8_t patch_green[48][104] = {0};   // <- will store the amount of green counts in each patch of the image
    int16_t p;                      // for the loops
    int16_t pp;                     // ,,
    int16_t q;                      // ,,
    int16_t qq;                     // ,,
    uint8_t g_count;                // count of green pixels in current patch
    uint8_t altHorLoc[104] = {0};         // the location of the first green top down along the width of the image
    uint32_t danger_count = 0;          // count the amount of patches that occur below the treshold "mark"

    // The loop is nested multiple times nevertheless it loops still 240*520 times (so once for each pixel

    switch (mark) {
        case 3:


            for (p = 0; p < 48; p++) {
                for (q = 0; q < 104; q++) {
                    // Now we will have to find the number of green in each patch
                    for (pp = p * 5; pp < (p * 5) + 5; pp++) {
                        for (qq = q * 5; qq < (q * 5) + 5; qq++) {   // looping through each pixel within the patch
                            // find yuv values
                            // Check if the color is inside the specified values
                            uint8_t *yp, *up, *vp;
                            if (pp % 2 == 0) {
                                // Even x
                                up = &buffer[qq * 2 * img->w + 2 * pp];      // U
                                yp = &buffer[qq * 2 * img->w + 2 * pp + 1];  // Y1
                                vp = &buffer[qq * 2 * img->w + 2 * pp + 2];  // V
                                //yp = &buffer[qq * 2 * img->w + 2 * pp + 3]; // Y2
                            } else {
                                // Uneven x
                                up = &buffer[qq * 2 * img->w + 2 * pp - 2];  // U
                                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                                vp = &buffer[qq * 2 * img->w + 2 * pp];      // V
                                yp = &buffer[qq * 2 * img->w + 2 * pp + 1];  // Y2
                            }
                            if ((*yp >= lum_min) && (*yp <= lum_max) &&
                                (*up >= cb_min) && (*up <= cb_max) &&
                                (*vp >= cr_min) && (*vp <= cr_max)) {
                                g_count++;
                            }
                        }
                    }
                    patch_green[p][q] = g_count;
                    g_count = 0;
                }
            }

            // Now we need to loop once more through the patches (240*520/25=4992 loops) to find where the green first appears looking from top to bottom

            for (q = 103; q > 0; q--) {
                for (p = 46; p > 0; p--) {
                    // screening the patches from top to bottom find the first patch with sufficient green and return its location
                    if (patch_green[p][q] > 15) {
                        altHorLoc[q] = (p * 5) + 5;
                        // check if the detected patch is to close to the bottom of the image and if yes add it to count
                        if ((q > 30) && (q < 73) && (altHorLoc[q] < treshold_loc)) {
                            danger_count++;
                        }
                        // draw the patch bright:
                        if ((q > 3) && (p > 3)) {
                            for (pp = -2; pp < 3; pp++) {
                                for (qq = -2; qq < 3; qq++) {
                                    uint8_t *yp, *up, *vp;
                                    if (pp % 2 == 0) {
                                        // Even x
                                        up = &buffer[((q * 5) + qq) * 2 * img->w + 2 * (((p * 5) + 5) + pp)];      // U
                                        yp = &buffer[((q * 5) + qq) * 2 * img->w + 2 * (((p * 5) + 5) + pp) + 1];  // Y1
                                        vp = &buffer[((q * 5) + qq) * 2 * img->w + 2 * (((p * 5) + 5) + pp) + 2];  // V
                                        //yp = &buffer[qq * 2 * img->w + 2 * pp + 3]; // Y2
                                    } else {
                                        // Uneven x
                                        up = &buffer[((q * 5) + qq) * 2 * img->w + 2 * (((p * 5) + 5) + pp) - 2];  // U
                                        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                                        vp = &buffer[((q * 5) + qq) * 2 * img->w + 2 * (((p * 5) + 5) + pp)];      // V
                                        yp = &buffer[((q * 5) + qq) * 2 * img->w + 2 * (((p * 5) + 5) + pp) + 1];  // Y2
                                    }
                                    if (draw) {
                                        *yp = 255;  // make pixel brighter in image
                                    }
                                }
                            }
                        }
                        break;
                    }

                }
                // if no patch was found than its also considered dangerous to fly
                if ((q > 23) && (q < 85) && (altHorLoc[q] < treshold_loc)) {
                    danger_count++;
                }
            }
            return danger_count;
    }


  switch (mark){ 
    case 0:
    //Scan for lower half and left 1/3 of the img
      lum_min = cod_lum_min1;
      for (uint16_t y = 0; y < img->h/4; y++) {
        for (uint16_t x = floor(img->w/5); x < img->w/2; x ++) {
          // Check if the color is inside the specified values
          uint8_t *yp, *up, *vp;
          if (x % 2 == 0) {
            // Even x
            up = &buffer[y * 2 * img->w + 2 * x];      // U
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
          } else {
              // Uneven x
              up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
              //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
              vp = &buffer[y * 2 * img->w + 2 * x];      // V
              yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
          }
          if ( (*yp >= lum_min) && (*yp <= lum_max) &&
             (*up >= cb_min ) && (*up <= cb_max ) &&
             (*vp >= cr_min ) && (*vp <= cr_max )) {
           cnt ++;
           tot_x += x;
           tot_y += y;
           if (draw){
              *yp = 30;  // make pixel brighter in image
            }
          }
        }
      }    
      if (cnt > 0) {
        *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
        *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
      } else {
        *p_xc = 0;
        *p_yc = 0;
      }
      return cnt;      
      break;

      //x = img->w/3*2; x < img->w; x ++
    
    case 1:
    //Scan for lower half and centeral 1/3 of the img
      lum_min = cod_lum_min1;
      for (uint16_t y = img->h/4; y < (img->h/4)*3; y++) {
        for (uint16_t x = floor(img->w/5); x < img->w/2; x ++) {
          // Check if the color is inside the specified values
          uint8_t *yp, *up, *vp;
          if (x % 2 == 0) {
            // Even x
            up = &buffer[y * 2 * img->w + 2 * x];      // U
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
          } else {
              // Uneven x
              up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
              //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
              vp = &buffer[y * 2 * img->w + 2 * x];      // V
              yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
          }
          if ( (*yp >= lum_min) && (*yp <= lum_max) &&
             (*up >= cb_min ) && (*up <= cb_max ) &&
             (*vp >= cr_min ) && (*vp <= cr_max )) {
           cnt ++;
           tot_x += x;
           tot_y += y;
           if (draw){
              *yp =  30;  // make pixel brighter in image
            }
          }
        }
      }    
      if (cnt > 0) {
        *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
        *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
      } else {
        *p_xc = 0;
        *p_yc = 0;
      }
      return cnt;
      break;
    
    case 2:
    //Scan for lower half and right 1/3 of the img
      lum_min = cod_lum_min1;
      for (uint16_t y = (img->h/4)*3; y < img->h; y++) {
        for (uint16_t x = floor(img->w/5); x < img->w/2; x ++) {
          // Check if the color is inside the specified values
          uint8_t *yp, *up, *vp;
          if (x % 2 == 0) {
            // Even x
            up = &buffer[y * 2 * img->w + 2 * x];      // U
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
          } else {
              // Uneven x
              up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
              //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
              vp = &buffer[y * 2 * img->w + 2 * x];      // V
              yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
          }
          if ( (*yp >= lum_min) && (*yp <= lum_max) &&
             (*up >= cb_min ) && (*up <= cb_max ) &&
             (*vp >= cr_min ) && (*vp <= cr_max )) {
           cnt ++;
           tot_x += x;
           tot_y += y;
           if (draw){
              *yp =  30;  // make pixel brighter in image
            }
          }
        }
      }    
      if (cnt > 0) {
        *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
        *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
      } else {
        *p_xc = 0;
        *p_yc = 0;
      }
      return cnt;
	break;
      default:
	return cnt;
  };

}




void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, local_filters[0].color_count_1, local_filters[0].color_count_2, local_filters[0].greenp_Loc);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, local_filters[0].color_count_1, local_filters[0].color_count_2, local_filters[0].greenp_Loc);
    local_filters[1].updated = false;
  }
}
