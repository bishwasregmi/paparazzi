/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_example.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"


int opencv_example(char *img, int width, int height) {
    // Create a new image, using the original bebop image.
    Mat M(height, width, CV_8UC2, img);
    Mat image;

#if OPENCVDEMO_GRAYSCALE
    //  Grayscale image example
    cvtColor(M, image, CV_YUV2GRAY_Y422);
    // Canny edges, only works with grayscale image
    int edgeThresh = 35;
    Canny(image, image, edgeThresh, edgeThresh * 3);
    // Convert back to YUV422, and put it in place of the original image
    grayscale_opencv_to_yuv422(image, img, width, height);
#else // OPENCVDEMO_GRAYSCALE
    // Color image example
    // Convert the image to an OpenCV Mat
    cvtColor(M, image, CV_YUV2BGR_Y422);



    // convert the image to HSV

    Mat HSVimg;

    cvtColor(image, HSVimg, CV_BGR2HSV);


    // patch x and y coordinate
    int patch_green[48][104];

    // defining alternative horizon

    int altHorLoc[104];

    // do the patch finding:

    // creating a loop to find the alternative horizon location by scanning the number of green pixels in all patches

    int count = 0;

    int x;
    int y;

    int xx;
    int yy;

    for (x = 0; x < 48; x++) {
        for (y = 0; y < 104; y++) {
            // Now that the grid of patches of 10x10 pixels is established find the number of green values within one patch
            for (xx = x * 5; xx < (x * 5) + 5; xx++) {
                for (yy = y * 5; yy < (y * 5) + 5; yy++) {
                    // intensity hsv per pixel
                    Vec3b intensity = HSVimg.at<Vec3b>(yy, xx);
                    uint8_t current_H = intensity.val[0];
                    uint8_t current_S = intensity.val[1];
                    uint8_t current_V = intensity.val[2];
                    //printf("%d\n", current_H);
                    //printf("%d\n", current_S);
                    //printf("%d\n", current_V);
                    // detect green and add to count
                    if ((current_H > 26) && (current_H < 80) &&
                        (current_S > 105) && (current_S < 255) &&
                        (current_V > 85) && (current_V < 255)) {
                        count++;
                    }
                }
            }
            patch_green[x][y] = count;
            count = 0;
        }
        int aa = 1;
    }

    // looping through patches to find where the green starts when looking from the vertical


    for (y = 103; y > 0; y--) {
        for (x = 46; x > 0; x--) {
            if (patch_green[x][y] > 7) {
                altHorLoc[y] = (x*5); // finding the location of the alternative horizon
                // paint the area bright on the image
                if ((y > 3) && (x > 3)) {
                    for (xx = -2; xx < 3; xx++) {
                        for (yy = -2; yy < 3; yy++) {
                            HSVimg.at<Vec3b>((y * 5) + yy, ((x * 5)+5) + xx) = Vec3b(40, 150, 120);
                        }
                    }
                }
                break;
            }
        }
    }




    Mat imgNew;

    cvtColor(HSVimg, imgNew, CV_HSV2BGR);


    // Blur it, because we can
    //blur(HSVimg, HSVimg, Size(10, 10));
    // Convert back to YUV422 and put it in place of the original image
    colorbgr_opencv_to_yuv422(imgNew, img, width, height);


#endif // OPENCVDEMO_GRAYSCALE

    return 0;
}


