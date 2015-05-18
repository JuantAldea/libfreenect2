/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <math.h>
#include <libfreenect2/registration.h>
#include <iostream>
#include <opencv2/opencv.hpp>
namespace libfreenect2
{

/*
 * most information, including the table layout in command_response.h, was
 * provided by @sh0 in https://github.com/OpenKinect/libfreenect2/issues/41
 */

// these seem to be hardcoded in the original SDK
static const float depth_q = 0.01;
static const float color_q = 0.002199;

void Registration::undistort_depth(int x, int y, float& mx, float& my)
{
  float dx = ((float)x - depth.cx) / depth.fx;
  float dy = ((float)y - depth.cy) / depth.fy;

  float ps = (dx * dx) + (dy * dy);
  float qs = ((ps * depth.k3 + depth.k2) * ps + depth.k1) * ps + 1.0;
  for (int i = 0; i < 9; i++) {
    float qd = ps / (qs * qs);
    qs = ((qd * depth.k3 + depth.k2) * qd + depth.k1) * qd + 1.0;
  }

  mx = dx / qs;
  my = dy / qs;
}



void Registration::undistort_depth(const Frame * const depth_in, Frame * const depth_out)
{
  const float * const depth_raw_in = (float*)depth_in->data;
  float * const depth_raw_out = (float*)depth_out->data;
  
  cv::Mat k = cv::Mat::eye(3, 3, CV_32FC1);
  k.at<float>(0, 0) = depth.fx;
  k.at<float>(1, 1) = depth.fy;
  k.at<float>(2, 2) = 1.0;
  k.at<float>(0, 2) = depth.cx;
  k.at<float>(1, 2) = depth.cy;
  
  cv::Mat dist_coeffs = cv::Mat::zeros(1, 8, CV_32F);
  dist_coeffs.at<float>(0,0) = depth.k1;
  dist_coeffs.at<float>(0,1) = depth.k2;
  dist_coeffs.at<float>(0,2) = depth.p1;
  dist_coeffs.at<float>(0,3) = depth.p2;
  dist_coeffs.at<float>(0,4) = depth.k3;

  cv::Mat image = cv::Mat(depth_in->height, depth_in->width, CV_32FC1, depth_in->data);
  cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(k, dist_coeffs, cv::Size(depth_in->height, depth_in->width), 0.0);
  cv::Mat map1, map2;
  cv::initUndistortRectifyMap(new_camera_matrix, dist_coeffs, cv::noArray(), new_camera_matrix, image.size(), CV_32FC1, map1, map2);
  
  cv::Mat rectified;
  cv::remap(image, rectified, map1, map2, cv::INTER_LINEAR);
  memcpy(depth_raw_out, rectified.data, rectified.rows * rectified.cols * rectified.elemSize());
  depth_out->height = rectified.rows;
  depth_out->width = rectified.cols;
}

void Registration::depth_to_color(float mx, float my, float& rx, float& ry)
{
  mx *= depth.fx * depth_q;
  my *= depth.fy * depth_q;

  float wx =
    (mx * mx * mx * color.mx_x3y0) + 
    (my * my * my * color.mx_x0y3) +
    (mx * mx * my * color.mx_x2y1) + 
    (my * my * mx * color.mx_x1y2) +
    (mx * mx * color.mx_x2y0) + 
    (my * my * color.mx_x0y2) + 
    (mx * my * color.mx_x1y1) +
    (mx * color.mx_x1y0) + 
    (my * color.mx_x0y1) +
    (color.mx_x0y0);

  float wy =
    (mx * mx * mx * color.my_x3y0) +
    (my * my * my * color.my_x0y3) +
    (mx * mx * my * color.my_x2y1) +
    (my * my * mx * color.my_x1y2) +
    (mx * mx * color.my_x2y0) +
    (my * my * color.my_x0y2) +
    (mx * my * color.my_x1y1) +
    (mx * color.my_x1y0) + 
    (my * color.my_x0y1) +
    (color.my_x0y0);

  rx = wx / (color.fx * color_q);
  ry = wy / (color.fx * color_q);
}

void Registration::apply( int dx, int dy, float dz, float& cx, float &cy)
{
  float rx = depth_to_color_map[dx][dy][0];
  float ry = depth_to_color_map[dx][dy][1];

  rx += (color.shift_m / dz) - (color.shift_m / color.shift_d);

  cx = rx * color.fx + color.cx;
  cy = ry * color.fy + color.cy;
}

void Registration::apply(Frame* rgb, Frame* depth, unsigned char* registered)
{
  if (!depth || !rgb || !registered)
    return;

  float* depth_raw = (float*)depth->data;
  float cx, cy;
  int c_off, d_off, r_off;

  for (int x = 0; x < depth->width; x++) {
    for (int y = 0; y < depth->height; y++) {

      d_off = y*depth->width + x;
      r_off = d_off*rgb->bytes_per_pixel;

      float z_raw = depth_raw[d_off];
      if (z_raw == 0.0) {
        registered[r_off+0] = 0;
        registered[r_off+1] = 0;
        registered[r_off+2] = 0;
        continue;
      }

      apply(x,y,z_raw,cx,cy);

      c_off = (round(cx) + round(cy) * rgb->width) * rgb->bytes_per_pixel;
      if ((c_off < 0) || (c_off > rgb->width*rgb->height*rgb->bytes_per_pixel)) {
        registered[r_off+0] = 0;
        registered[r_off+1] = 0;
        registered[r_off+2] = 0;
        continue;
      }

      registered[r_off+0] = rgb->data[c_off+0];
      registered[r_off+1] = rgb->data[c_off+1];
      registered[r_off+2] = rgb->data[c_off+2];
    }
  }

}

void Registration::apply(const std::vector<std::tuple<float, float>> &vectors, const Frame * const depth, std::vector<std::tuple<float, float, float>> &reprojected_vectors)
{
  if (!depth){
    return;
  }

  const float* depth_raw = (float*)depth->data;
  float cx, cy;
  
  reprojected_vectors.resize(vectors.size());
  
  for (size_t i = 0; i < vectors.size(); i++) {
    float float_x, float_y;
    std::tie(float_x, float_y) = vectors[i];
    const int x = round(float_x);
    const int y = round(float_y);
    
    //valid points cannot be in the back of the camera

    const int d_off = y * depth->width + x;

    const float z_raw = depth_raw[d_off];
    if (z_raw == 0.0) {
      reprojected_vectors[i] = std::make_tuple(-1, -1, -1);
      continue;
    }

    apply(x, y, z_raw, cx, cy);
    
    reprojected_vectors[i] = std::make_tuple(cx, cy, z_raw);
  }
}


Registration::Registration(Freenect2Device::IrCameraParams depth_p, Freenect2Device::ColorCameraParams rgb_p):
  depth(depth_p), color(rgb_p)
{
  float mx, my;
  float rx, ry;

  for (int x = 0; x < 512; x++)
    for (int y = 0; y < 424; y++) {

      undistort_depth(x,y,mx,my);
      undistort_map[x][y][0] = mx;
      undistort_map[x][y][1] = my;

      depth_to_color(mx,my,rx,ry);
      depth_to_color_map[x][y][0] = rx;
      depth_to_color_map[x][y][1] = ry;
  }
}

} /* namespace libfreenect2 */
