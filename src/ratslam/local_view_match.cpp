/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "local_view_match.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
using namespace std;
#include <boost/foreach.hpp>
#include <algorithm>

#include <stdio.h>

namespace ratslam
{



LocalViewMatch::LocalViewMatch(ptree settings)  //设置的值  try是settings中给的值  catch返回错误，就将最后面的值放入  暂时理解
{
  get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);//异常检测，非一般的检测，有赋值的
  get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);//
  get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) 0);
  get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
  get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
  get_setting_from_ptree(VT_PANORAMIC, settings, "vt_panoramic", 0);
 
  get_setting_from_ptree(VT_MATCH_THRESHOLD, settings, "vt_match_threshold", 0.03);
  get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
  get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_crop_x_max", -1);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_crop_y_max", -1);

  TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;  //模板大小

  templates.reserve(10000);  //调整容量大小  是一个结构体

  current_view.resize(TEMPLATE_SIZE); //设置内存大小

  current_vt = 0;
  prev_vt = 0;
}


LocalViewMatch::~LocalViewMatch()
{

}

void LocalViewMatch::on_image(const unsigned char *view_rgb, bool greyscale, unsigned int image_width, unsigned int image_height)
{
  if (view_rgb == NULL)  //如果不是rgb图，直接退出这个函数
    return;

  IMAGE_WIDTH = image_width;  //宽度
  IMAGE_HEIGHT = image_height; //高度

  if (IMAGE_VT_X_RANGE_MAX == -1)//根据utils中的get_setting_from_ptree中catch赋值-1后，会显示错误
    IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
  if (IMAGE_VT_Y_RANGE_MAX == -1)
    IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;

  this->view_rgb = view_rgb;
  this->greyscale = greyscale;

  convert_view_to_view_template(greyscale);
  prev_vt = get_current_vt();//当前模板编号变成旧的
  unsigned int vt_match_id;
  compare(vt_error, vt_match_id);  //这个误差可能是MSE
  if (vt_error <= VT_MATCH_THRESHOLD)  //当前模板匹配，比较	  
  {
    set_current_vt((int)vt_match_id); //如果当前模板编号不是(int)vt_match_id，则令prev_vt为当前模板，当前模板编号就变成(int)vt_match_id
    cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
    cout.flush();//刷新缓冲区
  } //这种情况就是匹配成功，与之前场景类似
  else
  {
    vt_relative_rad = 0;
    set_current_vt(create_template());  //创建的新的编号
    cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
    cout.flush();
  }//以前的模板没有与之匹配的，创建新的

}//结束on_image


void LocalViewMatch::clip_view_x_y(int &x, int &y)
{
  if (x < 0)
    x = 0;
  else if (x > TEMPLATE_X_SIZE - 1)
    x = TEMPLATE_X_SIZE - 1;

  if (y < 0)
    y = 0;
  else if (y > TEMPLATE_Y_SIZE - 1)
    y = TEMPLATE_Y_SIZE - 1;

} //设定x和y的范围

void LocalViewMatch::convert_view_to_view_template(bool grayscale)
{
  int data_next = 0;
  int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
  int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
  int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
  int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
  int pos;

  for (unsigned int i; i < current_view.size(); i++)  //size获取字符长度
    current_view[i] = 0;//赋值变成0

  if (grayscale)//此处可能判断是否为灰度图
  {
    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
        y_block_size, y_block_count++)//for循环中括号中第三个语句是最后一步
    {
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
          x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH);
            current_view[data_next] += (double)(view_rgb[pos]);
          }
        }
        current_view[data_next] /= (255.0);
        current_view[data_next] /= (x_block_size * y_block_size);
        data_next++;
      }
    }
  }
  else
  {
    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
        y_block_size, y_block_count++)
    {
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
          x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH) * 3;  //会不会是rgb图3个通道
            current_view[data_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1])
                + (double)(view_rgb[pos + 2]));
          }
        }
        current_view[data_next] /= (255.0 * 3.0);   //与if的区别在这
        current_view[data_next] /= (x_block_size * y_block_size);

        data_next++;
      }
    }
  }//最终将图片信息存入current_view中

  if (VT_NORMALISATION > 0)
  {
    double avg_value = 0;

    for (unsigned int i = 0; i < current_view.size(); i++)
    {
      avg_value += current_view[i];
    }

    avg_value /= current_view.size(); //向量中的值相加，然后除向量中个数

    for (unsigned int i = 0; i < current_view.size(); i++)
    {
      current_view[i] = std::max(0.0, std::min(current_view[i] * VT_NORMALISATION / avg_value, 1.0));//重新赋值，剔除大于1和小于0的
    }
  }//应该是对图片信息进行处理

  // now do patch normalisation
  // +- patch size on the pixel, ie 4 will give a 9x9
  if (VT_PATCH_NORMALISATION > 0)
  {
    int patch_size = VT_PATCH_NORMALISATION;
    int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);
    double patch_sum;
    double patch_mean;
    double patch_std;
    int patch_x_clip;
    int patch_y_clip;

    // first make a copy of the view
    std::vector<double> current_view_copy;  //定义向量
    current_view_copy.resize(current_view.size()); //字符大小给这个作内存大小
    for (unsigned int i = 0; i < current_view.size(); i++)
      current_view_copy[i] = current_view[i];

    // this code could be significantly optimimised ....  显著的优化
    for (int x = 0; x < TEMPLATE_X_SIZE; x++)
    {
      for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
      {
        patch_sum = 0;
        for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
        {
          for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
          {
            patch_x_clip = patch_x;
            patch_y_clip = patch_y;
            clip_view_x_y(patch_x_clip, patch_y_clip);

            patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];
          }
        }
        patch_mean = patch_sum / patch_total;

        patch_sum = 0;
        for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
        {
          for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
          {
            patch_x_clip = patch_x;
            patch_y_clip = patch_y;
            clip_view_x_y(patch_x_clip, patch_y_clip);

            patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean)
                * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
          }
        }

        patch_std = sqrt(patch_sum / patch_total);

        if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)
          current_view[x + y * TEMPLATE_X_SIZE] = 0.5;
        else {
          current_view[x + y * TEMPLATE_X_SIZE] = max((double) 0, min(1.0, (((current_view_copy[x + y * TEMPLATE_X_SIZE] - patch_mean) / patch_std) + 3.0)/6.0 ));
        }
      }
    }
  }

  double sum = 0;

  // find the mean of the data  数据平均值
  for (int i = 0; i < current_view.size(); i++)
    sum += current_view[i];

  current_mean = sum/current_view.size();

}

// create and add a visual template to the collection 创建可视化模板并将其添加到集合中
int LocalViewMatch::create_template() //创建模板
{
  templates.resize(templates.size() + 1);  //模板数加1  size是返回templates向量中元素个数
  VisualTemplate * new_template = &(*(templates.end() - 1));// 末尾元素-1？  这是一个结构体

  new_template->id = templates.size() - 1;//可能最新的都是减一个
  double * data_ptr = &current_view[0];  //第一个
  new_template->data.reserve(TEMPLATE_SIZE); //reserve容器c应该预留多少个元素的空间  每一个data存储一个模板的容量
  for (int i = 0; i < TEMPLATE_SIZE; i++)
    new_template->data.push_back(*(data_ptr++));//元素最后一个存入

  new_template->mean = current_mean;

  return templates.size() - 1;
}//返回的应该是创建模板的编号

// compare a visual template to all the stored templates, allowing for 
// slen pixel shifts in each direction
// returns the matching template and the MSE
void LocalViewMatch::compare(double &vt_err, unsigned int &vt_match_id)
{
  if (templates.size() == 0)
  {
    vt_err = DBL_MAX;
    vt_error = vt_err;
    return;
  }

  double *data = &current_view[0];
  double mindiff, cdiff;
  mindiff = DBL_MAX;

  vt_err = DBL_MAX;
  int min_template = 0;

  double *template_ptr;
  double *column_ptr;
  double *template_row_ptr;
  double *column_row_ptr;
  double *template_start_ptr;
  double *column_start_ptr;
  int row_size;
  int sub_row_size;
  double *column_end_ptr;
  VisualTemplate vt;
  int min_offset;

  int offset;
  double epsilon = 0.005;

  if (VT_PANORAMIC)
  {

	BOOST_FOREACH(vt, templates)  //BOOST_FOREACH遍历STL容器  vt就像是templates中的每一个元素
	{

	if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)  //匹配系数阈值+0.005
	  continue;//结束本次循环，进行下一次循环

	// for each vt try matching the view at different offsets
	// try to fast break based on error already great than previous errors
	// handles 2d images shifting only in the x direction
	// note I haven't tested on a 1d yet.
	for (offset = 0; offset < TEMPLATE_X_SIZE; offset += VT_STEP_MATCH)  //有点像SAD算法，比较像素点
	{
	  cdiff = 0;
	  template_start_ptr = &vt.data[0] + offset;
	  column_start_ptr = &data[0];
	  row_size = TEMPLATE_X_SIZE;
	  column_end_ptr = &data[0] + TEMPLATE_SIZE - offset; //TEMPLATE_SIZE 是模板x*y
	  sub_row_size = TEMPLATE_X_SIZE - offset;  //其中的一段  相当于匹配区域长度减去现在偏移位置

	  // do from offset to end  偏移量到末尾 每一次循环加的是row_size，总共有TEMPLATE_Y_SIZE次
	  for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
	  {      //循环TEMPLATE_X_SIZE - offset次
		for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
		{
		  cdiff += abs(*column_ptr - *template_ptr);  //最小差值
		}

		// fast breaks
		if (cdiff > mindiff)
		  break;
	  }

	  // do from start to offset   开始到偏移量 
	  template_start_ptr = &vt.data[0];
	  column_start_ptr = &data[0] + TEMPLATE_X_SIZE - offset;
	  row_size = TEMPLATE_X_SIZE;
	  column_end_ptr = &data[0] + TEMPLATE_SIZE;
	  sub_row_size = offset;
	  for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
	  {
		for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
		{
		  cdiff += abs(*column_ptr - *template_ptr);
		}

		// fast breaks
		if (cdiff > mindiff)
		  break;
	  }


	  if (cdiff < mindiff)
	  {
		mindiff = cdiff;
		min_template = vt.id;  //对应误差和偏移量最小的模板编号
		min_offset = offset;
	  }
	}

	}

	vt_relative_rad = (double) min_offset/TEMPLATE_X_SIZE * 2.0 * M_PI;
	if (vt_relative_rad > M_PI)
	vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
	vt_err = mindiff / (double) TEMPLATE_SIZE;
	vt_match_id = min_template;

	vt_error = vt_err;

  } else {

	BOOST_FOREACH(vt, templates)
	{

	if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
	  continue;

	// for each vt try matching the view at different offsets
	// try to fast break based on error already great than previous errors
	// handles 2d images shifting only in the x direction
	// note I haven't tested on a 1d yet.
	for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH) //与上述区别上面是TEMPLATE_X_SIZE，这里是VT_SHIFT_MATCH*2+1
	{
	  cdiff = 0;
	  template_start_ptr = &vt.data[0] + offset;
	  column_start_ptr = &data[0] + VT_SHIFT_MATCH;
	  row_size = TEMPLATE_X_SIZE;
	  column_end_ptr = &data[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
	  sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

	  for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
	  {
		for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
		{
		  cdiff += abs(*column_ptr - *template_ptr);
		}

		// fast breaks
		if (cdiff > mindiff)
		  break;
	  }

	  if (cdiff < mindiff)
	  {
		mindiff = cdiff;
		min_template = vt.id;
		min_offset = 0;
	  }
	}

	}

	vt_relative_rad = 0;
	vt_err = mindiff / (double)(TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);
	vt_match_id = min_template;

	vt_error = vt_err;

  }
}

}
