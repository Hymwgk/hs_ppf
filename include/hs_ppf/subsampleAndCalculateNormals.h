/**
 * \brief：主要是降采样以及点云法向量计算函数
 * 输入主要有两种1. 场景点云 2.带有法向量的模型点云；
 * 主要实现的是：1.对场景点云进行降采样以及法向量计算；2.对模型点法云进行降采样
 * 由于模型法向量使用外部mesh计算，因此不进行法向量计算，仅仅进行点云降采样
 */

#ifndef   SUBSAMPLEANDCALCULATENORMALS_
#define  SUBSAMPLEANDCALCULATENORMALS_

#include <pcl/point_cloud.h>   
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

using namespace pcl;
using namespace std;

/**
*\brief：对场景点云进行降采样并计算法向量；点云体素降采样->计算点云表面法向量->重新保留一些差异巨大点->返回处理后的点云结果
*\param[in]:cloud 待处理的场景点云
*\param[in]:subsampling_leaf_size_  体素降采样的尺寸，一般和离线训练模型的尺寸有关
*\param[in]:normal_r 法向量计算半径
*\return cloud_subsampled_with_normals 返回带有法向量的模型点云
*/
PointCloud<PointNormal>::Ptr
	subsampleAndCalculateNormals( const PointCloud<pcl::PointXYZ>::Ptr &cloud ,
		const Eigen::Vector4f &subsampling_leaf_size_,const float &normal_r);


/**
*\brief：对模型的点法云进行降采样；点云体素降采样->重新保留一些差异巨大点->返回处理后的点云结果
*\param[in]:modelNormal 模型点法云
*\param[in]:subsampling_leaf_size_  体素降采样的尺寸，一般和离线训练模型的尺寸有关
*\return modelNormalSubsampled 返回降采样后的模型点法云
*/
PointCloud<PointNormal>::Ptr
	subsamlpleModel(const PointCloud<PointNormal>::Ptr &modelNormal,
		const Eigen::Vector4f &subsampling_leaf_size_);

#endif
