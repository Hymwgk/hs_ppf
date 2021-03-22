/**
 * brief:主要是一个类对象，将传入的目标点云利用PCA主成分分析法，来构造一个长方形包围盒
 * 这个的中心以及三个维度的尺寸都是可知的 
 * 
 */ 

#ifndef BOUNDING_BOX_
#define BOUNDING_BOX_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

typedef pcl::PointXYZ PointType;

class Bounding_box
{
    public:
        pcl::PointCloud<PointType>::Ptr cloud;

        //构造函数，导入目标点云的指针
        explicit Bounding_box(pcl::PointCloud<PointType>::Ptr &cloud_);
   
        //读取并返回点云包围盒xyz三个维度中最大的尺寸
        inline float get_MaxModelDiameter();

        //读取并返回点云包围盒xyz三个维度中最小的尺寸
        inline float get_MinModelDiameter();

        //读取点云的质心 xyz坐标
        Eigen::Vector3f get_PcaCentroid();

    private:
        //点云包围盒的xyz三维尺寸
        Eigen::Vector3f whd1;
        //点云的质心
		Eigen::Vector4f pcaCentroid;      

        /**
         * brief:实际计算发生的地方，计算了输入点云的质心、以及包围盒xyz三个维度的尺寸
         * 利用PCA主元分析法获得点云的三个主方向，获取质心，计算协方差，
         * 获得协方差矩阵，求取协方差矩阵的特征值和特长向量，特征向量即为主方向。
         * input:目标点云的智能指针
         * output: 目标点云的质心、以及包围盒xyz三个维度的尺寸
         */
        void compute(pcl::PointCloud<PointType>::Ptr &cloud_);

};

#endif