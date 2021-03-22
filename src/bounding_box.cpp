#include "bounding_box.h"

Bounding_box::Bounding_box(pcl::PointCloud<PointType>::Ptr &cloud_)
{
    cloud = cloud_->makeShared();
	compute(cloud);
}

inline float Bounding_box::get_MaxModelDiameter()
{
	float temp = 0;
	temp = whd1[0];
	if (whd1[1] > whd1[0])
		temp = whd1[1];
	if (whd1[2] > temp)
		temp = whd1[2];
	printf("MaxModelDiameter: %f \n", temp);
	return temp;
}

inline float Bounding_box::get_MinModelDiameter()
{
	float temp = 0;
	temp = whd1[0];
	if (whd1[1] < whd1[0])
		temp = whd1[1];
	if (whd1[2] < temp)
		temp = whd1[2];
	printf("MinModelDiameter: %f \n", temp);
	return temp;
}

Eigen::Vector3f Bounding_box::get_PcaCentroid()
{
	Eigen::Vector3f Centroid;
	Centroid[0] = pcaCentroid[0];
	Centroid[1] = pcaCentroid[1];
	Centroid[2] = pcaCentroid[2];
	return Centroid;
}

void Bounding_box::compute(pcl::PointCloud<PointType>::Ptr &cloud_)
{
	
	//计算质心，并且存储
    pcl::compute3DCentroid(*cloud_, pcaCentroid);

    //协方差
	Eigen::Matrix3f covariance;  
	pcl::computeCovarianceMatrixNormalized(*cloud_, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
			
    //校正主方向间垂直
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	//std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	//std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	//std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
			
	//以质心为中心，以特征向量为轴，去计算坐标系，并后来将物体坐标系旋转到参考坐标系中。（就是世界坐标系）
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	//std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	//std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	//把原始的点云cloud_旋转之后放到了transformedCloud中
	pcl::transformPointCloud(*cloud_, *transformedCloud, tm);

	PointType min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

	//std::cout << "型心c1(3x1):\n" << c1 << std::endl;
	//仿射变换
	Eigen::Affine3f tm_inv_aff(tm_inv);
	//pcl::transformPoint(c1, c, tm_inv_aff);

	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	//whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

	//std::cout << "width1=" << whd1(0) << std::endl;//单位米
	//std::cout << "heght1=" << whd1(1) << std::endl;
	//std::cout << "depth1=" << whd1(2) << std::endl;
	//std::cout << "scale1=" << sc1 << std::endl;

}



