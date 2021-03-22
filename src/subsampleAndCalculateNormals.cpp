#include"subsampleAndCalculateNormals.h"

PointCloud<PointNormal>::Ptr
		subsampleAndCalculateNormals( const PointCloud<pcl::PointXYZ>::Ptr &cloud ,const Eigen::Vector4f &subsampling_leaf_size_,const float &normal_r)
{
	//===计算原始点云的法向量==============================================
	PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
	//多线程加速法向量计算，需要保证线程数目为正
	normal_estimation_filter.setNumberOfThreads(60);
	normal_estimation_filter.setInputCloud(cloud);
	//设置未降采样的点云作为搜索面
	pcl::search::KdTree<pcl::PointXYZ>::Ptr cloud_search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation_filter.setSearchMethod(cloud_search_tree);
	normal_estimation_filter.setRadiusSearch(normal_r);
	//normal_estimation_filter.setKSearch(15);
	normal_estimation_filter.compute(*cloud_normals);

	//===对输入点云进行体素降采样==============================================
	PointCloud<PointXYZ>::Ptr cloud_subsampled(new PointCloud<PointXYZ>());
	pcl::VoxelGrid<PointXYZ> subsampling_filter;
	subsampling_filter.setSaveLeafLayout(true);
	subsampling_filter.setInputCloud(cloud);
	subsampling_filter.setLeafSize(subsampling_leaf_size_);
	subsampling_filter.filter(*cloud_subsampled);

	//===计算降采样后的点云的法向量==============================================
	PointCloud<Normal>::Ptr cloud_subsampled_normals(new PointCloud<Normal>());
	//多线程加速法向量计算，需要保证线程数目为正
	normal_estimation_filter.setNumberOfThreads(60);
	normal_estimation_filter.setInputCloud(cloud_subsampled);
	//设置原始点云作为搜索面
	normal_estimation_filter.setSearchSurface(cloud);
	//search::KdTree<PointXYZ>::Ptr search_tree(new search::KdTree<PointXYZ>);
	//normal_estimation_filter.setSearchMethod(search_tree);
	normal_estimation_filter.setRadiusSearch(normal_r);
	//normal_estimation_filter.setKSearch(15);
	normal_estimation_filter.compute(*cloud_subsampled_normals);
	PCL_INFO("降采样后的点数 %d \n", cloud_subsampled ->points.size());

	//===根据HS-PPF,判断是否有一些网格内的点法相量差异巨大===========================
	vector<vector<uint32_t>>  added_points_index;
	added_points_index.resize(cloud_subsampled->points.size());
	//判断原始点云中的每个点是否与降采样之后的中心点，反正每个点都会检查
	for(uint32_t cloud_i=0;cloud_i<cloud->points.size();++cloud_i)
	{
		//获取该原始点对应降采样后的中心质点的index（数量不会太多，所以用unsigned int也行，此外getCentroidIndexAt返回的可能是-1）
		unsigned int Centroid_index = subsampling_filter.getCentroidIndexAt(
    		subsampling_filter.getGridCoordinates(cloud->points[cloud_i].x,cloud->points[cloud_i].y,cloud->points[cloud_i].z));
			
		//设定角度差阈值为30度
		const float threshold = M_PI*0.166 ;
		//如果场景点能够找到合理的中心质点坐标，就判断该场景点的法向量与中心质点的法向量是否相差巨大
		if (Centroid_index!=-1)
		{
			//获取原场景点和中心质点对应的法向量
			Eigen::Vector3f  cloud_i_normal, Centroid_normal;//
			//获取原始点云上点cloud_i的法向量
			cloud_i_normal = cloud_normals->points[cloud_i].getNormalVector3fMap();
			//获取对应的质心点的向量的法向量
			Centroid_normal = cloud_subsampled_normals->points[Centroid_index].getNormalVector3fMap();

			//计算两个向量之前的夹角
			float radian_angle = atan2(cloud_i_normal.cross(Centroid_normal).norm(), cloud_i_normal.dot(Centroid_normal));
			//如果两者的角度差超过30度，就检查是否已经有过近的点存在了。相当于聚一下类。就把当前点及其法向量添加在对应的点云下面
			if(radian_angle>= threshold)
			{
				//当前模型点与 容器中的点的距离角度
				float distance,angle ;
				Eigen::Vector3f added_point_i_normal_;
				Eigen::Vector3f dis_vector;
				//如果当前中心质点所在的方格中 还没有符合条件的值  
				//if(index ==90)
				//    index =90;
				if(added_points_index[Centroid_index].size()==0)
				{
					//就把当前的点标号存起来
					added_points_index[Centroid_index].push_back(cloud_i);
				}
				else //如果里面已经有符合条件的点了，开始做个类似聚类的筛选
				{
					uint16_t  size_old  =added_points_index[Centroid_index].size();
					for(uint16_t i = 0; i<size_old;++i )
					{
			
						//获取相同降采样方格中，之前保留下的点的法向量
						added_point_i_normal_= cloud_normals->points[added_points_index[Centroid_index][i]].getNormalVector3fMap();
						angle =  atan2(cloud_i_normal.cross(added_point_i_normal_).norm(), cloud_i_normal.dot( added_point_i_normal_));
						//计算距离差值
						dis_vector = cloud->points[added_points_index[Centroid_index][i]].getVector3fMap() - cloud->points[cloud_i].getVector3fMap();
						distance = dis_vector.norm();
				
						//如果角度差大于阈值30度，距离大于1/4的采样距离   就把该点存起来
						if((angle>= threshold)&&(distance > subsampling_leaf_size_[0]/2 ))
						{
							added_points_index[Centroid_index].push_back(cloud_i);
							break;
						}
					}
				}
			}
		}
	}

	uint16_t points_num=cloud_subsampled->points.size();
	//把二次筛选出来应该保留下来的点云点，添加到降采样点云中
	for(uint16_t i = 0; i<added_points_index.size();++i)
		for(uint16_t j =0; j<added_points_index[i].size();++j )
		{
			cloud_subsampled->points.push_back(cloud->points[added_points_index[i][j]]);
			cloud_subsampled_normals->points.push_back(cloud_normals->points[added_points_index[i][j]]);
			//cloud_test->points.push_back(cloud_->points[added_points_index[i][j]]);
			//cloud_test->width++;
			//cloud_subsampled->width++;
			//cloud_subsampled_normals->width++;
			//PCL_INFO("添加的点的索引 %d \n", added_points_index[i][j]);
		}
	PCL_INFO("法向量差异计算添加的点数 %d \n", cloud_subsampled ->points.size()-points_num);

	//===把法向量和点云接到一起，并存起来=========================================
	PointCloud<PointNormal>::Ptr cloud_subsampled_with_normals(new PointCloud<PointNormal>());
	concatenateFields(*cloud_subsampled, *cloud_subsampled_normals, *cloud_subsampled_with_normals);
	PCL_INFO("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size(), cloud_subsampled->points.size());

	return cloud_subsampled_with_normals;
}



PointCloud<PointNormal>::Ptr
	subsamlpleModel(const PointCloud<PointNormal>::Ptr &modelNormal,
		const Eigen::Vector4f &subsampling_leaf_size_)
{
	//===对输入模型点法云进行体素降采样==============================================
	PointCloud<PointNormal>::Ptr modelNormal_subsampled(new PointCloud<PointNormal>());
	pcl::VoxelGrid<PointNormal> subsampling_filter;
	subsampling_filter.setSaveLeafLayout(true);
	subsampling_filter.setInputCloud(modelNormal);
	subsampling_filter.setLeafSize(subsampling_leaf_size_);
	subsampling_filter.filter(*modelNormal_subsampled);
	PCL_INFO("降采样后的点数 %d \n", modelNormal_subsampled ->points.size());

	//===根据HS-PPF,判断是否有一些网格内的点法相量差异巨大===========================
	vector<vector<uint32_t>>  added_points_index;
	added_points_index.resize(modelNormal_subsampled->points.size());
	//判断原始点云中的每个点是否与降采样之后的中心点，反正每个点都会检查
	for(uint32_t cloud_i=0;cloud_i<modelNormal->points.size();++cloud_i)
	{
		//获取该原始点对应降采样后的中心质点的index（数量不会太多，所以用unsigned int也行，此外getCentroidIndexAt返回的可能是-1）
		unsigned int Centroid_index = subsampling_filter.getCentroidIndexAt(
    		subsampling_filter.getGridCoordinates(modelNormal->points[cloud_i].x,modelNormal->points[cloud_i].y,modelNormal->points[cloud_i].z));
			
		//设定角度差阈值为30度
		const float threshold = M_PI*0.166 ;
		//如果场景点能够找到合理的中心质点坐标，就判断该场景点的法向量与中心质点的法向量是否相差巨大
		if (Centroid_index!=-1)
		{
			//获取原场景点和中心质点对应的法向量
			Eigen::Vector3f  cloud_i_normal, Centroid_normal;//
			//获取原始点云上点cloud_i的法向量
			cloud_i_normal = modelNormal->points[cloud_i].getNormalVector3fMap();
			//获取对应的质心点的向量的法向量
			Centroid_normal = modelNormal_subsampled->points[Centroid_index].getNormalVector3fMap();

			//计算两个向量之前的夹角
			float radian_angle = atan2(cloud_i_normal.cross(Centroid_normal).norm(), cloud_i_normal.dot(Centroid_normal));
			//如果两者的角度差超过30度，就检查是否已经有过近的点存在了。相当于聚一下类。就把当前点及其法向量添加在对应的点云下面
			if(radian_angle>= threshold)
			{
				//当前模型点与 容器中的点的距离角度
				float distance,angle ;
				Eigen::Vector3f added_point_i_normal_;
				Eigen::Vector3f dis_vector;
				//如果当前中心质点所在的方格中 还没有符合条件的值  
				//if(index ==90)
				//    index =90;
				if(added_points_index[Centroid_index].size()==0)
				{
					//就把当前的点标号存起来
					added_points_index[Centroid_index].push_back(cloud_i);
				}
				else //如果里面已经有符合条件的点了，开始做个类似聚类的筛选
				{
					uint16_t  size_old  =added_points_index[Centroid_index].size();
					for(uint16_t i = 0; i<size_old;++i )
					{
						//获取相同降采样方格中，之前保留下的点的法向量
						added_point_i_normal_= modelNormal->points[added_points_index[Centroid_index][i]].getNormalVector3fMap();
						angle =  atan2(cloud_i_normal.cross(added_point_i_normal_).norm(), cloud_i_normal.dot( added_point_i_normal_));
						//计算距离差值
						dis_vector = modelNormal->points[added_points_index[Centroid_index][i]].getVector3fMap() - modelNormal->points[cloud_i].getVector3fMap();
						distance = dis_vector.norm();
				
						//如果角度差大于阈值30度，距离大于1/4的采样距离   就把该点存起来
						if((angle>= threshold)&&(distance > subsampling_leaf_size_[0]/2 ))
						{
							added_points_index[Centroid_index].push_back(cloud_i);
							break;
						}
					}
				}
			}
		}
	}

	uint16_t points_num=modelNormal_subsampled->points.size();
	//把二次筛选出来应该保留下来的点云点，添加到降采样点云中
	for(uint16_t i = 0; i<added_points_index.size();++i)
		for(uint16_t j =0; j<added_points_index[i].size();++j )
		{
            //这个，在降采样后的点云以及法向量中，是否可以
			modelNormal_subsampled->points.push_back(modelNormal->points[added_points_index[i][j]]);
		}
	PCL_INFO("法向量差异计算添加的点数 %d \n", modelNormal_subsampled ->points.size()-points_num);
	PCL_INFO("Cloud dimensions before / after subsampling: %u / %u\n", modelNormal->points.size(), modelNormal_subsampled->points.size());
	return modelNormal_subsampled;
}
