/**
 * \brief 主要是把场景点对或模型点对旋转到中间世界坐标系的过程抽离出来，完成了计算alpha_m与alpha_s的任务
 */

#ifndef POINTPAIRTO_WORLD
#define POINTPAIRTO_WORLD

#include <Eigen/Core>
#include <Eigen/Eigen>

/**
 * \brief 将模型或者场景的第一点（参考点），平移到世界坐标系原点，并将其法向量旋转到与x轴平行； 返回此时的变换矩阵
 * \param [in] reference_point  该参考点的坐标
 *  \param [in] reference_normal  该参考点的表面法向量
 * \return transform_msg变换矩阵
 */
inline Eigen::Affine3f pointpairToWorld(const Eigen::Vector3f &reference_point, const Eigen::Vector3f &reference_normal)
{
	//===计算当前点对与中间坐标系之间的alpha_m值================================================
	/**
	 * 首先将旋转方向统一为，m_r法向量转向x轴，且能保证转角小于180的方向
	 * 利用右手定则可以很轻易发现，上面一条规定，就决定了转轴的方向就是  m_r 法向量和 x轴正向的叉乘向量方向
	*/
	//计算m_r法向量与x轴正向间的夹角0~pi
	float rotation_angle = acosf(reference_normal.dot(Eigen::Vector3f::UnitX()) /reference_normal.norm());
	//看m_r法向量是否与x轴平行，如果平行就把标志位parallel_to_x 置1
	bool parallel_to_x = (reference_normal.y() == 0.0f && reference_normal.z() == 0.0f);
	/**
	 * 如果m_r法向量平行于x轴，就把旋转轴设置为与Y 轴平行的单位向量
	 * 如果参考点法向量不平行于x轴，设置旋转轴为m_r法向量叉乘x轴的单位法向量
	*/
	Eigen::Vector3f rotation_axis = (parallel_to_x) ? (Eigen::Vector3f::UnitY()) : 
		(reference_normal.cross(Eigen::Vector3f::UnitX()).normalized());
	
	//创建旋转分量对象，绕着rotation_axis旋转rotation_angle度，相当于创造旋转矩阵：代表把模型法向量旋转至与世界x轴平行
	Eigen::AngleAxisf rotation_mg(rotation_angle, rotation_axis);
	/**
	 * 使用平移和旋转分量，综合构建一个变换矩阵，顺序是先平移m_r点到世界坐标系原点，再使m_r法向量绕轴旋转到与x轴正向平行
	 * 由于所做变换都是相对于固定世界坐标系，因此需要向左乘
	 * 综合上面两条，就是平移矩阵在右边，旋转矩阵在左边
	*/ 
	Eigen::Affine3f transform_msg(rotation_mg * Eigen::Translation3f((-1) * reference_point));
 
    return transform_msg;
}


/**
 * \brief 计算alpha_m或者alpha_s，并返回
 * \param [in] secondPoint 第二点的三个坐标值
 * \param [in] transform_msg 需要依照的旋转矩阵
 * \return 角度alpha_m  或者  alpha_s
 */
inline float computeAlpha_M_S(const Eigen::Vector3f	&secondPoint, const Eigen::Affine3f &transform_msg)
{
	//把模型第二点按照旋转矩阵，旋转到与世界坐标系中，也是左乘，具体看论文
	Eigen::Vector3f point_transformed = transform_msg * secondPoint;
					
	//找到以x轴为旋转轴，从Y轴到(平移到原点之后的) mr_mi在Y-Z平面投影线的夹角,即alpha_m，这时候，返回的alpha_m值处于[-pi，pi]范围
	float angle = atan2f(point_transformed(2), point_transformed(1));
	//将alpha_m转换到[0,2pi]范围内
	if(angle<0)
		angle+=2 * M_PI;	
    
    return angle;
}

/**
 * \brief 通过计算出的alpha，来结算出模型点对转换到场景点对的变换矩阵
 * \param [in]  transform_mg  场景点转换矩阵
 * \param [in]  transform_sg 模型点转换矩阵
 * \param [in]  alpha 角度
 * \return  transform 模型点对转换到场景点对的变换矩阵
 */
inline Eigen::Affine3f ModelToScene(const Eigen::Affine3f  &transform_mg,const Eigen::Affine3f &transform_sg, const  float &alpha)
{
	//实际就是实现inverse(Ts->g)*Rx(alpha)*Tm->g *m_i的这个公式
	//实际上就是代表了三个步骤:
	//第一，把模型点对转到世界坐标系，与x轴对齐；
	//第二，把模型点对沿着x轴旋转alpha角，使与场景点对重合；
	//第三，把模型点对对转向场景
	//因为是相对于世界坐标系，所以，都是左乘顺序
	Eigen::Affine3f transform =
			transform_sg.inverse() *Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()) *transform_mg;

	return transform;
}



#endif
