#include "compute2points_ppf.h"


inline float angleBetween(const Eigen::Vector3f& a,	const Eigen::Vector3f& b) 
{
    //不需要输入的两个法向量进行单位化
	//const auto a_unit = a.normalized();
	//const auto b_unit = b.normalized();
    
    //计算axb
	const auto c = a.cross(b);
    //计算(axb)/(a*b)=sin/cos，并求反tan，atan2f函数的返回值范围是[-pi,pi]，但是这里肯定返回的值是[0,pi]，因为两个向量之间的一个角theta必然小于180，
	//而360-theta 又肯定小于-180，所以只能输出小于180的角theta
	return atan2f(c.norm(), a.dot(b));
}

bool computeContinuousPPF(const Eigen::Vector3f &p1, const Eigen::Vector3f &n1,const Eigen::Vector3f &p2, const Eigen::Vector3f &n2,
			float &f1, float &f2, float &f3, float &f4)
{
	Eigen::Vector3f d = p2 - p1;
	Eigen::Vector3f e = p1 - p2;

	//计算两点长度 
	f4 = d.norm();

	
	f1 = angleBetween(d, n1);
	f2 = angleBetween(e, n2);
	f3 = angleBetween(n1, n2);

	if (f4)
		return true;
	else
		return false;
}