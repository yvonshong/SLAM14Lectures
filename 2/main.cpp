
// Created by song on 12/15/17.
//

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

int main( int argc, char** argv )
{
    Eigen::Quaterniond q_1(0.35,0.2,0.3,0.1);
    Eigen::Quaterniond q_2(-0.5,0.4,-0.1,0.2);
    Eigen::Vector3d p_1(0.5,0,0.2);

    Eigen::Vector3d p_2;
    Eigen::Vector3d t_1(0.3,0.1,0.1);
    Eigen::Vector3d t_2(-0.1,0.5,0.3);
    p_2 = q_2*(t_1+q_1.conjugate()*p_1 -t_2 );
    cout<<p_2<<endl;
    return 0;
}
