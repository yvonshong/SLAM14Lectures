//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv)
{

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d
    // START YOUR CODE HERE
    ifstream fin(p3d_file);
    for (int i = 0; i < 76; i++)
    {
        double data[3] = {0};
        for (auto &d : data)
            fin >> d;
        Eigen::Vector3d 3d(data[0], data[1], data[2]);
        p3d.push_back(3d);
    }

    ifstream fin(p2d_file);
    for (int i = 0; i < 76; i++)
    {
        double data[2] = {0};
        for (auto &d : data)
            fin >> d;
        Eigen::Vector3d 2d(data[0], data[1]);
        p2d.push_back(2d);
    }

    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;
    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++)
    {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        Mat R;

        cost = 0;
        // compute cost

        for (int i = 0; i < nPoints; i++)
        {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE

            u = p2d[i][0];
            v = p2d[i][1];

            x = p3d[i][0];
            y = p3d[i][1];
            z = p3d[i][2];
            P_i = Vector4d(x, y, z, 1);
            P = Vector3d(K * T_esti.matrix() * P_i[0], K * T_esti.matrix() * P_i[1], K * T_esti.matrix() * P_i[2]);

            cost += sqrt(pow(fx * P[0] / P[2] + cx, 2) + pow(fy * P[1] / P[2] + cy, 2))

                // END YOUR CODE HERE

                // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            J << fx / P[2], 0, -fx * P[0] / (P[2] * P[2]), -fx * P[0] * P[1] / (P[2] * P[2]), fx + fx * P[0] * P[0] / (P[2] * P[2]), -fx * P[1] / P[2],
                0, fy / P[2], -fy * P[1] / (P[2] * P[2]), -fy - fy * P[1] * P[1] / (P[2] * P[2]), fy * P[0] * P[1] / (P[2] * P[2]), fy * P[0] / P[2];
            J = -J;
            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)
        {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti=Sophus::SE3::exp(dx)*T_esti;
        // END YOUR CODE HERE

        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n"
         << T_esti.matrix() << endl;
    return 0;
}
