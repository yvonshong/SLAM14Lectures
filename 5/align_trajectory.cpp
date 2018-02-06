#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/StdVector>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue

/*******************************************************************************************/
void DrawTrajectoryc(vector<Sophus::SE3> poses1, vector<Sophus::SE3> poses2)
{
    if (poses1.empty() || poses2.empty())
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++)
        {
            glColor3f(1 - (float)i / poses1.size(), 0.0f, (float)i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < poses2.size() - 1; i++)
        {
            glColor3f(1 - (float)i / poses2.size(), 0.0f, (float)i / poses2.size());
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
    }
}

Sophus::SE3 pose_estimation_3d3d(vector<Sophus::SE3> poses1, vector<Sophus::SE3> poses2)
{

    Eigen::Vector3d center1, center2;
    int Number = poses1.size();
    for (int i = 0; i < N; i++)
    {
        center1 += poses1[i].translation();
        center2 += poses2[i].translation();
    }
    center1 = center1 / Number;
    center2 = center2 / Number;

    vector<Eigen::Vector3d> p1(Number), p2(Number);
    for (int i = 0; i < N; i++)
    {
        p1[i] = poses1[i] - center1;
        p2[i] = poses2[i] - center2;
    }

    Eigen::Matrix3d error = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        error += p1[i] * p2[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(error, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    Eigen::Matrix3d R = U * (V.transpose());
    Eigen::Vector3d t = p1 - R * p2;
    Sophus::SE3 T(R,t);
    return T;
}

int main(int argc, char **argv)
{

    vector<Sophus::SE3> poses1;
    vector<Sophus::SE3> poses2;

    /// implement pose reading code
    // start your code here (5~10 lines)

    ifstream fin(trajectory_file);
    for (int i = 0; i < 620; i++)
    {
        double data[8] = {0};
        for (auto &d : data)
            fin >> d;
        Eigen::Vector3d track1_t(data[1], data[2], data[3]);
        Eigen::Quaterniond track1_q(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d track2_t(data[9], data[10], data[11]);
        Eigen::Quaterniond track2_q(data[12], data[13], data[14], data[15]);
        Sophus::SE3 SE3_1_qt(track1_q, track1_t);
        Sophus::SE3 SE3_2_qt(track2_q, track2_t);

        poses1.push_back(SE3_1_qt);
        poses2.push_back(SE3_2_qt);
    }
    // end your code here

    Sophus::SE3 T = pose_estimation_3d3d(vector<Sophus::SE3> poses1, vector<Sophus::SE3> poses2);
    for (int k = 0 ;k <poses2.size();k++)
    {
        poses2[k]  = T*poses2[k];
    }
    
    // draw trajectory in pangolin
    DrawTrajectory(poses1, poses2);

    return 0;
}