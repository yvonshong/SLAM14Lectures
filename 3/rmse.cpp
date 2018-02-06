#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <Eigen/StdVector>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string estimated_file = "./estimated.txt";
string groundtruth_file = "./groundtruth.txt";


void DrawTrajectory(vector<Sophus::SE3> estimated_poses,vector<Sophus::SE3> groundtruth_poses) {
    if (estimated_poses.empty() || groundtruth_poses.empty()) {
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
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);


        for (size_t i = 0; i < estimated_poses.size() - 1; i++) {
            glColor3f(1 - (float) i / estimated_poses.size(), 0.0f, (float) i / estimated_poses.size());
            glBegin(GL_LINES);
            auto p1 = estimated_poses[i], p2 = estimated_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < groundtruth_poses.size() - 1; i++) {
            glColor3f(1 - (float) i / groundtruth_poses.size(), 0.0f, (float) i / groundtruth_poses.size());
            glBegin(GL_LINES);
            auto p1 = groundtruth_poses[i], p2 = groundtruth_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }



        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

vector<Sophus::SE3> readPose(string filename)
{
    vector<Sophus::SE3> poses;
    /// implement pose reading code
    // start your code here (5~10 lines)

    ifstream fin(filename);
    for ( int i=0; i<612; i++ )
    {
        double data[8] = {0};
        for ( auto& d:data )
            fin>>d;
        Eigen::Quaterniond q( data[7], data[4], data[5], data[6] );
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Sophus::SE3 SE3_qt(q,t);
        poses.push_back(SE3_qt);
    }

    return poses;
    // end your code here

}

double RMSE(vector<Sophus::SE3> estimated_poses,vector<Sophus::SE3> groundtruth_poses)
{
    double sum = 0;
    for(int i=0;i<estimated_poses.size();i++)
    {
        double error = 0;
        double e = sqrt(  (groundtruth_poses.at(i).inverse() * estimated_poses.at(i)).log().transpose() *   (groundtruth_poses.at(i).inverse() * estimated_poses.at(i)).log()   );


        error = pow(abs(e),2);
        sum+= error;
    }
    double rmse = pow(sum/estimated_poses.size(),0.5);
    return rmse;
}


int main(int argc, char **argv) {

    vector<Sophus::SE3> estimated_poses;
    vector<Sophus::SE3> groundtruth_poses;

    estimated_poses = readPose(estimated_file);
    groundtruth_poses = readPose(groundtruth_file);

    
    std::cout<<RMSE(estimated_poses,groundtruth_poses)<<endl;;
    DrawTrajectory(estimated_poses,groundtruth_poses);
    return 0;
}

