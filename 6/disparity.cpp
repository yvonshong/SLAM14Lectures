//
// Created by song on 1/26/18.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

// this program shows how to use optical flow

string left_file = "./left.png";  // first image
string right_file = "./right.png";  // second image
string disparity_file = "./disparity.png" ;

double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
double baseline = 0.573;

// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] left_img the first image
 * @param [in] right_img the second image
 * @param [in] kp1 keypoints in left_img
 * @param [in|out] kp2 keypoints in right_img, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &left_img,
        const Mat &right_img,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] left_img the first pyramid
 * @param [in] right_img the second pyramid
 * @param [in] kp1 keypoints in left_img
 * @param [out] kp2 keypoints in right_img
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &left_img,
        const Mat &right_img,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat left_img = imread(left_file, 0);
    Mat right_img = imread(right_file, 0);
    Mat disparity_img = imread(disparity_file, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(left_img, kp1);


    // then test multi-level LK
    vector<KeyPoint> kp2;
    vector<bool> success;
    //TODO
    OpticalFlowMultiLevel(left_img, right_img, kp1, kp2, success);

    Mat left_img_show = left_img;
    cv::cvtColor(left_img, left_img_show, CV_GRAY2BGR);
    for (int i = 0; i < kp1.size(); i++) {

        cv::circle(left_img_show, kp1[i].pt, 2, cv::Scalar(0, 250, 0), 2);


    }
    // plot the differences of those functions
    Mat right_img_show;
    cv::cvtColor(right_img, right_img_show, CV_GRAY2BGR);
    for (int i = 0; i < kp2.size(); i++) {
        if (success[i]) {
            if(kp1[i].pt == kp2[i].pt) cout<<"==="<<endl;
            cv::circle(right_img_show, kp2[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(right_img_show, kp1[i].pt, kp2[i].pt, cv::Scalar(0, 250, 0));
        }
    }



    Mat match_img;
    vector<cv::DMatch> matches;
    for (int k =0; k <kp1.size();k++)
    {
        if(success[k])
        {
            cv::DMatch m;
            m.queryIdx=k;
            m.trainIdx=k;
            double disparity_ref = double(disparity_img.at<uchar>(kp1[k].pt.y,kp1[k].pt.x ));
            m.distance= float(abs(fx*baseline/(kp2[k].pt.x - kp1[k].pt.x) - fx*baseline /disparity_ref) );
            matches.push_back(m);

        }
    }


    // plot the matches
    cv::imshow("right_img_show", right_img_show);
    cv::imwrite("right_img_show.png", right_img_show);

    cv::drawMatches(left_img, kp1, right_img, kp2, matches, match_img);
    cv::imshow("matches", match_img);
    cv::imwrite("matches.png", match_img);

    cv::waitKey(0);

    double error = 0;
    int count = 0 ;
    for(auto m  : matches)
    {
        error+= m.distance;
        count ++;
    }
    cout<<"error: "<< error<<endl;
    cout<<"average error: "<<error/count<<endl;

    return 0;
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 10;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];

        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)

                    float x1 = kp.pt.x + x;
                    float y1 = kp.pt.y + y;
                    double error = GetPixelValue(img1,x1,y1)-GetPixelValue(img2,x1+dx,y1+dy);
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        J[0] = (GetPixelValue(img2,x1+dx+1,y1+dy) - GetPixelValue(img2,x1+dx-1,y1+dy))/2;
                        J[1] = (GetPixelValue(img2,x1+dx,y1+dy+1) - GetPixelValue(img2,x1+dx,y1+dy-1))/2;
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J[0] = (GetPixelValue(img1,x1+1,y1) - GetPixelValue(img1,x1-1,y1))/2;
                        J[1] = (GetPixelValue(img1,x1,y1+1) - GetPixelValue(img1,x1,y1-1))/2;
                    }

                    // compute H, b and set cost;
                    // TODO END YOUR CODE HERE
                    H += J * J.transpose() ;
                    b += J.transpose()*error;
                    cost += error * error;
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                //cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++) {
        Mat uplevel1;
        Mat uplevel2;
        cv::resize(img1, uplevel1, cv::Size(), scales[i], scales[i]);
        cv::resize(img2, uplevel2, cv::Size(), scales[i], scales[i]);
        pyr1.push_back(uplevel1.clone());
        pyr2.push_back(uplevel2.clone());
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE
    vector<KeyPoint> kp2_scale;
    for(int i = pyramids - 1; i >= 0; --i)
    {
        vector<KeyPoint> kp1_scale = kp1;
        for(int j = 0; j != kp1_scale.size(); ++j)
        {
            kp1_scale[j].pt.x *= scales[i];
            kp1_scale[j].pt.y *= scales[i];
        }

        for(int j = 0; j != kp2_scale.size(); ++j)
        {
            kp2_scale[j].pt.x *= scales[i];
            kp2_scale[j].pt.y *= scales[i];
        }

        OpticalFlowSingleLevel(pyr1[i], pyr2[i], kp1_scale, kp2_scale, success, inverse);

        for(int j = 0; j != kp2_scale.size(); ++j)
        {
            kp2_scale[j].pt.x /= scales[i];
            kp2_scale[j].pt.y /= scales[i];
        }
    }

    kp2 = kp2_scale;
    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}