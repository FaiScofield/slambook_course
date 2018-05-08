#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <chrono>

using namespace std;

// Camera intrinsics
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;    // 内参
double baseline = 0.573;    // 基线

// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
boost::format fmt_others("../%06d.png");    // other files

// useful typedefs
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


// TODO implement this function
/**
 * pose estimation using direct method
 * @param [in] img1
 * @param [in] img2
 * @param px_ref
 * @param depth_ref
 * @param [out] T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param [in] img1 the first input image
 * @param [in] img2 the second input image
 * @param [in] px_ref pixel coordinates of points
 * @param [in] depth_ref depth of points
 * @param [out] T21 transform from img1 to img2
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// bilinear interpolation
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

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // 随机返回在两参数范围内的值
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3 T_cur_ref;

    for (int i = 1; i < 6; i++) {  // 1~10
        cv::Mat img = cv::imread((fmt_others %i).str(), 0);
        if (img.empty()) {
            cout << "file " << (fmt_others %i).str() << " does not exist!" << endl;
            return -1;
        }
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
//        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "Image " << i << " processing time cost = "<< 1000000*time_used.count() << " s. T_cur_ref is \n" << T_cur_ref.matrix() << endl;
    }
}

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;
    vector<int> goodIndex;
    vector<cv::DMatch> matches;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();
        goodIndex.clear();
        matches.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {
            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            float u = 0, v = 0;                         // pixel frame of img2
            float u1 = px_ref[i][0], v1 = px_ref[i][1]; // pixel frame of img1

            Eigen::Vector4d Pw = Eigen::Vector4d::Zero();   // world frame
            Pw[3] = 1;
            Pw[2] = depth_ref[i];
            Pw[1] = (v1 - cy) * depth_ref[i] / fy;
            Pw[0] = (u1 - cx) * depth_ref[i] / fx;

            Eigen::Vector4d Pc_cur = T21.matrix() * Pw; // camera frame of img2
            double cur_x = Pc_cur[0];
            double cur_y = Pc_cur[1];
            double cur_z = Pc_cur[2];
            u = fx * cur_x / cur_z + cx;    // pixel frame of img2
            v = fy * cur_y / cur_z + cy;

            // check if points out of boarder
            if ( u1-half_patch_size  < 0 || u1+half_patch_size  >  img2.cols || // check points in img1
                 v1-half_patch_size  < 0 || v1+half_patch_size  >  img2.rows ||
                 u-half_patch_size-1 < 0 || u+half_patch_size+1 >= img2.cols || // check points in img2
                 v-half_patch_size-1 < 0 || v+half_patch_size+1 >= img2.rows) { // include the conditions of the calculation of gradients
                continue;
            }

            nGood++;
            goodProjection.push_back(Eigen::Vector2d(u, v));
            goodIndex.push_back(i);

            Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
            J_pixel_xi(0, 0) = fx / cur_z;
            J_pixel_xi(0, 1) = 0;
            J_pixel_xi(0, 2) = -fx*cur_x / (cur_z*cur_z);
            J_pixel_xi(0, 3) = -fx*cur_x*cur_y / (cur_z*cur_z);
            J_pixel_xi(0, 4) = fx + fx*cur_x*cur_x / (cur_z*cur_z);
            J_pixel_xi(0, 5) = -fx*cur_y / cur_z;
            J_pixel_xi(1, 0) = 0;
            J_pixel_xi(1, 1) = fy / cur_z;
            J_pixel_xi(1, 2) = -fy*cur_y / (cur_z*cur_z);
            J_pixel_xi(1, 3) = -fy - fy*cur_y*cur_y / (cur_z*cur_z);
            J_pixel_xi(1, 4) = fy*cur_x*cur_y / (cur_z*cur_z);
            J_pixel_xi(1, 5) = fy*cur_x / cur_z;

            // compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    double error = GetPixelValue(img1, u1+x, v1+y) - GetPixelValue(img2, u+x, v+y);;

                    Eigen::Vector2d J_img_pixel = Eigen::Vector2d::Zero();    // image gradients
                    J_img_pixel[0] = (GetPixelValue(img2, u+x+1, v+y) - GetPixelValue(img2, u+x-1, v+y))/2;
                    J_img_pixel[1] = (GetPixelValue(img2, u+x, v+y+1) - GetPixelValue(img2, u+x, v+y-1))/2;

                    // total jacobian
                    Vector6d J = -J_img_pixel.transpose() * J_pixel_xi;
                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        cv::DMatch match;
        for (int i=0; i<goodIndex.size(); i++) {
            match.queryIdx = goodIndex[i];
            match.trainIdx = i;
            match.distance = 10;
            matches.push_back(match);
        }

        if (std::isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased in iteration " << iter << ": " << cost << " --> " << lastCost << endl;
            break;
        }

        lastCost = cost;
//        cout << "iteration " << iter << " cost = " << cost << ", good = " << nGood << endl;
    }
    cout << "good projection: " << nGood << endl;
//    cout << "matches size:" << matches.size() << endl;
//    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    cv::Mat img_match_show(2*img1_show.rows, img1_show.cols, CV_8UC3);
    img1_show.copyTo(img_match_show(cv::Rect(0, 0, img1_show.cols, img1_show.rows)));
    img2_show.copyTo(img_match_show(cv::Rect(0, img1_show.rows, img2_show.cols, img2_show.rows)));

    vector<cv::KeyPoint> kp1, kp2;
    cv::KeyPoint kp;
    for (auto &px : px_ref) {
        kp.pt.x = px[0];
        kp.pt.y = px[1];
        kp1.push_back(kp);
    }
    for (auto &px : goodProjection) {
        kp.pt.x = cvRound(px[0]);
        kp.pt.y = cvRound(px[1]);
        kp2.push_back(kp);
    }

    int num_m = 0;
    for (auto &m : matches) {
        if (num_m % 3 == 0) {
            float b = 255*float ( rand() ) /RAND_MAX;
            float g = 255*float ( rand() ) /RAND_MAX;
            float r = 255*float ( rand() ) /RAND_MAX;
            cv::circle(img_match_show, cv::Point2d(kp1[m.queryIdx].pt.x, kp1[m.queryIdx].pt.y), 3, cv::Scalar(b,g,r), 2);
            cv::circle(img_match_show, cv::Point2d(kp2[m.trainIdx].pt.x, kp2[m.trainIdx].pt.y+img1.rows), 3, cv::Scalar(b,g,r), 2);
            cv::line(img_match_show, cv::Point2d(kp1[m.queryIdx].pt.x, kp1[m.queryIdx].pt.y), cv::Point2d(kp2[m.trainIdx].pt.x, kp2[m.trainIdx].pt.y+img1.rows), cv::Scalar(b,g,r), 1);
        }
        num_m ++;
    }

//    cv::drawMatches(img1_show, kp1, img2_show, kp2, matches, img_match_show,1);
    cv::imshow("matches", img_match_show);
    cv::waitKey();
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int pyramids = 4;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
    for (int l = 0; l < pyramids; l++) {
        cv::Mat img1_tmp, img2_tmp;
        resize(img1, img1_tmp, cv::Size(img1.cols*scales[l], img1.rows*scales[l]));
        resize(img2, img2_tmp, cv::Size(img1.cols*scales[l], img1.rows*scales[l]));
        pyr1.push_back(img1_tmp);
        pyr2.push_back(img2_tmp);
    }
    // END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values

    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px : px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG * scales[level]; fy = fyG * scales[level];
        cx = cxG * scales[level]; cy = cyG * scales[level];
        // END YOUR CODE HERE
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }
}
