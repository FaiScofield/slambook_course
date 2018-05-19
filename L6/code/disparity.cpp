#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>

using namespace std;

using namespace cv;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "../left.png";
string right_file = "../right.png";
string disparity_file = "../disparity.png";

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

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0; // dx need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            double H = 0.0;
            double b = 0.0;
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y <= half_patch_size || kp.pt.y  >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = 0;
                    error = GetPixelValue(img1, kp.pt.x+x, kp.pt.y+y) 
                                - GetPixelValue(img2, kp.pt.x+x+dx, kp.pt.y+y);
                    double J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        J = (GetPixelValue(img2, kp.pt.x+x+dx+1, kp.pt.y+y)
                                    - GetPixelValue(img2, kp.pt.x+x+dx-1, kp.pt.y+y))/2.0;
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx is updated, so we can store it and only compute error
                        J = (GetPixelValue(img1, kp.pt.x+x+1, kp.pt.y+y)
                                    - GetPixelValue(img1, kp.pt.x+x-1, kp.pt.y+y))/2.0;
                    }

                    // compute H, b and set cost;
                    H += J*J;
                    b += error*J;
                    cost += error*error;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            double update = b/H;
            // TODO END YOUR CODE HERE

            if (std::isnan(update)) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                //cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx 
            dx += update;
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + cv::Point2f(dx, 0.0);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, 0.0);
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
        bool inverse = false) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    pyr1.push_back(img1.clone());
    pyr2.push_back(img2.clone());
    for (int i = 1; i < pyramids; i++) {
        std::cout << "pyramids: " << i << endl;
        Mat dst;
        pyrDown(pyr1.back(), dst); 
        pyr1.push_back(dst.clone());
        pyrDown(pyr2.back(), dst);
        pyr2.push_back(dst.clone());
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE

    for(int i = pyramids-1; i >= 0; i--){
        vector<KeyPoint> pyrkp1 = kp1;
        for(size_t j = 0; j< pyrkp1.size(); ++j){
            pyrkp1[j].pt *= scales[i];
        }
        for(size_t j = 0; j < kp2.size(); ++j){
            kp2[j].pt *= 1/pyramid_scale;
        }
        OpticalFlowSingleLevel(pyr1[i], pyr2[i], pyrkp1, kp2, success);
        std::cout << "kp2[0] : "<< kp2[0].pt.x << " " << kp2[0].pt.y << std::endl;
        for(size_t j = 0; j < kp2.size(); ++j){
            if(success[j]){
                //std::cout << j << std::endl;
            }
        }
    }
    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}

int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = cv::imread(left_file, 0);
    Mat img2 = cv::imread(right_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(800, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi);

    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    int num_success = 0, num_1pix = 0, num_2pix = 0, num_other = 0;
    for( int i = 0; i < kp2_multi.size(); i++){
        if(success_multi[i]){
            int disparity = disparity_img.at<uchar>(kp1[i].pt.y, kp1[i].pt.x);
            double error = kp2_multi[i].pt.x -kp1[i].pt.x + disparity;
            cout << "disparity error: " << error <<endl;
            num_success ++;
            if(std::abs(error) < 1.0) num_1pix++;
            else if(std::abs(error) < 2.0) num_2pix++;
            else num_other++;
        }
    }
    std::cout << "ratio of close: " << float(num_1pix)/ num_success << endl;
    std::cout << "ratio of medium: " << float(num_2pix)/ num_success << endl;
    std::cout << "ratio of error: " << float(num_other)/ num_success << endl;
    
    cv::imshow("tracked multi level", img2_multi);
    cv::waitKey(0);
    return 0;
}