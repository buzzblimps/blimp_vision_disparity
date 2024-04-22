#pragma once

#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/stereo_camera_model.h>

// ============================== INCLUDES ==============================
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;

// ============================== DEFINES ==============================
#define CAMERA_WIDTH	    1280
#define CAMERA_HEIGHT	    960

#define RECT_WIDTH		    320
#define RECT_HEIGHT		    240

#define DISP_WIDTH		    160
#define DISP_HEIGHT		    120

// #define STEREO_CAL_PATH     "/home/pi/piOffboardStereoVisionSrc/stereo_rectify_maps240p.xml"
// #define STEREO_CAL_FILENAME     "stereo_rectify_maps240p.xml"

#define PRE_FILTER_SIZE	    7
#define PRE_FILTER_CAP	    2
#define UNIQUENESS_RATIO	5

#define LAMBDA			    17.8
#define SIGMA			    5.0

#define AVOID_DIST		    70
#define AVOID_AREA		    6000

#define MIN_AREA		    50
#define SIZE_RATIO		    3

//Colors
//Green ball
// #define B_CORRECTION	        Scalar(29,7,15)
// #define B_MIN			    Scalar(46,0,0)
// #define B_MAX			    Scalar(96,74,213)

//Purple ball
#define B_CORRECTION	        Scalar(0,19,0)
#define B_MIN			        Scalar(105,0,19)
#define B_MAX			        Scalar(171,255,255)

#define ORANGE_G_CORRECTION     Scalar(56,68,0)
#define ORANGE_G_MIN            Scalar(0,0,185)
#define ORANGE_G_MAX            Scalar(61,255,255)

#define YELLOW_G_CORRECTION	    Scalar(91,42,0)
#define YELLOW_G_MIN			Scalar(24,37,0)
#define YELLOW_G_MAX			Scalar(91, 255, 255)

#define CONVERSION		    0.15

#define E_ITER 1
#define D_ITER 1
#define E_SIZE 1
#define D_SIZE 1

#define GOAL_DILATION_ADJUST        4

#define G_POLY_APPROX_E             0.01
#define GOAL_INNER_CONTOUR_SCALE    0.7
#define GOAL_CONFIRM			    6000

#define F               420
#define BASELINE        0.062 //0.20341207

enum object {
    balloon,
    blimpB,
    blimpR,
    goalO,
    goalY,
    first = balloon,
    last = goalY
};

enum autoState{
	searching,
	approach,
	catching,
	caught,
	goalSearch,
	approachGoal,
	scoringStart,
	shooting,
	scored
};

enum blimpType{
	blue,
	red
};

enum goalType{
	orange,
	yellow
};

// ============================== CLASS ==============================

class ComputerVision {
    private:
        VideoCapture cap;

        image_geometry::StereoCameraModel model_;

        // contains scratch buffers for block matching
        // stereo_image_proc::StereoProcessor block_matcher_;

        // Stereo calibration
        // Mat Left_Stereo_Map1;
        // Mat Left_Stereo_Map2;
        // Mat Right_Stereo_Map1;
        // Mat Right_Stereo_Map2;
        // Mat Q;

        // Stereo matcher
        Ptr<StereoBM> left_matcher_;
        Ptr<ximgproc::DisparityWLSFilter> wls_filter_;
        Ptr<StereoMatcher> right_matcher_;

        // Identified objects
        std::vector<vector<float>> balloons;
        std::vector<vector<float>> goals;

        // Object avoidance quadrant
        int quad;
        
        // Goal detection
        double pixelDensityL = 0.2;
        double pixelDensityR = 0.2;
        Mat output,output_norm,output_norm_scaled;

        vector<Point> scaleContour(vector<Point> contour, float scale);

        string srcDir;

        void getFrames(Mat &imgL, Mat &imgR);
        bool getBall(float &X, float &Y, float &Z, float &area, Mat &imgL, Mat &imgR);
        void getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR);

        // Store left camera's corrected view
        cv::Mat left_correct_, right_correct_;

        //Stereo disparity matrix
        cv::Matx44d Q_;

        //Ball color correction matrices
        cv::Mat ballCorrect_L_, ballCorrect_R_;
    public:
        void init(const sensor_msgs::msg::CameraInfo &cinfo_left, const sensor_msgs::msg::CameraInfo &cinfo_right);

        // void readCalibrationFiles();
        void update(Mat imgL, Mat imgR, autoState mode, goalType goalColor); // Big image processing function
        vector<vector<float>> getTargetBalloon();
        vector<vector<float>> getTargetGoal();
        int getQuad();
};
