// ============================== INCLUDES ==============================
#include <iostream>
#include <vector>

#include "ComputerVision.hpp"

using namespace std;

// ============================== CLASS ==============================

void ComputerVision::init(const sensor_msgs::msg::CameraInfo &cinfo_left, const sensor_msgs::msg::CameraInfo &cinfo_right) {

    model_.fromCameraInfo(cinfo_left, cinfo_right);

    //Compute stereo reprojection matrix Q
    double Tx = -model_.baseline();
    Q_(0,0) =  1.0;
    Q_(0,3) = -model_.left().cx();
    Q_(1,1) =  1.0;
    Q_(1,3) = -model_.left().cy();
    Q_(2,3) =  model_.left().fx();
    Q_(3,2) = -1.0/Tx;
    Q_(3,3) =  (model_.left().cx() - model_.right().cx())/Tx;
    
    // Init stereo matcher
    // left_matcher = cv::StereoBM::create(16, 13); //Num disp, block size
    left_matcher_ = cv::StereoBM::create(16, 9);

    left_matcher_->setPreFilterType(1);
    // left_matcher_->setPreFilterSize(PRE_FILTER_SIZE);
    // left_matcher_->setPreFilterCap(PRE_FILTER_CAP);
    // left_matcher_->setUniquenessRatio(UNIQUENESS_RATIO);

    left_matcher_->setMinDisparity(9);
    left_matcher_->setSpeckleRange(4);
    left_matcher_->setSpeckleWindowSize(100);
    left_matcher_->setDisp12MaxDiff(0);


    left_matcher_->setPreFilterSize(9);
    left_matcher_->setPreFilterCap(31);
    left_matcher_->setUniquenessRatio(5);
    left_matcher_->setTextureThreshold(10);

    wls_filter_ = ximgproc::createDisparityWLSFilter(left_matcher_);
    right_matcher_ = ximgproc::createRightMatcher(left_matcher_);

    // wls_filter_->setLambda(LAMBDA);
    // wls_filter_->setSigmaColor(SIGMA);

    //Set the ball correction matrices
    ballCorrect_L_ = Mat::zeros(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), 16);
    ballCorrect_R_ = Mat::zeros(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), 16);
    ballCorrect_L_.setTo(B_CORRECTION);
    ballCorrect_R_.setTo(B_CORRECTION);
}

// void ComputerVision::readCalibrationFiles(){
//     // Check that program is still running
// 	if(!programData->program_running){
// 		fprintf(stdout, "ComputerVision trying to read calibration files with program_running=false. Stopping.\n");
// 		return;
// 	}

//     if(programData->verboseMode) fprintf(stdout, "Reading Stereo Camera Parameters.\n");
//     string stereo_cal_path = srcDir + "/" + STEREO_CAL_FILENAME;
//     if(programData->verboseMode) fprintf(stdout, "Stereo Camera Parameter File Path: %s\n", stereo_cal_path.c_str());
// 	FileStorage cv_file2 = FileStorage(stereo_cal_path, FileStorage::READ);
// 	if(!cv_file2.isOpened()){
//         fprintf(stderr, "Error: Failed to open stereo parameter file.\n");
//         programData->program_running = false;
//         return;
//     }

//     cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
// 	cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
// 	cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
// 	cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
// 	cv_file2["Q"] >> Q;
// 	cv_file2.release();

//     if(programData->verboseMode) fprintf(stdout, "Read complete.\n");
// }

// Big image processing function
void ComputerVision::update(Mat imgL, Mat imgR, autoState mode, goalType goalColor) {
    left_correct_ = imgL;
    right_correct_ = imgR;

    // Coord Variables
    float Ballx, Bally, Ballz = 0;
    float Goalx, Goaly, Goalz = 0;
    float ballArea = 0;
    float goalArea = 0;
    float goalAngle = 0;

    // Object Avoidance
    if (mode == searching || mode == goalSearch) {
        // Maybe Fix
        quad = 10;    
    }

    // Object detection based on state
    balloons.clear();
    goals.clear();
    if (mode == searching || mode == approach || mode == catching) {
        // Ball Detection
        bool detectedBall = getBall(Ballx, Bally, Ballz, ballArea, imgL, imgR);

        if(detectedBall) {
            if(true) {
                //cout << "Balloon Data:" << endl;
                //cout << "X: " << Ballx << endl;
                //cout << "Y: " << Bally << endl;
                // cout << "Z: " << Ballz << endl;
                //cout << "Area: " << ballArea << endl << endl;
            }

            std::vector<float> balloon;
            balloon.push_back(Ballx);
            balloon.push_back(Bally);
            balloon.push_back(Ballz);
            balloon.push_back(ballArea);
            balloons.push_back(balloon);
        }
    } else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
        // Goal Detection
        getGoal(Goalx, Goaly, Goalz, goalArea, goalAngle, imgL, imgR);

        std::vector<float> goal;
        goal.push_back(Goalx);
        goal.push_back(Goaly);
        goal.push_back(Goalz);
        goal.push_back(goalArea);
        //goal.push_back(goalAngle);
        goals.push_back(goal);
    }
}

vector<vector<float>> ComputerVision::getTargetBalloon() {
    std::vector<std::vector<float> > target;
    //fprintf(stdout, "BalloonsSize=%d\n", balloons.size());
    //send back balloon data
    float area = 0;
    int index = -1;
    for (int i = 0; i < balloons.size(); i++) {
        if (area < balloons[i][3]) {
            area = balloons[i][3];
            index = i;
        }
    }

    if (index != -1) {
        target.push_back(balloons[index]);
    }
    return target;
}

vector<vector<float>> ComputerVision::getTargetGoal() {
    std::vector<std::vector<float> > target;
    //send back goal data
    float area = 0;
    int index = -1;
    for (int i = 0; i < goals.size(); i++) {
        if (area < goals[i][3]) {
            area = goals[i][3];
            index = i;
        }
    }

    if (index != -1) {
        target.push_back(goals[index]);
    }
    return target;
}
    
int ComputerVision::getQuad() {
    return quad;
}

vector<Point> ComputerVision::scaleContour(vector<Point> contour, float scale) {
    Moments moment = moments(contour);
    double cx = moment.m10 / moment.m00; //int(M['m10']/M['m00'])
    double cy = moment.m01 / moment.m00; //int(M['m01']/M['m00'])

    //shift all points
    for (unsigned int i = 0; i < contour.size(); i++) {
        contour[i].x = ((contour[i].x-cx)*scale)+cx;
        contour[i].y = ((contour[i].y-cy)*scale)+cy;
    }

    return contour;
}

bool ComputerVision::getBall(float &X, float &Y, float &Z, float &area, Mat &left_rect, Mat &right_rect) {

    //Uncomment for disparity testing
    //Convert left and right to grayscale
    cv::Mat left_rect_mono, right_rect_mono;
    cv::cvtColor(left_rect,  left_rect_mono,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_rect, right_rect_mono, cv::COLOR_BGR2GRAY);

    imshow("Left Mono",  left_rect_mono);
    imshow("Right Mono", right_rect_mono);

    //Compute stereo disparities and weighted least squares filter
    cv::Mat left_disp, right_disp;
    left_matcher_->compute(left_rect_mono, right_rect_mono, left_disp);
    right_matcher_->compute(right_rect_mono, left_rect_mono, right_disp);

    cv::Mat filtered_disp;
    wls_filter_->filter(left_disp, left_rect_mono, filtered_disp, right_disp);
    // std::cout << left_disp << std::endl;

    double min_val, max_val;
    cv::minMaxLoc(filtered_disp, &min_val, &max_val);
    Mat disparity_vis;
    filtered_disp.convertTo(disparity_vis, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

    namedWindow("Filtered Disparity");
    imshow("Filtered Disparity", disparity_vis);

    return true;

    //Apply correction
    Mat ball_CL, ball_CR;
    add(left_rect, ballCorrect_L_, ball_CL);
    add(right_rect, ballCorrect_R_, ball_CR);

    //Apply HSV
    Mat left_HSV, right_HSV;
    cvtColor(ball_CL, left_HSV, cv::COLOR_BGR2HSV);
    cvtColor(ball_CR, right_HSV, cv::COLOR_BGR2HSV);

    //Visualize only H
    /*
    vector<Mat> HSV;
    split(left_HSV, HSV); // Split image into HSV channels
    (HSV[1]).setTo(255); // Set S channel to max
    (HSV[2]).setTo(255); // Set V channel to max
    Mat newHSV, newHSVRGB;
    merge(HSV, newHSV); // Merge channels back into one image
    cvtColor(newHSV, newHSVRGB, COLOR_HSV2BGR); // Convert to RGB
    piComm->setStreamFrame(newHSVRGB, "Left_HSV");*/

    //Isolate Ball
    Mat bMask_L, bMask_R;
    inRange(left_HSV, B_MIN, B_MAX, bMask_L);
    inRange(right_HSV, B_MIN, B_MAX, bMask_R);

    // imshow("bMask_L", bMask_L);

    //Noise reduction
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat bMask_L_cleaned, bMask_R_cleaned;
    morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
    morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

    morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
    morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

    //DEBUG: see mask
    //namedWindow("bMask_L");
    //imshow("bMask_L", bMask_L_cleaned);
    // piComm->setStreamFrame(bMask_L_cleaned, "bMask_L");

    //namedWindow("bMask_R");
    //imshow("bMask_R", bMask_R_cleaned);

    //Find Largest Contour (Largest Ball)
    vector<vector<Point> > contoursL;
    findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point> largestContour_L;
    float largestArea_L = 0;
    int index_L = 0;

    for (unsigned int i = 0; i < contoursL.size(); i++) {
        double area = contourArea(contoursL[i]);
        if (area > largestArea_L) {
            largestArea_L = area;
            largestContour_L = contoursL[i];
            index_L = i;
        }
    }

    vector<vector<Point> > contoursR;
    findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point> largestContour_R;
    float largestArea_R = 0;
    int index_R = 0;

    for (unsigned int i = 0; i < contoursR.size(); i++) {
        double area = contourArea(contoursR[i]);
        if (area > largestArea_R) {
            largestArea_R = area;
            largestContour_R = contoursR[i];
            index_R = i;
        }
    }

    // Exclude contours that are too small
    if (largestArea_L < 50 || largestArea_R < 50) {
        largestContour_L.clear();
        largestContour_R.clear();

        return false;
    }

    // Debug draw contours
    //namedWindow("contL");
    Mat imgLcountours;
    left_rect.copyTo(imgLcountours);
    drawContours(imgLcountours, contoursL, index_L, Scalar(255, 255, 255), -1);
    imshow("imgLcountours", imgLcountours);

    //piComm->setStreamFrame(imgL, "Draw Contours");

    //namedWindow("contR");
    //drawContours(imgR, contoursR, index_R, Scalar(255, 255, 255), -1);
    //imshow("contR", imgR);

    // Center detection with blob centroid
    Moments m_L = moments(largestContour_L, true);
    Point p_L(m_L.m10/m_L.m00, m_L.m01/m_L.m00);

    //cout << largestArea_R << endl;
    //cout << largestArea_L << endl;

    Moments m_R = moments(largestContour_R, true);
    Point p_R(m_R.m10/m_R.m00, m_R.m01/m_R.m00);

    // Reveal area around chosen point in original image
    Mat maskL = Mat::zeros(left_rect.size(), CV_8UC1);
    Mat maskR = Mat::zeros(right_rect.size(), CV_8UC1);

    // Create Circle masks
    int mask_radius = 50;
    // int radius = 300;

    circle(maskL, p_L, mask_radius, Scalar(255, 255, 255), -1);
    circle(maskR, p_R, mask_radius, Scalar(255, 255, 255), -1);

    Mat imgLCircle;
    left_rect.copyTo(imgLCircle);
    circle(imgLCircle, p_L, 50, Scalar(0,0,0), -1);
    // imshow("imgLCircle", imgLCircle);

    threshold(maskL, maskL, 127, 255, THRESH_BINARY);
    threshold(maskR, maskR, 127, 255, THRESH_BINARY);

    // Apply mask
    Mat masked_imgL, masked_imgR;
    bitwise_and(left_rect,  left_rect,  masked_imgL, maskL);
    bitwise_and(right_rect, right_rect, masked_imgR, maskR);
    
    try {
        // Perform ORB feature extraction and matching
        int num_features = 25;
        Ptr<ORB> orb = ORB::create(num_features);

        vector<KeyPoint> keypointsL, keypointsR;
        Mat descriptorsL, descriptorsR;

        // Detect keypoints in the image
        orb->detect(masked_imgL, keypointsL);
        orb->detect(masked_imgR, keypointsR);

        //Define radius
        // float radius = 15.0f;

        //Filter keypoints
        vector<KeyPoint> kp_filt_L;
        for (const auto& k : keypointsL) {
            //Calculate Float
            float dist = sqrt(pow(k.pt.x - p_L.x, 2) + pow(k.pt.y - p_L.y, 2));

            if (dist < mask_radius) {
                kp_filt_L.push_back(k);
            }
        }

        vector<KeyPoint> kp_filt_R;
        for (const auto& k : keypointsR) {
            //Calculate Float
            float dist = sqrt(pow(k.pt.x - p_R.x, 2) + pow(k.pt.y - p_R.y, 2));

            if (dist < mask_radius) {
                kp_filt_R.push_back(k);
            }
        }

        //DEBUG: Filtering
        //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
        //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

        //Compute matches
        orb->compute(masked_imgL, kp_filt_L, descriptorsL);
        orb->compute(masked_imgR, kp_filt_R, descriptorsR);

        // Match descriptors using Brute-Force matcher
        BFMatcher matcher(NORM_HAMMING);
        vector<vector<DMatch>> knn_matches;
        matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

        // Filter matches using ratio test
        const float ratio_thresh = 0.8f;
        vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
              good_matches.push_back(knn_matches[i][0]);
            }
        }

        // Draw good matches
        Mat img_matches;
        drawMatches(masked_imgL, kp_filt_L, masked_imgR, kp_filt_R, knn_matches, img_matches);

        namedWindow("ORB Matches");
        imshow("ORB Matches", img_matches);

        //waitKey(1);
        // piComm->setStreamFrame(img_matches, "Matches");

        // Calculate average distance of all matched points
        double avg_distance = 0.0;
        for (const auto& matches : knn_matches) {
            if (matches.size() < 2) continue;  // Skip if not enough matches
            const auto& kp_L = kp_filt_L[matches[0].queryIdx];
            const auto& kp_R1 = kp_filt_R[matches[0].trainIdx];
            const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
            double disparity = abs(kp_L.pt.x - kp_R1.pt.x);
            double ratio = matches[0].distance / matches[1].distance;
            if (ratio < ratio_thresh) {
                double distance = (F * BASELINE) / disparity;
                avg_distance += distance;
            }
        }

        avg_distance /= good_matches.size();

        // Add RANSAC maybe?

        if (isnan(avg_distance)) {
            //Assign XYZ of ball
            Z = 1000;
            X = (p_L.x + p_R.x)/2;
            Y = (p_L.y + p_R.y)/2;
            area = (largestArea_L + largestArea_R)/2;
        } else {
            //Assign XYZ of ball
            Z = avg_distance;
            X = (p_L.x + p_R.x)/2;
            Y = (p_L.y + p_R.y)/2;
            area = (largestArea_L + largestArea_R)/2;
        }

        //DEBUG READ OUTPUT
        if (false) {
            cout << "Distance : " << Z << endl;
            cout << "X: " << X << endl;
            cout << "Y: " << Y << endl;
            cout << area << endl;
            cout << isnan(Z) << endl << endl;
        }

        return true;

    } catch (const cv::Exception){
        cout << "Failed" << endl;
        return false;
    }
}

void ComputerVision::getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR){
    // Applying blur to reduce noise
    Mat imgBlurredL;
    GaussianBlur(imgL, imgBlurredL, Size(5, 5), 2);

    Mat imgBlurredR;
    GaussianBlur(imgR, imgBlurredR, Size(5, 5), 2);

    // Initialize matrix for rectified stereo images
    Mat Left_nice, Right_nice;

    // Applying stereo image rectification on the left image
    // remap(imgBlurredL,
    //       Left_nice,
    //       Left_Stereo_Map1,
    //       Left_Stereo_Map2,
    //       INTER_LANCZOS4,
    //       BORDER_CONSTANT,
    //       0);

    // Applying stereo image rectification on the right image
    // remap(imgBlurredR,
    //       Right_nice,
    //       Right_Stereo_Map1,
    //       Right_Stereo_Map2,
    //       INTER_LANCZOS4,
    //       BORDER_CONSTANT,
    //       0);

    // TEMP: Skip rectification
    imgL.copyTo(Left_nice);
    imgR.copyTo(Right_nice);

    //Apply correction
    Mat goal_CL, goal_CR;
    Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
    Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
    balloonCorrect_L.setTo(ORANGE_G_CORRECTION);
    balloonCorrect_R.setTo(ORANGE_G_CORRECTION);

    add(Left_nice, balloonCorrect_L, goal_CL);
    add(Right_nice, balloonCorrect_R, goal_CR);

    //Apply HSV
    Mat left_HSV, right_HSV;
    cvtColor(goal_CL, left_HSV, cv::COLOR_BGR2HSV);
    cvtColor(goal_CR, right_HSV, cv::COLOR_BGR2HSV);

    //Isolate Goal
    Mat bMask_L, bMask_R;
    inRange(left_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_L);
    inRange(right_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_R);

    //Noise reduction
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat bMask_L_cleaned, bMask_R_cleaned;
    morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
    morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

    morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
    morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

    //DEBUG: see mask
    //namedWindow("bMask_L");
    //imshow("bMask_L", bMask_L_cleaned);
    //waitKey(1);

    //namedWindow("bMask_R");
    //imshow("bMask_R", bMask_R_cleaned);
    //waitKey(1);

    //Find Contours
    vector<vector<Point>> contoursL;
    vector<vector<Point>> contoursR;
    findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Apply morphological operations to fill in the gaps and complete the rectangle
    dilate(bMask_L_cleaned, bMask_L_cleaned, kernel);
    erode(bMask_L_cleaned, bMask_L_cleaned, kernel);
    dilate(bMask_R_cleaned, bMask_R_cleaned, kernel);
    erode(bMask_R_cleaned, bMask_R_cleaned, kernel);

    // Find the bounding rectangle of the largest contour
    Rect rectL;
    Rect rectR;

  // Try Catch for numerous failure points
  try {
    if (!contoursL.empty()) {
      size_t maxAreaIndexL = 0;
      for (size_t i = 1; i < contoursL.size(); i++) {
          if (contourArea(contoursL[i]) > contourArea(contoursL[maxAreaIndexL])) {
              maxAreaIndexL = i;
          }
      }

      if (pixelDensityL < 0.35){
        rectL = boundingRect(contoursL[maxAreaIndexL]);
      } else if (pixelDensityL > 0.35 && maxAreaIndexL != 0) {
        rectL = boundingRect(contoursL[maxAreaIndexL-1]);
      } 
      //std::cout << "index: " << maxAreaIndex << endl;
    }

    if (!contoursR.empty()) {
      size_t maxAreaIndexR = 0;
      for (size_t i = 1; i < contoursR.size(); i++) {
          if (contourArea(contoursR[i]) > contourArea(contoursR[maxAreaIndexR])) {
              maxAreaIndexR = i;
          }
      }

      if (pixelDensityR < 0.35){
        rectR = boundingRect(contoursR[maxAreaIndexR]);
      } else if (pixelDensityR > 0.35 && maxAreaIndexR != 0) {
        rectR = boundingRect(contoursR[maxAreaIndexR-1]);
      } 
      //std::cout << "index: " << maxAreaIndex << endl;
    }

    // Draw a new rectangle with the same aspect ratio and orientation
    if (!rectL.empty() && !rectR.empty()) {
        double aspectRatioL = (double)rectL.width / rectL.height;
        double aspectRatioR = (double)rectR.width / rectR.height;

        int newWidthL = (int)(aspectRatioL * rectL.height);
        int newWidthR = (int)(aspectRatioR * rectR.height);

        //center and 4 corners of the bounding box
        Point centerL = Point(rectL.x + rectL.width / 2, rectL.y + rectL.height / 2);
        Point ltCorner = Point(rectL.x, rectL.y + rectL.height);;
        Point rtCorner = Point(rectL.x + rectL.width, rectL.y + rectL.height);
        Point lbCorner = Point(rectL.x, rectL.y);
        Point rbCorner = Point(rectL.x + rectL.width, rectL.y);;

        Point centerR = Point(rectR.x + rectR.width / 2, rectR.y + rectR.height / 2);

        //show center and 4 corners
        circle(bMask_L_cleaned,centerL,1,Scalar(255,255,0),3,4,0);
        // circle(bMask_L_cleaned,ltCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,lbCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,rtCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,rbCorner,1,Scalar(255,255,0),20,4,0);
        circle(bMask_R_cleaned,centerR,1,Scalar(255,255,0),3,4,0);

        rectL = Rect(centerL.x - newWidthL / 2, rectL.y, newWidthL, rectL.height);
        rectR = Rect(centerR.x - newWidthR / 2, rectR.y, newWidthR, rectR.height);
        // std::cout << "Center(Left) (x,y): " << centerL.x << ", " << centerL.y << endl;
        // std::cout << "Center(Right) (x,y): " << centerR.x << ", " << centerR.y << endl;

        Mat cropedMaskL = bMask_L_cleaned.colRange(rectL.x,rectL.x + rectL.width).rowRange(rectL.y,rectL.y + rectL.height);
        Mat cropedMaskR = bMask_R_cleaned.colRange(rectR.x,rectR.x + rectR.width).rowRange(rectR.y,rectR.y + rectR.height);  

        int whitePixelsL = countNonZero(cropedMaskL);
        int whitePixelsR = countNonZero(cropedMaskR);
        //std::cout << "white pixels: " << whitePixels << endl;

        pixelDensityL = double(whitePixelsL)/double(rectL.width*rectL.height);
        pixelDensityR = double(whitePixelsR)/double(rectR.width*rectR.height);
        //std::cout << "pixel density: " << pixelDensity << endl;

        if (pixelDensityL > 0.1 && pixelDensityL < 0.35 && pixelDensityR > 0.1 && pixelDensityR < 0.35 ){
          rectangle(bMask_L_cleaned, rectL, Scalar(255), 2);

          //Coners
          vector<Point2f> corners;

          output = Mat::zeros(bMask_R_cleaned.size(), CV_32FC1);
          //cornerHarris for corner finding 
          cornerHarris(bMask_R_cleaned,output,5,3,0.04);
          //normalize to get the outputs 
          normalize(output,output_norm,0,255,NORM_MINMAX,CV_32FC1, Mat());
          convertScaleAbs(output_norm,output_norm_scaled);

          //draw the points on the image
          for (int j = 0; j<output_norm.rows;j++){
            for (int i = 0; i<output_norm.cols;i++){
              // the threshold can be changed to filter how many points are getting pushed to coners vector
              if ((int)output_norm.at<float>(j,i) > 150){
                //coordiates of the corners are (i,j), they can be stored in a vector 
                //circle(bMask_R_cleaned,Point(i,j),4,Scalar(0,0,255),2,8,0);
                corners.push_back(Point(i,j));
              }
            }
          }

          // K means clustering to remove close corners

          // Set the number of clusters
          int num_clusters = 4;

          // Convert the vector of corners to a matrix
          Mat corners_mat(corners.size(), 2, CV_32F);
          for (int i = 0; i < corners.size(); i++) {
              corners_mat.at<float>(i, 0) = corners[i].x;
              corners_mat.at<float>(i, 1) = corners[i].y;
          }

          // Perform k-means clustering
          Mat labels, centers;
          kmeans(corners_mat, num_clusters, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

          // Group the corners based on their cluster labels
          vector<vector<Point>> corner_groups(num_clusters);
          for (int i = 0; i < corners_mat.rows; i++) {
              int label = labels.at<int>(i);
              corner_groups[label].push_back(Point(corners_mat.at<float>(i, 0), corners_mat.at<float>(i, 1)));
          }

          // Compute the average of each group of corners
          vector<Point> averaged_corners;
          for (int i = 0; i < corner_groups.size(); i++) {
              if (corner_groups[i].size() > 0) {
                  int sum_x = 0, sum_y = 0;
                  for (int j = 0; j < corner_groups[i].size(); j++) {
                      sum_x += corner_groups[i][j].x;
                      sum_y += corner_groups[i][j].y;
                  }
                  int avg_x = round(sum_x / corner_groups[i].size());
                  int avg_y = round(sum_y / corner_groups[i].size());
                  averaged_corners.push_back(Point(avg_x, avg_y));
              }
          }

          std::cout << "corners:" << endl << averaged_corners << endl;

          // Draw circles at the averaged corners on the original image
          for (int i = 0; i < averaged_corners.size(); i++) {
              circle(bMask_R_cleaned, averaged_corners[i], 4, Scalar(0, 255, 0), 2);
          }

          // Set the radius of the circle aropiComm->setStreamFrame(bMask_L_cleaned, "bMask_L");und each corner point
          int radius = 30;

          // Create a grayscale image mask with the same size as the original image
          Mat mask(Left_nice.size(), CV_8UC1, Scalar(0));

          cout << averaged_corners.size() << endl;

          // Draw circles with the specified radius around each averaged corner point
          //for (int i = 0; i < averaged_corners.size(); i++) {
          //    circle(mask, averaged_corners[i], radius, Scalar(255), -1);
          //}
          
          Mat masked_imgL_;
          threshold(mask, mask, 127, 255, THRESH_BINARY);
          bitwise_and(imgL, imgL, masked_imgL_, mask);

          namedWindow("bruh");
          imshow("bruh", masked_imgL_);
          //waitKey(1);

          double widthHeightRatioL = (double)rectL.width / rectL.height;
          double widthHeightRatioR = (double)rectR.width / rectR.height;
          //std::cout << "Ratio: " << widthHeightRatio << endl;
          double areaL = rectL.width*rectL.height;
          double areaR = rectR.width*rectR.height;
          double areaRelDiff =  double((areaL-areaR)/(areaR)*100);

          if (areaRelDiff < 5) {
            //Calculate dist
            double disparity = abs(centerL.x - centerR.x);

            double distance = (F * BASELINE) / disparity;
            
            double distance2 = -0.000037*(areaL+areaR)/2 + 3.371;

            if (distance > 5.0){
              distance = 1000.0;
            } 
            
            //cout << distance2 << endl;
            //std::cout << "Area (Left,Right): " << areaL << ", " << areaR << endl;

            // Set Outputs
            X = (centerL.x + centerR.x)/2;
            Y = (centerL.y + centerR.y)/2;
            Z = distance2;  // For now
            area = (areaL + areaR)/2;

            //corners 

            //C = P^-1*W
            //W = PC

            float ratio = (widthHeightRatioL+widthHeightRatioR)/2;
            //std::cout << "Ratio: " << ratio << endl;
            angle = acosf(ratio)/3.14159*180;
            if (ratio > 1){
              angle = 0;
            } 
            std::cout << "angle" << angle << endl;

          }

        } else {
          pixelDensityL = 0.2; //reinitiate
          pixelDensityR = 0.2;
        }      
    }

    // Display the result
    // piComm->setStreamFrame(bMask_L_cleaned, "bMask_L");

    //imshow("ApproximationsL", bMask_L_cleaned);
    //waitKey(1);
    // piComm->setStreamFrame(bMask_L_cleaned, "bMask_L");
    //imshow("ApproximationsR", bMask_R_cleaned);
    //waitKey(1);

    Mat masked_imgR_;
    bitwise_and(Left_nice, Left_nice, masked_imgR_, bMask_R_cleaned);

    //namedWindow("Test");
    //imshow("Test", masked_imgR_);
    //waitKey(1);
    
    /*
    //Approximate shapes
    std::vector<std::vector<cv::Point>> approximationsL(contoursL.size());
    for (int i = 0; i < contoursL.size(); i++) {
        approxPolyDP(contoursL[i], approximationsL[i], 50, false);
    }

    std::vector<std::vector<cv::Point>> approximationsR(contoursR.size());
    for (int i = 0; i < contoursR.size(); i++) {
        approxPolyDP(contoursR[i], approximationsR[i], 50, false);
    }

    // Visualize approximated contours
    Mat approx_image = bMask_L_cleaned.clone();
    for (int i = 0; i < approximationsL.size(); i++) {
        cv::drawContours(bMask_L_cleaned, approximationsL, i, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Approximations", bMask_L_cleaned);
    cv::waitKey(1);

    std::vector<cv::Point2f> corners;
      double qualityLevel = 0.01;
      double minDistance = 10;
      int blockSize = 3;
      bool useHarrisDetector = false;
      double k = 0.04;
      cv::goodFeaturesToTrack(bMask_L_cleaned, corners, 500, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

      for (size_t i = 0; i < corners.size(); i++)
      {
          cv::circle(bMask_L_cleaned, corners[i], 5, cv::Scalar(255), -1);
      }

    cv::imshow("Corners", bMask_L_cleaned);
    cv::waitKey(1);

    // Show image with detected incomplete rectangles
    imshow("Detected rectangles", bMask_L_cleaned);
    waitKey(1);

    */

  } catch (const cv::Exception){
    cout << "Failed" << endl;
    return;
  }
}