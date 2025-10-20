/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */



#ifndef VISION_UTILS_H 
#define VISION_UTILS_H

#include <vector>
#include <random>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <visp3/vision/vpHomography.h>

#include <tuple>
#include <sstream>

#include <visp3/core/vpPoint.h>
#include <cmath> // para std::sqrt
#include <string>

using namespace cv;


#ifdef __USE_SINGLE_PRECISION__
typedef float FPTYPE;
#else
typedef double FPTYPE;
#endif

typedef Eigen::Vector3d Point3D_t;
typedef Eigen::Vector3d Vector3D_t;
typedef Eigen::Vector2d Point2D_t ;

typedef Eigen::Matrix<FPTYPE, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix_t;
typedef Eigen::Transform<FPTYPE, 3, Eigen::Affine> AffineTransformation;


// functions to get X, Y, Z coordinates
template <typename T>
inline FPTYPE& X(T& p) { return p(0); }

template <typename T>
inline FPTYPE& Y(T& p) { return p(1); }

inline FPTYPE& Z(Point3D_t& p) { return p(2); }

template <typename T>
inline const FPTYPE X(const T& p) { return p(0); }

template <typename T>
inline const FPTYPE Y(const T& p) { return p(1); }

inline const FPTYPE& Z(const Point3D_t& p) { return p(2); }


enum class homographySolution {
    SOLUTION_1=0,
    SOLUTION_2=1,
    NONE=-1
};

// Colores por segmento: izquierda (rojo), centro (verde), derecha (azul)
const cv::Scalar colors[3] = {
    cv::Scalar(0, 0, 255),   // rojo
    cv::Scalar(0, 255, 0),   // verde
    cv::Scalar(255, 0, 0)    // azul
};


Matrix_t ComputeHomography( const std::vector<cv::Point2d> &cv_reference_points,
                            const std::vector<cv::Point2d> &cv_current_points,
                            const Matrix_t &K);

Matrix_t ComputeEssential( const std::vector<cv::Point2d> &cv_reference_points,
                           const std::vector<cv::Point2d> &cv_current_points,
                           const Matrix_t &K);

void RecoverFromHomography(const Matrix_t &homography,Matrix_t &R, Vector3D_t &t,
                           Vector3D_t &n, FPTYPE &d, const int current_iteration, 
                           homographySolution &homograpy_solution);

void RecoverFromEssential(const Matrix_t &essential,Matrix_t &R, Vector3D_t &t);

void EstimateHomography(const cv::Mat &refImg, const cv::Mat &currImg, const Matrix_t &K, Matrix_t &H, const int counter );

//================================================================================================

bool isNonColinear(FPTYPE x1, FPTYPE y1, FPTYPE x2, FPTYPE y2, FPTYPE x3, FPTYPE y3);

void reorderVector(std::vector<double>& v, int i, int j, int k);

void selectTripletFromSegments( const std::vector<FPTYPE> &ref_x,
                                const std::vector<FPTYPE> &ref_y,
                                std::tuple<int,int,int> &selected_idxs);


void selectBestTriplet(std::vector<FPTYPE>& ref_x, std::vector<FPTYPE>& ref_y,
                       std::vector<FPTYPE>& cur_x, std::vector<FPTYPE>& cur_y,
                       const std::vector<cv::KeyPoint>& keypointsRef,
                       const std::vector<cv::DMatch>& matches,
                       std::tuple<int, int, int>& selected_idxs);


void drawSelectedTriplet(const std::vector<cv::KeyPoint>& keypointsRef,
                         const std::vector<cv::DMatch>& goodMatches,
                         const std::tuple<int, int, int>& best_idxs,
                         const cv::Mat& refImg, int counter);

void drawSegmentedKeypoints(const cv::Mat& image,
                            const std::vector<cv::KeyPoint>& keypoints,
                            const std::string& filename);

double evaluateHomographyProjection(
    const std::vector<double> &ref_x,
    const std::vector<double> &ref_y,
    const std::vector<double> &curr_x,
    const std::vector<double> &curr_y,
    const vpHomography &H);


void detectORBInSegments(  const cv::Mat &img,
    std::vector<cv::KeyPoint> &keypoints_out,
    cv::Mat &descriptors_out,
    int nFeaturesPerRegion );


//=================================================================================================

void EstimateHomographyVisp(const cv::Mat &refImg, const cv::Mat &currImg, const Matrix_t &K, vpHomography &H, const int counter );

void Rodriguez(const Matrix_t &R, Vector3D_t &u);

void PBVSController(const Matrix_t &R, const Vector3D_t &t, const Vector3D_t &u, 
                    Vector3D_t &Uv, Vector3D_t &Uw,const FPTYPE lambdav_,const FPTYPE lambdaw_);

Point2D_t NormalicePoint(const Point2D_t &point, const Matrix_t &Kinv_);

void RecoverFromHomographyVisp(vpHomography &H, vpRotationMatrix &R, vpTranslationVector &t, vpColVector &n, vpThetaUVector &u);

void PBVSController(const vpRotationMatrix &R, const vpTranslationVector &t, const vpThetaUVector &u, 
                    vpColVector &Uv, vpColVector &Uw,const FPTYPE lambdav_,const FPTYPE lambdaw_);

FPTYPE YRotationAngleFromRotationMatrix(const Matrix_t& rotation_matrix);
    

#endif // VISION_UTILS_H
