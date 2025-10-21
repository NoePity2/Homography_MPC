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

#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

#include <iostream>
#include <tuple>
#include <vector>


#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <visp3/vision/vpHomography.h>

#include "vision_utils.h"

using namespace cv;

/*******************************************************************************
 *
 *                                functions
 *
 ******************************************************************************/


namespace {


    inline std::pair<FPTYPE, FPTYPE> unitize(FPTYPE x, FPTYPE y)
    {
        FPTYPE magnitude = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        FPTYPE l = x / magnitude;
        FPTYPE m = y / magnitude;
        return std::make_pair(l, m);
    }

    Vector3D_t vex(const Matrix_t &A) {
        if (A.rows()!=3 || A.cols()!=3)
            throw std::domain_error("vex: expects 3x3 matrices as an input");
        Vector3D_t v;
        v(0)=0.5*(A(2,1)-A(1,2));
        v(1)=0.5*(A(0,2)-A(2,0));
        v(2)=0.5*(A(1,0)-A(0,1));
        return v;
    }

}; // namespace



Matrix_t ComputeHomography( const std::vector<cv::Point2d> &cv_reference_points,
                            const std::vector<cv::Point2d> &cv_current_points,
                            const Matrix_t &K)
{
    cv::Mat cv_homography = cv::findHomography(cv_reference_points, cv_current_points);
    Matrix_t homography(3, 3);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            homography(i,j) = cv_homography.at<double>(i,j);
    return K.inverse()*homography*K;
}

Matrix_t ComputeEssential( const std::vector<cv::Point2d> &cv_reference_points,
                           const std::vector<cv::Point2d> &cv_current_points,
                           const Matrix_t &K)
{
    cv::Mat cv_fundamental_matrix =
        findFundamentalMat(cv_reference_points, cv_current_points, cv::FM_8POINT, 3, 0.99);
    Matrix_t fundamental(3, 3);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            fundamental(i,j) = cv_fundamental_matrix.at<double>(i,j);
    return K.transpose()*fundamental*K;
}

void RecoverFromEssential(const Matrix_t &homography,Matrix_t &R, Vector3D_t &t)
{
    // Decomposes the essential matrix E (3x3) into the camera motion
    // In practice there are multiple solutions and S (4x4xN) is a set of homogeneous
    // transformations representing possible camera motion.
    //
    // Reference::
    //
    // Y.Ma, J.Kosecka, S.Soatto, S.Sastry,
    // "An invitation to 3D",
    // Springer, 2003.
    // p116, p120-122
    // Notes::
    // - The transformation is from view 1 to view 2.
    // See also CentralCamera.E.
    // we return T from view 1 to view 2
    Matrix_t U;
    Matrix_t S;
    Matrix_t V;

    Eigen::JacobiSVD<Matrix_t> svd(homography, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    // Ma etal solution, p116, p120-122
    // Fig 5.2 (p113), is wrong, (R,t) is from camera 2 to 1
    if (V.determinant() < 0) {
        V = -V;
        S = -S;
    }
    if (U.determinant() < 0) {
        U = -U;
        S = -S;
    }
    static Eigen::Matrix3d mp(Eigen::AngleAxisd(+0.5*M_PI, Eigen::Vector3d::UnitZ()));
    static Eigen::Matrix3d mm(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()));
    Matrix_t R1 = U*mp.transpose()*V.transpose();
    Matrix_t R2 = U*mm.transpose()*V.transpose();
    Vector3D_t t1 = vex(U*mp*S.asDiagonal()*U.transpose());
    Vector3D_t t2 = vex(U*mm*S.asDiagonal()*U.transpose());   
    if (R1(0,0)>0&&R1(1,1)>0&&R1(2,2)>0) 
       R=R1;
    else
       R=R2;
    if (t1(2)>0) // Suppose here that the target camera position is in front of us, not behind 
       t=t1;
    else
       t=t2;
}

void RecoverFromHomography(const Matrix_t &homography,Matrix_t &R, Vector3D_t &t, Vector3D_t &n, FPTYPE &distancePlane, const int current_iteration, homographySolution &homograpy_solution)
{
    Matrix_t U;
    Matrix_t S;
    Matrix_t V;

    {
        Eigen::JacobiSVD<Matrix_t> svd(homography, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        S = svd.singularValues();
        V = svd.matrixV();
    }

    // Eigen returns the transpose of what we need here
    V.transposeInPlace();

    FPTYPE s1 = S(0) / S(1);
    FPTYPE s3 = S(2) / S(1);
    FPTYPE zeta = s1 - s3;
    if (fabs(zeta)<std::numeric_limits<FPTYPE>::epsilon()) {
        distancePlane = 1.0;
        R = Matrix_t::Identity(3,3);
        n = Vector3D_t::Zero(3); n(0)=1.0;
        t = Vector3D_t::Zero(3); t(0)=1.0;
        return;
    }
    FPTYPE a1 = std::sqrt(1 - std::pow(s3, 2));
    FPTYPE b1 = std::sqrt(std::pow(s1, 2) - 1);
    std::pair<FPTYPE,FPTYPE> p,q,r;
    p = unitize(a1, b1);               FPTYPE &a = p.first;  FPTYPE &b = p.second;
    q = unitize(1 + s1 * s3, a1 * b1); FPTYPE &c = q.first;  FPTYPE &d = q.second;
    r = unitize(-b / s1, -a / s3);     FPTYPE &e = r.first;  FPTYPE &f = r.second;

    Matrix_t v1 = V.row(0).transpose();
    Matrix_t v3 = V.row(2).transpose();

    Vector3D_t n1 = b * v1 - a * v3;
    Vector3D_t n2 = b * v1 + a * v3;

    Matrix_t tmp(3, 3);
    tmp << c, 0.0, d,
           0, 1.0, 0,
          -d, 0.0, c;

    Matrix_t R1 = U * (tmp * V);
    Matrix_t R2 = U * (tmp.transpose() * V);

    Vector3D_t t1 = e * v1 + f * v3;
    Vector3D_t t2 = e * v1 - f * v3;


    if (Z(n1) < 0) {
        t1 = -t1;
        n1 = -n1;
    }

    if (Z(n2) < 0) {
        t2 = -t2;
        n2 = -n2;
    }
    if(current_iteration == 0) {
        if (Z(n1) > Z(n2)) { //Solution 1
            R = R1;
            t = t1;
            n = n1;
            homograpy_solution = homographySolution::SOLUTION_1;
        } else { //Solution 2
            R = R2;
            t = t1;
            n = n2;
            homograpy_solution = homographySolution::SOLUTION_2;
        }
    }else{
        if(homograpy_solution == homographySolution::SOLUTION_1){
            R = R1;
            t = t1;
            n = n1;
        }else{
            R = R2;
            t = t2;
            n = n2;
        }
    }

    distancePlane = 1.0 / zeta;
}


void EstimateHomography(const cv::Mat &refImg, const cv::Mat &currImg, const Matrix_t &K,  Matrix_t &H, const int counter )
{


    //*----------------- Step 1: Detect keypoints and compute keypoints using ORB Detector    -------------------------*//
    int nFeatures = 600;
    
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nFeatures);

    std::vector<cv::KeyPoint> keypointsRef, keypointsCurr;
    cv::Mat descriptorsRef, descriptorsCurr; 

    detector->detectAndCompute( refImg, noArray(), keypointsRef, descriptorsRef );
    detector->detectAndCompute( currImg, noArray(), keypointsCurr, descriptorsCurr );

    //detectORBInSegments(refImg, keypointsRef, descriptorsRef, 500);
    //detectORBInSegments(currImg, keypointsCurr, descriptorsCurr, 500);

    descriptorsRef.convertTo(descriptorsRef, CV_8U);
    descriptorsCurr.convertTo(descriptorsCurr, CV_8U);

    
    //*----------------- Step 2: Matching descriptor vectors with a FLANN based matcher  -------------------------*//
    //*----------------- Since ORB is a binary-point descriptor Hamming distance is used -------------------------*//

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptorsRef, descriptorsCurr, matches, 4);

    std::vector<cv::DMatch> goodMatches;
    for(const auto m : matches)
        if(m[0].distance < m[1].distance * 0.7f)
            goodMatches.push_back(m[0]);


    //*----------------- Step 3: Selecting good matches ans compute homography  -------------------------*//
    std::vector<Point2f> refPoints2f;
    std::vector<Point2f> currentPoints2f;

    for( size_t i = 0; i < goodMatches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        refPoints2f.push_back( keypointsRef[ goodMatches[i].queryIdx ].pt );
        currentPoints2f.push_back( keypointsCurr[ goodMatches[i].trainIdx ].pt );
    }

    cv::Mat Hom = cv::findHomography( currentPoints2f, refPoints2f,  RANSAC, 1 );
    cv::cv2eigen(Hom,H);

    //std::cout << "H: " << H << std::endl;
    H = K.inverse()*H*K;
    H = (1/H(1,1))*H;
    //std::cout << "H: " << H << std::endl;

    //std::stringstream ss;
    //ss << "/home/noe/ddr_ws/src/visual_predictive/src/folder_matches/" << counter << ".jpg";

    // cv::Mat out;
    // cv::drawMatches(refImg, keypointsRef, currImg, keypointsCurr, goodMatches, out);
    // cv::imwrite(ss.str(),out);

    //cv::Mat out;
    //cv::drawKeypoints(currImg, keypointsCurr, out, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DEFAULT);
    //cv::imwrite(ss.str(),out);

    //std::cout <<"End of the EstimateHomography function... " << std::endl;

}


//==================================================================================================

// ===================== FUNCIONES AUXILIARES =========================== //

//********************************************************* */
// Returns true if the three points are not collinear
bool isNonColinear(FPTYPE x1, FPTYPE y1, FPTYPE x2, FPTYPE y2, FPTYPE x3, FPTYPE y3) {
    double area = std::abs(x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
    return area > 1e-6;
}

//********************************************************* */
void reorderVector(std::vector<double>& v, int i, int j, int k) {
    std::vector<double> tmp = {v[i], v[j], v[k]};
    for (int n = 0; n < v.size(); ++n)
        if (n != i && n != j && n != k)
            tmp.push_back(v[n]);
    v = tmp;
}


// ===================== SELECCIÓN DE TRIPLETA DE PUNTOS =================== //

void selectBestTriplet(std::vector<FPTYPE>& ref_x, std::vector<FPTYPE>& ref_y,
                       std::vector<FPTYPE>& cur_x, std::vector<FPTYPE>& cur_y,
                       const std::vector<cv::KeyPoint>& keypointsRef,
                       const std::vector<cv::DMatch>& matches,
                       std::tuple<int, int, int>& selected_idxs) {
    const int N = matches.size();
    if (N < 3) return;

    const int width = 640; // o usa refImg.cols si lo tienes disponible
    std::vector<int> left_idxs, center_idxs, right_idxs;

    for (int i = 0; i < N; ++i) {
        float x = keypointsRef[matches[i].queryIdx].pt.x;
        if (x < width / 3)
            left_idxs.push_back(i);
        else if (x < 2 * width / 3)
            center_idxs.push_back(i);
        else
            right_idxs.push_back(i);
    }

    if (left_idxs.empty() || center_idxs.empty() || right_idxs.empty()) {
        selected_idxs = {0, 1, 2};
        return;
    }

    double max_area = 0.0;
    std::tuple<int, int, int> best = {left_idxs[0], center_idxs[0], right_idxs[0]};

    for (int i : left_idxs)
        for (int j : center_idxs)
            for (int k : right_idxs)
                if (isNonColinear(ref_x[i], ref_y[i], ref_x[j], ref_y[j], ref_x[k], ref_y[k])) {
                    double area = std::abs(ref_x[i]*(ref_y[j]-ref_y[k]) +
                                           ref_x[j]*(ref_y[k]-ref_y[i]) +
                                           ref_x[k]*(ref_y[i]-ref_y[j]));
                    if (area > max_area) {
                        max_area = area;
                        best = {i, j, k};
                    }
                }

    selected_idxs = best;

    int i, j, k;
    std::tie(i, j, k) = best;

    reorderVector(ref_x, i, j, k);
    reorderVector(ref_y, i, j, k);
    reorderVector(cur_x, i, j, k);
    reorderVector(cur_y, i, j, k);
}

// ===================== VISUALIZACIÓN =================== //

void drawSelectedTriplet(const std::vector<cv::KeyPoint>& keypointsRef,
                         const std::vector<cv::DMatch>& goodMatches,
                         const std::tuple<int, int, int>& best_idxs,
                         const cv::Mat& refImg, int counter) {
    cv::Mat img_copy = refImg.clone();
    int i, j, k;
    std::tie(i, j, k) = best_idxs;

    // Extraer los puntos de la imagen
    cv::Point2f p1 = keypointsRef[goodMatches[i].queryIdx].pt;
    cv::Point2f p2 = keypointsRef[goodMatches[j].queryIdx].pt;
    cv::Point2f p3 = keypointsRef[goodMatches[k].queryIdx].pt;

    // Dibujar todos los puntos en azul
    for (const auto& m : goodMatches)
        cv::circle(img_copy, keypointsRef[m.queryIdx].pt, 2, cv::Scalar(255, 0, 0), -1);

    // Dibujar los tres puntos seleccionados en rojo
    cv::circle(img_copy, p1, 6, cv::Scalar(0, 0, 255), -1);
    cv::circle(img_copy, p2, 6, cv::Scalar(0, 0, 255), -1);
    cv::circle(img_copy, p3, 6, cv::Scalar(0, 0, 255), -1);

    // Dibujar el triángulo
    cv::line(img_copy, p1, p2, cv::Scalar(0, 255, 0), 2);
    cv::line(img_copy, p2, p3, cv::Scalar(0, 255, 0), 2);
    cv::line(img_copy, p3, p1, cv::Scalar(0, 255, 0), 2);

    std::stringstream ss;
    ss << "/home/axel/Homography_MPC/src/visual_predictive/src/areas_virtual_plane/" << counter << ".jpg";
    cv::imwrite(ss.str(), img_copy);
}

void drawSegmentedKeypoints(const cv::Mat& image,
                            const std::vector<cv::KeyPoint>& keypoints,
                            const std::string& filename) {
    cv::Mat output = image.clone();
    int width = image.cols;

    for (const auto& kp : keypoints) {
        int segment = kp.pt.x < width / 3 ? 0 :
                      (kp.pt.x < 2 * width / 3 ? 1 : 2);
        cv::circle(output, kp.pt, 3, colors[segment], -1);
    }

    cv::imwrite(filename, output);
}

void detectORBInSegments(
    const cv::Mat &img,
    std::vector<cv::KeyPoint> &keypoints_out,
    cv::Mat &descriptors_out,
    int nFeaturesPerRegion = 500)
{
    cv::Ptr<cv::ORB> detector = cv::ORB::create(nFeaturesPerRegion);
    int w = img.cols;
    int h = img.rows;

    std::vector<cv::KeyPoint> keypoints_all;
    std::vector<cv::Mat> descriptors_parts;

    // Dividir en 3 regiones verticales: izquierda, centro, derecha
    for (int i = 0; i < 3; ++i) {
        int x0 = (w / 3) * i;
        int width = (i == 2) ? (w - x0) : (w / 3);  // Asegura que el último segmento cubra todo

        cv::Rect roi(x0, 0, width, h);
        cv::Mat segment = img(roi);

        std::vector<cv::KeyPoint> keypoints_seg;
        cv::Mat descriptors_seg;

        detector->detectAndCompute(segment, cv::noArray(), keypoints_seg, descriptors_seg);

        // Ajustar coordenadas al sistema global de la imagen
        for (auto &kp : keypoints_seg)
            kp.pt.x += x0;

        // Guardar resultados
        keypoints_all.insert(keypoints_all.end(), keypoints_seg.begin(), keypoints_seg.end());
        if (!descriptors_seg.empty())
            descriptors_parts.push_back(descriptors_seg);
    }

    // Concatenar descriptores
    if (!descriptors_parts.empty())
        cv::vconcat(descriptors_parts, descriptors_out);

    keypoints_out = keypoints_all;
}

//=================================================================================================

//==================================================================================================
// ---------- Triplete de Malis rápido (sin 3 for) ----------
static inline double cross2D(const Eigen::Vector2d& a,
                             const Eigen::Vector2d& b,
                             const Eigen::Vector2d& c){
    return (b.x()-a.x())*(c.y()-a.y()) - (b.y()-a.y())*(c.x()-a.x());
}

static inline bool colinear2D(const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c,
                              double eps=1e-12){
    return std::abs(cross2D(a,b,c)) <= eps;
}

static std::vector<int> convexHullIdx(const std::vector<Eigen::Vector2d>& P){
    const int n=(int)P.size();
    std::vector<int> ord(n); std::iota(ord.begin(), ord.end(), 0);
    std::sort(ord.begin(), ord.end(), [&](int i,int j){
        if (P[i].x()==P[j].x()) return P[i].y()<P[j].y();
        return P[i].x()<P[j].x();
    });
    std::vector<int> H; H.reserve(2*n);
    auto crossIdx = [&](int a,int b,int c){ return cross2D(P[a],P[b],P[c]); };
    for(int id:ord){
        while(H.size()>=2 && crossIdx(H[H.size()-2],H[H.size()-1],id)<=0) H.pop_back();
        H.push_back(id);
    }
    size_t low=H.size();
    for(int k=n-2;k>=0;--k){
        int id=ord[k];
        while(H.size()>low && crossIdx(H[H.size()-2],H[H.size()-1],id)<=0) H.pop_back();
        H.push_back(id);
    }
    if(!H.empty()) H.pop_back();
    return H;
}

static inline long double areaProdSq(const Eigen::Vector2d& ar,
                                     const Eigen::Vector2d& br,
                                     const Eigen::Vector2d& cr,
                                     const Eigen::Vector2d& ac,
                                     const Eigen::Vector2d& bc,
                                     const Eigen::Vector2d& cc){
    const long double Aref = cross2D(ar,br,cr);
    const long double Acur = cross2D(ac,bc,cc);
    return (Aref*Aref)*(Acur*Acur);
}

std::array<int,3> selectBestMalisTripletFast(const std::vector<Eigen::Vector2d>& ref,
                                             const std::vector<Eigen::Vector2d>& cur){
    const int n=(int)ref.size();
    if(n!=(int)cur.size() || n<8) throw std::runtime_error(">=8 matches y tamaños iguales.");

    auto Href = convexHullIdx(ref);
    auto Hcur = convexHullIdx(cur);

    std::vector<int> cand = Href;
    cand.insert(cand.end(), Hcur.begin(), Hcur.end());
    std::sort(cand.begin(), cand.end());
    cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
    if((int)cand.size()<3){ cand.resize(n); std::iota(cand.begin(), cand.end(), 0); }

    std::vector<char> isCand(n,0); for(int id:cand) isCand[id]=1;
    std::vector<int> loop; loop.reserve(Href.size());
    for(int id:Href) if(isCand[id]) loop.push_back(id);
    if((int)loop.size()<3) loop = Href;

    const int h=(int)loop.size();
    if(h<3) throw std::runtime_error("Hull degenerada.");

    auto idxAt = [&](int t){ return loop[(t%h+h)%h]; };

    long double best=-1.0L; std::array<int,3> bestIdx{loop[0],loop[1],loop[2]};
    int k=2;
    for(int i=0;i<h;++i){
        int ii=idxAt(i);
        k = std::max(k, i+2);
        for(int j=i+1; j<i+h-1; ++j){
            int jj=idxAt(j);
            auto scoreAt = [&](int kk)->long double{
                int kkid=idxAt(kk);
                if (colinear2D(ref[ii],ref[jj],ref[kkid])) return -1.0L;
                if (colinear2D(cur[ii],cur[jj],cur[kkid])) return -1.0L;
                return areaProdSq(ref[ii],ref[jj],ref[kkid], cur[ii],cur[jj],cur[kkid]);
            };
            long double s0 = scoreAt(k);
            while(true){
                long double s1 = scoreAt(k+1);
                if(s1>s0){ ++k; s0=s1; } else break;
            }
            if(s0>best){ best=s0; bestIdx={ ii, jj, idxAt(k) }; }
        }
    }
    if(best<0.0L) throw std::runtime_error("No hay triplete válido (no colineal en ambas vistas).");
    return bestIdx;
}
// ---------- fin helpers ----------

double symmetricTransferError(const vpHomography& H,
                              const Eigen::Vector2d& p_cur,
                              const Eigen::Vector2d& p_ref)
{
    auto lift = [](const Eigen::Vector2d& p){ return Eigen::Vector3d(p.x(), p.y(), 1.0); };
    Eigen::Matrix3d He;
    for (int r=0;r<3;++r) for (int c=0;c<3;++c) He(r,c) = H[r][c];

    const Eigen::Vector3d m  = lift(p_cur);
    const Eigen::Vector3d mp = lift(p_ref);

    const Eigen::Vector3d Hmp   = He * mp;
    const Eigen::Vector3d Hinv_m= He.inverse() * m;

    const Eigen::Vector2d pred_in_cur = (Hmp   / Hmp(2)).head<2>();
    const Eigen::Vector2d pred_in_ref = (Hinv_m/ Hinv_m(2)).head<2>();

    const double d1 = (pred_in_cur - p_cur).squaredNorm();
    const double d2 = (pred_in_ref - p_ref).squaredNorm();
    return d1 + d2;
}

std::vector<int> filterInliersBySTE(const vpHomography& H,
                                    const std::vector<Eigen::Vector2d>& cur,
                                    const std::vector<Eigen::Vector2d>& ref,
                                    double tau_sq,
                                    size_t min_keep,
                                    size_t max_keep)
{
    const size_t n = std::min(cur.size(), ref.size());
    struct Item { int idx; double e; };
    std::vector<Item> items; items.reserve(n);
    for (size_t i=0;i<n;++i){
        const double e = symmetricTransferError(H, cur[i], ref[i]);
        if (std::isfinite(e)) items.push_back({(int)i, e});
    }
    std::sort(items.begin(), items.end(), [](const Item&a, const Item&b){ return a.e < b.e; });

    // Primero todos los < tau, garantizando un mínimo; luego cap opcional
    std::vector<int> keep; keep.reserve(std::min(max_keep, items.size()));
    for (auto &it : items) if (it.e <= tau_sq) keep.push_back(it.idx);

    if (keep.size() < min_keep) {
        // rellena con los mejores aunque superen el umbral
        for (auto &it : items) {
            if (std::find(keep.begin(), keep.end(), it.idx) == keep.end()) keep.push_back(it.idx);
            if (keep.size() >= min_keep) break;
        }
    }
    if (keep.size() > max_keep) keep.resize(max_keep);
    return keep;
}

bool sanityCheckHomography(const vpHomography& H,
                           double min_det,
                           double max_cond)
{
    Eigen::Matrix3d He;
    for (int r=0;r<3;++r) for (int c=0;c<3;++c) He(r,c) = H[r][c];

    const double det = He.determinant();
    if (!std::isfinite(det) || std::abs(det) < min_det) return false;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(He);
    const auto sv = svd.singularValues();
    if (sv(2) <= 0) return false;
    const double cond = sv(0) / sv(2);
    if (!std::isfinite(cond) || cond > max_cond) return false;

    return true;
}


//=================================================================================================

void EstimateHomographyVisp(const cv::Mat &refImg,const cv::Mat &currImg,const Matrix_t &K, vpHomography &H, const int /*counter*/ )
{
    // ---------------- 1) Detect + describe (ORB) ----------------
    const int nFeatures = 600;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nFeatures);
    std::vector<cv::KeyPoint> keypointsRef, keypointsCurr;
    cv::Mat descriptorsRef, descriptorsCurr;

    detector->detectAndCompute(refImg, cv::noArray(), keypointsRef, descriptorsRef);
    detector->detectAndCompute(currImg, cv::noArray(), keypointsCurr, descriptorsCurr);

    descriptorsRef.convertTo(descriptorsRef, CV_8U);
    descriptorsCurr.convertTo(descriptorsCurr, CV_8U);

    // ---------------- 2) Matching (Hamming + Lowe ratio) --------
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptorsRef, descriptorsCurr, matches, 4);

    std::vector<cv::DMatch> goodMatches;
    goodMatches.reserve(matches.size());
    for (const auto &m : matches) {
        if (m.size() >= 2 && m[0].distance < 0.7f * m[1].distance)
            goodMatches.push_back(m[0]);
    }
    if (goodMatches.size() < 8) {
        throw std::runtime_error("EstimateHomographyVisp: menos de 8 coincidencias válidas.");
    }

    // ---------------- 3) Normalizar a plano calibrado -----------
    const Matrix_t Kinv_ = K.inverse();

    std::vector<double> refPoints_x(goodMatches.size());
    std::vector<double> refPoints_y(goodMatches.size());
    std::vector<double> curPoints_x(goodMatches.size());
    std::vector<double> curPoints_y(goodMatches.size());

    std::vector<Eigen::Vector2d> refPts; refPts.reserve(goodMatches.size());
    std::vector<Eigen::Vector2d> curPts; curPts.reserve(goodMatches.size());

    Point2D_t desnPoint, normPoint;

    for (size_t i = 0; i < goodMatches.size(); ++i) {
        // referencia
        desnPoint(0) = keypointsRef[goodMatches[i].queryIdx].pt.x;
        desnPoint(1) = keypointsRef[goodMatches[i].queryIdx].pt.y;
        normPoint = NormalicePoint(desnPoint, Kinv_);
        refPoints_x[i] = normPoint(0);
        refPoints_y[i] = normPoint(1);

        // actual
        desnPoint(0) = keypointsCurr[goodMatches[i].trainIdx].pt.x;
        desnPoint(1) = keypointsCurr[goodMatches[i].trainIdx].pt.y;
        normPoint = NormalicePoint(desnPoint, Kinv_);
        curPoints_x[i] = normPoint(0);
        curPoints_y[i] = normPoint(1);

        refPts.emplace_back(refPoints_x[i], refPoints_y[i]);
        curPts.emplace_back(curPoints_x[i], curPoints_y[i]);
    }

    // ---------------- 4) Triplete óptimo (Malis, sin O(n^3)) ----
    const auto best3 = selectBestMalisTripletFast(refPts, curPts);

    // Helper para empaquetar índices en los vectores HLM
    std::vector<double> x_ref, y_ref, x_cur, y_cur;
    auto push_idx = [&](int idx){
        x_ref.push_back(refPoints_x[idx]); y_ref.push_back(refPoints_y[idx]);
        x_cur.push_back(curPoints_x[idx]); y_cur.push_back(curPoints_y[idx]);
    };

    // ---------------- 5) H inicial con paquete mínimo -----------
    x_ref.reserve(std::max<size_t>(8, refPts.size()));
    y_ref.reserve(x_ref.capacity());
    x_cur.reserve(x_ref.capacity());
    y_cur.reserve(x_ref.capacity());

    // 3 del triplete + completar hasta 8
    push_idx(best3[0]); push_idx(best3[1]); push_idx(best3[2]);
    for (size_t i = 0; i < refPts.size() && x_ref.size() < 8; ++i) {
        if ((int)i == best3[0] || (int)i == best3[1] || (int)i == best3[2]) continue;
        push_idx((int)i);
    }

    vpHomography H0;
    // isplanar=false → múltiples planos → homografía respecto a plano virtual
    vpHomography::HLM(x_cur, y_cur, x_ref, y_ref, /*isplanar=*/false, H0);
    H0 /= H0[1][1];

    // ---------------- 6) Filtrar inliers por STE -----------------
    // Umbral en coordenadas normalizadas (px^2). Ajusta según ruido.
    const double tau_sq = 1.0;                 // ~1 px^2
    const size_t min_keep = 12;                // asegura >8
    const size_t max_keep = 300;               // limita coste si hay miles de matches
    auto inliers = filterInliersBySTE(H0, curPts, refPts, tau_sq, min_keep, max_keep);

    if (inliers.size() < 8) {
        // Si por alguna razón el filtrado es muy estricto, usa H0 como salida segura.
        H = H0;
        return;
    }

    // ---------------- 7) Re-estimar con MUCHOS puntos ----------
    x_ref.clear(); y_ref.clear(); x_cur.clear(); y_cur.clear();
    x_ref.reserve(inliers.size()); y_ref.reserve(inliers.size());
    x_cur.reserve(inliers.size()); y_cur.reserve(inliers.size());

    for (int idx : inliers) push_idx(idx);

    vpHomography::HLM(x_cur, y_cur, x_ref, y_ref, /*isplanar=*/false, H);
    H /= H[1][1];

    // ---------------- 8) Sanity check + fallback ----------------
    if (!sanityCheckHomography(H)) {
        if (sanityCheckHomography(H0)) {
            H = H0; // fallback suave
        } // si no, dejamos H como está; el llamador puede manejar el error
    }

}


double evaluateHomographyProjection(
    const std::vector<double> &ref_x,
    const std::vector<double> &ref_y,
    const std::vector<double> &curr_x,
    const std::vector<double> &curr_y,
    const vpHomography &H)
{
    assert(ref_x.size() == curr_x.size());
    assert(ref_y.size() == curr_y.size());
    size_t N = ref_x.size();
    double error_sum = 0.0;

    std::cout << "\n--- Homography Projection Evaluation ---\n";
    for (size_t i = 0; i < N; ++i)
    {
        vpPoint p_curr, p_proj, p_ref;
        p_curr.set_x(curr_x[i]);  p_curr.set_y(curr_y[i]);  p_curr.set_Z(1);
        p_ref.set_x(ref_x[i]);    p_ref.set_y(ref_y[i]);    p_ref.set_Z(1);

        p_proj = H * p_curr;
        double x_proj = p_proj.get_x() / p_proj.get_w();
        double y_proj = p_proj.get_y() / p_proj.get_w();

        double x_ref = p_ref.get_x();
        double y_ref = p_ref.get_y();

        double error = std::sqrt((x_proj - x_ref)*(x_proj - x_ref) + (y_proj - y_ref)*(y_proj - y_ref));
        error_sum += error * error;

        std::cout << "Point " << i << ": ";
        std::cout << "ref=(" << x_ref << ", " << y_ref << "), ";
        std::cout << "proj=(" << x_proj << ", " << y_proj << "), ";
        std::cout << "error=" << error << std::endl;
    }

    double rmse = std::sqrt(error_sum / N);
    std::cout << "\nRMSE projection error: " << rmse << std::endl;
    return rmse;
}

void Rodriguez(const Matrix_t &R, Vector3D_t &u){

    cv::Mat Rcv, ucv;
    cv::eigen2cv(R,Rcv);
    cv::Rodrigues(Rcv,ucv);
    cv::cv2eigen(ucv,u);
}

void PBVSController(const Matrix_t &R, const Vector3D_t &t, const Vector3D_t &u, Vector3D_t &Uv,
                     Vector3D_t &Uw,const FPTYPE lambdav_, const FPTYPE lambdaw_)
{

    //*--------------- Linear velocity control --------------
    Uv = -lambdav_*R.transpose()*(t);
    //*--------------- Linear velocity control --------------

    //*--------------- Angular velocity control --------------
    Uw = -lambdaw_*u;
    //*--------------- Angular velocity control --------------

}

Point2D_t NormalicePoint(const Point2D_t &point, const Matrix_t &Kinv_ )
{
    
    Point3D_t normPoint = (Kinv_ * point.colwise().homogeneous());

    return normPoint.colwise().hnormalized();
}

void RecoverFromHomographyVisp(vpHomography &H, vpRotationMatrix &R, vpTranslationVector &t, vpColVector &n, vpThetaUVector &u)
{
    H.computeDisplacement(R,t,n);
    u.buildFrom(R);
}

void PBVSController(const vpRotationMatrix &R, const vpTranslationVector &t, const vpThetaUVector &u, 
                    vpColVector &Uv, vpColVector &Uw,const FPTYPE lambdav_,const FPTYPE lambdaw_)

{
        //*--------------- Linear velocity control --------------
    Uv = -lambdav_*R.t()*(t);
    //*--------------- Linear velocity control --------------

    //*--------------- Angular velocity control --------------
    Uw = -lambdaw_*u;
    //*--------------- Angular velocity control --------------

}

FPTYPE YRotationAngleFromRotationMatrix(const Matrix_t& rotation_matrix) {
        return std::atan(rotation_matrix(0, 2) / rotation_matrix(0, 0));
    }