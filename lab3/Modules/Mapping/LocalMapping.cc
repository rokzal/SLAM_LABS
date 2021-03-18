/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Mapping/LocalMapping.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"

using namespace std;

LocalMapping::LocalMapping() {

}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    if(pCurrKeyFrame && pCurrKeyFrame->getId() != 1){
        return;
    }

    currKeyFrame_->checkAllMapPointsAreGood();
    pMap_->checkKeyFrame(currKeyFrame_->getId());

    //Triangulate new MapPoints
    triangulateNewMapPoints();
    cout << "Triangulated" << endl;

    currKeyFrame_->checkAllMapPointsAreGood();

    checkDuplicatedMapPoints();


    //Run a local Bundle Adjustment
    localBundleAdjustment(pMap_.get(),currKeyFrame_->getId());

    currKeyFrame_->checkAllMapPointsAreGood();

}

void LocalMapping::triangulateNewMapPoints() {
    //Get a list of the best covisible KeyFrames with the current one
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());

    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    //Get data from the current KeyFrame
    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    int depth1 = 0, depth2 = 0, rep1 = 0, rep2 = 0, parallax = 0;
    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(pKF->getId() == currKeyFrame_->getId())
            continue;

        //Check that baseline between KeyFrames is not too short
        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01){
            continue;
        }

        Sophus::SE3f T2w = pKF->getPose();

        Sophus::SE3f T21 = T2w*T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        //Match features between the current and the covisible KeyFrame
        //TODO: this can be further improved using the orb vocabulary
        int nMatches = searchForTriangulation(currKeyFrame_.get(),pKF.get(),settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(),E,vMatches);

        vector<cv::KeyPoint> vTriangulated1, vTriangulated2;
        vector<int> vMatches_;
        //Try to triangulate a new MapPoint with each match
        for(size_t i = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                //Get matched KeyPoints
                cv::KeyPoint kp1 = currKeyFrame_->getKeyPoint(i);
                cv::KeyPoint kp2 = pKF->getKeyPoint(vMatches[i]);

                //Check parallax between rays
                Eigen::Vector3f ray1 = calibration1->unproject(kp1.pt.x,kp1.pt.y).normalized();
                Eigen::Vector3f ray2 = calibration1->unproject(kp2.pt.x,kp2.pt.y).normalized();

                Eigen::Vector3f ray1_w = T1w.inverse().rotationMatrix() * ray1;
                Eigen::Vector3f ray2_w = T2w.inverse().rotationMatrix() * ray2;

                const float cosParallax = cosRayParallax(ray1_w,ray2_w);

                if(cosParallax < settings_.getMinCos()){
                    //Triangulate a 3D point
                    Eigen::Vector3f p3D;
                    triangulate(ray1,ray2,T1w,T2w,p3D);

                    //Check that the triangulated point lies in front of the cameras
                    Eigen::Vector3f p3D1 = T1w * p3D;
                    Eigen::Vector3f p3D2 = T2w * p3D;

                    if(p3D1(2) <= 0){
                        depth1++;
                        continue;
                    }

                    if(p3D2(2) <= 0){
                        depth2++;
                        continue;
                    }

                    //Check reprojection error
                    cv::Point2f uv1 = calibration1->project(p3D1);
                    float sigmaSquared1 = currKeyFrame_->getSigma2(kp1.octave);
                    if(squaredReprojectionError(kp1.pt,uv1) > 5.991*sigmaSquared1){
                        rep1++;
                        continue;
                    }

                    cv::Point2f uv2 = calibration1->project(p3D2);
                    float sigmaSquared2 = pKF->getSigma2(kp2.octave);
                    if(squaredReprojectionError(kp2.pt,uv2) > 5.991*sigmaSquared2){
                        rep2++;
                        continue;
                    }

                    //Check scale consistency
                    Eigen::Vector3f normal1 = p3D - currKeyFrame_->getPose().inverse().translation();
                    Eigen::Vector3f normal2 = p3D - pKF->getPose().inverse().translation();

                    float ratioDistance = normal2.norm() / normal1.norm();
                    float ratioOctave = currKeyFrame_->getScaleFactor(kp1.octave) / pKF->getScaleFactor(kp2.octave);
                    float ratioFactor = (currKeyFrame_->getNumberOfScales() > 1) ? 1.5f*currKeyFrame_->getScaleFactor(1)
                            : 1.5f;

                    if(ratioDistance*ratioFactor<ratioOctave || ratioDistance>ratioOctave*ratioFactor)
                        continue;

                    auto mp1 = currKeyFrame_->getMapPoints();
                    auto mp2 = pKF->getMapPoints();
                    assert(!mp1[i]);
                    assert(!mp2[vMatches[i]]);

                    //Triangulation is successful
                    shared_ptr<MapPoint> pMP(new MapPoint(p3D));

                    //Add the new MapPoint to the map
                    pMap_->insertMapPoint(pMP);

                    pMap_->checkKeyFrame(currKeyFrame_->getId());
                    pMap_->checkKeyFrame(pKF->getId());

                    //Add observations
                    currKeyFrame_->setMapPoint(i,pMP);
                    pKF->setMapPoint(vMatches[i],pMP);

                    pMap_->addObservation(currKeyFrame_->getId(),pMP->getId(),i);
                    pMap_->checkKeyFrame(currKeyFrame_->getId());
                    pMap_->addObservation(pKF->getId(),pMP->getId(),vMatches[i]);
                    pMap_->checkKeyFrame(pKF->getId());

                    nTriangulated++;

                    vMatches_.push_back(vTriangulated1.size());
                }
                else{
                    parallax++;
                }
            }
        }
    }
    cout << "Triangulated: " << nTriangulated << endl;
}

void LocalMapping::checkDuplicatedMapPoints() {
    vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<shared_ptr<MapPoint>> vCurrMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vKFcovisible.size(); i++){
        if(vKFcovisible[i].first == currKeyFrame_->getId())
            continue;
        int nFused = fuse(pMap_->getKeyFrame(vKFcovisible[i].first),settings_.getMatchingFuseTh(),vCurrMapPoints,pMap_.get());
        pMap_->checkKeyFrame(vKFcovisible[i].first);
        pMap_->checkKeyFrame(currKeyFrame_->getId());
    }
}
