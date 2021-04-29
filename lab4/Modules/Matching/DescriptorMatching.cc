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

#include "DescriptorMatching.h"

using namespace std;

int HammingDistance(const cv::Mat &a, const cv::Mat &b){
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++){
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}


int searchForInitializaion(Frame& refFrame, Frame& currFrame, int th, vector<int>& vMatches, std::vector<cv::Point2f>& vPrevMatched){
    fill(vMatches.begin(),vMatches.end(),-1);

    vector<size_t> vIndicesToCheck(100);

    vector<cv::KeyPoint>& vRefKeys = refFrame.getKeyPoints();

    vector<int> vMatchedDistance(vRefKeys.size(),INT_MAX);
    vector<int> vnMatches21(vRefKeys.size(),-1);

    cv::Mat refDesc = refFrame.getDescriptors();
    cv::Mat currDesc = currFrame.getDescriptors();

    int nMatches = 0;
    const int minOctave = 0, maxOctave = 0;
    for(size_t i = 0; i < vRefKeys.size(); i++){
        //Only search matches with KeyPoints in the finest scale
        if(vRefKeys[i].octave > maxOctave) {
            continue;
        }
        /*
        * Your code for task 1 here!
        */
        //Clear previous matches
        vIndicesToCheck.clear();
        //get keypoint
        auto keypoint = vRefKeys[i];

        //Get keypoints within selected distance.
        float radius = 50;
        currFrame.getFeaturesInArea(keypoint.pt.x,keypoint.pt.y,radius,0,10,vIndicesToCheck);

        //Match with the one with the smallest Hamming distance
        int bestDist = 255, secondBestDist = 255;
        size_t bestIdx;
        for(auto j : vIndicesToCheck){
        //for(size_t j = 0; j < currDesc.size[0];j++){
            //if(currFrame.getMapPoint(j)){
            //    continue;
            //}

            int dist = HammingDistance(refDesc.row(i),currDesc.row(j));

            if(dist < bestDist){
                secondBestDist = bestDist;
                bestDist = dist;
                bestIdx = j;
            }
            else if(dist < secondBestDist){
                secondBestDist = dist;
            }
        }
        if(bestDist <= th && (float)bestDist < (float(secondBestDist)*0.9)){
            if(vnMatches21[bestIdx]>=0){
                vMatches[vnMatches21[bestIdx]]=-1;
                nMatches--;
            }

            vMatches[i] = bestIdx;
            vnMatches21[bestIdx]= i;
            nMatches++;
        }

    }

    for(size_t i = 0; i < vMatches.size(); i++){
        if(vMatches[i] != -1){
            vPrevMatched[i]=currFrame.getKeyPoint(vMatches[i]).pt;
        }
    }
    return nMatches;
}

int guidedMatching(Frame& refFrame, Frame& currFrame, int th, std::vector<int>& vMatches, int windowSizeFactor){
    currFrame.clearMapPoints();
    fill(vMatches.begin(),vMatches.end(),-1);
    vector<size_t> vIndicesToCheck(100);

    vector<shared_ptr<MapPoint>> vRefMapPoints = refFrame.getMapPoints();

    shared_ptr<CameraModel> currCamera = currFrame.getCalibration();
    Sophus::SE3f Tcw = currFrame.getPose();

    int nMatches = 0;
    int nMappoints = 0;
    for(size_t i = 0; i < vRefMapPoints.size(); i++){
        if(!vRefMapPoints[i]){
            continue;
        }
        //Project map point into the current frame
        Eigen::Vector3f p3Dcam = Tcw * vRefMapPoints[i]->getWorldPosition();
        cv::Point2f uv = currCamera->project(p3Dcam);

        //Clear previous matches
        vIndicesToCheck.clear();

        //Get last octave in which the point was seen
        int nLastOctave = refFrame.getKeyPoint(i).octave;

        //Search radius depends on the size of the point in the image
        float radius = 15 * windowSizeFactor * currFrame.getScaleFactor(nLastOctave);

        //Get candidates whose coordinates are close to the current point
        currFrame.getFeaturesInArea(uv.x,uv.y,radius,nLastOctave-1,nLastOctave+1,vIndicesToCheck);

        cv::Mat desc = vRefMapPoints[i]->getDescriptor();

        //Match with the one with the smallest Hamming distance
        int bestDist = 255, secondBestDist = 255;
        size_t bestIdx;
        for(auto j : vIndicesToCheck){
            if(currFrame.getMapPoint(j)){
                continue;
            }

            int dist = HammingDistance(desc.row(0),currFrame.getDescriptors().row(j));

            if(dist < bestDist){
                secondBestDist = bestDist;
                bestDist = dist;
                bestIdx = j;
            }
            else if(dist < secondBestDist){
                secondBestDist = dist;
            }
        }
        if(bestDist <= th && (float)bestDist < (float(secondBestDist)*0.9)){
            vMatches[i] = bestIdx;
            currFrame.setMapPoint(bestIdx,vRefMapPoints[i]);
            nMatches++;
        }
        nMappoints ++;

    }
    cout<<"Nmappoints"<<nMappoints<<endl;
    return nMatches;
}

int searchWithProjection(Frame& currFrame, int th, std::vector<std::shared_ptr<MapPoint>>& vMapPoints){
    CameraModel* currCalibration = currFrame.getCalibration().get();
    Sophus::SE3f Tcw = currFrame.getPose();
    cv::Mat currDesc = currFrame.getDescriptors();

    vector<size_t> vIndicesToCheck(100);

    int nMatches = 0, vCos = 0, invDist = 0, noClose = 0;
    for(shared_ptr<MapPoint> pMP : vMapPoints){
        //Clear previous matches
        vIndicesToCheck.clear();

        //Check normal orientation
        assert(pMP);
        Eigen::Vector3f rayWorld = pMP->getWorldPosition() - currFrame.getPose().inverse().translation();
        float viewCos = rayWorld.normalized().dot(pMP->getNormalOrientation());

        if(viewCos < 0.5){
            vCos++;
            continue;
        }

        //Check distance is in the scale invariance region of the MapPoint
        float dist = rayWorld.norm();
        float maxDistance = pMP->getMaxDistanceInvariance();
        float minDistance = pMP->getMinDistanceInvariance();

        if(dist < minDistance || dist > maxDistance){
            invDist++;
            continue;
        }

        //Predict scale
        int predictedOctave = ceil(log(maxDistance/dist)/log(currFrame.getScaleFactor(1)));
        if(predictedOctave < 0)
            predictedOctave = 0;
        else if(predictedOctave > currFrame.getNumberOfScales())
            predictedOctave = currFrame.getNumberOfScales();

        //Project MapPoint into the Frame
        Eigen::Vector3f p3Dc = Tcw*pMP->getWorldPosition();
        cv::Point2f uv = currCalibration->project(p3Dc);

        float radius = currFrame.getScaleFactor(predictedOctave);
        if(viewCos>0.998)
            radius *= 2.5;
        else
            radius *= 4.0;

        /*
         * Your matching code for Task 4 goes here
         */

        auto world_point = pMP->getWorldPosition();
        auto transformed = Tcw * world_point;
        auto pP = currCalibration->project(transformed);
        currFrame.getFeaturesInArea(pP.x,pP.y,radius,predictedOctave-1,predictedOctave+1,vIndicesToCheck);

        //Match with the one with the smallest Hamming distance
        int bestDist = 255, secondBestDist = 255;
        size_t bestIdx;
        for(auto j : vIndicesToCheck){
            //Comented ¿?
            if(currFrame.getMapPoint(j)){
                continue;
            }

            auto candidate = currDesc.row(j);
            int dist = HammingDistance(pMP->getDescriptor(),candidate);
            if(dist < bestDist){
                secondBestDist = bestDist;
                bestDist = dist;
                bestIdx = j;
            }
            else if(dist < secondBestDist){
                secondBestDist = dist;
            }
        }
        if(bestDist <= th  && (float)bestDist < (float(secondBestDist)*0.9)){
            currFrame.setMapPoint(bestIdx,pMP);
            nMatches++;
        }

    }

    //cout<<"Number of matches: "<<nMatches<<endl;



    return nMatches;
}


int searchForTriangulation(KeyFrame* kf1, KeyFrame* kf2, int th, float fEpipolarTh, Eigen::Matrix<float,3,3>& E, std::vector<int>& vMatches){
    //Clear matches
    fill(vMatches.begin(),vMatches.end(),-1);

    //Retrieve KeyFrame descriptors
    cv::Mat& desc1 = kf1->getDescriptors();
    cv::Mat& desc2 = kf2->getDescriptors();

    //Get MaPoints
    vector<shared_ptr<MapPoint>>& vMapPoints1 = kf1->getMapPoints();
    vector<shared_ptr<MapPoint>>& vMapPoints2 = kf2->getMapPoints();

    shared_ptr<CameraModel> calibration = kf1->getCalibration();

    vector<int> vMatchedDistance(vMapPoints1.size(),INT_MAX);
    vector<int> vnMatches21(vMapPoints1.size(),-1);

    vector<bool> vbMatched2(vMapPoints2.size(),false);

    int nMatches = 0;
    for(size_t i = 0; i < vMapPoints1.size(); i++){
        //Check that there is no MapPoint already triangulated
        if(vMapPoints1[i]){
            continue;
        }

        //Unrpoject KeyPoint
        Eigen::Vector3f ray1 = (E*calibration->unproject(kf1->getKeyPoint(i).pt).transpose()).normalized();

        //Search a match in the other KeyFrame
        int bestDist = 255, secondBestDist = 255;
        size_t bestIdx;
        for(size_t j = 0; j < vMapPoints2.size(); j++){
            if(vMapPoints2[j] || vbMatched2[2])
                continue;

            int dist = HammingDistance(desc1.row(i),desc2.row(j));

            if(dist > 50 || dist > bestDist)
                continue;

            if(vMatchedDistance[j] <= dist)
                continue;

            //Unproject KeyPoint
            Eigen::Vector3f ray2 = calibration->unproject(kf2->getKeyPoint(j).pt).normalized();

            //Check epipolar constrain
            float err = fabs(M_PI/2 - acos(ray2.dot(ray1)));
            if(err < fEpipolarTh){
                bestDist = dist;
                bestIdx = j;
            }
        }

        if(bestDist < th){
            if(vMapPoints2[bestIdx])
                continue;

            if(vnMatches21[bestIdx]>=0){
                vMatches[vnMatches21[bestIdx]]=-1;
                nMatches--;
            }

            vMatches[i] = bestIdx;
            vnMatches21[bestIdx]= i;
            vbMatched2[bestDist] = true;
            vMatchedDistance[bestIdx]=bestDist;
            nMatches++;
        }
    }
    //cout<<"Number of matches " << nMatches<<endl;
    return nMatches;
}

int fuse(std::shared_ptr<KeyFrame> pKF, int th, std::vector<std::shared_ptr<MapPoint>>& vMapPoints, Map* pMap){
    Sophus::SE3f Tcw = pKF->getPose();
    shared_ptr<CameraModel> calibration = pKF->getCalibration();
    cv::Mat currDesc = pKF->getDescriptors();

    vector<size_t> vIndicesToCheck(100);

    vector<shared_ptr<MapPoint>>& vKFMps = pKF->getMapPoints();

    cv::Mat descMat = pKF->getDescriptors();
    int nFused = 0;

    for(size_t i = 0; i < vMapPoints.size(); i++){
        //Clear previous matches
        vIndicesToCheck.clear();

        shared_ptr<MapPoint> pMP = vMapPoints[i];

        if(!pMP)
            continue;

        if(pMap->getNumberOfObservations(pMP->getId()) == 0)
            continue;

        if(pMap->isMapPointInKeyFrame(pMP->getId(),pKF->getId()) != -1){
            continue;
        }

        /*
         * Your code for Lab 4 - Task 3 here!
         */
        auto world_point = pMP->getWorldPosition();
        auto transformed = Tcw * world_point;
        auto pP = calibration->project(transformed);

        //Get last octave in which the point was seen
        int nLastOctave = pKF->getKeyPoint(i).octave;

        //Search radius depends on the size of the point in the image
        float radius = 15*pKF->getScaleFactor(nLastOctave);

        pKF->getFeaturesInArea(pP.x,pP.y,radius,nLastOctave-1,nLastOctave+1,vIndicesToCheck);

        //Match with the one with the smallest Hamming distance
        int bestDist = 255, secondBestDist = 255;
        size_t bestIdx;
        for(auto j : vIndicesToCheck){
            //if(pKF->getMapPoint(j)){
            //    continue;
            //}

            auto candidate = currDesc.row(j);
            int dist = HammingDistance(pMP->getDescriptor(),candidate);
            if(dist < bestDist){
                secondBestDist = bestDist;
                bestDist = dist;
                bestIdx = j;
            }
            else if(dist < secondBestDist){
                secondBestDist = dist;
            }
        }
        if(bestDist <= th && (float)bestDist < (float(secondBestDist)*0.9)){
            //No previous match.
            auto aMappoint = pKF->getMapPoints()[bestIdx];
            if (!aMappoint) {
                pMap->addObservation(pKF->getId(),pMP->getId(),bestIdx);
                pKF->setMapPoint(bestIdx, pMP);
            }
            //Previous match, fuse.
            else{
                //Alredy done in Fuse map Points, not really sure if we should do it here or
                //not, or somewhere else. Also adds observations.
                //if(pMap->isMapPointInKeyFrame(aMappoint->getId(),pKF->getId())>0)
                //   pMap->removeObservation(pKF->getId(),aMappoint->getId());
                pMap->fuseMapPoints(pMP->getId(),aMappoint->getId());
            }
            nFused++;
        }

    }

    return nFused;
}
