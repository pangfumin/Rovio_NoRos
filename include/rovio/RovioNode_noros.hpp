/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <rovio/Struct_definition.hpp>

#include <iostream>
#include <sstream>
#include "rovio/RovioFilter.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode_noros{
 public:
  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
  typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  mtPoseMeas poseUpdateMeas_;
  mtPoseUpdate* mpPoseUpdate_;

  struct FilterInitializationState {
    FilterInitializationState()
        : WrWM_(V3D::Zero()),
          state_(State::WaitForInitUsingAccel) {}

    enum class State {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const {
      return isInitialized();
    }

    bool isInitialized() const {
      return (state_ == State::Initialized);
    }
  };
  FilterInitializationState init_state_;

  bool forceOdometryPublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;
  std::mutex m_filter_;

  
  int msgSeq_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput cameraOutput_;
  MXD cameraOutputCov_;
  mtOutput imuOutput_;
  MXD imuOutputCov_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::LandmarkOutput landmarkOutput_;
  MXD featureOutputCov_;
  MXD landmarkOutputCov_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  MXD featureOutputReadableCov_;

  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;
  
  
  std::thread consumerThread_;
  int argc_;
  char** argv_;
 
  /** \brief Constructor
   */
  RovioNode_noros(int argc, char** argv,std::shared_ptr<mtFilter> mpFilter )
      : argc_(argc),argv_(argv),mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_), landmarkOutputImuCT_(&mpFilter->multiCamera_),
        cameraOutputCov_((int)(mtOutput::D_),(int)(mtOutput::D_)), featureOutputCov_((int)(FeatureOutput::D_),(int)(FeatureOutput::D_)), landmarkOutputCov_(3,3),
        featureOutputReadableCov_((int)(FeatureOutputReadable::D_),(int)(FeatureOutputReadable::D_)){
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
    mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
    forceOdometryPublishing_ = false;
    forceTransformPublishing_ = false;
    forceExtrinsicsPublishing_ = false;
    forceImuBiasPublishing_ = false;
    forcePclPublishing_ = false;
    forceMarkersPublishing_ = false;
    forcePatchPublishing_ = false;
    gotFirstMessages_ = false;
    
    // start loopping
    consumerThread_ = std::thread(&RovioNode_noros::Loop, this);
  
  }

  /** \brief Destructor
   */
  virtual ~RovioNode_noros(){}

 

 
  void imuCallback(StampedIMUData  imu_msg){
    std::lock_guard<std::mutex> lock(m_filter_);
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg.imudata.a(0),imu_msg.imudata.a(1),imu_msg.imudata.a(2));
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg.imudata.g(0),imu_msg.imudata.g(1),imu_msg.imudata.g(2));
    if(init_state_.isInitialized()){
      mpFilter_->addPredictionMeas(predictionMeas_,imu_msg.timestamp);
      updateAndPublish();
    } else {
      switch(init_state_.state_) {
        case FilterInitializationState::State::WaitForInitExternalPose: {
          std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
          mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, imu_msg.timestamp);
          break;
        }
        case FilterInitializationState::State::WaitForInitUsingAccel: {
          std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
          mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),imu_msg.timestamp);
          break;
        }
        default: {
          std::cout << "Unhandeld initialization type." << std::endl;
          abort();
          break;
        }
      }

      //std::cout << std::setprecision(12);
      std::cout << "-- Filter: Initialized at t = " << imu_msg.timestamp << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
  }
  

  /** \brief Image callback for the camera with ID 0
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback0( cv::Mat & img,double & t){
    std::lock_guard<std::mutex> lock(m_filter_);
    imgCallback(img,t);
  }

  void imgCallback(cv::Mat & img,double & t){
 
    cv::Mat cv_img = img.clone();
   
    if(init_state_.isInitialized() && !cv_img.empty()){
      double msgTime = t;
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        for(int i=0;i<mtState::nCam_;i++){
          if(imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]){
            std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
          }
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[0].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[0] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
        updateAndPublish();
      }
    }
  }

 

  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    if(init_state_.isInitialized()){
      // Execute the filter update.
      const double t1 = (double) cv::getTickCount();
      static double timing_T = 0;
      static int timing_C = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      double lastImageTime;
      if(std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)){
        mpFilter_->updateSafe(&lastImageTime);
      }
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;
      bool plotTiming = false;
      if(plotTiming){
        //ROS_INFO_STREAM(" == Filter Update: " << (t2-t1)/cv::getTickFrequency()*1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C);
      }
      if(mpFilter_->safe_.t_ > oldSafeTime){ // Publish only if something changed
        for(int i=0;i<mtState::nCam_;i++){
          if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_){
            cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
            cv::waitKey(3);
          }
        }
        if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_){
          cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
          cv::waitKey(3);
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
        mtState& state = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        imuOutputCT_.transformState(state,imuOutput_);

        // Cout verbose for pose measurements
        if(mpImgUpdate_->verbose_){
          if(mpPoseUpdate_->inertialPoseIndex_ >=0){
            std::cout << "Transformation between inertial frames, IrIW, qWI: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_) << std::endl;
          }
          if(mpPoseUpdate_->bodyPoseIndex_ >=0){
            std::cout << "Transformation between body frames, MrMV, qVM: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_) << std::endl;
          }
        }

      
        gotFirstMessages_ = true;
      }
    }
  }
  
  
  void Loop()
  {
       
      std::string datasetBase = std::string(argv_[1]);
      
  
      
      std::string imgBasePath = datasetBase + "cam0/data/";
      std::string strTimeStampFile = datasetBase + "cam0/data.txt";
      std::string strImuFile = datasetBase + "imu0/data.txt";
      std::string strGroundTruthFile = datasetBase + "state_groundtruth_estimate0/data.txt";
      std::ifstream timestampfile(strTimeStampFile);
      std::vector<ImageNameStruct> imageList = getImageList(timestampfile);
      timestampfile.close();
      
     
      
      std::ifstream imuFile(strImuFile);
      std::vector<StampedIMUData> imuList;
      imuList = getIMUReadingEuroc(imuFile);
      imuFile.close();
      //std::cout<<imuList.at(0).imudata.a<<std::endl;
      
      
      // groundtruth
      std::ifstream groundTruthFile(strGroundTruthFile);
      std::vector<IMUGroundTruth> groundTruthList;
      groundTruthList = getGroundTruthEuroc(groundTruthFile);
      groundTruthFile.close();
      std::cout<<groundTruthList.size()<<std::endl;
      
      unsigned int imageStart ;
      unsigned int imageEnd ;
      
      std::string s = std::string(argv_[2]);
      std::stringstream ss;
      ss<<s;
      ss>>imageStart;
      
      std::string s1 = std::string(argv_[3]);
      std::stringstream ss1;
      ss1<<s1; 
      ss1>>imageEnd;
      
     
      
	
      double image_t =  imageList.at(imageStart).imageTimeStamp;
      unsigned int imuStart= 0;	
      unsigned int groundTruthStart = 0;

      for (size_t i = 0; i < imuList.size() && imuList.at(i).timestamp < image_t; i++)
	imuStart = i;
    
		   
      std::cout<<"imuStart: "<<imuStart<<std::endl;
      std::cout<<"groundTruthStart: "<<groundTruthStart<<std::endl;
      
      unsigned int imuCnt  = imuStart;
      unsigned int groundTruthCnt = groundTruthStart;
      for (unsigned int imageCnt = imageStart+1; imageCnt < imageEnd; imageCnt ++)
      {
	while( groundTruthList.at(groundTruthCnt).timeStamp < imageList.at(imageCnt).imageTimeStamp )
	    groundTruthCnt ++;
	
	
	
	
	while(imuList.at(imuCnt).timestamp < imageList.at(imageCnt).imageTimeStamp)
	{
	 
	  imuCallback(imuList.at(imuCnt));
	 
	  imuCnt ++;
	  cv::waitKey(3);
	}
	
	std::string  imageName = imgBasePath + imageList.at(imageCnt).imageName;
	double t = imageList.at(imageCnt).imageTimeStamp;
	cv::Mat image = cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);
	imgCallback0(image,t);

      }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
