/*

// Struct_definition.h defines different types of sturctures being used in the implementation of MSCKF

*/

#ifndef STRUCT_DEF_H

#define STRUCT_DEF_H



#include <Eigen/Dense>


#include <map>

#include <set>

#include "Parameters.hpp"

#include <deque>


#define PI 3.1415926535

#define NaN -1

typedef Eigen::Matrix<double, 15, 15> Matrix15;

typedef Eigen::Matrix<double, 16, 16> Matrix16;

typedef Eigen::Matrix<double,4,1> quaternion;



struct Feature

{

	/* ----- Structure to store Camera Feature Points ----- */

	double u;

	double v;

};



struct IMURawData

{

	Eigen::Vector3d imuData;

	double timeStamp;

};



struct IMUdata

{

	/* --------- Structure to store Raw IMU Data ---------- */

	Eigen::Vector3d a;

	Eigen::Vector3d g;



	/* -------- Operator to copy structure members -------- */

	IMUdata& operator=(const IMUdata& old)

	{

		a = old.a;

		g = old.g;



		return *this;

	}

};

struct StampedIMUData

{

	double timestamp;

	IMUdata imudata;



};



struct StampedAccData{

	double timestamp;

	Eigen::Vector3d accdata;





};



struct StampedGyroData{



	double timestamp;

	Eigen::Vector3d gyrodata;



};



struct CalibrationParameter{

	float  Td;

	CalibrationParameter()

	{

		 Td = 0;

	}

};



struct IMUstate

{

	/* ------ Structure of IMU Part of State Vector ------- */

	Eigen::Vector3d p_b_G;

	Eigen::Vector3d v_b_G;

	quaternion q_B_G;

	Eigen::Vector3d ba;

	Eigen::Vector3d bg;

	CalibrationParameter calibParam;



	Eigen::Vector3d w_b;

	/* ------------------- Constructor -------------------- */

	IMUstate()

	{

		p_b_G.setZero();

		v_b_G.setZero();

		q_B_G.setIdentity();

		ba.setZero();

		bg.setZero();

		calibParam. Td = 0;

		w_b.setZero();

	}



	/* -------- Operator to copy structure members -------- */

	IMUstate& operator=(const IMUstate& old)

	{

		p_b_G = old.p_b_G;

		v_b_G = old.v_b_G;

		q_B_G = old.q_B_G;

		ba = old.ba;

		bg = old.bg;

		w_b = old.w_b;

		calibParam = old.calibParam;



		return *this;

	}

};



struct IMUGroundTruth

{

	/* ------ Structure of IMU Part of State Vector ------- */

	double timeStamp;

	Eigen::Vector3d p_b_G;

	Eigen::Vector3d v_b_G;

	quaternion q_B_G;

	Eigen::Vector3d bias_w;

	Eigen::Vector3d bias_a;

	/* ------------------- Constructor -------------------- */

	IMUGroundTruth()

	{

		timeStamp = 0.0;

		p_b_G.setZero();

		v_b_G.setZero();

		q_B_G.setIdentity();

		bias_a.setZero();

		bias_w.setZero();

	}



	/* -------- Operator to copy structure members -------- */

	IMUGroundTruth& operator=(const IMUGroundTruth& old)

	{

		p_b_G = old.p_b_G;

		v_b_G = old.v_b_G;

		q_B_G = old.q_B_G;

		bias_a = old.bias_a;

		bias_w = old.bias_w;

		timeStamp = old.timeStamp;

		return *this;

	}

};



struct Bodystate

{

	/* --- Structure of Camera/Body Part of State Vector -- */

	int state_k;



	Eigen::Vector3d p_b_G;

	Eigen::Vector3d v_b_G;

	quaternion q_B_G;



	Eigen::Vector3d w_b;

	std::set<int> trackedFeatureID;   // TODO:: need to be modified as a set



	/* -------- Operator to copy structure members -------- */

	Bodystate& operator=(const Bodystate& old)

	{

		state_k = old.state_k;

		p_b_G = old.p_b_G;

		v_b_G = old.v_b_G;

		q_B_G = old.q_B_G;

		trackedFeatureID = old.trackedFeatureID;



		return *this;

	}



	Bodystate()

	{

		state_k = 0;

		p_b_G.setZero();

		v_b_G.setZero();

		q_B_G.setIdentity();

		trackedFeatureID.clear();

	}

};



struct Camstate{

	Eigen::Vector3d p_c_G;

	quaternion q_C_G;

	Camstate(){

		p_c_G.setZero();

		q_C_G.setIdentity();

	}

};

struct FeatureTrack

{

	/* - Structure of Features and corresponding Body States - */

	int featureID;

	std::vector<Feature> observations;

	std::map<int, std::vector<Feature>, std::less<int>, Eigen::aligned_allocator<std::pair<int, std::vector<Feature>>>> featureTracks;

	std::vector<int> bodyStateID;

	std::vector<Bodystate, Eigen::aligned_allocator<Bodystate>> bodyStateBuffer;



	/* -------- Operator to copy structure members -------- */

	FeatureTrack& operator=(const FeatureTrack& old)

	{

		featureID = old.featureID;

		observations = old.observations;

		featureTracks = old.featureTracks;

		bodyStateID = old.bodyStateID;

		bodyStateBuffer = old.bodyStateBuffer;



		return *this;

	}

	FeatureTrack(){

		featureID = 0;

		observations.clear();

		featureTracks.clear();

		bodyStateID.clear();

		bodyStateBuffer.clear();

	}

};



struct StateVector

{
	/* -------- Structure of Complete State Vector -------- */

	IMUstate imuState;

	

	Eigen::Matrix<double, 16, 16> imuCovariance; // imu + calib

	Eigen::MatrixXd bodyCovariance;

	Eigen::MatrixXd imuBodyCovariance;

	std::vector<Bodystate, Eigen::aligned_allocator<Bodystate>> bodyStateBuffer;

	//CalibrationParameter camCalibState;

	//Eigen::MatrixXd camCalibCovariance;

	/* -------- Operator to copy structure members -------- */

	StateVector& operator=(const StateVector& old)

	{

		imuState = old.imuState;

		imuCovariance = old.imuCovariance;

		bodyCovariance = old.bodyCovariance;

		imuBodyCovariance = old.imuBodyCovariance;

		bodyStateBuffer = old.bodyStateBuffer;

		//camCalibState = old.camCalibState;

		//camCalibCovariance = old.camCalibCovariance;

		return *this;

	}



	StateVector(){

		imuCovariance.setZero();

		bodyCovariance.setZero();

		imuBodyCovariance.setZero();

		//bodyStateBuffer.clear();

		//camCalibCovariance.setZero();

	}

};

struct ImageNameStruct{

	std::string imageName;

	double imageTimeStamp;

};
typedef struct IMUMeanData_{

	double gx_mean;
	double gy_mean;
	double gz_mean;

	double ax_mean;
	double ay_mean;
	double az_mean;

}IMUMeanData;

void getParameter(std::string cameParam,std::string imuParam,cameraParameters & camParam,	noiseParameters & initNoiseParam);

std::vector<ImageNameStruct> getImageList( std::ifstream& imageListFile);

std::vector<StampedIMUData> getIMUReading(std::ifstream& IMURecordFile);
std::vector<StampedIMUData> getIMUReadingEuroc(std::ifstream& IMURecordFile);

std::vector<StampedIMUData> getIMUReadingDvt(std::ifstream& IMURecordFile);

std::vector<ImageNameStruct> getImageListDvt( std::ifstream& imageListFile);

std::vector<StampedAccData> getAccReading(std::ifstream & AccRecordFile);

std::vector<StampedGyroData> getGyroReading(std::ifstream & gyroRecordFile);
std::vector<IMUGroundTruth> getGroundTruthEuroc(std::ifstream & groundTruthFile);

#endif