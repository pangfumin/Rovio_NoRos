/*
// Parameters.h defines Parameters of the hardware/Simulator which are not direct part of MSCKF Algorithm
*/
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

struct noiseParameters
{
	// define noise params	-> noiseParams

	double u_var_prime;
	double v_var_prime;
	/*
	double sigma_gc;					//rot vel var
	double sigma_ac;					//lin accel var
	double sigma_wgc;				//gyro bias change var
	double sigma_wac;				//accel bias change var
	*/
	Eigen::Vector3d var_gc;
	Eigen::Vector3d var_ac;
	Eigen::Vector3d var_wgc;
	Eigen::Vector3d var_wac;


	Eigen::Vector3d aBias;
	Eigen::Vector3d gBias;

	Eigen::Vector3d aScale;
	Eigen::Vector3d gScale;

	noiseParameters& operator=(const noiseParameters& old){
		u_var_prime = old.u_var_prime;
		v_var_prime = old.v_var_prime;
		var_ac = old.var_ac;
		var_gc = old.var_gc;
		var_wgc = old.var_wgc;
		var_wac = old.var_wac;
		return *this;
	};
};

struct msckfParameters
{
	// define msckf params	-> msckfParams

	int minTrackLength;				// Set to inf to dead-reckon only
	int maxTrackLength;				// Set to inf to wait for features to go out of view
	double maxGNCostNorm;			// Set to inf to allow any triangulation, no matter how bad
	double minRCOND;
	bool doNullSpaceTrick;
	bool doQRdecomp;
	double fDistanceMin ;
    double fDistanceMax ;
};

struct cameraParameters
{
	// define camera params	-> camera

	unsigned int width;
	unsigned int height;

	double c_u;						// Principal point [u pixels] 
	double c_v;						// Principal point [v pixels]
	double f_u;						// Focal length [u pixels]
	double f_v;						// Focal length [v pixels]
	
	double w;

	double Wc0;
	double Wc1;
	double Wc2;

	double Tc0;
	double Tc1;
	double Tc2;

	Eigen::Matrix3d R_C_B;
	Eigen::Vector3d Bp_c0;
	cv::Mat  distortCoeff;

	cameraParameters &operator=(const cameraParameters &old){
		c_u = old.c_u;
		c_v = old.c_v;
		f_u = old.f_u;
		f_v = old.f_v;
		w = old.w;
		R_C_B = old.R_C_B;
		Bp_c0 = old.Bp_c0;
		Wc0 = old.Wc0;
		Wc1 = old.Wc1;
		Wc2 = old.Wc2;
		Tc0 = old.Tc0;
		Tc1 = old.Tc1;
		Tc2 = old.Tc2;
		return *this;
	};

};

struct IMUCalibrationParameters
{
	// define IMU Calibration params -> 

	Eigen::Matrix3d Ta;
	Eigen::Matrix3d Tg;
	Eigen::Matrix3d Ts;
};

#endif