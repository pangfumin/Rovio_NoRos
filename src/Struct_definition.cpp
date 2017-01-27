#include "rovio/Struct_definition.hpp"

#include <iostream>

#include <fstream>

#include <opencv2/core/core.hpp>



void getParameter(std::string cameParam,std::string imuParam,cameraParameters & camParam,	noiseParameters & initNoiseParam)

{



	cv::FileStorage fs(cameParam,cv::FileStorage::READ);



	cv::Mat T_BS, intrinsics, distortion_coefficients,resolution;

	fs["T_BS"] >> T_BS;

	//std::cout<<T_BS<<std::endl;

	Eigen::Matrix3d R_B_C;

	R_B_C<< T_BS.at<double>(0,0),T_BS.at<double>(0,1),T_BS.at<double>(0,2),

		T_BS.at<double>(1,0),T_BS.at<double>(1,1),T_BS.at<double>(1,2),

		T_BS.at<double>(2,0),T_BS.at<double>(2,1),T_BS.at<double>(2,2);

	camParam.R_C_B = R_B_C.transpose();



	camParam.Bp_c0 <<T_BS.at<double>(0,3),T_BS.at<double>(1,3),T_BS.at<double>(2,3);







	fs["intrinsics"] >> intrinsics;

	std::cout<<intrinsics<<std::endl;

	camParam.f_u = intrinsics.at<double>(0);

	camParam.f_v = intrinsics.at<double>(1);

	camParam.c_u = intrinsics.at<double>(2);

	camParam.c_v = intrinsics.at<double>(3);

	





	fs["distortion_coefficients"] >> distortion_coefficients;

	std::cout<<distortion_coefficients<<std::endl;

	camParam.distortCoeff = distortion_coefficients;



	fs["resolution"] >> resolution;

	std::cout<<resolution<<std::endl;

	camParam.width = 752;

	camParam.height = 480;

	





	cv::FileStorage fs1(imuParam,cv::FileStorage::READ);

	Eigen::Vector3d I(1,1,1);

	

	double gyroscope_noise_density;

	fs1["gyroscope_noise_density"] >> gyroscope_noise_density;

	std::cout<<gyroscope_noise_density<<std::endl; 

	initNoiseParam.var_gc = gyroscope_noise_density*gyroscope_noise_density*I;



	double gyroscope_random_walk;

	fs1["gyroscope_random_walk"] >> gyroscope_random_walk;

	std::cout<<gyroscope_random_walk<<std::endl; 

	initNoiseParam.var_wgc = gyroscope_random_walk*gyroscope_random_walk*I;



	double accelerometer_noise_density;

	fs1["accelerometer_noise_density"] >> accelerometer_noise_density;

	std::cout<<accelerometer_noise_density<<std::endl; 

	initNoiseParam.var_ac = accelerometer_noise_density*accelerometer_noise_density*I;



	double accelerometer_random_walk;

	fs1["accelerometer_random_walk"] >> accelerometer_random_walk;

	initNoiseParam.var_wac = accelerometer_random_walk*accelerometer_random_walk*I;





	initNoiseParam.u_var_prime = (8/camParam.f_u)*(8/camParam.f_u);

	initNoiseParam.v_var_prime = initNoiseParam.u_var_prime;



}





std::vector<ImageNameStruct> getImageList( std::ifstream& imageListFile)

{

	std::string filename;

	std::string line;

	std::string timestamp;

	double time;

	std::vector<ImageNameStruct> imageNameList;

	while(std::getline(imageListFile,line)&& !line.empty())

	{



		std::istringstream iss(line);



		iss >> timestamp >> filename;

		time = std::stod(timestamp);

		//std::cout<<filename<<" "<<time<<std::endl;

		time = time * 1e-9;



		ImageNameStruct imageNameStruct;

		imageNameStruct.imageName = filename;

		imageNameStruct.imageTimeStamp = time;

		imageNameList.push_back(imageNameStruct);

			

	

	}

	return imageNameList;

}



std::vector<ImageNameStruct> getImageListDvt( std::ifstream& imageListFile)

{

	std::string filename;

	std::string line;

	std::string timestamp;

	double time;

	std::vector<ImageNameStruct> imageNameList;

	while(std::getline(imageListFile,line)&& !line.empty())

	{



		std::istringstream iss(line);



		iss >> filename >> timestamp;



		//std::cout<< timestamp<<" "<<filename<<std::endl;



		time = std::stod(timestamp);

		time = time * 1e-6; // [s]

		ImageNameStruct imageNameStruct;

		imageNameStruct.imageName = filename;

		imageNameStruct.imageTimeStamp = time;

		imageNameList.push_back(imageNameStruct);

			

	

	}

	return imageNameList;

}



std::vector<StampedIMUData> getIMUReading(std::ifstream& IMURecordFile)

{



	

	std::string gx;

	std::string gy;

	std::string gz;

	std::string ax;

	std::string ay;

	std::string az;



	std::string line;

	std::string timestamp;

	std::vector<StampedIMUData> imuReadingList;

	while(std::getline(IMURecordFile,line)&& !line.empty())

	{



		std::istringstream iss(line); 

		

		iss>>timestamp>>ax>>ay>>az>>gx>>gy>>gz;

		double time = std::stod(timestamp);

		time = time * 1e-9;

		//std::cout<<"IMU timestamp: "<<time<<std::endl;



		StampedIMUData imuReadingStruct;

		imuReadingStruct.timestamp= time;

		imuReadingStruct.imudata.a<<stod(ax),stod(ay),stod(az);

		imuReadingStruct.imudata.g<<stod(gx),stod(gy),stod(gz);



		



		imuReadingList.push_back(imuReadingStruct);

			

	

	}

	return imuReadingList;



}







std::vector<StampedIMUData> getIMUReadingDvt(std::ifstream& IMURecordFile)

{



	

	std::string gx;

	std::string gy;

	std::string gz;

	std::string ax;

	std::string ay;

	std::string az;



	std::string line;

	std::string timestamp;

	std::vector<StampedIMUData> imuReadingList;

	while(std::getline(IMURecordFile,line)&& !line.empty())

	{





		std::istringstream iss(line); 

        std::getline(iss, timestamp, ',');

		

		double time = std::stod(timestamp);

		time = time* 1e-6;

		StampedIMUData imuReadingStruct;

		imuReadingStruct.timestamp= time;

		

		std::string s;

        for (int j = 0; j < 3; ++j) {

          std::getline(iss, s, ',');

          imuReadingStruct.imudata.a[j] = std::stof(s);

        }



		 for (int j = 0; j < 3; ++j) {

          std::getline(iss, s, ',');

          imuReadingStruct.imudata.g[j] = std::stof(s);

        }



        /*

		std::cout<< timestamp<<" "<<imuReadingStruct.imudata.a[0]<< " "<<imuReadingStruct.imudata.a[1]<<" "<<imuReadingStruct.imudata.a[2]

		<< imuReadingStruct.imudata.g[0]<< " "<<imuReadingStruct.imudata.g[1]<<" "<<imuReadingStruct.imudata.g[2]<<std::endl;

		*/



		imuReadingList.push_back(imuReadingStruct);

			

	

	}

	return imuReadingList;



}







std::vector<StampedAccData> getAccReading(std::ifstream & AccRecordFile)

{

	std::string ax;

	std::string ay;

	std::string az;



	std::string line;

	std::string timestamp;

	std::vector<StampedAccData> AccReadingList;

	while(std::getline(AccRecordFile,line)&& !line.empty())

	{



		std::istringstream iss(line); 

        std::getline(iss, timestamp, ',');

		

		double time = std::stod(timestamp);

		time = time* 1e-6;

		StampedAccData accReadingStruct;

		accReadingStruct.timestamp= time;

		

		std::string s;

        for (int j = 0; j < 3; ++j) {

          std::getline(iss, s, ',');

          accReadingStruct.accdata[j] = std::stof(s);

        }



		//std::cout<< timestamp<<" "<<accReadingStruct.accdata[0]<< " "<<accReadingStruct.accdata[1]<<" "<<accReadingStruct.accdata[2]<<std::endl;

	

		AccReadingList.push_back(accReadingStruct);

	}

	return AccReadingList;



}





std::vector<StampedGyroData> getGyroReading(std::ifstream & gyroRecordFile)

{

	std::string gx;

	std::string gy;

	std::string gz;



	std::string line;

	std::string timestamp;

	std::vector<StampedGyroData> GyroReadingList;

	while(std::getline(gyroRecordFile,line)&& !line.empty())

	{



		

		std::istringstream iss(line); 

        std::getline(iss, timestamp, ',');

		

		double time = std::stod(timestamp);

		time = time* 1e-6;

		StampedGyroData gyroReadingStruct;

		gyroReadingStruct.timestamp= time;

		

		std::string s;

        for (int j = 0; j < 3; ++j) {

          std::getline(iss, s, ',');

          gyroReadingStruct.gyrodata[j] = std::stof(s);

        }



	    //std::cout<< timestamp<<" "<< gyroReadingStruct.gyrodata[0]<< " "<< gyroReadingStruct.gyrodata[1]<<" "<< gyroReadingStruct.gyrodata[2]<<std::endl;

		GyroReadingList.push_back(gyroReadingStruct);

	}

	return GyroReadingList;



}



