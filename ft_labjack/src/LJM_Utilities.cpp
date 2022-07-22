#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include "LabJackM.h"
#include "../include/ft_labjack/LJM_Utilities.h"
#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "voltage_publisher_node");

	ros::NodeHandle nh;

	ros::Publisher ros_pub = nh.advertise<std_msgs::Float64MultiArray>("/ft_labjack", 10);

	ros::Rate rate(200);

	std_msgs::Float64MultiArray msg;

	int count = 1;
	int err;
	int handle;
	int i;
	int errorAddress = INITIAL_ERR_ADDRESS;

	// const double calibrationMatrixLFoot[6][6] = {
	// 	{10.33210006, 2.870928506, 35.22072861, -523.6694973, -77.00177467, 511.9011159},
	// 	{-105.743662,	605.4978365, 	26.33334729, 	-303.4779539,	39.6309077,	-298.3805753},
	// 	{886.3198703,	26.78106668,	893.893759,	29.2209897	,877.0217466,	47.63717902},
	// 	{-2.298244201,	8.507400105	,-18.31270999	,-4.996775944	,20.13146101	,-3.042080899},
	// 	{21.97888714,	0.7301285954,	-12.10184833	,6.96877549,	-9.49941147	,-7.780829137},
	// 	{1.345444842	,-8.622316201,	0.7316302626,	-8.694377615	,1.102496696	,-8.461854155}};
    const double calibrationMatrixLFoot[6][6] = {
    {3.66373, -2.53887, -56.93166, 472.72786, 55.48633, -480.00204},
    {69.76364, -550.39726, -18.98287, 268.38284, -51.71793, 281.63366},
    {-694.83314, -48.95910, -677.24401, -53.91778, -721.44018, -44.23236},
    {0.81357, -7.81489, 14.99121, 5.16497, -16.16116, 2.94277},
    {-17.34220, -1.31783, 9.88078, -5.83100, 7.85218, 7.37678},
    {-0.78857, 7.80856, -1.07929, 7.63313, -0.74397, 7.78721}};
    
	enum
	{
		NUM_FRAMES_CONFIG = 24
	};
	const char *aNamesConfig[NUM_FRAMES_CONFIG] =
		{"AIN0_NEGATIVE_CH", "AIN0_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN2_NEGATIVE_CH", "AIN2_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN4_NEGATIVE_CH", "AIN4_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN6_NEGATIVE_CH", "AIN6_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN8_NEGATIVE_CH", "AIN8_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN10_NEGATIVE_CH", "AIN10_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US"};
	const double aValuesConfig[NUM_FRAMES_CONFIG] =
		{1, 10, 8, 80,
		 3, 10, 8, 80,
		 5, 10, 8, 80,
		 7, 10, 8, 80,
		 9, 10, 8, 80 ,
		 11, 10, 8, 80};

	// Set up for reading AIN values
	enum
	{
		NUM_FRAMES_AIN = 6
	};
	double aValuesAIN[NUM_FRAMES_AIN] = {0};
	const char *aNamesAIN[NUM_FRAMES_AIN] =
		{"AIN0",
		 "AIN2",
		 "AIN4",
		 "AIN6",
		 "AIN8",
		 "AIN10"};

	// Open first found LabJack
	handle = OpenOrDie(LJM_dtANY, LJM_ctANY, "LJM_idANY");
	// handle = OpenSOrDie("LJM_dtANY", "LJM_ctANY", "LJM_idANY");

	PrintDeviceInfoFromHandle(handle);

	// Setup and call eWriteNames to configure AINs on the LabJack.
	err = LJM_eWriteNames(handle, NUM_FRAMES_CONFIG, aNamesConfig, aValuesConfig, &errorAddress);
	ErrorCheckWithAddress(err, errorAddress, "LJM_eWriteNames");

	printf("\nSet configuration:\n");
	for (i = 0; i < NUM_FRAMES_CONFIG; i++)
	{
		printf("    %s : %f\n", aNamesConfig[i], aValuesConfig[i]);
	}

	while (ros::ok())
	{
		err = LJM_eReadNames(handle, NUM_FRAMES_AIN, aNamesAIN, aValuesAIN, &errorAddress);
		ErrorCheckWithAddress(err, errorAddress, "LJM_eReadNames");

		double _lf = 0.0;
		double leftFootAxisData[6];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				_lf += calibrationMatrixLFoot[i][j] * aValuesAIN[j];
			}

            if(initalize_flag_ == true){
                
                leftFootAxisData_temp[i] = _lf;
            }
            std::cout << " v_curr : " << _lf << std::endl;
            _lf = _lf - leftFootAxisData_temp[i];
			leftFootAxisData[i] = _lf;
			msg.data.push_back(_lf);
		}
        tick_count += 1;
        if(tick_count == 5){
            initalize_flag_ = false;
        }
        
        std::cout << " v_init : " << leftFootAxisData_temp[0] << " v_init : " << leftFootAxisData_temp[1] << " v_init : " << leftFootAxisData_temp[2] << " v_init : " << leftFootAxisData_temp[3] << " v_init : " << leftFootAxisData_temp[4] << " v_init : " << leftFootAxisData_temp[5] << std::endl;
        
        
        
		ros_pub.publish(msg);
		rate.sleep();
		msg.data.erase(msg.data.begin(), msg.data.end());
		std::cout << " v0 : " << leftFootAxisData[0] << " v0 : " << leftFootAxisData[1] << " v0 : " << leftFootAxisData[2] << " v0 : " << leftFootAxisData[3] << " v0 : " << leftFootAxisData[4] << " v0 : " << leftFootAxisData[5] << std::endl;
        // std::cout << " v0 : " << aValuesAIN[0]<< " v0 : " <<aValuesAIN[1] << " v0 : " << aValuesAIN[2] << " v0 : " << aValuesAIN[3] << " v0 : " << aValuesAIN[4] << " v0 : " << aValuesAIN[5] << std::endl;
	}

	CloseOrDie(handle);

	WaitForUserIfWindows();

	return 0;
}
