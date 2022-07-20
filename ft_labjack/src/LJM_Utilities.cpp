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

	ros::Rate rate(1);

	std_msgs::Float64MultiArray msg;

	int count = 1;

	int err;
	int handle;
	int i;
	int errorAddress = INITIAL_ERR_ADDRESS;

	const double calibrationMatrixLFoot[6][6] = {
		{-0.74957, 0.24281, -23.71506, 466.60093, 18.22998, -468.17685},
		{7.19002, -546.10473, -6.38277, 266.54205, -11.70446, 277.80902},
		{-695.34548, -16.86802, -637.23056, 11.25426, -777.28613, 38.67756},
		{-0.77968, -7.75893, 14.91040, 3.51086, -16.01169, 4.68867},
		{-17.44361, -0.36882, 8.83623, -6.62325, 9.17076, 6.19479},
		{-0.31783, 7.67933, -0.32218, 7.62097, -0.22801, 7.95418}};

	enum
	{
		NUM_FRAMES_CONFIG = 24
	};
	const char *aNamesConfig[NUM_FRAMES_CONFIG] =
		{"AIN0_NEGATIVE_CH", "AIN0_RANGE", "AIN0_RESOLUTION_INDEX", "AIN0_SETTLING_US",
		 "AIN2_NEGATIVE_CH", "AIN2_RANGE", "AIN2_RESOLUTION_INDEX", "AIN2_SETTLING_US",
		 "AIN4_NEGATIVE_CH", "AIN4_RANGE", "AIN4_RESOLUTION_INDEX", "AIN4_SETTLING_US",
		 "AIN6_NEGATIVE_CH", "AIN6_RANGE", "AIN6_RESOLUTION_INDEX", "AIN6_SETTLING_US",
		 "AIN8_NEGATIVE_CH", "AIN8_RANGE", "AIN8_RESOLUTION_INDEX", "AIN8_SETTLING_US",
		 "AIN10_NEGATIVE_CH", "AIN10_RANGE", "AIN10_RESOLUTION_INDEX", "AIN10_SETTLING_US"};
	const double aValuesConfig[NUM_FRAMES_CONFIG] =
		{1, 10, 0, 0,
		 3, 10, 0, 0,
		 5, 10, 0, 0,
		 7, 10, 0, 0,
		 9, 10, 0, 0,
		 11, 10, 0, 0};

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
			leftFootAxisData[i] = _lf;
			msg.data.push_back(_lf);
		}

		ros_pub.publish(msg);
		rate.sleep();
		msg.data.erase(msg.data.begin(), msg.data.end());
		std::cout << " v0 : " << leftFootAxisData[0] << " v0 : " << leftFootAxisData[1] << " v0 : " << leftFootAxisData[2] << " v0 : " << leftFootAxisData[3] << " v0 : " << leftFootAxisData[4] << " v0 : " << leftFootAxisData[5] << std::endl;
	}

	CloseOrDie(handle);

	WaitForUserIfWindows();

	return 0;
}
