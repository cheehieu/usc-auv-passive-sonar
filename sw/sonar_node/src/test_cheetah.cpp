#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <cstring>
#include <time.h>
#include "sonar/cheetah.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
//#include <csignal>

//Writes ADC data into a text file
void outputFile(long int data_array[], int length, int adc_id)
{
	std::ostringstream oss_filename;
	oss_filename << adc_id << "_adc_samples.txt";
	const std::string& s_filename = oss_filename.str();
	const char* filename = s_filename.c_str();
	
	std::ofstream file (filename);
	if ( file.is_open() )
	{
		for (int i=0 ; i<length ; i++)
		{
			file << data_array[i] << ",";
		}
		file.close();
	}
	else	std::cout << "Error opening file!" << std::endl;
	std::cout << "Finished writing data from ADC" << adc_id << std::endl;
	return;
}

// Globals
const int    NUM_SAMPLES	   = 1000000;
const int    DATA_BLOCK_SIZE       = NUM_SAMPLES * 3; //#of ADCs

// Declare Cheetah Variables
Cheetah    handle;
int        port, bitrate;
uint8_t    mode = 0;
int        ret, input, ready_bit, valid_data_point;

// Raw Data
const int  TX_LENGTH = DATA_BLOCK_SIZE * 2;
uint8_t    data_in[TX_LENGTH];
uint8_t    data_out[2];

// Data Arrays
long int   data1[TX_LENGTH/3];
long int   data2[TX_LENGTH/3];
long int   data3[TX_LENGTH/3];


int main (int argc, char *argv[]) {
  if (argc < 2) {
    ROS_ERROR("sonar_node usage: sudo rosrun sonar_node sonar_node PORT [Freq 1] [Freq 1] [Enable debug]\n");
    return 1;
  }

  ros::init(argc, argv, "find_pingers"); 
  ros::NodeHandle n("~");
  
  // Initialize SIGINT Handler
  //signal(SIGINT, sigintHandler);
 
  // Initialize Cheetah connection parameters
  port = atoi(argv[1]);
  bitrate = 40000;	//10000kHz = 10MHz

  // Open the device
  handle = ch_open(port);
  if (handle <= 0) {
    ROS_ERROR("Unable to open Cheetah device on port %d (Error code = %d: %s)", port, handle, ch_status_string(handle));
    exit(1);
  }
  ROS_INFO("Opened Cheetah device on port %d", port);
  ROS_INFO("Host interface is %s", (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

  // Ensure that the SPI subsystem is configured.
  //ch_spi_configure(handle, CheetahSpiPolarity(mode >> 1), CheetahSpiPhase(mode & 1), CH_SPI_BITORDER_MSB, 0x0);
  ch_spi_configure(handle, CH_SPI_POL_RISING_FALLING, CH_SPI_PHASE_SETUP_SAMPLE, CH_SPI_BITORDER_MSB, 0x0);
  ROS_INFO("SPI configuration set to mode %d, %s shift, SS[2:0] active low", mode, "MSB");

  // Power the target using the Cheetah adapter's power supply.
  ch_target_power(handle, CH_TARGET_POWER_ON);
  ch_sleep_ms(100);

  // Set the bitrate.
  bitrate = ch_spi_bitrate(handle, bitrate);
  ROS_INFO("Bitrate set to %d kHz", bitrate);

  // Make a simple queue to just assert OE.
  ch_spi_queue_clear(handle);
  ch_spi_queue_oe(handle, 1);
  ROS_INFO("Pre 0 batch shift...");
  ch_spi_batch_shift(handle, 0, 0);	//what is this for?

  // Queue the batch, which is a sequence of SPI packets (back-to-back) each of length 2.
  ROS_INFO("Beginning to queue SPI packets...");
  data_out[0] = 0xff;
  data_out[1] = 0xff;
  ch_spi_queue_clear(handle);
  
  for (int i = 0; i < DATA_BLOCK_SIZE; ++i) {
    // Convert Slave 1
    ch_spi_queue_ss(handle, 0xF);
    ch_spi_queue_array(handle, 2, data_out);
    ch_spi_queue_ss(handle, 0xE);

    // Convert Slave 2
    ch_spi_queue_ss(handle, 0xF);
    ch_spi_queue_array(handle, 2, data_out);
    ch_spi_queue_ss(handle, 0xD);

    // Convert Slave 3
    ch_spi_queue_ss(handle, 0xF);
    ch_spi_queue_array(handle, 2, data_out);
    ch_spi_queue_ss(handle, 0xB);
  }

  ROS_INFO("Done queueing packets");
  
  // Submit the first batch
  ch_spi_async_submit(handle);

  //while (ros::ok()) {

    ch_spi_async_submit(handle);
    
    ret = ch_spi_async_collect(handle, TX_LENGTH, data_in);
    ROS_INFO("ASYNC_DATA_COLLECT returns: %d bytes", ret);

    // Post processing
    int data_idx = 0;
    for (int j = 0; j < TX_LENGTH; j += 6) {
      // SS3 Data
      input = (data_in[j] << 8) + data_in[j+1];
      valid_data_point = (input & 0x3ffc) >> 2;
      if(valid_data_point >= 0x0800)	valid_data_point = valid_data_point - 0x1000;	//convert 2's comp to signed
      data3[data_idx] = valid_data_point;

      // SS1 Data
      input = (data_in[j+2] << 8) + data_in[j+3];
      valid_data_point = (input & 0x3ffc) >> 2;
      if(valid_data_point >= 0x0800)	valid_data_point = valid_data_point - 0x1000;	//convert 2's comp to signed
      data1[data_idx] = valid_data_point;

      // SS2 Data
      input = (data_in[j+4] << 8) + data_in[j+5];
      valid_data_point = (input & 0x3ffc) >> 2;
      if(valid_data_point >= 0x0800)	valid_data_point = valid_data_point - 0x1000;	//convert 2's comp to signed
      data2[data_idx] = valid_data_point;
      ++data_idx;
    }

    outputFile(data1, NUM_SAMPLES, 1);
    outputFile(data2, NUM_SAMPLES, 2);
    outputFile(data3, NUM_SAMPLES, 3);

    ros::spinOnce();
  //}

  return 0;
}

