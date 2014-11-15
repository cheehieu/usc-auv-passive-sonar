#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "cheetah.h"

#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif

static s64 _timeMillis () {
#ifdef _WIN32
    return ((s64)clock()) * 1000 / CLOCKS_PER_SEC;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return ((s64)tv.tv_sec * 1000L) + (s64)(tv.tv_usec / 1000L);
#endif
}


// Globals
const int 	NUM_SAMPLES		= 1000000;
const int 	DATA_BLOCK_SIZE		= NUM_SAMPLES * 3; //#of ADCs
const int  	TX_LENGTH = DATA_BLOCK_SIZE * 2;
uint8_t    	data_in[TX_LENGTH]; //raw data
uint8_t    	data_out[2];
long int   	data1[TX_LENGTH/3];
long int   	data2[TX_LENGTH/3];
long int   	data3[TX_LENGTH/3];

// Declare Cheetah Variables
Cheetah 	handle;
int 		port, bitrate;
uint8_t 	mode = 0;
int 		ret, input, ready_bit, valid_data_point;



int main( int argc, char ** argv )
{
	ros::init( argc, argv, "sonar_driver" );
	if (argc < 3) {
		ROS_ERROR("sonar_node usage: sudo rosrun sonar_node sonar_node PORT BITRATE\n");
  		return 1;
  	}

	// Initialize Cheetah parameters
	port = atoi(argv[1]);
	bitrate = atoi(argv[2]); //kHz

	// Open the device
	handle = ch_open(port);
	if (handle <= 0) {
		ROS_ERROR("Unable to open Cheetah device on port %d (Error code = %d: %s)", port, handle, ch_status_string(handle));
	  	exit(1);
	}
	ROS_INFO("Opened Cheetah device on port %d", port);
	ROS_INFO("Host interface is %s", (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

	// Configure SPI subsystem
	ch_spi_configure(handle, CH_SPI_POL_RISING_FALLING, CH_SPI_PHASE_SETUP_SAMPLE, CH_SPI_BITORDER_MSB, 0x0);
	ROS_INFO("SPI configuration set to mode %d, %s shift, SS[2:0] active low", mode, "MSB");

	// Power the target using the Cheetah's 5V power supply
	ch_target_power(handle, CH_TARGET_POWER_ON);
	ch_sleep_ms(100);

	// Set the bitrate
	bitrate = ch_spi_bitrate(handle, bitrate);
	ROS_INFO("Bitrate set to %d kHz", bitrate);

  
	// Make a simple queue to assert OE
	ch_spi_queue_clear(handle);
	ch_spi_queue_oe(handle, 1);
	ch_spi_batch_shift(handle, 0, 0);

	// Queue the batch
	ROS_INFO("Beginning to queue SPI packets...");
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
	ROS_INFO("Finished queueing packets\n");

	// Open output filestreams
	std::ofstream file1 ("1_adc_samples.txt");
	std::ofstream file2 ("2_adc_samples.txt");
	std::ofstream file3 ("3_adc_samples.txt");

	int batch_cnt = 1; //count number of sample batches
	double elapsed;
	s64    start;

	while( ros::ok() ) {
		// Submit the batch and collect data
		ROS_INFO("Collecting data from batch %d...", batch_cnt);
		start = _timeMillis();
   		ch_spi_async_submit(handle);
    		ret = ch_spi_async_collect(handle, TX_LENGTH, data_in);
		elapsed = ((double)(_timeMillis() - start)) / 1000;
		ROS_INFO("collected %d bytes from batch #%04d in %.4lf seconds\n", ret, batch_cnt, elapsed);
		if (ret < 0)  ROS_INFO("status error: %s\n", ch_status_string(ret));
		batch_cnt++;

		// Process raw data for the 12-bit ADC's, and write data to text files
    		int data_idx = 0;
		if ( file1.is_open() && file2.is_open() && file3.is_open() ) {
    			for (int j = 0; j < TX_LENGTH; j += 6) {
    				// SS3 Data
		    		input = (data_in[j] << 8) + data_in[j+1];
		    		valid_data_point = (input & 0x3ffc) >> 2;
		    		if(valid_data_point >= 0x0800)	valid_data_point = valid_data_point - 0x1000; //convert 2's comp to signed
		    		data3[data_idx] = valid_data_point;
				file3 << data3[data_idx] << ",";
	
		    		// SS1 Data
		    		input = (data_in[j+2] << 8) + data_in[j+3];
		    		valid_data_point = (input & 0x3ffc) >> 2;
		    		if(valid_data_point >= 0x0800)	valid_data_point = valid_data_point - 0x1000;
		    		data1[data_idx] = valid_data_point;
				file1 << data1[data_idx] << ",";
	
		    		// SS2 Data
		    		input = (data_in[j+4] << 8) + data_in[j+5];
		    		valid_data_point = (input & 0x3ffc) >> 2;
		    		if(valid_data_point >= 0x0800)	valid_data_point = valid_data_point - 0x1000;
		    		data2[data_idx] = valid_data_point;
				file2 << data2[data_idx] << ",";
		    		++data_idx;
    			}
		}
		else std::cout << "Error opening output filestream!" << std::endl;
	}

	// Close the filestreams on SIGINT
	file1.close();
	file2.close();
	file3.close();
	return 0;
}
