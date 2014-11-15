/*=========================================================================
| (c) 2005-2007  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : async.c
|--------------------------------------------------------------------------
| Use the asynchronous interface of the Cheetah host adapter
|--------------------------------------------------------------------------
| Redistribution and use of this file in source and binary forms, with
| or without modification, are permitted.
|
| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
| FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
| COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
| LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
| CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
| LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
| ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
| POSSIBILITY OF SUCH DAMAGE.
 ========================================================================*/

//=========================================================================
// INCLUDES
//=========================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cheetah.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>


// Globals
const int       NUM_SAMPLES             = 1000000;
const int       DATA_BLOCK_SIZE         = NUM_SAMPLES * 3; //#of ADCs
const int       TX_LENGTH = DATA_BLOCK_SIZE * 2;
uint8_t         data_in[TX_LENGTH]; //raw data
uint8_t         data_out[2];
long int        data1[TX_LENGTH/3];
long int        data2[TX_LENGTH/3];
long int        data3[TX_LENGTH/3];

// Declare Cheetah Variables
Cheetah         handle;
int             port, bitrate;
uint8_t         mode = 0;
int             ret, input, ready_bit, valid_data_point;
	// Open output filestreams
	std::ofstream file1 ("1_adc_samples.txt");
	std::ofstream file2 ("2_adc_samples.txt");
	std::ofstream file3 ("3_adc_samples.txt");

#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif


//=========================================================================
// CONSTANTS
//=========================================================================
// Uncomment to show the data returned by the shifting device
#define SHOW_DATA

// Add a delay between bytes by changing this constant (in nanoseconds)
#define BYTE_DELAY 0


//=========================================================================
// UTILITY FUNCTIONS
//=========================================================================
static s64 _timeMillis () {
#ifdef _WIN32
    return ((s64)clock()) * 1000 / CLOCKS_PER_SEC;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return ((s64)tv.tv_sec * 1000L) + (s64)(tv.tv_usec / 1000L);
#endif
}


//=========================================================================
// FUNCTIONS
//=========================================================================
static void _blast_async (Cheetah handle, u32 txnlen, u32 iter) {
    double elapsed;
    u32    i;
    int    count = 0;
    u08    data_out[4];
    s64    start;
    int    ret;

    // Make a simple queue to just assert OE.
    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 1);
    ch_spi_batch_shift(handle, 0, 0);

    
    // Queue the batch which is a sequence of SPI packets
    // (back-to-back) each of length 4.
    ch_spi_queue_clear(handle);
    for (i = 0; i < txnlen; ++i) {
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


    start = _timeMillis();

    // First, submit first batch 
    ch_spi_async_submit(handle);

    for (i = 0; i < iter-1; ++i) {
        // Submit another batch, while the previous one is in
        // progress.  The application may even clear the current
        // batch queue and queue a different set of SPI
        // transactions before submitting this batch
        // asynchronously.
        ch_spi_async_submit(handle);
        
        // The application can now perform some other functions
        // while the Cheetah is both finishing the previous batch
        // and shifting the current batch as well.  In order to
        // keep the Cheetah's pipe full, this entire loop must
        // complete AND another batch must be submitted
        // before the current batch completes.
        ch_sleep_ms(25);
        
        // Collect the previous batch
        ret = ch_spi_async_collect(handle, 0, 0);
        elapsed = ((double)(_timeMillis() - start)) / 1000;
        printf("collected batch #%03d in %.2lf seconds\n", i+1, elapsed);
        if (ret < 0)  printf("status error: %s\n", ch_status_string(ret));
        fflush(stdout);

        start = _timeMillis();
        
        // The current batch is now shifting out on the SPI
        // interface. The application can again do some more tasks
        // here but this entire loop must finish so that a new
        // batch is armed before the current batch completes.
        ch_sleep_ms(25);
    }

    // Collect batch the last batch
    ret = ch_spi_async_collect(handle, 0, 0);
    elapsed = ((double)(_timeMillis() - start)) / 1000;
    printf("collected batch #%03d in %.2lf seconds\n", i+1, elapsed);
    if (ret < 0)  printf("status error: %s\n", ch_status_string(ret));
    fflush(stdout);

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


//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main (int argc, char *argv[]) {
    Cheetah handle;
    int port     = 0;
    int bitrate  = 0;
    u08 mode     = 0;
    u32 txnlen;
    u32 iter;

    if (argc < 5) {
        printf("usage: async PORT BITRATE TXN_LENGTH ITER\n");
        printf("\n");
        printf("TXN_LENGTH is the number of SPI packets, each of length\n");
        printf("4 to queue in a single batch.\n");
        printf("\n");
        printf("ITER is the number of batches to process asynchronously.\n");
        return 1;
    }

    port     = atoi(argv[1]);
    bitrate  = atoi(argv[2]);
    txnlen   = atoi(argv[3]);
    iter     = atoi(argv[4]);
    
    // Open the device
    handle = ch_open(port);
    if (handle <= 0) {
        printf("Unable to open Cheetah device on port %d\n", port);
        printf("Error code = %d (%s)\n", handle, ch_status_string(handle));
        return 1;
    }
    printf("Opened Cheetah device on port %d\n", port);

    printf("Host interface is %s\n",
           (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

    // Ensure that the SPI subsystem is configured.
    //ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0);
	ch_spi_configure(handle, CH_SPI_POL_RISING_FALLING, CH_SPI_PHASE_SETUP_SAMPLE, CH_SPI_BITORDER_MSB, 0x0);
    printf("SPI configuration set to mode %d, %s shift, SS[2:0] active low\n",
           mode, "MSB");
    fflush(stdout);

    // Power the target using the Cheetah adapter's power supply.
    ch_target_power(handle, CH_TARGET_POWER_ON);
    ch_sleep_ms(100);

    // Set the bitrate.
    bitrate = ch_spi_bitrate(handle, bitrate);
    printf("Bitrate set to %d kHz\n", bitrate);
    fflush(stdout);

    _blast_async(handle, txnlen, iter);
    
	file1.close();
	file2.close();
	file3.close();


    // Close and exit.
    ch_close(handle);
    return 0;
}
