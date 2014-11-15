#include <ros/ros.h>
#include <math.h>
#include <fftw3.h>
#include <sonar_node/SonarScan.h>
//#include <sonar_node/SonarScanArray.h>
#include <sonar/fft_bin_prediction.h>

extern "C" {
#include <sonar/cheetah.h>
}

double sign(double val) {
  if (val < 0)
    return -1.0;
  else if (val > 0)
    return 1.0;
  return 0.0;
}

// Global Constants
const double SPEED_SOUND_WATER = 1482; // [m/s]
const double SENSOR_SPACING = 0.024;    // [m]

// Declare Cheetah Variables
Cheetah    handle;
int        port, bitrate;
uint8_t    mode = 0;
int        ret, input, ready_bit, valid_data_point;

// FFT & Bin Prediction
const int  N_FFT = 512;
double     BIN_PREDICTION_M = 10.59;
double     BIN_PREDICTION_B = -1193.82;
int        target_frequency1, target_frequency2;
double     target_wavelength1, target_wavelength2;
int        halfwidth = 3;
double     gaussian_weights[] = {0.006, 0.061, 0.242, 0.383, 0.242, 0.061, 0.006};
double     maxIndex1, maxValue1;
double     maxIndex2, maxValue2;
double     maxIndex3, maxValue3;
double     sum1, sum2, sum3;
double     angle_estimate1;

// Raw Data
const int  TX_LENGTH = N_FFT * 3 * 2;
uint8_t    data_in[TX_LENGTH];
uint8_t    data_out[2];

int main( int argc, char *argv[]) {
  if (argc < 5) {
    ROS_ERROR("sonar_node usage: rosrun sonar_node sonar_node __name:=sonar_node PORT [Freq 1] [Freq 1]\n");
    return 1;
  }

	ros::init(argc, argv, "sonar_node");
	ros::NodeHandle n;
	ros::Publisher sonar_pub = n.advertise<sonar_node::SonarScan>("sonar", 100);
  ros::Rate loop_rate(5);

  // Assign Target Frequencies
  target_frequency1  = atoi(argv[2]);
  target_frequency2  = atoi(argv[3]);
  target_wavelength1 = (double)SPEED_SOUND_WATER / (double)target_frequency1;
  target_wavelength2 = (double)SPEED_SOUND_WATER / (double)target_frequency2;

  // Define Connection Values
  port    = 0;
  bitrate = 10000;

  // Open the Cheetah Connection
  handle = ch_open(port);

  // Check that the Cheetah was opened successfully
  if (handle <= 0) {
    ROS_ERROR("Sonar: Unable to open Cheetah device on port %d (Error code = %d: %s)", port, handle, ch_status_string(handle));
    exit(1);
  }
  ROS_DEBUG("Opened Cheetah device on port %d", port);

  // Check that the Cheetah is operating at "full speed"
  ROS_DEBUG("Host interface is %s",
      (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");
//  if (handle != 0)
//    ROS_ERROR("The Cheetah was unable to open at full speed");

  // Ensure that the SPI subsystem is configured.
  ch_spi_configure(handle, CheetahSpiPolarity(mode >> 1), CheetahSpiPhase(mode & 1), CH_SPI_BITORDER_MSB, 0x0);
  ROS_DEBUG("SPI configuration set to mode %d, %s shift, SS[2:0] active low", mode, "MSB");

  // Power the target using the Cheetah adapter's power supply.
  ch_target_power(handle, CH_TARGET_POWER_ON);
  ch_sleep_ms(100);

  // Set the bitrate.
  bitrate = ch_spi_bitrate(handle, bitrate);
  ROS_INFO("Bitrate set to %d kHz", bitrate);

  // Make a simple queue to just assert OE.
  ch_spi_queue_clear(handle);
  ch_spi_queue_oe(handle, 1);
  ch_spi_batch_shift(handle, 0, 0);

  // Queue the batch, which is a sequence of SPI packets (back-to-back) each of length 2.
  ROS_DEBUG("Sonar: Beginning to queue SPI packets");
  data_out[0] = 0xff;
  data_out[1] = 0xff;
  ch_spi_queue_clear(handle);

  for (int i = 0; i < TX_LENGTH; ++i) {
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

  // Calculate Sampling Frequency and Target Bin
  // For affine bin prediction, use m = 10.59, b = -1193.82 and a
  // window-halfwidth of 5.
  FFTBinAffinePrediction bin_predictor1(bitrate,
                                       N_FFT,
                                       target_frequency1,
                                       BIN_PREDICTION_M,
                                       BIN_PREDICTION_B,
                                       halfwidth);
  FFTBinAffinePrediction bin_predictor2(bitrate,
                                       N_FFT,
                                       target_frequency2,
                                       BIN_PREDICTION_M,
                                       BIN_PREDICTION_B,
                                       halfwidth);
  ROS_DEBUG("Sonar: Target Bin 1: %d", bin_predictor1.getTargetBin());
  ROS_DEBUG("Sonar: Target Bin 2: %d", bin_predictor2.getTargetBin());

  // Declare Data Vectors
  double *data1, *data2, *data3;
  fftw_complex *fft1, *fft2, *fft3;
  fftw_plan fft_plan1, fft_plan2, fft_plan3;

  // Allocate Memory for Data Vectors
  data1 = (double *) fftw_malloc ( sizeof ( double ) * N_FFT );
  data2 = (double *) fftw_malloc ( sizeof ( double ) * N_FFT );
  data3 = (double *) fftw_malloc ( sizeof ( double ) * N_FFT );
  memset(data1, 0, sizeof ( double ) * N_FFT);
  memset(data2, 0, sizeof ( double ) * N_FFT);
  memset(data3, 0, sizeof ( double ) * N_FFT);
  fft1 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( N_FFT / 2 ) + 1) );
  fft2 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( N_FFT / 2 ) + 1) );
  fft3 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( N_FFT / 2 ) + 1) );

  // Define FFT Operations
  fft_plan1 = fftw_plan_dft_r2c_1d(N_FFT, data1, fft1, FFTW_ESTIMATE);
  fft_plan2 = fftw_plan_dft_r2c_1d(N_FFT, data2, fft2, FFTW_ESTIMATE);
  fft_plan3 = fftw_plan_dft_r2c_1d(N_FFT, data3, fft3, FFTW_ESTIMATE);

  // Submit the first batch
  ch_spi_async_submit(handle);

  while (ros::ok()) {
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
    //ch_sleep_ms(1);

    // Collect the previous batch
    // The length of the batch, N_FFT * 6, come from the fact that 3 ADCs
    // are batched and the return data requires 2 bytes.  (2 * 3 = 6)
    ret = ch_spi_async_collect(handle, N_FFT * 6, data_in);

    int data_idx = 0;
    for (int j = 0; j < N_FFT * 6; j += 6) {
      // SS3 Data
      input = (data_in[j] << 8) + data_in[j+1];
      ready_bit = input & 0x4000;
      valid_data_point = (input & 0x3ffc) >> 2;
      data3[data_idx] = valid_data_point;

      // SS2 Data
      input = (data_in[j+2] << 8) + data_in[j+3];
      ready_bit = input & 0x4000;
      valid_data_point = (input & 0x3ffc) >> 2;
      data2[data_idx] = valid_data_point;

      // SS1 Data
      input = (data_in[j+4] << 8) + data_in[j+5];
      ready_bit = input & 0x4000;
      valid_data_point = (input & 0x3ffc) >> 2;
      data1[data_idx] = valid_data_point;
      
      

      //fprintf(output_file, "%10f, %10f, %10f", data1[data_idx], data2[data_idx], data3[data_idx]);
      ++data_idx;
    }
    
    // Perform FFT on current input data
    fftw_execute(fft_plan1);
    fftw_execute(fft_plan2);
    fftw_execute(fft_plan3);

    // Check the ADCs for a peak.
    maxValue1 = 0;
    maxIndex1 = 0;
    maxValue2 = 0;
    maxIndex2 = 0;
    maxValue3 = 0;
    maxIndex3 = 0;

    // Find maxValue and maxIndex for each channel
    for (int i = bin_predictor1.getTargetBin() - bin_predictor1.getHalfwidth(); i < bin_predictor1.getTargetBin() + bin_predictor1.getHalfwidth() + 1; ++i) {
//    for (int i = 50; i < N_FFT / 2; ++i) {
//      /*fprintf(stderr, "%1.3f * %10.2f + ", gaussian_weights[i - (bin_predictor1.getTargetBin() - bin_predictor1.getHalfwidth())], sqrt(
//          pow(fft1[i][0],2.0) +
//          pow(fft1[i][1],2.0)));*/
//      weighted_sum += gaussian_weights[i - (bin_predictor1.getTargetBin() - bin_predictor1.getHalfwidth())] *
//                      sqrt(pow(fft1[i][0],2.0) + pow(fft1[i][1],2.0));
      if (fft1[i][0] * fft1[i][0] + fft1[i][1] * fft1[i][1] > maxValue1) {
        maxValue1 = fft1[i][0] * fft1[i][0] + fft1[i][1] * fft1[i][1];
        maxIndex1 = i;
        }
      if (fft2[i][0] * fft2[i][0] + fft2[i][1] * fft2[i][1] > maxValue2) {
        maxValue2 = fft2[i][0] * fft2[i][0] + fft2[i][1] * fft2[i][1];
        maxIndex2 = i;
      }
      if (fft3[i][0] * fft3[i][0] + fft3[i][1] * fft3[i][1] > maxValue3) {
        maxValue3 = fft3[i][0] * fft3[i][0] + fft3[i][1] * fft3[i][1];
        maxIndex3 = i;
      }
    }

    // Calculate Gaussian weighted sums surrounding predicted bin
    int targetBin = bin_predictor1.getTargetBin();
    sum1 = gaussian_weights[0] *
              (fft1[targetBin-3][0] * fft1[targetBin-3][0]  +
               fft1[targetBin-3][1] * fft1[targetBin-3][1]) +
           gaussian_weights[1] *
              (fft1[targetBin-2][0] * fft1[targetBin-2][0]  +
               fft1[targetBin-2][1] * fft1[targetBin-2][1]) +
           gaussian_weights[2] *
              (fft1[targetBin-1][0] * fft1[targetBin-1][0]  +
               fft1[targetBin-1][1] * fft1[targetBin-1][1]) +
           gaussian_weights[3] *
              (fft1[targetBin][0] * fft1[targetBin][0]  +
               fft1[targetBin][1] * fft1[targetBin][1]) +
           gaussian_weights[4] *
              (fft1[targetBin+1][0] * fft1[targetBin+1][0]  +
               fft1[targetBin+1][1] * fft1[targetBin+1][1]) +
           gaussian_weights[5] *
              (fft1[targetBin+2][0] * fft1[targetBin+2][0]  +
               fft1[targetBin+2][1] * fft1[targetBin+2][1]) +
           gaussian_weights[6] *
              (fft1[targetBin+3][0] * fft1[targetBin+3][0]  +
               fft1[targetBin+3][1] * fft1[targetBin+3][1]);
    sum2 = gaussian_weights[0] *
              (fft2[targetBin-3][0] * fft2[targetBin-3][0]  +
               fft2[targetBin-3][1] * fft2[targetBin-3][1]) +
           gaussian_weights[1] *
              (fft2[targetBin-2][0] * fft2[targetBin-2][0]  +
               fft2[targetBin-2][1] * fft2[targetBin-2][1]) +
           gaussian_weights[2] *
              (fft2[targetBin-1][0] * fft2[targetBin-1][0]  +
               fft2[targetBin-1][1] * fft2[targetBin-1][1]) +
           gaussian_weights[3] *
              (fft2[targetBin][0] * fft2[targetBin][0]  +
               fft2[targetBin][1] * fft2[targetBin][1]) +
           gaussian_weights[4] *
              (fft2[targetBin+1][0] * fft2[targetBin+1][0]  +
               fft2[targetBin+1][1] * fft2[targetBin+1][1]) +
           gaussian_weights[5] *
              (fft2[targetBin+2][0] * fft2[targetBin+2][0]  +
               fft2[targetBin+2][1] * fft2[targetBin+2][1]) +
           gaussian_weights[6] *
              (fft2[targetBin+3][0] * fft2[targetBin+3][0]  +
               fft2[targetBin+3][1] * fft2[targetBin+3][1]);
    sum3 = gaussian_weights[0] *
              (fft3[targetBin-3][0] * fft3[targetBin-3][0]  +
               fft3[targetBin-3][1] * fft3[targetBin-3][1]) +
           gaussian_weights[1] *
              (fft3[targetBin-2][0] * fft3[targetBin-2][0]  +
               fft3[targetBin-2][1] * fft3[targetBin-2][1]) +
           gaussian_weights[2] *
              (fft3[targetBin-1][0] * fft3[targetBin-1][0]  +
               fft3[targetBin-1][1] * fft3[targetBin-1][1]) +
           gaussian_weights[3] *
              (fft3[targetBin][0] * fft3[targetBin][0]  +
               fft3[targetBin][1] * fft3[targetBin][1]) +
           gaussian_weights[4] *
              (fft3[targetBin+1][0] * fft3[targetBin+1][0]  +
               fft3[targetBin+1][1] * fft3[targetBin+1][1]) +
           gaussian_weights[5] *
              (fft3[targetBin+2][0] * fft3[targetBin+2][0]  +
               fft3[targetBin+2][1] * fft3[targetBin+2][1]) +
           gaussian_weights[6] *
              (fft3[targetBin+3][0] * fft3[targetBin+3][0]  +
               fft3[targetBin+3][1] * fft3[targetBin+3][1]);

    if (sum1 > 8000 && sum2 > 8000 && sum3 > 8000)
        ROS_INFO("Weighted Sums: %f               %f               %f\n", sum1, sum2, sum3);

    //ROS_DEBUG("MaxIndex: %f, %f, %f", maxIndex1, maxIndex2, maxIndex3);
    //if (maxIndex1 > bin_predictor1.getTargetBin()-2 && maxIndex2 > bin_predictor1.getTargetBin()-2) {
    ROS_INFO("Signal 1 Peak: %1.0f, %10.2f", maxIndex1, maxValue1);
    ROS_INFO("Signal 2 Peak: %1.0f, %10.2f", maxIndex2, maxValue2);
    ROS_INFO("Signal 3 Peak: %1.0f, %10.2f", maxIndex3, maxValue3);
    //}

    // Calculate Phase of each signal found on each ADC
    //double phase1 = atan2((double)fft1[(int)maxIndex1][1], (double)fft1[(int)maxIndex1][0]);
    double phase2 = atan2((double)fft2[(int)maxIndex2][1], (double)fft2[(int)maxIndex2][0]);
    //double phase3 = atan2((double)fft3[(int)maxIndex3][1], (double)fft3[(int)maxIndex3][0]);
    double phase1 = atan2((double)fft1[(int)maxIndex2][1], (double)fft1[(int)maxIndex2][0]);
    double phase3 = atan2((double)fft3[(int)maxIndex2][1], (double)fft3[(int)maxIndex2][0]);

    // Calculate usable phase differences
    double delta1 = phase2 - phase1;
    double delta2 = phase3 - phase2;
    double delta3 = phase1 - phase3;

    // Determine minimum phase difference for pair selection
    int min_index = 3;
    if (fabs(delta2) < fabs(delta1) && fabs(delta2) < fabs(delta3))
      min_index = 2;
    if (fabs(delta1) < fabs(delta2) && fabs(delta1) < fabs(delta3))
      min_index = 1;

    
    double delta_tmp;
    switch (min_index) {
      case 1:
        delta_tmp = delta1;
        if (delta3 > delta2)
          delta_tmp = -1.0 * delta1 + sign(delta_tmp) * 2.0 * M_PI;
        angle_estimate1 = delta_tmp * (double)(((double)target_wavelength1 / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
        break;
      case 2:
        delta_tmp = delta2;
        if (delta1 > delta3)
          delta_tmp = -1.0 * delta2 + 2.0 * M_PI;
        angle_estimate1 = (delta_tmp - 4.0 / 3.0 * M_PI) * ((target_wavelength1 / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
        break;
      case 3:
        delta_tmp = delta3;
        if (delta2 > delta1)
          delta_tmp = -1.0 * delta3 - 2.0 * M_PI;
        //fprintf(stderr, "%4.8f\n", (double)SPEED_SOUND_WATER);
        //fprintf(stderr, "%4.8f\n", (double)target_frequency1);
        //fprintf(stderr, "%4.8f\n", (double)SPEED_SOUND_WATER / (double)target_frequency1);
        //fprintf(stderr, "%4.8f\n", (double)target_wavelength);
        //fprintf(stderr, "%4.8f\n", ((double)target_wavelength / 2.0));
        //fprintf(stderr, "%4.8f\n", (((double)target_wavelength / 2.0) / SENSOR_SPACING));
        //fprintf(stderr, "%4.8f\n", 180.0 / M_PI / 2.0);
        //fprintf(stderr, "%4.8f\n", (((double)target_wavelength / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0);
        angle_estimate1 = (delta_tmp + 4.0 / 3.0 * M_PI ) * (((double)target_wavelength1 / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
        break;
      default:
        ROS_ERROR("Sonar: Invalid min_index for phase difference.");
    }
    if (sum1 > 8000 && sum2 > 8000 && sum3 > 8000)
      ROS_INFO("Detected Angle: %4.4f degrees", angle_estimate1);

    // Ensures communication is successful
    if (ret < 0)
      ROS_ERROR("Sonar: status error: %s", ch_status_string(ret));

    // Form the Sonar Node message to publish
    sonar_node::SonarScan msg;
    msg.F1_magnitude = (sum1 + sum2 + sum3) / 3.0;
    msg.F1_heading   = angle_estimate1;
    msg.F2_magnitude = 0;
    msg.F2_heading   = 0;

    // Publish the message
    if (sum1 > 8000 && sum2 > 8000 && sum3 > 8000)
      sonar_pub.publish(msg);
    
    ROS_INFO("Sonar: I published!");
    //ros::spinOnce();
    //loop_rate.sleep();
  }

  // Clean up allocated memory
  fftw_destroy_plan(fft_plan1);
  fftw_destroy_plan(fft_plan2);
  fftw_destroy_plan(fft_plan3);
  fftw_free(data1);
  fftw_free(data2);
  fftw_free(data3);
  fftw_free(fft1);
  fftw_free(fft2);
  fftw_free(fft3);

  return 0;
}

