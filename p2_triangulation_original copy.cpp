/* 
 * Project 2 Part 2: Centroid of a Triangle 
 * 
 * Built From: barcode_test.cpp (by Raven)
 * Authors: Chris Beckley and Laura Hovatter
 * Most Recent Update: 10-10-2014
 *
 *    This program finds three barcodes, calculates the centroid of them and drives to that point.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#include "flockbot_api.h" 
#include "barcode.hpp"    

/*** Constant Definitions ***/
#define CAM_ANGLE 75     // The angle of the camera: set between 90 (forward) and 0 (facing down). Typical value = 75
#define ROBOT_SPEED 50   // The speed at which the robot will move and rotate. Typical value = 10
#define INCR_ANGLE 25    // The incremental angle used to randomly move when 3 cans are not seen. SET TO A VALUE THAT DOES NOT DIVIDE EVENLY INTO 360. Typical value = 25
#define DAMP_FACTOR 30.0 // The damping factor for error in the angle to rotate towards the centroid. The calculation later is DAMP_FACTOR / distance to centroid
const double Z = 20.3;  // The height of the camera in centimeters. Typical value = 20.3, but we subtracted 1 to account for barcodes being 1cm off floor (calibrated for a CAM_ANGLE of 75)
const double M = 802.0; // The focal distance of the camera (I think? This value was given by Prof. Luke). Value = 802.0
const double RAD_CONV = 0.0174532925; // The constant to convert from degrees to radians. Equal to PI / 180
const double DEG_CONV = 57.29577951;  // The constant to convert from radiant to degrees. Equal to 180 / PI

int vision_running = 1;  // This governs the main barcode processing loop
pthread_t vision_thread; // Thread to run the barcode processing module

/*** Global Variables ***/
int image_width = 800;   // The width of the camera image
int image_height = 600;  // The height of the camera image
int angle_turned = 0;    // The amount of angle turned so far while moving "randomly"

/*** Helper Functions ***/

/*
 * The next two functions help to center the zero point of the camera image. A raw image
 * has a zero point in the upper left corner, and y-values increase going down. After
 * running both of these, the zero point will be in the center of the image, and y-values
 * will increase going up (and x-values will still increase going to the right).
 */
// Convert the x-value of the camera image to a zero point in the middle of the image
int convert_camera_x(int x) {
  return x - (image_width / 2);
}
// Convert the y-value of the camera image to a zero point in the middle of the image
int convert_camera_y(int y) {
  return (image_height - y) - (image_height / 2);
}

double calculate_error_damp(double dist) {
  return DAMP_FACTOR / dist;
}

// X = Z / tan(theta + gamma)
double can_xpos(int y_cam) {
  // gamma = angle of camera in radians
  double gamma = RAD_CONV * ((double) (CAM_ANGLE-90)); // IS THIS RIGHT???
  double theta = atan((double) y_cam / M);
  double xpos = (Z / tan(theta + gamma));
  printf("Converting X --- Y'=%d, Gamma=%f, Theta=%f, X=%f\n",y_cam,gamma,theta,xpos);
  return xpos;
}

// Y = -X * tan(omega)
double can_ypos(double x, int x_cam) {
  double neg_x_prime = (double) x_cam * -1.0;
  double omega = atan(neg_x_prime / M);
  double ypos = -x * tan(omega);
  printf("Converting Y --- X'=%d, Omega=%f, Y=%f\n",x_cam,omega,ypos);
  return ypos;
}

// Move in a pseudo-random direction. Used to locate a position where all three barcodes can be seen
void random_move(int num_barcodes_seen) {
  if (angle_turned < 360) {
    turn_robot_wait(ROBOT_SPEED, INCR_ANGLE / (num_barcodes_seen + 1));
    angle_turned += INCR_ANGLE;
  }
  else{
    move_distance_wait(ROBOT_SPEED, 10);
    angle_turned = 0;
  }
}

/*** Main Triangulation Function ***/
int main() 
{
  initialize_api(); // Initializes the robot API
  robot_connect("127.0.0.1"); // This always needs to be 127.0.0.1 to connect to the robot
  
  barcode_configure(2 /*number of digits*/, 10 /*barcodes per pass*/, 2 /*kept passes*/, 
                    2 /*skip pixel width*/, 2 /*min bar width*/, 100 /* Allowed skew */, -1 /*Otsu thresholding*/,
                    image_width /*image width*/, image_height /*image height*/);
  sleep(1);
  
  int** barcodes; // Pointer for a two-dimensional array, will hold returned barcodes
  int* xy; // Pointer to an array [x,y]
  camera_set(CAM_ANGLE); // Sets the tilt on the camera to the current defined camera angle
  
  // Create a thread to handle grabbing images for checking barcodes
  pthread_create(&vision_thread, NULL, barcode_start, NULL);
  
  // The final x and y coordinates of the three barcodes
  double x1,y1,x2,y2,x3,y3;
  x1 = y1 = x2 = y2 = x3 = y3 = 0.0;

  // Grab the current seen barcodes
  barcode_frame_wait(); // Waits for the next frame from the current counter
  barcodes = barcode_get_barcodes(); // Retrieves the barcodes in the 2D array
  
  int num_codes = (int)barcodes[0];  // Number of barcodes detected
  int num_digits = (int)barcodes[1]; // Number of digits per barcode (will be 2)
  printf("Detected %d barcodes %d digits long:\n", num_codes, num_digits);
  
  
  // Find three barcodes if we didn't see three at first 
  while (num_codes < 3) {
    // Code to shutdown when the high bump is pressed. Just in case we never find the cans...
    if (get_high_bump() == 1) {
      free(barcodes);
      robot_stop();
      
      /*** Teardown Code ***/
      vision_running = 0; // Tells the background thread to nicely finish
      pthread_join(vision_thread, NULL); // Waits nicely for the barcode module to shutdown
      shutdown_api(); // Shuts down the robot API
      return 0; // Returns 0
    }
    printf("Didn't see 3 barcodes. Moving to find them...\n\n");
    random_move((int)barcodes[0]);
    free(barcodes);
    sleep(3);
    barcode_frame_wait_start(); // Reset the frame wait counter to wait for next full frame
    barcode_frame_wait(); // Waits for the next frame from the current counter
    barcodes = barcode_get_barcodes(); // Retrieves the barcodes in the 2D array
    
    num_codes = (int)barcodes[0];  // Number of barcodes detected
    num_digits = (int)barcodes[1]; // Number of digits per barcode (will be 2)
    printf("Detected %d barcodes %d digits long:\n", num_codes, num_digits);
  }
    
  int i;
  /* Runs through the barcodes detected, which start at index [2] */
  for(i = 2; i < num_codes+2; i++)
    {
      /* This block converts the raw digits into actual numbers */
      int j;
      uint32_t sum = 0;
      for(j = 0; j < num_digits; j++)
	{
	  sum += barcodes[i][j] * pow(10,(num_digits-1)-j);
	}
      
      /* Get the X and Y positions of the cans relative to the robot */
      xy = barcode_get_cur_xy(sum);
      int bottom_y = convert_camera_y(barcodes[i][j+3]);
      double x_pos = can_xpos(bottom_y);
      double y_pos = can_ypos(x_pos,convert_camera_x(xy[0]));
      
      printf("Barcode: %d\tX_CAM: %d\tY_CAM: %d\n",sum,convert_camera_x(xy[0]),bottom_y);
      printf("X_POS: %f\n",x_pos);
      printf("Y_POS: %f\n",y_pos);
      
      // Save the (x,y) coordinates in the proper variables for use later
      switch(i - 1) {
      case 1:
	x1 = x_pos;
	y1 = y_pos;
	break;
      case 2:
	x2 = x_pos;
	y2 = y_pos;
	break;
      case 3:
	x3 = x_pos;
	y3 = y_pos;
	break;
      default:
	break;
      }    
      
      free(xy);
    }

  // We don't need the camera or barcodes any more, so free them up
  vision_running = 0; // Tells the background thread to nicely finish
  pthread_join(vision_thread, NULL); // Waits nicely for the barcode module to shutdown
  free(barcodes);

  /*** Calculate the centroid, and the distance and angle of rotation to it ***/
  double x_centroid = (x1 + x2 + x3) / 3.0;
  double y_centroid = (y1 + y2 + y3) / 3.0;
  double dist_centroid = sqrt((x_centroid * x_centroid) + (y_centroid * y_centroid));
  int16_t dist_cm = (int16_t) round(dist_centroid);
  double theta_centroid = atan((y_centroid / x_centroid));
  int16_t theta_deg = (int16_t) round(DEG_CONV * (theta_centroid * calculate_error_damp(dist_centroid)));
  printf("\n\nCentroid X: %f\nCentroid Y: %f\nDistance: %f\nAngle: %d\n\n",x_centroid,y_centroid,dist_centroid,theta_deg);

  // Use the distance and angle calculations to set the proper robot turn and move values
  int8_t speed;
  if(theta_deg >= 0){
    speed = ROBOT_SPEED;
  }
  else{ // If the angle is negative, turn clockwise (= negative speed)
    speed = ROBOT_SPEED * (-1);
    theta_deg = theta_deg*(-1); // Angle must be positive for turn_robot_wait()
  }
  printf("Turning by %d degrees\n", theta_deg);
  turn_robot_wait(speed,theta_deg);
  printf("Moving %d cm\n",dist_cm);
  move_distance_wait(ROBOT_SPEED,dist_cm);

  // Make sure the robot has stopped
  robot_stop();

  /*** Teardown Code ***/
  shutdown_api(); // Shuts down the robot API
  return 0; // Returns 0
 }
