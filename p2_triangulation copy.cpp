/* 
 * Project 2 Part 2: Centroid of a Triangle 
 * 
 * Built From: barcode_test.cpp (by Raven)
 * Authors: Chris Beckley and Laura Hovatter
 * Most Recent Update: 10-14-2014
 *
 *    This program finds three barcodes, calculates the centroid of them and drives to that point.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include "flockbot_api.h" 
#include "barcode.hpp"    


/*****************************************************************************************************
 *                                                                                                   *
 *                                       CONSTANT DEFINITIONS                                        *
 *                                                                                                   * 
 *****************************************************************************************************/
#define CAM_ANGLE 90     // The angle of the camera: set between 90 (forward) and 0 (facing down). Typical value = 75
#define ROBOT_SPEED 50   // The speed at which the robot will move and rotate. Typical value = 10
#define INCR_ANGLE 25    // The incremental angle used to randomly move when 3 cans are not seen. SET TO A VALUE THAT DOES NOT DIVIDE EVENLY INTO 360. Typical value = 25
#define TWO_PI 6.283185  // The constant value of 2*PI
#define PI_OVER_TWO 1.570796
#define PI 3.141593
#define DAMP_FACTOR 30.0 // The damping factor for error in the angle to rotate towards the centroid. The calculation later is DAMP_FACTOR / distance to centroid
const float Z = 20.6;    // The height of the camera in centimeters. Calibrated for a CAM_ANGLE of 90
const float M = 802.0;   // The focal distance of the camera (I think? This value was given by Prof. Luke). Value = 802.0
const float RAD_CONV = 0.0174532925; // The constant to convert from degrees to radians. Equal to PI / 180
const float DEG_CONV = 57.29577951;  // The constant to convert from radians to degrees. Equal to 180 / PI


/*****************************************************************************************************
 *                                                                                                   *
 *                                       GLOBAL VARIABLES                                            *
 *                                                                                                   * 
 *****************************************************************************************************/
int image_width = 800;   // The width of the camera image
int image_height = 600;  // The height of the camera image
int angle_turned = 0;    // The amount of angle turned so far while moving "randomly"
float cur_omega = 0.0;   // The current value of Omega (used for converting can positions to the robot reference frame)
int vision_running = 1;  // This governs the main barcode processing loop
pthread_t vision_thread; // Thread to run the barcode processing module

// The struct used to hold information on all three unique cans found
typedef struct can{
  int id;      // The number on the can
  float x_pos; // Position with regards to the robot reference frame
  float y_pos; 
}can_ref, *can_ptr;

can_ptr cans[3];

/*****************************************************************************************************
 *                                                                                                   *
 *                                       FUNCTION HEADERS                                            *
 *                                                                                                   * 
 *****************************************************************************************************/
int convert_camera_x(int x);
int convert_camera_y(int y);
float calculate_error_damp(float dist);
float can_xpos(int y_cam);
float can_ypos(float x, int x_cam);
void random_move(int num_barcodes_seen);
void transformation(float x, float y, int can_num);
void initialize_cans_array();
void breakdown_cans_array();
void go_to_pos(float x, float y);
int16_t convert_angle_flockbot(float rad);
int16_t convert_xy_flockbot(float x, float y);

/*****************************************************************************************************
 *                                                                                                   *
 *                                       HELPER FUNCTIONS                                            *
 *                                                                                                   * 
 *****************************************************************************************************/

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

// Calculate how much to damp down the final angle we calculate
float calculate_error_damp(float dist) {
  return DAMP_FACTOR / dist;
}

// Finad the X-position of a can based on the formula X = Z / tan(theta + gamma)
float can_xpos(int y_cam) {
  // gamma = angle of camera in radians
  float gamma = RAD_CONV * ((float) (CAM_ANGLE-90)); // IS THIS RIGHT???
  float theta = atan((float) y_cam / M);
  float xpos = (Z / tan(theta + gamma));
  printf("Converting X --- Y'=%d, Gamma=%f, Theta=%f, X=%f\n",y_cam,gamma,theta,xpos);
  return xpos;
}

// Find the Y-position of a can based on the formula Y = -X * tan(omega)
float can_ypos(float x, int x_cam) {
  float neg_x_prime = (float) x_cam * -1.0;
  cur_omega = atan(neg_x_prime / M);
  float ypos = -x * tan(cur_omega);
  printf("Converting Y --- X'=%d, Omega=%f, Y=%f\n",x_cam,cur_omega,ypos);
  return ypos;
}

// Move in a pseudo-random direction. Used to locate a position where all three barcodes can be seen
void random_move(int num_barcodes_seen) {
  // Turn in a full circle first to try and find any cans around
  if (angle_turned < 360) {
    // If we have just seen a can or two, slow down the turn to hopefully find the other cans
    turn_robot_wait(ROBOT_SPEED, INCR_ANGLE / (num_barcodes_seen + 1));
    angle_turned += INCR_ANGLE;
  }
  // Once we have gone in a full circle, move in to a nearby random point in order to try again
  else{
    // Move to a random point that is at least 14cm away from the current robot position
    int sign = 1;
    int r_x = 0;
    int r_y = 0;
    if (rand() % 100 < 50) {sign = -1;} 
    else {sign = 1;}
    r_x = sign * ((rand() % 11) + 10);
    if (rand() % 100 < 50) {sign = -1;} 
    else {sign = 1;}
    r_y = sign * ((rand() % 11) + 10);
    float* robot_pos = get_pos();
    float x = robot_pos[0] + (float) r_x;
    float y = robot_pos[1] + (float) r_y;

    go_to_pos(x,y);
    angle_turned = 0;
    free(robot_pos); // THIS MIGHT BREAK
  }
}

// Transform the position of the can from the current to the initial robot reference frame
void transformation(float x, float y, int can_num) {
  /*
   * This function is only called after finding a unique can, at which point the
   * horizontal angle from the robot to the can will have been calculated (cur_omega).
   * We add this to the current orientation of the robot and use that angle to do a
   * rotational transformation on the x and y parameters. Finally, we do a translation
   * based on the robot's current x and y position and set the values for the desired
   * can.
   *
   * The formula for this can be summed up as:
   *      T_robot_pos(R_total_angle((x,y)))
   */

  // Rotation by total_angle
  float total_angle = get_angle() + cur_omega;
  float x_prime = (x * cos(total_angle)) - (y * sin(total_angle));
  float y_prime = (x * sin(total_angle)) + (y * cos(total_angle));

  // Translation by robot_pos
  float* robot_pos = get_pos();
  x_prime = x_prime - robot_pos[0];
  y_prime = y_prime - robot_pos[1];

  // Update the can position in the cans array
  cans[can_num]->x_pos = x_prime;
  cans[can_num]->y_pos = y_prime;

  // Finally, we free the position array obtained from the integrator
  free(robot_pos); // THIS MIGHT BREAK
}

// Create space for everything in the cans array and initialize all values to zero
void initialize_cans_array() {
  int i = 0;
  for (i; i < 3; i++) {
    // The calloc function both creates space and initializes all can values to zero
    cans[i] = (can_ptr)calloc(sizeof(can_ref),1);
  } 
}

// Free all space allocated for the cans array
void breakdown_cans_array() {
  int i = 0;
  for(i; i < 3; i++) {
    free(cans[i]);
  }
}

// Convert the given radian angle into a degrees int16_t value between 0 and 2PI
int16_t convert_angle_flockbot(float rad) {
  // If rad is > 2PI or < -2PI, get it to a value in the range (-2PI, 2PI)
  if(rad > 0.0) {
    while(rad > TWO_PI){
      rad = rad - TWO_PI;
    }
  }
  else{
    while(rad < -TWO_PI){
      rad = rad + TWO_PI;
    }
  }

  // Convert the angle to degrees and a 16 bit integer
  return (int16_t) round(rad * DEG_CONV);
}

int16_t convert_xy_flockbot(float x, float y) {
  float rad = 0.0;
  int x_round = (int) round(x);
  int y_round = (int) round(y);

  if (x_round == 0) {
    if (y_round < 0) {
      rad = -PI_OVER_TWO;
    }
    else if (y_round > 0) {
      rad = PI_OVER_TWO;
    }
    else {
      rad = 0.0;
    } 
  }
  else if (y_round == 0) {
    if (x_round < 0) {
      rad = PI;
    }
    else {
      rad = 0.0;
    }
  }
  else {
    rad = atan(y / x);
  }
  
  return convert_angle_flockbot(rad);
}

// Drive to the desired (x,y) position in the original robot reference frame
void go_to_pos(float x, float y) {
  
  /*
   * First we need to turn back to the angle that we initially
   * began from. This is accomplished by using the get_angle()
   * function from the integrator, converting that to proper
   * input for the flockbot turn function, and executing the
   * turn.
   */
  int8_t speed = -ROBOT_SPEED;
  int16_t angle_to_origin = convert_angle_flockbot(get_angle());
  
  if(angle_to_origin > 0.0){
    speed = ROBOT_SPEED;
  }
  else {
    angle_to_origin = angle_to_origin * (-1.0);
  }
  
  printf("GoToPos Turning to Origin: Degrees=%d\n",angle_to_origin);
  turn_robot_wait(speed, angle_to_origin);

  /*
   * Now that we are back in the initial orientation, we can
   * figure out our how much we need to turn (angle_from_origin) 
   * and how far we need to go (total_dist) in order to acheive the 
   * desired position. We acheive both of these by shifting the 
   * translating the destination point to the original robot reference
   * frame and then calculating distance and angle to there as if we
   * are sitting at the origin and facing down the positive x-axis.
   */
  float* robot_pos = get_pos();
  float x_dist = x - robot_pos[0];
  float y_dist = y - robot_pos[1];
  int16_t total_dist = (int16_t) round(sqrt((x_dist*x_dist)+(y_dist*y_dist)));
  int16_t angle_from_origin = convert_xy_flockbot(x_dist,y_dist);

  speed = -ROBOT_SPEED;
  if(angle_from_origin > 0.0){
    speed = ROBOT_SPEED;
  }
  else {
    angle_from_origin = angle_from_origin * (-1.0);
  }

  printf("GoToPos Going to Position: Degrees=%d\tDistance=%d\n",angle_from_origin,total_dist);
  turn_robot_wait(speed, angle_from_origin);  
  move_distance_wait(ROBOT_SPEED, total_dist);
 
  // Finally, we free the position array obtained from the integrator
  free(robot_pos); // THIS MIGHT BREAK
}



/*****************************************************************************************************
 *                                                                                                   *
 *                                     MAIN TRIANGULATION FUNCTION                                   *
 *                                                                                                   * 
 *****************************************************************************************************/
int main() 
{
  /*** INITIALIZATION ***/
  srand(time(NULL)); // Seed the random number generator with the current time
  initialize_api(); 
  robot_connect("127.0.0.1");
  barcode_configure(2 /*number of digits*/, 10 /*barcodes per pass*/, 2 /*kept passes*/, 
                    2 /*skip pixel width*/, 2 /*min bar width*/, 100 /* Allowed skew */, -1 /*Otsu thresholding*/,
                    image_width /*image width*/, image_height /*image height*/);
  sleep(1);
  
  int** barcodes; // Pointer for a two-dimensional array, will hold returned barcodes
  int* xy; // Pointer to an array [x,y]
  camera_set(CAM_ANGLE); // Sets the tilt on the camera to the current defined camera angle
  
  // Create a thread to handle grabbing images for checking barcodes
  pthread_create(&vision_thread, NULL, barcode_start, NULL);
  
  // Initialize the array of cans to be found and the integrator
  initialize_cans_array();
  reset_integrator();

  // Grab the current barcodes in camera view
  barcode_frame_wait(); // Waits for the next frame from the current counter
  barcodes = barcode_get_barcodes(); // Retrieves the barcodes in the 2D array
  
  int num_codes = (int)barcodes[0];  // Number of barcodes detected
  int num_digits = (int)barcodes[1]; // Number of digits per barcode (will be 2)
  
  // This variable is used to keep track of how many unique cans we have seen so far
  int num_unique_cans_found = 0;
  
  /*
   * MAIN LOOP:
   *
   *     Search until we find all three *unique* cans.
   */
  while (num_unique_cans_found < 3) {

    // Code to shutdown when the high bump is pressed. Just in case we never find the cans...
    if (get_high_bump() == 1) {
      // Stop the robot, then break down all memory and threads used
      robot_stop();
      /*** Teardown Code ***/
      free(barcodes);
      breakdown_cans_array();
      vision_running = 0; // Tells the background thread to nicely finish
      pthread_join(vision_thread, NULL); // Waits nicely for the barcode module to shutdown
      shutdown_api(); 
      return 0; 
    }
    
    // Debug print of barcodes currently in view
    printf("Detected %d barcodes %d digits long:\n", num_codes, num_digits);
    
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
	
	/* Check to see if the current can has been seen before */
	int seen_can = 0;
	int can_iterator;
	for (can_iterator = 0; can_iterator < num_unique_cans_found; can_iterator++) {
	  if (sum == cans[can_iterator]->id) {
	    seen_can = 1;
	  }
	}
	
	/*
	 * If the can is unique, we must do all calculations to find its absolute x and y
	 * position according to the robot's initial reference frame and save those values.
	 */
	if (!seen_can) {
	  
	  /* Get the X and Y positions of the cans relative to the robot */
	  xy = barcode_get_cur_xy(sum);
	  int bottom_y = convert_camera_y(barcodes[i][j+3]);
	  float x_pos = can_xpos(bottom_y);
	  float y_pos = can_ypos(x_pos,convert_camera_x(xy[0]));

	  /* Save the id of the can and its position after transformation */
	  cans[num_unique_cans_found]->id = sum;
	  transformation(x_pos,y_pos,num_unique_cans_found);
	  
	  // Debug print statements
	  printf("Found Unique Barcode:\n");
	  printf("ID: %d\tX_CAM: %d\tY_CAM: %d\n",cans[num_unique_cans_found]->id,convert_camera_x(xy[0]),bottom_y);
	  printf("CAN_POS: (%f,%f)\n",cans[num_unique_cans_found]->x_pos,cans[num_unique_cans_found]->y_pos);
	  float* robot_pos = get_pos();
	  printf("ROBOT_POS: (%f,%f)\n",robot_pos[0],robot_pos[1]);
	  free(robot_pos); // THIS MIGHT BREAK
	  
	  // Remember that we found one more unique can
	  num_unique_cans_found++;
	  free(xy);
	}
	/* Otherwise, we just print out a debug statement */
	else {
	  printf("Found barcode %d, which is not unique.\n");
	}
      }
    
    // Keep the debug prints looking nice and free barcodes so we can reset it
    printf("\n\n");
    free(barcodes);
    
    /* If we are not done, move slightly and get the next barcode image */
    if (num_unique_cans_found < 3) {
      random_move((int)barcodes[0]);
      sleep(3);
      barcode_frame_wait_start(); 
      barcode_frame_wait(); 
      barcodes = barcode_get_barcodes(); 
      num_codes = (int)barcodes[0];  
      num_digits = (int)barcodes[1]; 
    }
  }
   
  // We don't need the camera or barcodes any more, so free them up
  vision_running = 0; // Tells the background thread to nicely finish
  pthread_join(vision_thread, NULL); // Waits nicely for the barcode module to shutdown
  
  /*** Calculate the centroid, and the distance and angle of rotation to it ***/
  float x_centroid = (cans[0]->x_pos + cans[1]->x_pos + cans[2]->x_pos) / 3.0;
  float y_centroid = (cans[0]->y_pos + cans[1]->y_pos + cans[2]->y_pos) / 3.0;
  printf("\n\nCentroid X: %f\nCentroid Y: %f\n",x_centroid,y_centroid);
  // Finally, drive to the centroid!
  go_to_pos(x_centroid,y_centroid);

  // Make sure the robot has stopped
  robot_stop();

  /*** Teardown Code ***/
  breakdown_cans_array();
  shutdown_api(); 
  return 0;
 }
