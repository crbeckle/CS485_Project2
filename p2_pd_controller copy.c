#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "flockbot_api.h"

#define K -1

int16_t lffir_zero, rffir_zero;

// The P Controller function
int16_t p_controller(int16_t d, int16_t zero_point) {
  // Force = K * x
  return K*(zero_point - d);
}

// Debug printing function
void print_sensors(int t) {
  int num_times = 0;
  while (num_times < t) {
    printf("LFFIR: %d\n", get_avg_ir(0));
    printf("RFFIR: %d\n", get_avg_ir(4));
    sleep(1);
    num_times++;
  }
}

// Calibrate the zero point for the IR sensors 
void calibrate_ir_zero() {
  lcd_clear();
  lcd_set_cursor(1,1);
  lcd_write("Calibrating");
  lcd_set_cursor(2,1);
  lcd_write("Zero Points");

  // Wait until we can get an average reading
  sleep(1);
  // Grab average IR values and calibrate to those
  lffir_zero = get_avg_ir(0);
  rffir_zero = get_avg_ir(4);
  printf("LFFIR Zero: %d\n",lffir_zero);
  printf("RFFIR Zero: %d\n",rffir_zero);
  
  sleep(2);
}

int main()
{
  // Initialization
  initialize_api();
  robot_connect("127.0.0.1");
  
  // Uncomment for debug printing
  //print_sensors(30);
  
  int16_t force_left, force_right, lffir, rffir;
  int8_t high_bump;
  calibrate_ir_zero();
  
  lcd_clear();
  lcd_set_cursor(1,1);
  lcd_write("Running");

  // Run the p controller until the bumper is hit
  while (1) {
    high_bump = get_high_bump();
    if (high_bump == 1) {
      break;
    }
    // Grab the average IR values and zero out erratic values
    lffir = get_avg_ir(0);
    if (lffir > 100) {lffir = 0;}
    rffir = get_avg_ir(4);
    if (rffir > 100) {rffir = 0;}

    // Find the p controller forces for both wheels and set them
    force_left = p_controller(lffir, lffir_zero);
    force_right = p_controller(rffir, rffir_zero);
    move_wheels(force_left,force_right);
    usleep(40000);
  }
  
  robot_stop();
  
  // Shutdown code
  shutdown_api();
  return 0;
}
