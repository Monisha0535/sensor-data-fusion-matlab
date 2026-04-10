
/* Includes */
#include "mbed.h"
#include "XNucleoIKS01A2.h"


/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;

Serial pc(PA_2, PA_3, 921600);

union myInt16{
    int16_t value;
    uint8_t byte[2];
}axesx[3], axesg[3], ns;

union myfloat{
    float value;
    uint8_t byte[4];
}sp;

Timer tim; // To use a timer, we need to define a variable of type Timer

/* Simple main function */
int main() {
  uint8_t id;
  acc_gyro->set_g_odr(1666); // Set LSM6DSL Gyroscope output data rate.
  acc_gyro->set_x_odr(1666); // Set LSM6DSL Accelerometer output data rate.

  /* Enable all sensors */
  acc_gyro->enable_x(); // Enable LSM6DSL Accelerometer
  acc_gyro->enable_g(); // Enable LSM6DSL Gyroscope

while (1) {
    do{
    
  }while(pc.getc()!= 's');

  pc.putc('s');
  for (int i=0; i<4; i++) {
      sp.byte[i]=pc.getc();
  }

  pc.putc('s');
  for (int i=0; i<2; i++) {
      ns.byte[i]= pc.getc();
  }

  tim.reset(); // reset the timer
  tim.start(); // the counting begins with the invocation of a specific  method start()
 
  for(int i=0; i<ns.value; i++) {

    tim.reset();  
    acc_gyro->get_x_axes_raw((int16_t *)axesx); // read raw data from LSM6DSL Accelerometer
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            pc.putc(axesx[i].byte[j]);
        }
    }

    acc_gyro->get_g_axes_raw((int16_t *)axesg); // read raw data from LSM6DSL Gyroscope.
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            pc.putc(axesg[i].byte[j]);
        }
    }

    float elapsed_time=tim.read(); // the time elapsed since startup is obtained by invoking the read methods
    if (elapsed_time>sp.value) {
        elapsed_time=0;
    }
    wait(sp.value-elapsed_time); //I subtract the time taken by the cycle from the sampling time

/*
    Next, a check is made to see if the elapsed time elapsed_time
     is greater than the sampling time sp.value. If this is true, it means the time taken
     for the execution of the previous cycle has exceeded the desired sampling time.
     In this case, elapsed_time is set to 0, indicating that no interval has been exceeded
     of sampling. This is done to compensate for any delays or discrepancies
     in the execution time of the cycle. Finally, the wait() function is used to wait for the
     remaining time of the current cycle. The difference between the desired sampling time sp.value
     and the elapsed time elapsed_time represents the time remaining to wait before starting
     the next sampling cycle.
*/

  }
}
}