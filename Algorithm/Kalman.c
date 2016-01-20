#include "Kalman.h"
#include "stm32f4xx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
/* Kalman filter variables */
float Q_angle = 0.001; //0.0004 //0.005 // Process noise variance for the accelerometer
float Q_bias = 0.003; //0.0002 //0.0003 //Process noise variance for the gyro bias
float R_measure = 0.03; //0.002 //0.008 // Measurement noise variance - this is actually the variance of the measurement noise
    
kalman_t kalmanX;
kalman_t kalmanY;
kalman_t kalmanZ;

kalman_single_t  kalman_single_X;
kalman_single_t  kalman_single_Y;
kalman_single_t  kalman_single_Z;

/*=====================================================================================================*/
/*=====================================================================================================*/
float kalman_filter_angle(kalman_t *kalman, float newAngle, float newRate, float looptime)
{
    //calculate dt
    //float dt = (float)(looptime)/1000;
    float dt = looptime;
    /* Step 1 
    Discrete Kalman filter time update equations - Time Update ("Predict")*/
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    /* Step 2 
    Update estimation error covariance - Project the error covariance ahead*/
    kalman->P_00 += dt * (dt*kalman->P_11 - kalman->P_01 - kalman->P_10 + Q_angle);
    kalman->P_01 -= dt * kalman->P_11;
    kalman->P_10 -= dt * kalman->P_11;
    kalman->P_11 += Q_bias * dt;

    /* Step 4 
    Calculate Kalman gain - Compute the Kalman gain*/
    kalman->S = kalman->P_00 + R_measure;   //(Pk-) +R
    /* Step 5 */
    kalman->K_0 = kalman->P_00 / kalman->S;        // K[0] = Kk = (Pk-)/ [(Pk-) +R]
    kalman->K_1 = kalman->P_10 / kalman->S;
          
    /* Step 3 
    Calculate angle and bias - Update estimate with measurement zk (newAngle)*/
    kalman->y = newAngle - kalman->angle;      // Zk - (Xk-)  = y
    /* Step 6 */
    kalman->angle += kalman->K_0 * kalman->y;         // Xk = (Xk-) + Kk * [Zk - (Xk-)]
    kalman->bias += kalman->K_1 * kalman->y;

    /* Step 7 
    Calculate estimation error covariance - Update the error covariance*/
    kalman->P_00 -= kalman->K_0 * kalman->P_00;   // Pk.new = (Pk-) - Kk*(Pk-)
    kalman->P_01 -= kalman->K_0 * kalman->P_01;
    kalman->P_10 -= kalman->K_1 * kalman->P_00;
    kalman->P_11 -= kalman->K_1 * kalman->P_01;
          
    return kalman->angle;
}
/*=====================================================================================================*/
/*=====================================================================================================*/

float kalman_single(kalman_single_t *kalman_single, float acc, float measure_noise, float process_noise)
{
  kalman_single->R=measure_noise*measure_noise;
  kalman_single->Q=process_noise*process_noise; 
 
 /********* calculate kalman ***************/
    kalman_single->P_ = kalman_single->P + kalman_single->Q;                     // P_ = A*P*A' + Q;
    kalman_single->K = kalman_single->P_/(kalman_single->P_ + kalman_single->R);                // K = P_*H'*inv(H*P_*H' + R);
    kalman_single->x_hat = kalman_single->x_hat + kalman_single->K*(acc - kalman_single->x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
    kalman_single->P = ( 1 - kalman_single->K)*kalman_single->P_ ;               // P = ( 1 - K*H)*P_ ;
 /****************************************/ 
 return kalman_single->x_hat;
}
/*=====================================================================================================*/
/*=====================================================================================================*/