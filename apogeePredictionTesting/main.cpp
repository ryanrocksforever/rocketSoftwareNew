#include <iostream>
#include <sstream>
#include <cmath>
#include<windows.h>           // for windows
using namespace std;

double dragConst = 0.561; // average drag coefficient
double mass = 0.61; // mass of rocket
double g = 9.80665;

double getAcceleration(double zAccel, double angle) {//event is a struct of type sensors_event_t
	
	double theta = angle;
	//using spherical coordinates
	double a = cos(theta)*zAccel;
	return a;
}

/*
This function uses the average of two accelerometer readings multiplied by
the elapsed time, summed with the previous velocity reading.
Thus it integrates over the acceleration to find velocity.

velocity is a global variable that is initialized to 0 on the pad
*/
// double getVelocity(void) {
// 	time_t start,end;
// 	double acc_first,acc_second;
// 	start = time(NULL); //get begin time
// 	acc_first = getAcceleration();//get first acceleration
// 	delay(100);//delay for 100ms
// 	acc_second = getAcceleration();//get acceleration again
// 	end = time(NULL);//get finish time
// 	velocity=velocity + (end-start)*(acc_first+acc_second)/(2);//Vo + at
// 	return velocity;
// }

// /*
// Using spherical coordinates, get angle of rocket to vertical
// event is a structure of type sensors_event_t which holds information from the
// gyro sensor used to derive the angles
// */
// double getAngleToVertical(double angle){
// 	// hardware read
// 	sensors_event_t event;//orient event
// 	bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);//get orientation
// 	double x=event.orientation.x*180/pi;//get angle to x in radians, convert to deg
// 	double y=event.orientation.y*180/pi;//get angle to y in radians, convert to deg
	
// 	// conversion to spherical
// 	double theta=atan(y/x);//rectangular to spherical conversion
// 	return theta;
// }

double getDrag(double velocity, double a) {
	//double a = getAcceleration();
	// from mg+cv^2=a, we rearrange to solve for c
	double c = (a-mass*g)/(velocity*velocity); 
	return c;
}

double referenceArea = 0.35;
double airDensity = 1.2;  // air density
double getProjectedAltitude(double Cd,double v,double alt) {
  double y = (mass/Cd)*log(cos(atan(v/sqrt(mass*g/Cd)))); // Calculate additional altitude projected
  //double deltaHeight = (v * v) / (2 * g) * (mass / (Cd * referenceArea)) +
                        (v / g) * log(mass / (mass - (airDensity * referenceArea * Cd * v * v) / (2 * g)));
  double ypro = y ;  // Projected altitude is the additional + current alt
  return ypro;
}

//digitalWrite(PYRO_PINL, HIGH);
  //delay(1000);
  //digitalWrite(PYRO_PINL, LOW);
  double angles1[] = {80.267, 80.319, 80.382, 80.345, 80.179, 79.972, 79.795, 79.754, 79.818, 79.871, 79.828, 79.635, 79.367, 79.104, 78.858, 78.672, 78.57, 78.567, 78.655, 78.776, 78.845, 78.773, 78.537, 78.174, 77.775, 77.448, 77.254, 77.159, 77.113, 77.115, 77.147, 77.15, 77.08, 76.892, 76.551, 76.117, 75.712, 75.408, 75.227, 75.166, 75.204, 75.282, 75.323, 75.262, 75.062, 74.712, 74.246, 73.701, 73.125, 72.587, 72.137, 71.789, 71.538, 71.397, 71.359, 71.367, 71.352, 71.27, 71.104, 70.849, 70.517, 70.137, 69.714, 69.221, 68.663, 68.079, 67.484, 66.88, 66.274, 65.665, 65.054, 64.437, 63.802, 63.153, 62.483, 61.778, 61.041, 60.251, 59.407, 58.519, 57.58, 56.585, 55.532, 54.42, 53.251, 52.025, 50.747, 49.425, 48.076, 46.712, 45.319, 43.879, 42.386, 40.832, 39.212, 37.516, 35.737, 33.868, 31.905, 29.849, 27.693, 25.43, 23.058, 20.584, 18.011};
  double altitudes1[] = {114.416, 115.81, 117.883, 120.862, 123.8, 126.697, 129.553, 132.369, 135.146, 137.883, 140.581, 143.242, 145.864, 148.449, 150.996, 153.507, 155.981, 158.419, 160.82, 163.187, 165.518, 167.814, 170.077, 172.305, 174.499, 176.659, 178.786, 180.88, 182.941, 184.969, 186.964, 188.928, 190.86, 192.761, 194.63, 196.468, 198.275, 200.052, 201.798, 203.513, 205.198, 206.853, 208.478, 210.074, 211.641, 213.179, 214.688, 216.168, 217.62, 219.043, 220.438, 221.804, 223.142, 224.451, 225.733, 226.987, 228.214, 229.413, 230.585, 231.73, 232.848, 233.94, 235.004, 236.043, 237.054, 238.039, 238.998, 239.931, 240.837, 241.718, 242.572, 243.401, 244.203, 244.98, 245.731, 246.457, 247.157, 247.832, 248.482, 249.106, 249.706, 250.28, 250.829, 251.354, 251.853, 252.328, 252.778, 253.204, 253.604, 253.98, 254.332, 254.659, 254.961, 255.239, 255.493, 255.723, 255.929, 256.112, 256.27, 256.405, 256.517, 256.605, 256.67, 256.713, 256.732};
  double speeds1[] = {60.965, 60.576, 59.998, 59.167, 58.345, 57.53, 56.723, 55.923, 55.134, 54.355, 53.586, 52.826, 52.07, 51.322, 50.58, 49.845, 49.116, 48.394, 47.679, 46.973, 46.276, 45.586, 44.902, 44.222, 43.546, 42.872, 42.204, 41.543, 40.888, 40.237, 39.593, 38.956, 38.324, 37.697, 37.076, 36.456, 35.837, 35.222, 34.61, 34.002, 33.4, 32.803, 32.213, 31.628, 31.047, 30.469, 29.893, 29.32, 28.747, 28.176, 27.606, 27.04, 26.475, 25.914, 25.356, 24.804, 24.256, 23.713, 23.172, 22.633, 22.096, 21.56, 21.028, 20.498, 19.969, 19.441, 18.915, 18.39, 17.868, 17.347, 16.827, 16.31, 15.795, 15.281, 14.769, 14.259, 13.751, 13.246, 12.741, 12.238, 11.736, 11.236, 10.738, 10.24, 9.744, 9.249, 8.755, 8.261, 7.767, 7.273, 6.783, 6.294, 5.807, 5.323, 4.84, 4.36, 3.882, 3.407, 2.934, 2.464, 1.998, 1.535, 1.075, 0.616, 0.161};
  double accels1[] = {1, 1.27, 1.536, 1.482, 1.251, 1.077, 0.805, 0.835, 1.086, 1.257, 1.368, 1.194, 0.975, 0.954, 0.841, 0.769, 0.712, 0.698, 0.803, 0.972, 1.164, 1.208, 1.116, 0.895, 0.637, 0.473, 0.542, 0.653, 0.634, 0.644, 0.765, 0.853, 0.94, 1.026, 0.887, 0.58, 0.398, 0.337, 0.331, 0.356, 0.481, 0.639, 0.783, 0.852, 0.881, 0.788, 0.691, 0.591, 0.413, 0.294, 0.242, 0.258, 0.2, 0.206, 0.314, 0.473, 0.556, 0.587, 0.577, 0.549, 0.464, 0.439, 0.502, 0.493, 0.387, 0.343, 0.335, 0.314, 0.307, 0.299, 0.292, 0.323, 0.304, 0.308, 0.34, 0.322, 0.364, 0.369, 0.333, 0.343, 0.356, 0.355, 0.353, 0.347, 0.338, 0.33, 0.304, 0.26, 0.226, 0.247, 0.29, 0.301, 0.315, 0.329, 0.349, 0.372, 0.393, 0.403, 0.403, 0.422, 0.436, 0.439, 0.418, 0.396, 0.385};
  //note acceleration is lateral accel ex what the sensor returns
  double angles[] = {80.267, 80.319, 80.382, 80.345, 80.179};
  double altitudes[] = {114.416, 115.81, 117.883, 120.862, 123.8};
  double speeds[] = {60.965, 60.576, 59.998, 59.167, 58.345};
  double accels[] = {1, 1.27, 1.536, 1.482, 1.251};
  
  double alt, v, Cd;
  // projectedApogee = calcApogee(0, dragConst, 80, 58);
  // Serial.println(projectedApogee);
  // projectedApogee = calcApogee(0, dragConst, 78, 56);
  // Serial.println(projectedApogee);
  bool notappoge = true;
  int index = 0;
  int targetVel;
  int actualVel;
  double InputY;
  double angle;


int main() {
  //demolaunch with preset values
  cout << "DEMO LAUNCh\n";

  
  for(double loopangle : altitudes) {
    
    //Serial.print("start loop 2-2");

                cout << index;
                // if (true) {
                     

                //     // angle = arccosine(w) * 2
                     angle = angles[index];
                //     lastAngle = angle;
                  

                // } else {
                //   angle = lastAngle;
                // }
                 
                    
                

                //InputP = (pitch);
                cout << "\n";
                cout << "Data\n";
                cout << altitudes[index];
                cout << dragConst;
                cout << angle;
                cout << speeds[index];
                //projectedApogeeList = calcApogeeList(0, dragConst, angle, speeds[index]);
                
                //Serial.println((char*)(projectedApogeeList));
                
                //projectedApogee = calcApogee(0, dragConst, angle, speeds[index]);
                
                


                alt = altitudes[index];//get data from altimeter
                cout << alt;
                cout << "\n";
                v = speeds[index];//get velocity from altimeter do50hz funct
                cout << v;
                cout << "\n";
                cout << getAcceleration(accels[index], angle);
                cout << "\n";
                //Cd = getDrag(v, getAcceleration(accels[index], angle));
                
                //Serial.println(Cd);
                Cd = 0.5;
                InputY = getProjectedAltitude(Cd, v, alt);//calculate Input to PID
                
                cout << "ProjectedApogee: ";
                
                cout << InputY;
                cout << "\n";
                // if (projectedApogee < 100 || projectedApogee > 400)
                // {
                //   Serial.println("ProjectedApogee Failed");
                //   Serial.println(projectedApogee);
                //   //projectedApogee = 250;
                  
                // }
                //SetpointY = targetVel
                //InputY = actualVel;
                //upPID.Compute();
                //yawPID.Compute();
                //pitchPID.Compute();
                
                
                //Serial.print("yaw"); // limits 153 front and 100 back
                //Serial.print(yservopos);
                //Serial.print("\n");
                
               
             

                index ++;
                //Sleep(0.0015);
  }
  //appogee hit, deploy chute
  
  

}
