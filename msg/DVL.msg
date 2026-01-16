std_msgs/Header header
#Time
float32 time
#Measured velocity [m/s]
geometry_msgs/Vector3 velocity
#Velocity covariance matrix (row-major 3x3: xx, xy, xz, yx, yy, yz, zx, zy, zz)
float64[9] covariance
#Figure of Merit
float32 fom
#Altitude of the sensor
float32 altitude
#Beams/Transducers
DVLBeam[] beams
#Validity of velocity
bool velocity_valid
#Status message
int32 status
#Formatting of json
string form