Header header
#Time
float32 time
#Measured velocity [m/s]
geometry_msgs/Vector3 velocity
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