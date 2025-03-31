Header header
#Time
float64 time
#Measured velocity [m/s]
geometry_msgs/Vector3 velocity
#Figure of Merit
float64 fom
#Altitude of the sensor
float64 altitude
#Beams/Transducers
DVLBeam[] beams
#Validity of velocity
bool velocity_valid
#Status message
int64 status
#Formatting of json
string form
#Dead reckoning position
geometry_msgs/Point position
#Dead reckoning attitude: roll, pitch, yaw
float64[] attitude
#Standard deviation (figure of merit) for position (m)
float64 std