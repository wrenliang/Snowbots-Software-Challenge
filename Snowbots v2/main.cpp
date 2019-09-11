/**
 * Hey there! Welcome to the Snowbots Software Challenge!
 *
 * As a prospective team member, we'd like to see you demonstrate your skills
 * by programming our brand spankin' new Mars rover.
 * The goal is to make the rover drive to a set of waypoints, read in from a
 * text file. We'll provide a lot of the rough framework you'll need, including
 * a _swanky_ simulator, we just need you to fill in a couple of functions
 * below..........
 *
 */

#include <iostream>
#include <stack>
#include <vector>
#include <cmath>
#include <math.h>
#include <fstream>

/** Constants **/
/** (you probably don't need to care about these, and should *NOT* change them) **/
// Length of the robot (used for kinematic model in simulator)
static const double ROBOT_LENGTH = 0.2;
// The time step that the simulator takes on each iteration
static const double SIM_TIME_STEP = 0.05;
// The tolerance we want to come within for each waypoint
static const double WAYPOINT_TOLERANCE = 0.2;
// The physical limit for the robot turning angle
static const double MAX_TURN_ANGLE = M_PI/3;

/** You CAN change these constants **/
// The name of the file to read the waypoints in from
static const char* WAYPOINTS_FILE_NAME = "waypoints.txt";
// The maximum number of iterations the simulator will run (presuming
// "waypoints" is never empty)
static const double MAX_SIM_ITERS = 2000;


// A 2D Point In Space (measured in meters)
struct Point {
    double x;
    double y;
};

// A message giving the status of the robot
struct StatusMsg {
    // The current position of the robot
    Point position;
    
    // The current heading of the robot
    double heading;
};

// A command we can send to the robot to control it
struct MovementMsg {
    // An angle in radians, within [-pi/3, pi/3], with -pi/3 being turn right,
    // pi/3 being turn left
    double steering_angle;
    
    // How fast the robot should move, in [-1, 1] with -1 being reverse,
    // and 1 being forward. The value is in meters/second.
    double throttle;
};

/** Test Macros **/
// Test that two values are exactly equal
#define TEST_EQ(EXPECTED, ACTUAL) \
if (EXPECTED != ACTUAL) { \
std::cout << "FAILURE: " << #ACTUAL << " gave " << ACTUAL << ", but expected " << EXPECTED << std::endl; \
}

// Test that two values are equal to within some tolerance
#define TEST_EQ_WITHIN_TOLERANCE(EXPECTED, ACTUAL, TOLERANCE) \
if (std::abs(EXPECTED - ACTUAL) > TOLERANCE) { \
std::cout << "FAILURE: " << #ACTUAL << " gave " << ACTUAL << ", but expected " << EXPECTED << " (with tolerance of " << TOLERANCE << ")" << std::endl; \
}

// Pre-declarations for functions
double distanceBetweenPoints(Point p1, Point p2);
double angleBetweenPoints(Point p1, Point p2);
MovementMsg computeNewMovement(StatusMsg status);
void runTests();

// Waypoints to navigate to
std::stack<Point> waypoints;

int main(int argc, char** argv) {
    /** SETUP THE WAYPOINTS **/
    
    // Read the waypoints in from a file
    //std::ifstream infile(WAYPOINTS_FILE_NAME);
    
    std::ifstream inFile;
    inFile.open(WAYPOINTS_FILE_NAME);
    
    if(inFile.is_open()){
        std::cout << "opened\n";
    } else {
        std::cout << "error\n";
    }
    
    double x, y;
    while (inFile >> x >> y){
        waypoints.push({x,y});
        std::cout << x << "," << y << "\n";
    }
    
    // Flip the waypoints stack, since we want the first waypoint at the top
    auto waypoints_copy = waypoints;
    waypoints = {};
    while (!waypoints_copy.empty()) {
        Point waypoint = waypoints_copy.top();
        waypoints_copy.pop();
        waypoints.push(waypoint);
    }
    
    /** RUN THE SIMULATOR **/
    
    // This is our crude "simulator". You don't need to touch this, though
    // it might help to at least somewhat understand what we're doing.
    StatusMsg curr_robot_status{ (Point){0,0}, 0 };
    int curr_sim_iter = 0;
    
    std::cout << "x, y, heading" << std::endl << "-------------" << std::endl;
    while (!waypoints.empty() && curr_sim_iter < MAX_SIM_ITERS) {
        curr_sim_iter++;
        
        // Print Robot State
        std::cout << curr_robot_status.position.x << ", "
        << curr_robot_status.position.y << ", "
        << curr_robot_status.heading << std::endl;
        
        MovementMsg mv_msg = computeNewMovement(curr_robot_status);
        
        // Constrain turn angle to physical limitations
        mv_msg.steering_angle = std::min(std::max(-MAX_TURN_ANGLE, mv_msg.steering_angle), MAX_TURN_ANGLE);
        
        // Update Robot State
        // You don't need to understand this, but we're using a simple
        // "tricycle" ackermann model here to update the robot state
        double d_theta = (mv_msg.throttle / ROBOT_LENGTH) *
        std::tan(mv_msg.steering_angle);
        double d_x = mv_msg.throttle *
        std::sin(curr_robot_status.heading + (d_theta * SIM_TIME_STEP)/2.0);
        double d_y = mv_msg.throttle *
        std::cos(curr_robot_status.heading + (d_theta * SIM_TIME_STEP)/2.0);
        
        curr_robot_status.position.x += d_x * SIM_TIME_STEP;
        curr_robot_status.position.y += d_y * SIM_TIME_STEP;
        curr_robot_status.heading += d_theta * SIM_TIME_STEP;
    }
    
    if (waypoints.empty()) {
        std::cout << "All waypoints reached!!" << std::endl;
    }
    
    /** RUN TESTS **/
    runTests();
    
}

/* ------- YOUR CODE GOES BELOW HERE ------- */

void runTests() {
    // HINT: Add tests for your functions here!!
    std::cout << "-------------" << std::endl;
    std::cout << "Failed Tests:" << std::endl;
    std::cout << "-------------" << std::endl;
    
    TEST_EQ(10, distanceBetweenPoints({0,0}, {10, 0}));
    TEST_EQ(5, distanceBetweenPoints({0,0}, {3, 4}));
    
    // We don't test for exact equality here because of floating point error
    TEST_EQ_WITHIN_TOLERANCE(0, angleBetweenPoints({0,0}, {0, 10}), 0.0000001);
    TEST_EQ_WITHIN_TOLERANCE(M_PI/2, angleBetweenPoints({0,0}, {10, 0}), 0.0000001);
    
    std::cout << "-------------" << std::endl;
}

/**
 * Calculates the distance between two given points
 *
 * @param p1 the first point
 * @param p2 the second point
 *
 * @return the absolute distance between p1 and p2
 */
double distanceBetweenPoints(Point p1, Point p2) {
    // TODO #1
    double xDistance = abs(p1.x - p2.x);
    double yDistance = abs(p1.y - p2.y);
    double orthDistance = sqrt(pow(xDistance,2) + pow(yDistance,2)); //Pythagorean theorem
    
    return orthDistance;
}

/**
 * Calculates the angle from the first point to the second point
 *
 * The angle is measured assuming +Y is 0 radians, +X is +PI/2 radians,
 * -X is -PI/2 radians, -Y is -PI or +PI
 *
 * @param p1 the first point
 * @param p2 the second point
 *
 * @return the angle between between p1 and p2 in *radians*
 */

//NEED TO FIX IF GREATER THAN PI/2 BECAUSE IT NEEDS TO KNOW TO TURN AROUND
double angleBetweenPoints(Point p1, Point p2) {
    //TODO #2
    double angle;
    double deltaX = p2.x - p1.x;
    double deltaY = p2.y - p1.y;
    
    angle = tan(deltaX/deltaY);
    
    return angle;
}

/**
 * Calculate a new MovementMsg based on the given StatusMsg
 *
 * This MovementMsg should move the Robot towards the next waypoint.
 * If we're close enough (ie. within "WAYPOINT_TOLERANCE") to the waypoint,
 * this will update "waypoints" so that we start moving towards the next one.
 *
 * @param status The current status of the robot
 *
 * @return A MovementMsg moving the robot towards the next waypoint
 */
MovementMsg computeNewMovement(StatusMsg status) {
    // TODO #3
    // NOTE: The "simulator" will stop once "waypoints" is empty
    // HINT: You are allowed to change the waypoints file for testing (by
    //       modifying "WAYPOINTS_FILE_NAME" at the top of this file) (but we
    //       will be testing with our own secret waypoints!)
    // HINT: You probably want to use the functions you filled in above....
    // HINT: If you want to visualize what your robot is doing, you probably
    //       want to take the points that the simulator prints and put them
    //       into a scatter plot in excel (or equivalent). You may also want
    //       to change "MAX_SIM_ITERS" at the top of this file to limit the
    //       number of simulator iterations at first
    
    
    /*CONSTANTS*/
    static const double THRESHOLD_ANGLE = M_PI/60;
    static const double TURNING_THROTTLE = 0.2;
    static const double DISTANCE_THRESHOLD = 0.1;
    
    MovementMsg outputMsg = { 0, 0 }; //initialized to arbitrary value, will be changed later
    
    double targetAngle = angleBetweenPoints(status.position, waypoints.top()); //gives the unit circle angle that the robot should be heading at

    double angleDiff = targetAngle - status.heading; //difference between target angle and current heading
    
    //if close enough, pop the top waypoint and move to next waypoint
    if( abs(status.position.x - waypoints.top().x) <= DISTANCE_THRESHOLD && abs(status.position.y - waypoints.top().y) <= DISTANCE_THRESHOLD ){
        std::cout << "Waypoint reached! Popping off stack \n";
        waypoints.pop();
    } else {
        //if the heading is close enough, we can drive quickly and make small turning adjustments
        if( abs(angleDiff) < THRESHOLD_ANGLE) {
            outputMsg.steering_angle = angleDiff;
            outputMsg.throttle = 1;
        } else {
            //checks if turn angle is possible
            if (abs(angleDiff) > MAX_TURN_ANGLE) {
                std::cout << "Max turn reached, drive with turn throttle\n";
                
                if(angleDiff < 0){
                    outputMsg.steering_angle = -M_PI/3;
                } else {
                    outputMsg.steering_angle = M_PI/3;
                }
                
                //drive with slower speed while at max turning circle
                outputMsg.throttle = TURNING_THROTTLE;
                
            } else {
                std::cout << "Regular turn, turn throttle\n";
                outputMsg.steering_angle = angleDiff;
                outputMsg.throttle = TURNING_THROTTLE;
            }
        }
    }
    

    return outputMsg;
}
