#ifndef TU_ILMENAU_SOP_H
#define TU_ILMENAU_SOP_H

#include <string>
#include <vector>

//#define AUTO_A
#ifndef AUTO_A
#define AUTO_B
#endif


#ifdef AUTO_A
#define MAX_POSITIVE_STEERING_ANGLE           30.0//28.2  //Degree//20//31//right
#define MAX_NEGATIVE_STEERING_ANGLE           28.0//252.5.8 //25.8//19//26//25//left
#define POSITIVE_STEERING_ANGLE_TO_PERCENT    (90.0 / MAX_POSITIVE_STEERING_ANGLE)
#define NEGATIVE_STEERING_ANGLE_TO_PERCENT    (90.0 / MAX_NEGATIVE_STEERING_ANGLE)
#else
#define MAX_POSITIVE_STEERING_ANGLE           32.5//28.2  //Degree//20//31
#define MAX_NEGATIVE_STEERING_ANGLE           32//35  //25.8//19//26//25
#define POSITIVE_STEERING_ANGLE_TO_PERCENT    (90.0 / MAX_POSITIVE_STEERING_ANGLE)
#define NEGATIVE_STEERING_ANGLE_TO_PERCENT    (90.0 / MAX_NEGATIVE_STEERING_ANGLE)
#endif



#define CAMERA_DISTANCE         18
#define CAMERA_DISTANCE_MAX     25
#define CAMERA_DISTANCE_MIN     18  //The distance from the camera to the front in cm
#define REAR_CAMERA_DISTANCE    37  //The distance from the front camera to the rear camera in cm

#define CAR_CENTER_TO_FRONT     30
#define LASERSCANNER_CENTER_TO_CAR_FRONT     6



/* Vehicle parameters */
const double lr = 0.18;  // Distance from cog to the rear axis [m]
const double lf = 0.18;  // Distance from cog to the front axis [m]
const double l = lr + lf; // Total distance between axes [m]


#define PI 3.14159265358979323846

#define RADIAN_TO_DEGREES (180.0/PI)
#define DEGREES_TO_RADIAN (PI/180.0)


#define PARKING_READY_FLAG_OFF 0
#define PARKING_READY_FLAG_ON 1

#define UNMARKED_INTERSECTION    0
#define STOP_GIVE_WAY            1
#define PARKEN                   2
#define HAVEWAY                  3
#define GIVE_WAY                 5
#define PEDESTRIAN_CROSSING      6
#define TESTSTRECKER_A9          10
#define ROADWORKS                12
#define NO_TRAFFIC_SIGN          999
#define ROAD_SIGN_NOT_IN_XML     888


#define LANE_DETECTION       0x01
#define STOP_LINE_DETECTION  0x02
#define ADULT_DETECTION      0x04
#define CHILD_DETECTION      0x08


#define CROSSING_BUSY_L 0x01
#define CROSSING_BUSY_R 0x02
#define CROSSING_BUSY_T 0x04


enum  DRIVING_MODEL{
	CAR_STOP = 0, 
	LANE_KEEPING, 
	TURN_LEFT, 
	TURN_RIGHT, 
	STRAIGHT, 
	PARKING, 
	PULL_OUT_LEFT, 
	PULL_OUT_RIGHT, 
        MERGE_LEFT,
        EMERGENCY_BREAK,
        AVOIDANCE
};

enum  VEHICLE_DIRECTION{
        DEGREE_0 = 0,
        DEGREE_90,
        DEGREE_MINUS_90,
        DEGREE_180
};

enum  RAMPER_STATE{
        NO_RAMP = 0,
        RAMP_UP = 1,
        RAMP_IN = 2,
        RAMP_DOWN = 3
};

enum FLAG {OFF = 0, ON};
enum POINT_STATE {NORMAL, WARNING, EMERGENCY};
enum KURVE_PARAMETER {K, M, B};
enum COORDINATE {X ,Y, SPEED, HEADING, RADIUS};
enum ULTRASONIC {F_LEFT , F_CENTER_LEFT, F_CENTER, F_CENTER_RIGHT, F_RIGHT, S_LEFT, S_RIGHT, R_LEFT, R_CENTER, R_RIGHT};
enum DETECTIONMODE_t {LSEARCH, LTRACE, SL_SEARCH, SL_TRACE};
enum FindOneSideLane {SL_NotFound, SL_Left, SL_Right};
enum IMAGEP_ROCESSING {IMAGE_STOP, IMAGE_RUN};
enum CROSSING {CROSSING_FLAG_OFF, CROSSING_FLAG_STOP_LINE, CROSSING_FLAG_TRAFFIC_SIGNS, CROSSING_FLAG_INTERSECTION};
enum PEDESTRIAN_STATE {NO_PESDESTRIAN, PEDESTRIAN_GOING, PEDESTRIAN_LEAVING, PEDESTRIAN_CHILDREN};
enum CROSSING_VEHICLE_STATE {NO_VEHICLES, VEHICLES_RIGHT, VEHICLES_LEFT, VEHICLES_FRONT, VEHICLES_THERE};
enum STOP_DECISION_STATE {STOP_DECISION, NOSTOP_DECISION};

enum LIGHT {HEAD, BRAKE, REVERSE, HAZARD, LEFT, RIGHT};
enum parkingSlot{slot1, slot2, slot3, slot4};

/*! the enum for the different states of the state machine (i.e. states of the car) */
enum stateCar{stateCar_ERROR = -1, stateCar_READY = 0, stateCar_RUNNING = 1, stateCar_COMPLETE = 2, stateCar_STARTUP = -2};
/*! the enum for the different states of the state machine (i.e. states of the car) */
enum juryActions{action_STOP = -1, action_GETREADY = 0, action_START = 1};


enum SPACE_NAME{HOME, OFFICE, RESTAURANT, POST, SUPERMARKET,GARAGE};


typedef struct _CAR_POSITION_STRUCT
{
    float X_Position;
    float Y_Position;
    float HeadingAngle;
    float speed;
    float radius;

}CAR_POSITION_STRUCT;

typedef struct _CCONTROL_SIGNAL_STRUCT
{
    float speed;
    float steering;

}CONTROL_SIGNAL_STRUCT;


typedef struct _LANE_DETECTION_STRUCT
{
    float kurve_parameter[3];
    float detect_mode;
    float width;
    float left_or_right_line;
    float adult_flag;
    float child_flag;

}LANE_DETECTION_STRUCT;


typedef struct _REFERENCE_POINT
{
    float X[21];
    float Y[21];
}REFERENCE_POINT;

typedef struct _X_Y_POINT
{
    float X;
    float Y;

}X_Y_POINT;

typedef struct _SECTION_BOUNDARY
{
    float left;
    float right;
    float top;
    float bom;
    float HeadingAngle;
    X_Y_POINT center;
}SECTION_BOUNDARY;

typedef struct _LASER_SCANNER_DATA
{
    int number_of_scan_point;
    X_Y_POINT coordinate[360];
    int point_state[360];

}LASER_SCANNER_DATA;

typedef struct _ULTRASONIC_STR
{
    X_Y_POINT side_left;
    X_Y_POINT side_right;
    X_Y_POINT rear_left;
    X_Y_POINT rear_center;
    X_Y_POINT rear_right;
    int s_l_state;
    int s_r_state;
    int r_l_state;
    int r_c_state;
    int r_r_state;

}ULTRASONIC_STR;


typedef struct _MANEUVER_LIST
{
    int current_id;
    int number_of_id;
    int action[150][2];
    int state;
    bool ready_flag;
    bool stop_flag;

}MANEUVER_LIST;


#endif
