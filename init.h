#ifndef INIT_H
#define INIT_H

//const int ledPin = 0;
const int rideHeightOffset = -32; // 32 means body at 100mm off ground


/* Servo IDs */
const int RM_TIBIA_ID   = 18;
const int RF_COXA_ID    = 2;
const int LR_TIBIA_ID   = 11;
const int LF_FEMUR_ID   = 3;
const int RF_TIBIA_ID   = 6;
const int RM_FEMUR_ID   = 16;
const int RM_COXA_ID    = 14;
const int RR_COXA_ID    = 8;
const int LF_TIBIA_ID   = 5;
const int LF_COXA_ID    = 1;
const int LR_FEMUR_ID   = 9;
const int RR_FEMUR_ID   = 10;
const int LM_TIBIA_ID   = 17;
const int RF_FEMUR_ID   = 4;
const int LM_FEMUR_ID   = 15;
const int RR_TIBIA_ID   = 12;
const int LM_COXA_ID    = 13;
const int LR_COXA_ID    = 7;

const int RIGHT_FRONT   = 0;
const int RIGHT_MIDDLE  = 1;
const int RIGHT_REAR    = 2;
const int LEFT_REAR     = 3;
const int LEFT_MIDDLE   = 4;
const int LEFT_FRONT    = 5;


/* Body Dimensions */
const int X_COXA        = 120;    // MM between front and back legs /2
const int Y_COXA_FB     = 60;     // MM between front/back legs /2
const int Y_COXA_M      = 100;    // MM between two middle legs /2
const float COXA_ANGLE  = M_PI_4; //45deg;     // Angle of coxa from straight forward (deg)

/* Legs */
const int LENGTH_COXA   = 52;     // MM distance from coxa servo to femur servo
const int LENGTH_FEMUR  = 66;     // MM distance from femur servo to tibia servo
const int LENGTH_TIBIA  = 132;    // MM distance from tibia servo to foot

const int SERVO_UPDATE_PERIOD = 20; //ms

/* Parameters for Commander input */
typedef struct{
  int   Xspeed;
  int   Yspeed;
  int   Rspeed;
  int   bodyTransX;
  int   bodyTransY;
  int   bodyTransZ;
  float bodyRotX;
  float bodyRotY;
  float bodyRotZ;
}commanderStruct;
extern commanderStruct commanderInput;


/* Leg parts */

typedef struct{
  float coxa;
  float femur;
  float tibia;
} floatAnglesStruct; //for joint angles

typedef struct{
  int coxa;
  int femur;
  int tibia;
} intAnglesStruct; //for servo position

typedef struct{
  float x;
  float y;
  float z;
} floatCoordsStruct; //for x,y,z coords in millimeters

typedef struct{
  int x;
  int y;
  int z;
} intCoordsStruct; //for x,y,z coords in millimeters

typedef struct{
  intCoordsStruct       footPos;
  intCoordsStruct       footPosCalc;
  floatAnglesStruct     jointAngles;
  intAnglesStruct       servoPos;
  intCoordsStruct       initialFootPos;
  intCoordsStruct       legBasePos;
  float                 bodyRotZ;
} legStruct;
extern legStruct        leg[6];


/* Body parts */
typedef struct{
  floatAnglesStruct    angles;
  intCoordsStruct      pos;
} bodyStruct;
//extern bodyStruct body;
  


void readCommandInputs();
void runIK();
void legIK();
void bodyIK();
void driveServos();
void legAngleCorrections();
void tripodGait();
void rippleGait();
void syncWriteServos();



#endif
