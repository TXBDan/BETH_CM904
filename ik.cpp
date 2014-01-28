#include <Arduino-compatibles.h>
#include "init.h"
#include "ik.h"



/******************************************************************************
 * Inverse Kinematics for hexapod
 *
 * FRONT VIEW       ^        ==0         0==
 *     /\___/\      |       |  0==[___]==0  |
 *    /       \     -Z      |               |
 *
 * TOP VIEW
 *    \       /     ^
 *     \_____/      |
 *  ___|     |___   X
 *     |_____|
 *     /     \      Y->
 *    /       \
 *****************************************************************************/


/*********************************************************************************************************
    runIK()
**********************************************************************************************************/
void runIK(){
  
    footPosCalc();
    legIK(); 
    driveServos();
}

/**********************************************************************************************************
    footPosCalc()
    Calculates necessary foot position (leg space) to acheive commanded body rotations, translations, and gait inputs
***********************************************************************************************************/
void footPosCalc(){
    
    float sinRotX, cosRotX, sinRotY, cosRotY, sinRotZ, cosRotZ;
    int totalX, totalY, totalZ;
    int tempFootPosX[6], tempFootPosY[6], tempFootPosZ[6];
    int bodyRotOffsetX[6], bodyRotOffsetY[6], bodyRotOffsetZ[6];
  
    //sinRotX = sin(radians(-commanderInput.bodyRotX)); // CHANGE COMMANDER TO GIVE RADIANS!!!
    sinRotX = sin(-commanderInput.bodyRotX); // CHANGE COMMANDER TO GIVE RADIANS!!!
    cosRotX = cos(-commanderInput.bodyRotX);
    sinRotY = sin(-commanderInput.bodyRotY);
    cosRotY = cos(-commanderInput.bodyRotY);
    sinRotZ = sin(-commanderInput.bodyRotZ);
    cosRotZ = cos(-commanderInput.bodyRotZ);
  
    for( int legNum=0; legNum<6; legNum++){ 
        //SerialUSB.print ("footPosCalc() Leg: "); SerialUSB.println (legNum);

        //sinRotZ = sin(radians(leg[legNum].bodyRotZ - commanderInput.bodyRotZ));
        //cosRotZ = cos(radians(leg[legNum].bodyRotZ - commanderInput.bodyRotZ));
    
        totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x; 
        totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
        totalZ = leg[legNum].initialFootPos.z + leg[legNum].legBasePos.z;

        bodyRotOffsetX[legNum] = round(( totalY*cosRotY*sinRotZ + totalY*cosRotZ*sinRotX*sinRotY + totalX*cosRotZ*cosRotY - totalX*sinRotZ*sinRotX*sinRotY - totalZ*cosRotX*sinRotY) - totalX);  
        bodyRotOffsetY[legNum] = round(  totalY*cosRotX*cosRotZ - totalX*cosRotX*sinRotZ         + totalZ*sinRotX         - totalY);
        bodyRotOffsetZ[legNum] = round(( totalY*sinRotZ*sinRotY - totalY*cosRotZ*cosRotY*sinRotX + totalX*cosRotZ*sinRotY + totalX*cosRotY*sinRotZ*sinRotX + totalZ*cosRotX*cosRotY) - totalZ);   
      
        // Calculated foot positions to acheive xlation/rotation input. Not coxa mounting angle corrected
        tempFootPosX[legNum] = leg[legNum].initialFootPos.x + bodyRotOffsetX[legNum] - commanderInput.bodyTransX + leg[legNum].footPos.x;      
        //SerialUSB.print("Foot Pos X: "); SerialUSB.println(tempFootPosX[legNum]);        
        tempFootPosY[legNum] = leg[legNum].initialFootPos.y + bodyRotOffsetY[legNum] - commanderInput.bodyTransY + leg[legNum].footPos.y;
        //SerialUSB.print("Foot Pos Y: "); SerialUSB.println(tempFootPosY[legNum]); 
        tempFootPosZ[legNum] = leg[legNum].initialFootPos.z + bodyRotOffsetZ[legNum] - commanderInput.bodyTransZ + leg[legNum].footPos.z;
        //SerialUSB.print("Foot Pos Z: "); SerialUSB.println(tempFootPosZ[legNum]); 
    }
     
    // Rotates X,Y about coxa to compensate for coxa mounting angles.
    leg[0].footPosCalc.x = round( tempFootPosY[0]*cos(COXA_ANGLE)   - tempFootPosX[0]*sin(COXA_ANGLE) ); 
    leg[0].footPosCalc.y = round( tempFootPosY[0]*sin(COXA_ANGLE)   + tempFootPosX[0]*cos(COXA_ANGLE) );
    //SerialUSB.print("Foot Pos Y: "); SerialUSB.println(leg[0].footPosCalc.y); 
    leg[0].footPosCalc.z =        tempFootPosZ[0];
    leg[1].footPosCalc.x = round( tempFootPosY[1]*cos(COXA_ANGLE*2) - tempFootPosX[1]*sin(COXA_ANGLE*2) );
    leg[1].footPosCalc.y = round( tempFootPosY[1]*sin(COXA_ANGLE*2) + tempFootPosX[1]*cos(COXA_ANGLE*2) );
    //SerialUSB.print("Foot Pos Y: "); SerialUSB.println(leg[1].footPosCalc.y); 
    leg[1].footPosCalc.z =        tempFootPosZ[1];
    leg[2].footPosCalc.x = round( tempFootPosY[2]*cos(COXA_ANGLE*3) - tempFootPosX[2]*sin(COXA_ANGLE*3) );
    leg[2].footPosCalc.y = round( tempFootPosY[2]*sin(COXA_ANGLE*3) + tempFootPosX[2]*cos(COXA_ANGLE*3) );
    //SerialUSB.print("Foot Pos Y: "); SerialUSB.println(leg[3].footPosCalc.y); 
    leg[2].footPosCalc.z =        tempFootPosZ[2];
    leg[3].footPosCalc.x = round( tempFootPosY[3]*cos(COXA_ANGLE*5) - tempFootPosX[3]*sin(COXA_ANGLE*5) );
    leg[3].footPosCalc.y = round( tempFootPosY[3]*sin(COXA_ANGLE*5) + tempFootPosX[3]*cos(COXA_ANGLE*5) );
    leg[3].footPosCalc.z =        tempFootPosZ[3];
    leg[4].footPosCalc.x = round( tempFootPosY[4]*cos(COXA_ANGLE*6) - tempFootPosX[4]*sin(COXA_ANGLE*6) );
    leg[4].footPosCalc.y = round( tempFootPosY[4]*sin(COXA_ANGLE*6) + tempFootPosX[4]*cos(COXA_ANGLE*6) );
    leg[4].footPosCalc.z =        tempFootPosZ[4];
    leg[5].footPosCalc.x = round( tempFootPosY[5]*cos(COXA_ANGLE*7) - tempFootPosX[5]*sin(COXA_ANGLE*7) );
    leg[5].footPosCalc.y = round( tempFootPosY[5]*sin(COXA_ANGLE*7) + tempFootPosX[5]*cos(COXA_ANGLE*7) );
    leg[5].footPosCalc.z =        tempFootPosZ[5];
    
//    for( int legNum=0; legNum<6; legNum++){ 
//        SerialUSB.print ("footPosCalc() (after coxa rotat) Leg: "); SerialUSB.println (legNum+1);
//        SerialUSB.print("footPosCalcX: "); SerialUSB.println(leg[legNum].footPosCalc.x);  //these are off by +/- 1
//        SerialUSB.print("footPosCalcY: "); SerialUSB.println(leg[legNum].footPosCalc.y);
//        SerialUSB.print("footPosCalcZ: "); SerialUSB.println(leg[legNum].footPosCalc.z);
//    }
    // ALL OK UP TO HERE
    
}


/**************************************************************************************************************
    legIK()
    Translates foot x,y,z positions (body space) to leg space and adds goal foot positon input (leg space).
    Calculates the coxa, femur, and tibia angles for these foot positions (leg space).
***************************************************************************************************************/
void legIK(){
  
    float CoxaFootDist, IKSW, IKA1, IKA2, tibAngle;
                                                         
    for( int legNum=0; legNum<6; legNum++ ){
      
        //SerialUSB.print ("legIK() Leg: "); SerialUSB.println (legNum+1);
   
        CoxaFootDist = sqrt( sq(leg[legNum].footPosCalc.y) + sq(leg[legNum].footPosCalc.x) );
        //SerialUSB.print("CoxaFootDist: "); SerialUSB.println(CoxaFootDist);       
        IKSW = sqrt( sq(CoxaFootDist-LENGTH_COXA) + sq(leg[legNum].footPosCalc.z) );
        //SerialUSB.print("IKSW: "); SerialUSB.println(IKSW);       
        IKA1 = atan2( (CoxaFootDist - LENGTH_COXA) , leg[legNum].footPosCalc.z );
        //SerialUSB.print("IKA1: "); SerialUSB.println(IKA1);       
        IKA2 = acos( (sq(LENGTH_TIBIA) - sq(LENGTH_FEMUR) - sq(IKSW) ) / (-2*IKSW*LENGTH_FEMUR) );
        //SerialUSB.print("IKA2: "); SerialUSB.println(IKA2);       
        tibAngle = acos( (sq(IKSW) - sq(LENGTH_TIBIA) - sq(LENGTH_FEMUR)) / (-2*LENGTH_FEMUR*LENGTH_TIBIA) );
        //SerialUSB.print("tibAngle: "); SerialUSB.println(tibAngle);
        
        leg[legNum].jointAngles.coxa  = M_PI_2 - atan2( leg[legNum].footPosCalc.y , leg[legNum].footPosCalc.x ); 
        leg[legNum].jointAngles.femur = M_PI_2 - (IKA1 + IKA2);
        leg[legNum].jointAngles.tibia = M_PI_2 - tibAngle;
                      
//        SerialUSB.print("Coxa Angle: "); SerialUSB.println(leg[legNum].jointAngles.coxa);
//        SerialUSB.print("Femur Angle: "); SerialUSB.println(leg[legNum].jointAngles.femur); 
//        SerialUSB.print("Tibia Angle: "); SerialUSB.println(leg[legNum].jointAngles.tibia);
    }
    
    // Applies necessary corrections to servo angles to account for hardware
    for( int legNum=0; legNum<3; legNum++ ){
        //SerialUSB.print ("legIK() Leg: "); SerialUSB.println (legNum+1);
        //SerialUSB.print("Leg Num: "); SerialUSB.println(legNum);
        leg[legNum].jointAngles.coxa  = leg[legNum].jointAngles.coxa;
        //SerialUSB.print("Coxa Position: "); SerialUSB.println(leg[legNum].jointAngles.coxa);
        leg[legNum].jointAngles.femur = leg[legNum].jointAngles.femur - 0.237;              // accounts for offset servo bracket on femur
        leg[legNum].jointAngles.tibia = leg[legNum].jointAngles.tibia + 0.958; //counters offset servo bracket on femur, accounts for 90deg mounting, and bend of tibia
        
        //SerialUSB.print("Coxa Angle: "); SerialUSB.println(leg[legNum].jointAngles.coxa);
        //SerialUSB.print("Femur Angle: "); SerialUSB.println(leg[legNum].jointAngles.femur); 
        //SerialUSB.print("Tibia Angle: "); SerialUSB.println(leg[legNum].jointAngles.tibia);
    }    
   for( int legNum=3; legNum<6; legNum++ ){
        //SerialUSB.print ("legIK() Leg: "); SerialUSB.println (legNum+1);
        leg[legNum].jointAngles.coxa  =   leg[legNum].jointAngles.coxa;
        leg[legNum].jointAngles.femur = -(leg[legNum].jointAngles.femur - 0.237);
        leg[legNum].jointAngles.tibia = -(leg[legNum].jointAngles.tibia + 0.958);
        
        //SerialUSB.print("Coxa Angle: "); SerialUSB.println(leg[legNum].jointAngles.coxa);
        //SerialUSB.print("Femur Angle: "); SerialUSB.println(leg[legNum].jointAngles.femur); 
        //SerialUSB.print("Tibia Angle: "); SerialUSB.println(leg[legNum].jointAngles.tibia);
    }
    
}



/*************************************************
    driveServos()
    Commands servos to angles in joinitAngles
    converts to AX12 definition of angular rotation and divides by number of degrees per bit. 300deg/1024bit
    0deg = 512 = straight servo
    ax12SyncWrite() writes all servo values out to the AX12 bus using the SYNCWRITE instruction
**************************************************/
void driveServos(){
  
    for( int legNum=0; legNum<6; legNum++ ){
      
        //SerialUSB.print ("legIK() Leg: "); SerialUSB.println (legNum+1);
       
        leg[legNum].servoPos.coxa  = round((abs( leg[legNum].jointAngles.coxa  - 3.665) - 1.047 )  / 0.0051);
        leg[legNum].servoPos.femur = round((abs( leg[legNum].jointAngles.femur - 3.665) - 1.047 )  / 0.0051);
        leg[legNum].servoPos.tibia = round((abs( leg[legNum].jointAngles.tibia - 3.665) - 1.047 )  / 0.0051);
        //leg[legNum].servoPos.tibia = round((abs( leg[legNum].jointAngles.tibia - 210) - 60 )  / 0.293);
//        SerialUSB.print("Coxa Servo: "); SerialUSB.println(leg[legNum].servoPos.coxa);
//        SerialUSB.print("Femur Servo: "); SerialUSB.println(leg[legNum].servoPos.femur); 
//        SerialUSB.print("Tibia Servo: "); SerialUSB.println(leg[legNum].servoPos.tibia);
    }
      
    syncWriteServos();  
}


