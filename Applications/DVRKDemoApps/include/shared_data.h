#ifndef SHARED_DATA_H
#define SHARED_DATA_H
//============================================================================
#include "robot/CDaVinciRobot.h"
#include "robot/CDaVinciRobotMTM.h"
#include "robot/CDaVinciRobotPSM.h"
#include "timer/CPrecisionClock.h"
#include "math/CMath.h"
//============================================================================
enum robotControlType
{
  JointCtrlWithGC,
  CartesianCtrlWithGC,
  NoCtrlWithGC
};

//============================================================================
//---------------------------Shared data structure----------------------------
typedef struct
{
    FirewirePort* p_port;
    CDaVinciRobotMTM* p_daVinciMTML;
    CDaVinciRobotMTM* p_daVinciMTMR;
    CDaVinciRobotPSM* p_daVinciPSM1;
    CDaVinciRobotPSM* p_daVinciPSM2;

    // MTML variables
    cVector3d m_daVinciMTMLPosition;
    cVector3d m_daVinciMTMLVelocity;
    cMatrix3d m_daVinciMTMLRotation;
    cVector3d m_daVinciMTMLVelocityAngular;
    double m_daVinciMTMLGripAngle;

    double m_daVinciMTMLActuatorAngles[8];
    double m_daVinciMTMLJointAngles[8];
    double m_daVinciMTMLJointVeloicities[8];

    cVector3d m_daVinciMTMLForce;
    cVector3d m_daVinciMTMLTorque;
    double m_daVinciMTMLJointForces[8];

    // MTMR variables
    cVector3d m_daVinciMTMRPosition;
    cVector3d m_daVinciMTMRVelocity;
    cMatrix3d m_daVinciMTMRRotation;
    cVector3d m_daVinciMTMRVelocityAngular;
    double m_daVinciMTMRGripAngle;

    double m_daVinciMTMRActuatorAngles[8];
    double m_daVinciMTMRJointAngles[8];
    double m_daVinciMTMRJointVeloicities[8];

    cVector3d m_daVinciMTMRForce;
    cVector3d m_daVinciMTMRTorque;
    double m_daVinciMTMRJointForces[8];

    // Foot Pedal variables
    bool m_footPedalInput[6];
    bool m_footPedalClutch;

    // PSM1 variables
    cVector3d m_daVinciPSM1Position;
    cVector3d m_daVinciPSM1Velocity;
    cMatrix3d m_daVinciPSM1Rotation;
    cVector3d m_daVinciPSM1VelocityAngular;
    double m_daVinciPSM1GripAngle;
    double m_daVinciPSM1GripVelocity;

    double m_daVinciPSM1ActuatorAngles[8];
    double m_daVinciPSM1JointAngles[8];
    double m_daVinciPSM1JointVeloicities[8];

    cVector3d m_daVinciPSM1Force;
    cVector3d m_daVinciPSM1Torque;
    double m_daVinciPSM1JointForces[8];
    double m_daVinciPSM1GripperForce;

    bool m_daVinciPSM1ArmInput;
    bool m_daVinciPSM1SterileAdapterInserted;
    bool m_daVinciPSM1ToolInserted;

    // PSM2 variables
    cVector3d m_daVinciPSM2Position;
    cVector3d m_daVinciPSM2Velocity;
    cMatrix3d m_daVinciPSM2Rotation;
    cVector3d m_daVinciPSM2VelocityAngular;
    double m_daVinciPSM2GripAngle;
    double m_daVinciPSM2GripVelocity;

    double m_daVinciPSM2ActuatorAngles[8];
    double m_daVinciPSM2JointAngles[8];
    double m_daVinciPSM2JointVeloicities[8];

    cVector3d m_daVinciPSM2Force;
    cVector3d m_daVinciPSM2Torque;
    double m_daVinciPSM2JointForces[8];
    double m_daVinciPSM2GripperForce;

    bool m_daVinciPSM2ArmInput;
    bool m_daVinciPSM2SterileAdapterInserted;
    bool m_daVinciPSM2ToolInserted;

    // Manipulator position offset
    cVector3d m_mtmlPositionOffset;
    cMatrix3d m_mtmlRotationOffset;

    cVector3d m_mtmrPositionOffset;
    cMatrix3d m_mtmrRotationOffset;

    cVector3d m_psm1PositionOffset;
    cMatrix3d m_psm1RotationOffset;

    cVector3d m_psm2PositionOffset;
    cMatrix3d m_psm2RotationOffset;

    // Master-Slave orientation error
    double m_mtmlpsm1RotationError;
    double m_mtmrpsm2RotationError;

    // Master and slave robot joint control parameters
    double m_mtmJointKp[8];
    double m_mtmJointKd[8];
    double m_psmJointKp[8];
    double m_psmJointKd[8];

    // Master and slave robot cartesian control parameters
    cVector3d m_mtmCartesianKpForce;
    cVector3d m_mtmCartesianKdForce;
    cVector3d m_mtmCartesianKpTorque;
    cVector3d m_mtmCartesianKdTorque;
    cVector3d m_psmCartesianKpForce;
    cVector3d m_psmCartesianKdForce;
    cVector3d m_psmCartesianKpTorque;
    cVector3d m_psmCartesianKdTorque;
    double m_gripperKp;
    double m_gripperKd;

    // Master and slave desired Cartessian coordinates
    cVector3d m_daVinciMTMLDesiredCartesianCoord;
    cVector3d m_daVinciMTMRDesiredCartesianCoord;
    cVector3d m_daVinciPSM1DesiredCartesianCoord;
    cVector3d m_daVinciPSM2DesiredCartesianCoord;

    // Master and slave desired joint angles (variable)
    double m_daVinciMTMLJointAnglesDesired[8];
    double m_daVinciMTMRJointAnglesDesired[8];
    double m_daVinciPSM1JointAnglesDesired[8];
    double m_daVinciPSM2JointAnglesDesired[8];

    // Master and slave home angles (fixed)
    double m_daVinciMTMLJointHomeAngles[8];
    double m_daVinciMTMRJointHomeAngles[8];
    double m_daVinciPSM1JointHomeAngles[8];
    double m_daVinciPSM2JointHomeAngles[8];

    // Orientation misalignment threshold
    double m_misalignmentThreshold;

    // A shared clock
    cPrecisionClock sharedClock;

    // Slave to master base frame rotation and vice versa
    cMatrix3d m_slaveToMasterRotation;
    cMatrix3d m_masterToSlaveRotation;

//============================================================================
    // ------------------- Flags ------------------------
    // Flags for state machine
    bool m_flagHomeRobot;
    bool m_flagStartTeleoperation;

    // Flags for robot control types
    robotControlType m_flagMTMLControlType;
    robotControlType m_flagPSM1ControlType;
    robotControlType m_flagMTMRControlType;
    robotControlType m_flagPSM2ControlType;

    // Flags for power on and relay on
    bool m_flagEnableSafetyRelay;
    bool m_flagEnablePower;

    // Flags for threads
    bool m_flagControlThreadEnd;
    bool m_flagStateMachineThreadEnd;
    bool m_flagDisplayThreadEnded;

    // Flags for head presence
    bool m_flagHeadPresent;

    int m_controlRate;

}sd_SharedData;


#endif
