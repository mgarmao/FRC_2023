// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 *  This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *s
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Operator and Driver Controllers */
    public static final int CONTROLLER_OPERATOR = 0;
    public static final int JOYSTICK_LEFT = 1;
    public static final int JOYSTICK_RIGHT = 2;
    public static final double JOYSTICK_CURVE   = 5/3;
    //public static final double DRIVER_INPUT_CURVE = 3;
    public static final int DRIVETRAIN_GEAR_RATIO = 9;
    public static final double WHEEL_CIRCUMFRANCE = 18.06;

    /** Drive Train */
    public static final int DRIVE_FRONT_LEFT = 1;
    public static final int DRIVE_REAR_LEFT = 5;
    public static final int DRIVE_FRONT_RIGHT = 4;
    public static final int DRIVE_REAR_RIGHT = 2; 

    //** Intake Motor */
    public static final int INTAKE_WHEELS = 3; 
    public static final double INTAKE_SPEED = 0.99; //Subject to Josh
    public static final double EJECT_SPEED = 0.99; //Subject to Josh

    
    //Pneumatics
    public static final int COMPRESSOR = 0;
    public static final int [] INTAKE_SOLENOID = new int[] {2, 1};

    /** Elevator */
    public static final int ELEVATOR_LEFT = 6;
    public static final int ELEVATOR_RIGHT = 7;
    public static final double ELEVATOR_POWER = 0.90;//xxxxx
    public static final double ELEVATOR_POWER_BOOST = 0.9;
    public static final float ELEVATOR_LOWER_LIMIT = -73;
    public static final float ELEVATOR_UPPER_LIMIT = 0;
    public static final double EL_GEAR_RATIO = 9;
    public static final double EL_GEAR_CIRCUMFRANCE = 9;


    //arm
    public static int ARM = 8;
    public static double ARM_POWER = 0.35;
    public static final int ARM_LOWER_LIMIT = -62;
    public static final int ARM_UPPER_LIMIT = 67;
    public static final double ARM_LENGTH = 36;

    //wrist
    public static int WRIST = 9;
    public static double WRIST_MAX_POWER = 0.25;
    public static final float WRIST_LOWER_LIMIT = 0;
    public static final float WRIST_UPPER_LIMIT = 58;
    public static double Wrist_Position = 0;

    //Arm Deadzone
    public static double MIN_ARM_SITUATION = 30;
    public static double MIN_EL_EXTENTION_FOR_ARM = -18;
    public static double elPosition = 0;
    public static double UPPER_ARM_DEADZONE = 45;
    public static double LOWER_ARM_DEADZONE = 26;

    //Preset Positions
    ///X conew
    public static double CONE_FRONT_PICKUP_POV = 90;
    public static double CONE_FRONT_PICKUP_WRIST = 26;
    public static double CONE_FRONT_PICKUP_ARM =  53;
    public static double CONE_FRONT_PICKUP_EL = -1;
    
    /// Retract X
    public static double RETRACT_POV = 180;
    public static double RETRACT_WRIST = 6;
    public static double RETRACT_ARM =  -62;
    public static double RETRACT_EL = -74;

    //X hgh
    public static double CUBE_SCORE_HIGH_POV = 0;
    public static double CUBE_SCORE_HIGH_WRIST = 26; 
    public static double CUBE_SCORE_HIGH_ARM = 29;
    public static double CUBE_SCORE_HIGH_EL = -75.5;

    //Mid Cube X
    public static double CONE_SCORE_MID_POV = 270;
    public static double CONE_SCORE_MID_WRIST = 10; 
    public static double CONE_SCORE_MID_ARM = 28;
    public static double CONE_SCORE_MID_EL = -56;

    public static double CONE_PICKUP_WRIST = 51;
    public static double CONE_PICKUP_ARM = 101;
    public static double CONE_PICKUP_EL = -40;

    public static boolean elInPosition = false;
    public static boolean ARM_IN_POSITION = false;

    //changing each match
    public static double startYaw = 0;
    public static boolean startingConfig = false;
    public static int startingApriltag = 100;
    public static int requestedPipeline = 0;
    public static double currentAngle = 0;
}
