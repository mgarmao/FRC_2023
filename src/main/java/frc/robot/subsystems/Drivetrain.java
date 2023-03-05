package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.RobotContainer.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax m_motorFrontLeft, m_motorRearLeft, m_motorFrontRight, m_motorRearRight;
    private MotorControllerGroup m_motorsLeft, m_motorsRight;
    private DifferentialDrive m_drivetrain;
    private RelativeEncoder FL_encoder;
    boolean movingDistance= false;
    double distanceToMove;

    double kP0 = 0.02;
    double kI0 = 0.0;
    double kD0 = 0.04;

    double kP1 = 0.025;
    double kI1 = 0.0;
    double kD1 = 0.01;
  
    PIDController driveToDistancePID = new PIDController(kP0, kI0, kD0);
    PIDController keepAnglePID = new PIDController(kP1, kI1, kD1);

    public Drivetrain() {
        /** Create new objects to control the SPARK MAX motor controllers. */
        m_motorFrontLeft = new CANSparkMax(Constants.DRIVE_FRONT_LEFT, MotorType.kBrushless);
        m_motorRearLeft = new CANSparkMax(Constants.DRIVE_REAR_LEFT, MotorType.kBrushless);
        m_motorFrontRight = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT, MotorType.kBrushless);
        m_motorRearRight = new CANSparkMax(Constants.DRIVE_REAR_RIGHT, MotorType.kBrushless);

        /** Create new motor controller groups for the left and right side of the robot. */
        m_motorsLeft = new MotorControllerGroup(m_motorFrontLeft, m_motorRearLeft);
        m_motorsRight = new MotorControllerGroup(m_motorFrontRight, m_motorRearRight);

        /** Construct a drive train for driving differential drive. */
        m_drivetrain = new DifferentialDrive(m_motorsLeft, m_motorsRight);

         /**
         * Restore motor controller parameters to factory default until the next controller 
         * reboot.
         */
        m_motorFrontLeft.restoreFactoryDefaults();
        m_motorRearLeft.restoreFactoryDefaults();
        m_motorFrontRight.restoreFactoryDefaults();
        m_motorRearRight.restoreFactoryDefaults();

        /**
         * When the SPARK MAX is receiving a neutral command, the idle behavior of the motor 
         * will effectively disconnect all motor wires. This allows the motor to spin down at 
         * its own rate. 
         */
        m_motorFrontLeft.setIdleMode(IdleMode.kCoast);
        m_motorFrontRight.setIdleMode(IdleMode.kCoast);
        m_motorRearLeft.setIdleMode(IdleMode.kCoast);
        m_motorRearRight.setIdleMode(IdleMode.kCoast);

        /** Invert the direction of the right-side speed controllers. */
        m_motorsRight.setInverted(false);
        m_motorsLeft.setInverted(true);
        
        // m_motorFrontLeft.getEncoder().setInverted(true);
        FL_encoder = m_motorFrontLeft.getEncoder();
    }

    /** This function makes use of the driver input to control the robot like a tank. */ 
    public void tankDrive(double leftSpeed, double rightSpeed) {
        /** Set the drivetrain to operate as tank drive. */
        m_drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public void moveDistance(double m_distanceToMove, int gearRatio, double wheelCircumfrance){
        distanceToMove = m_distanceToMove;
        double encoderStartPos = FL_encoder.getPosition();
        double initAngle = gyro.getYaw();

        while(((-FL_encoder.getPosition()-encoderStartPos)/gearRatio)*wheelCircumfrance<=distanceToMove){
            double anglePID = keepAnglePID.calculate(initAngle,gyro.getYaw());
            double distancePID = driveToDistancePID.calculate(((FL_encoder.getPosition()-encoderStartPos)/gearRatio), distanceToMove);

            // SmartDashboard.putNumber("Move FWRD PID", distancePID);
            if(distancePID>0.5){
                distancePID = 0.5;
            }
            if(distancePID<-0.5){
                distancePID = -0.5;
            }

            double driveLeft = distancePID+anglePID;
            double driveRight = distancePID-anglePID;

            tankDrive(driveLeft, driveRight);
            // SmartDashboard.putNumber("FL Position", FL_encoder.getPosition()); 
            // SmartDashboard.putNumber("distance driven", ((FL_encoder.getPosition()-encoderStartPos)/gearRatio)*wheelCircumfrance); 
        }
    } 
      

    public void driveOntoCharger(double stopingDistance, int gearRatio, double wheelCircumfrance){
        distanceToMove = stopingDistance;
        double encoderStartPos = FL_encoder.getPosition();
        double initAngle = gyro.getYaw();
        while(gyro.getPitch()<=3&&!TeleopIndicator.getTeleopMode()){
            double anglePID = keepAnglePID.calculate(initAngle,gyro.getYaw());
            double distancePID = driveToDistancePID.calculate(((FL_encoder.getPosition()-encoderStartPos)/gearRatio), distanceToMove);

            // SmartDashboard.putNumber("Move FWRD PID", distancePID);
            if(distancePID>0.5){
                distancePID = 0.5;
            }
            if(distancePID<-0.5){
                distancePID = -0.5;
            }

            double driveLeft = distancePID+anglePID;
            double driveRight = distancePID-anglePID;

            tankDrive(driveLeft, driveRight);
            if(((-FL_encoder.getPosition()-encoderStartPos)/gearRatio)*wheelCircumfrance<=distanceToMove){
                break;
            }
        }

        while(gyro.getPitch()>=3&&!TeleopIndicator.getTeleopMode()){
            double anglePID = keepAnglePID.calculate(initAngle,gyro.getYaw());
            double distancePID = driveToDistancePID.calculate(((FL_encoder.getPosition()-encoderStartPos)/gearRatio), distanceToMove);

            // SmartDashboard.putNumber("Move FWRD PID", distancePID);
            if(distancePID>0.7){
                distancePID = 0.7;
            }
            if(distancePID<-0.7){
                distancePID = -0.7;
            }

            double driveLeft = distancePID+anglePID;
            double driveRight = distancePID-anglePID;

            tankDrive(driveLeft, driveRight);
            if(((-FL_encoder.getPosition()-encoderStartPos)/gearRatio)*wheelCircumfrance<=distanceToMove){
                break;
            }
        }

        while((((-FL_encoder.getPosition()-encoderStartPos)/gearRatio)*wheelCircumfrance<=distanceToMove)&&!TeleopIndicator.getTeleopMode()){
            distanceToMove=0;
            double anglePID = keepAnglePID.calculate(initAngle,gyro.getYaw());
            double distancePID = driveToDistancePID.calculate(((FL_encoder.getPosition()-encoderStartPos)/gearRatio), distanceToMove);

            // SmartDashboard.putNumber("Move FWRD PID", distancePID);
            if(distancePID>0.4){
                distancePID = 0.4;
            }
            if(distancePID<-0.4){
                distancePID = -0.4;
            }

            double driveLeft = distancePID+anglePID;
            double driveRight = distancePID-anglePID;

            tankDrive(driveLeft, driveRight);
        }
    }

    /** This function is called once each time the the command ends or is interrupted. */
    public void stop() {
        /**
         * Stop motor movement. Motor can be moved again by calling set without having to 
         * re-enable the motor.
         */
        m_drivetrain.stopMotor();
    }

    @Override
    public void periodic() {        
        // SmartDashboard.putNumber("FL Position", FL_encoder.getPosition()); 
    }

}