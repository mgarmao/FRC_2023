// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class ReverseMoveDistance extends CommandBase {
    double DISTANCE_TO_MOVE;
    int gearRatio = Constants.DRIVETRAIN_GEAR_RATIO;
    double wheelCircumfrance = Constants.WHEEL_CIRCUMFRANCE;
    double encoderStartPos = 0;
    double initAngle = 0;

    double kP0 = 0.02;
    double kI0 = 0.0;
    double kD0 = 0.04;

    double kP1 = 0.0012;
    double kI1 = 0.0;
    double kD1 = 0.001;
    PIDController keepAnglePID = new PIDController(kP1, kI1, kD1);
    PIDController driveToDistancePID = new PIDController(kP0, kI0, kD0);

    double maxSpeed =0.5;

    public ReverseMoveDistance(double distanceToMoveInches,double mMaxSpeed) {
        DISTANCE_TO_MOVE = distanceToMoveInches;
        maxSpeed = mMaxSpeed;
        addRequirements(m_drivetrain);
        addRequirements(gyro);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        encoderStartPos = m_drivetrain.getFrontLeftEncoder();
        initAngle = gyro.getYaw();
        keepAnglePID.reset();
        driveToDistancePID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        double anglePID = keepAnglePID.calculate(initAngle,gyro.getYaw());
        double distancePID = driveToDistancePID.calculate(((m_drivetrain.getFrontLeftEncoder()-encoderStartPos)/gearRatio), DISTANCE_TO_MOVE);

        // SmartDashboard.putNumber("Move FWRD PID", distancePID);
        if(distancePID>maxSpeed){
            distancePID = maxSpeed;
        }
        if(distancePID<-maxSpeed){
            distancePID = -maxSpeed;
        }

        double driveLeft = distancePID-anglePID;
        double driveRight = distancePID+anglePID;

        m_drivetrain.tankDrive(-driveLeft, -driveRight);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false
        if(((((m_drivetrain.getFrontLeftEncoder()-encoderStartPos)/gearRatio)*wheelCircumfrance)>=DISTANCE_TO_MOVE)){
            SmartDashboard.putBoolean("Reversing", true);
            return true;
        }
        else{
            SmartDashboard.putBoolean("Reversing", false);
            return false;
        }
    }
}
