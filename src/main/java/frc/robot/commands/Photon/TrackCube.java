// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Photon;

import static frc.robot.RobotContainer.m_drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Photon;

public class TrackCube extends CommandBase {
    Photon photon = new Photon();
    int gearRatio = Constants.DRIVETRAIN_GEAR_RATIO;
    double kP = 0.05;
    double kI = 0.0;
    double kD = 0.01;

    double PID =1;

    double DISTANCE_TO_MOVE,encoderStartPos,loopTime=0;
    PIDController pid = new PIDController(kP, kI, kD);

    double wheelCircumfrance = Constants.WHEEL_CIRCUMFRANCE;

    public TrackCube(double distanceToMoveInches) {
        DISTANCE_TO_MOVE = distanceToMoveInches;
        addRequirements(m_drivetrain);
        addRequirements(photon);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        encoderStartPos = m_drivetrain.getFrontLeftEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Cone Yaw",photon.getCubeYaw());
        PID = pid.calculate(photon.getCubeYaw(), 0);
        double speedLimit = 0.5;
        double driveLeft = 0.4+PID;
        double driveRight = 0.4-PID;

        driveLeft = Math.max(-speedLimit, Math.min(driveLeft, speedLimit));
        driveRight = Math.max(-speedLimit, Math.min(driveRight, speedLimit));

        m_drivetrain.tankDrive(driveLeft, driveRight);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        m_drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(((((m_drivetrain.getFrontLeftEncoder()-encoderStartPos)/gearRatio)*wheelCircumfrance)>=DISTANCE_TO_MOVE)){
            return true;
        }
        else{
            return false;
        }
    }
}