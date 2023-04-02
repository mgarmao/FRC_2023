// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class DriveKeepingY extends CommandBase {
  double kP = 0.05;
  double kI = 0.00;
  double kD = 0.15;
  double distancePID = 1; 
  double driveLeft = 0;
  double driveRight = 0;

  double m_setpoint,m_stopDistance;
  PIDController pid = new PIDController(kP, kI, kD);

  public DriveKeepingY(double setpoint, double stopDistance) {
    m_stopDistance = stopDistance;
    m_setpoint = setpoint;
    addRequirements(photon);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    distancePID = pid.calculate(photon.getApriltagDistanceY(Constants.startingApriltag), m_setpoint);
    
    if(distancePID>=1){
      distancePID=1;
    }

    if(distancePID<=-1){
      distancePID=-1;
    }

    if(photon.getApriltagYaw(Constants.startingApriltag)>=18){
      driveLeft = 0.5;
      driveRight = 0.3;
    }
    else if (photon.getApriltagYaw(Constants.startingApriltag)<=-18){
      driveLeft = 0.3;
      driveRight = 0.5;
    }
    else{
      driveLeft = 0.5+distancePID;
      driveRight = 0.5-distancePID;
    }

    m_drivetrain.tankDrive(driveLeft, driveRight);
    
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    if(photon.apriltagHasTarget()&&(photon.getApriltagHypot()>=m_stopDistance)){
      return false;
    }
    else{
      return true;
    }
  }
}