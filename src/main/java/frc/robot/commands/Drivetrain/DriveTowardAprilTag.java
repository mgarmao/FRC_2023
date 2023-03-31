// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class DriveTowardAprilTag extends CommandBase {
  double kP = 0.03;
  double kI = 0.0;
  double kD = 0.01;
  double yawPID = 1; 
  double driveLeft = 0;
  double driveRight = 0;

  double m_setpoint;
  PIDController pid = new PIDController(kP, kI, kD);

  public DriveTowardAprilTag(double setpoint) {
    m_setpoint = setpoint;
    addRequirements(photon);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    yawPID = pid.calculate(photon.getApriltagDistanceY(Constants.startingApriltag), m_setpoint);
    
    if(yawPID>=1){
      yawPID=1;
    }

    if(yawPID<=-1){
      yawPID=-1;
    }

    driveLeft = 0.5-yawPID;
    driveRight = 0.5+yawPID;

    m_drivetrain.tankDrive(driveLeft, driveRight);
    
    SmartDashboard.putBoolean("Drive Stage",true);
    SmartDashboard.putNumber("DriveLeft",driveLeft);
    SmartDashboard.putNumber("driveRight",driveRight);    
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    if(photon.apriltagHasTarget()&&photon.getApriltagDistanceX(Constants.startingApriltag)>=1.5){
      return false;
    }
    else{
      return true;
    }
  }
}