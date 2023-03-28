// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Photon;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class AlignWithApriltag extends CommandBase {
  double kP = 0.06;
  double kI = 0.0;
  double kD = 0.03;
  double yawPID = 1; 
  double driveLeft = 0;
  double driveRight = 0;

  double m_setpoint;
  PIDController pid = new PIDController(kP, kI, kD);
  int targetID;

  public AlignWithApriltag(double setpoint) {
    photon.setPipline(0);
    m_setpoint = setpoint;
    addRequirements(photon);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    photon.setPipline(0);
    targetID = Constants.startingApriltag;
  }

  @Override
  public void execute() {
    if(targetID!=100){
      yawPID = pid.calculate(photon.apriltagDistanceY(targetID), m_setpoint);
    }
    else{
      yawPID = pid.calculate(photon.apriltagDistanceYBest(), m_setpoint);
    }
    
    if(yawPID>=1){
      yawPID=1;
    }

    if(yawPID<=-1){
      yawPID=-1;
    }

    driveLeft = 0.5-yawPID;
    driveRight = 0.5+yawPID;
    if(photon.apriltagDistanceY(4)!=0){
      m_drivetrain.tankDrive(driveLeft, driveRight);
    }
    else{
      m_drivetrain.tankDrive(0.4, 0.4);
    }
    
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
    if((photon.apriltagDistanceX()>=0.5)||(photon.apriltagDistanceX()==0)){
      return false;
    }
    else{
      return true;
    }
  }
}