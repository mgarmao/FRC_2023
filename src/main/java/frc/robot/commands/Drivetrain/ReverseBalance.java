// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class ReverseBalance extends CommandBase {

  double kP = 0.01;
  double kI = 0.0;
  double kD = 0.01;

  boolean aparentLevel = false;
  double balancePID = 1; 
  double initAngle = 0;
  double m_setpoint = 0;
  PIDController pid = new PIDController(kP, kI, kD);

  public ReverseBalance() {
    addRequirements(gyro);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    initAngle = gyro.getYaw();
  }

  @Override
  public void execute() {
    double anglePID = pid.calculate(initAngle,gyro.getYaw());

    if(anglePID>=0.25){
      anglePID=0.25;
    }

    if(anglePID<=-0.25){
      anglePID=-0.25;
    }

    if((gyro.getPitch()<-12)){
      double driveLeft = 0.4-anglePID;
      double driveRight = 0.4+anglePID;
      m_drivetrain.tankDrive(-driveLeft, -driveRight); 
      SmartDashboard.putNumber("Motor",driveLeft);   

    }
    else if((gyro.getPitch()>12)){
      double driveLeft = 0.42+anglePID;
      double driveRight = 0.42-anglePID;
      m_drivetrain.tankDrive(driveLeft, driveRight);
      SmartDashboard.putNumber("Motor",driveLeft);  
    }
    else{
      m_drivetrain.tankDrive(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setBrakeMode();
    m_drivetrain.stop();
    m_drivetrain.setCoast();
    
  }

  @Override
  public boolean isFinished() {
    if((gyro.getPitch()>-12)&&(gyro.getPitch()<12)){
      SmartDashboard.putBoolean("balancing", true);
      m_drivetrain.setBrakeMode();
      m_drivetrain.stop();
    }
    else{
      SmartDashboard.putBoolean("balancing", false);
    }
    return false;
  }
}