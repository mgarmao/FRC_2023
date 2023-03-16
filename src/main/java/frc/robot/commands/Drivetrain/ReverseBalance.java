// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class ReverseBalance extends CommandBase {

  double kP = 0.0012;
  double kI = 0.0;
  double kD = 0.001;

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

    if((gyro.getPitch()<-10)){
      double driveLeft = 0.35-anglePID;
      double driveRight = 0.35+anglePID;
      m_drivetrain.tankDrive(-driveLeft, -driveRight); 
      SmartDashboard.putNumber("Motor",driveLeft);   

    }
    else if((gyro.getPitch()>10)){
      double driveLeft = 0.35+anglePID;
      double driveRight = 0.35-anglePID;
      m_drivetrain.tankDrive(driveLeft, driveRight);
      SmartDashboard.putNumber("Motor",driveLeft);  
      aparentLevel = true; 
    }
    else{
      m_drivetrain.tankDrive(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setBrakeMode();
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    if((gyro.getPitch()>-13)&&(gyro.getPitch()<13)&&aparentLevel){
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