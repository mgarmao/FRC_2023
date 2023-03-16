// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import frc.robot.subsystems.Photon;

public class FowardBalance extends CommandBase {
  Photon photon = new Photon();

  double kP = 0.0012;
  double kI = 0.0;
  double kD = 0.001;

  double balancePID = 1; 
  double initAngle = 0;
  double m_setpoint = 0;
  PIDController pid = new PIDController(kP, kI, kD);

  public FowardBalance() {
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

    double driveLeft = 0.7+anglePID;
    double driveRight = 0.7-anglePID;
    SmartDashboard.putNumber("gyro pitch", gyro.getPitch());

    m_drivetrain.tankDrive(driveLeft, driveRight);    
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setBrakeMode();
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    if((gyro.getPitch()>-0.3)&&(gyro.getPitch()<0.3)){
      SmartDashboard.putBoolean("balancing", false);
      return true;
    }
    else{
      SmartDashboard.putBoolean("balancing", true);
      return false;
    }
  }
}