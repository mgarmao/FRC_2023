// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import frc.robot.subsystems.Photon;

public class Balance extends CommandBase {
  Photon photon = new Photon();

  double kP = 0.05;
  double kI = 0.00;
  double kD = 0.15;
  double balancePID = 1; 

  double m_setpoint = 0;
  PIDController pid = new PIDController(kP, kI, kD);

  public Balance() {
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    while((gyro.getPitch()>1||gyro.getPitch()<-1)&&!TeleopIndicator.getTeleopMode()){
        balancePID = pid.calculate(photon.apriltagDistanceY(), m_setpoint);
        
        if(balancePID>=0.4){
            balancePID=0.4;
        }

        if(balancePID<=-0.4){
            balancePID=-0.4;
        }

        double driveLeft = balancePID;
        double driveRight = balancePID;

        m_drivetrain.tankDrive(driveLeft, driveRight);
        
        SmartDashboard.putBoolean("Drive Stage",true);
        SmartDashboard.putNumber("DriveLeft",driveLeft);
        SmartDashboard.putNumber("driveRight",driveRight);
        SmartDashboard.putBoolean("TELEOP",TeleopIndicator.getTeleopMode());
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("PID AprilTag",false);

    return false;
  }
}