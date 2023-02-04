// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import frc.robot.subsystems.Photon;

public class DriveTowardAprilTag extends CommandBase {
    
  Photon photon = new Photon();

  double kP = 0.04;
  double kI = 0.0;
  double kD = 0.01;
  double yawPID = 1; 
  double driveLeft = 0;
  double driveRight = 0;

  double m_setpoint;
  PIDController pid = new PIDController(kP, kI, kD);

  public DriveTowardAprilTag() {
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // (PID>=0.05||PID<=-0.05)&&
    while(photon.hasTarget()&&!TeleopIndicator.getTeleopMode()){
      yawPID = pid.calculate(photon.getYaw(), m_setpoint);
      
      if(yawPID>=1){
        yawPID=1;
      }

      if(yawPID<=-1){
        yawPID=-1;
      }



      driveLeft = 0.3-yawPID;
      driveRight = 0.3+yawPID;

      m_drivetrain.tankDrive(driveLeft, driveRight);
      
      SmartDashboard.putBoolean("PID AprilTag",true);
      SmartDashboard.putNumber("yaw",photon.getYaw());
      SmartDashboard.putNumber("DriveLeft",driveLeft);
      SmartDashboard.putNumber("driveRight",driveRight);
      SmartDashboard.putNumber("Yaw PID",yawPID);
      SmartDashboard.putNumber("hypot",photon.hypot());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("PID AprilTag",false);

    return false;
  }
}
