// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import frc.robot.subsystems.Photon;

public class RotateAlignApriltagAngle extends CommandBase {
    
  Photon photon = new Photon();

  double kP = 0.04;
  double kI = 0.0;
  double kD = 0.01;
  double PID = 1; 
  double m_setpoint;
  PIDController pid = new PIDController(kP, kI, kD);

  public RotateAlignApriltagAngle(double setpoint) {
    m_setpoint = setpoint;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 
    while(photon.hasTarget()&&!TeleopIndicator.getTeleopMode()&&(PID>=0.1||PID<=-0.1)){
      PID = pid.calculate(photon.getYaw(), m_setpoint);
      if(PID>=1){
        PID=1;
      }
      if(PID<=-1){
        PID=-1;
      }
      
      m_drivetrain.tankDrive(-PID, PID);
      SmartDashboard.putBoolean("PID AprilTag",true);
      SmartDashboard.putNumber("yaw",photon.getYaw());
      SmartDashboard.putNumber("Rotate Setpoint",m_setpoint);
      SmartDashboard.putNumber("Rotate PID",PID);
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
