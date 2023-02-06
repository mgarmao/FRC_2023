// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import frc.robot.subsystems.Photon;

public class RotateToCone extends CommandBase {
  Photon photon = new Photon();

  double kP = 0.05;
  double kI = 0.0;
  double kD = 0.01;

  double PID =1;

  double loopTime = 0.0;

  PIDController pid = new PIDController(kP, kI, kD);

  public void RotateCone() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //PID>=0.1||PID<=-0.1
    while(true){
      SmartDashboard.putNumber("Cone Yaw",photon.getConeYaw());
      PID = pid.calculate(photon.getConeYaw(), 0);
      m_drivetrain.tankDrive(PID, -PID);
      SmartDashboard.putNumber("Rotate PID",PID);
      SmartDashboard.putNumber("Angle",photon.getAngle(0));
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
    return false;
  }
}
