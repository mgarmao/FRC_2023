// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import java.time.Duration;
import java.time.Instant;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;


import frc.robot.subsystems.Limelight;

public class RotateToCone extends CommandBase {
    
  Limelight vision = new Limelight();

  double kP = 0.05;
  double kI = 0.0;
  double kD = 0.01;

  double P = 0.0;
  double I = 0.0;
  double preI = 0.0;
  double D = 0.0;

  double error = 0.0;
  double pError = 0.0;

  double loopTime = 0.0;
  Duration timeElapsed = Duration.between(Instant.now(), Instant.now());

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

    error = vision.ConeX();
    double PID = MathUtil.clamp(pid.calculate(error, 0), -1, 1);
    m_drivetrain.tankDrive(PID, -PID);
    SmartDashboard.putNumber("Rotate PID",PID);
   
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
