// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

import java.time.Duration;
import java.time.Instant;

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
  
  public RotateToCone() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(true){
      Instant start = Instant.now();

      error = vision.ConeX();

      P = kP*error;
      I = preI+kI*error;
      Instant end = Instant.now();
      Duration timeElapsed = Duration.between(start, end);
      D = kD*(pError-error);
      double PID= P+I+D;
      
      if(PID>=1){
        PID=1;
      }
      if(PID<=-1){
        PID=-1;
      }

      m_drivetrain.tankDrive(PID, -PID);

      pError=error;
      SmartDashboard.putNumber("PID",PID);
      SmartDashboard.putNumber("Millis", timeElapsed.toMillis());
    }

    // while(vision.ConeX()==0){
    //   m_drivetrain.tankDrive(0.6, -0.6);
    // }


    // while(vision.ConeX()>4) {
      
    //   m_drivetrain.tankDrive(0.6, -0.6);
    // }
    
    // while(vision.ConeX()<-4) {
    //   m_drivetrain.tankDrive(-0.6, 0.6);
    // }
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
