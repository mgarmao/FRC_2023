package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class DriveOffChargeStation extends CommandBase {

  double kP = 0.01;
  double kI = 0.0;
  double kD = 0.01;

  boolean aparentLevel = false;
  double balancePID = 1; 
  double initAngle = 0;
  double m_setpoint = 0;
  PIDController pid = new PIDController(kP, kI, kD);

  boolean drivingUp, drivingDown, level = false;

  public DriveOffChargeStation() {
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

    if((gyro.getPitch()<-5)){
      double driveLeft = 0.45-anglePID;
      double driveRight = 0.45+anglePID;
      m_drivetrain.tankDrive(-driveLeft, -driveRight); 
      SmartDashboard.putNumber("Motor",driveLeft);   
      drivingUp=true;
    }
    else if((gyro.getPitch()>5&&drivingUp)){
      double driveLeft = 0.45-anglePID;
      double driveRight = 0.45+anglePID;
      m_drivetrain.tankDrive(-driveLeft, -driveRight);
      SmartDashboard.putNumber("Motor",driveLeft);  
      drivingDown=true;
    }
    else if(drivingUp&&drivingDown){
      level=true;
      m_drivetrain.tankDrive(0, 0);
    }
    else{
      double driveLeft = 0.45-anglePID;
      double driveRight = 0.45+anglePID;
      m_drivetrain.tankDrive(-driveLeft, -driveRight); 
      SmartDashboard.putNumber("Motor",driveLeft);   
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setBrakeMode();
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    if(level){
      return true;
    }
    else{
      return false;
    }
  }
}