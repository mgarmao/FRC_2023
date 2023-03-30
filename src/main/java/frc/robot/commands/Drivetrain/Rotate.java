package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class Rotate extends CommandBase {
  private double m_initAngle, inputAngle;
  double kP = 0.005;
  double kI = 0.00019;
  double kD = 0.015;
  PIDController pid = new PIDController(kP, kI, kD);
  double initalAngle = 0;
  public Rotate(double m_inputAngle) {
    inputAngle = -m_inputAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalAngle = gyro.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = 1;
    drive = pid.calculate(gyro.getYaw(),inputAngle+initalAngle);
    
    if(gyro.getYaw()>(inputAngle+initalAngle+40)){
      if(drive>0.55){
        drive=0.55;
      }
      if(drive<-0.55){
        drive=-0.55;
      }
    }
    else{
      if(drive>0.37){
        drive=0.37;
      }
      if(drive<-0.37){
        drive=-0.37;
      }
    }

    SmartDashboard.putNumber("ROTATE PID", drive);
    SmartDashboard.putNumber("Gyroscope (Degrees)", gyro.getYaw());
    SmartDashboard.putNumber("Ending Angle", (inputAngle+m_initAngle)-2);
    m_drivetrain.tankDrive(drive, -drive);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(gyro.getYaw()>(inputAngle+initalAngle)){
      return false;
    }
    else{
      return true;
    }
  }
}