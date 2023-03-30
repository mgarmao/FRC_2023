package frc.robot.commands.Drivetrain;

import static frc.robot.RobotContainer.gyro;
import static frc.robot.RobotContainer.m_drivetrain;
import static frc.robot.RobotContainer.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Scan extends CommandBase {
    private double initAngle, inputAngle;
    double kP = 0.005;
    double kI = 0.00019;
    double kD = 0.015;
    double scanDegrees=0;
    PIDController pid = new PIDController(kP, kI, kD);
    double initalAngle = 0;

    int revolutions = 0;
    boolean previousLeft = true;
    boolean goingLeft = true;
    boolean goingRight = false;

    public Scan(double MscanDegrees) {
        scanDegrees = MscanDegrees; 
        addRequirements(m_drivetrain);
        addRequirements(photon);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        photon.setPipline(0);
        initalAngle = gyro.getYaw();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(revolutions>=4){
            double drive = 1;
            drive = pid.calculate(gyro.getYaw(),inputAngle+initAngle);

            //go right
            if((gyro.getYaw()<(scanDegrees+initalAngle))&&goingRight){
                if(drive>0.45){
                    drive=0.45;
                }
                if(drive<-0.45){
                    drive=-0.45;
                }
                m_drivetrain.tankDrive(-drive, drive);
            }
            else{
                goingLeft = false;
                goingRight = true;
            }

            //go left
            if((gyro.getYaw()>(-scanDegrees+initalAngle))&&goingLeft){
                if(drive>0.45){
                    drive=0.45;
                }
                if(drive<-0.45){
                    drive=-0.45;
                }
                m_drivetrain.tankDrive(drive, -drive);
            }
            else{
                goingLeft = false;
                goingRight = true;
            }

            if(previousLeft!=goingLeft){
                revolutions++;
            }

            previousLeft = goingLeft;
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
        if(photon.hasApriltag(Constants.startingApriltag)){
            return true;
        }
        else if (revolutions>=4){
            return true;
        }
        else{
            return false;
        }
    }
}