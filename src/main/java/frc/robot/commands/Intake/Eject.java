// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class Eject extends CommandBase {
    double speed;
    int gearRatio = Constants.DRIVETRAIN_GEAR_RATIO;
    double wheelCircumfrance = Constants.WHEEL_CIRCUMFRANCE;

    public Eject(double m_speed) {
        speed = m_speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Intake.eject(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        end(isFinished());
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
