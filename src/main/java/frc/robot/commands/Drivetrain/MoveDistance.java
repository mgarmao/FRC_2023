// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class MoveDistance extends CommandBase {
    double DISTANCE_TO_MOVE;
    int gearRatio = Constants.DRIVETRAIN_GEAR_RATIO;
    double wheelCircumfrance = Constants.WHEEL_CIRCUMFRANCE;

    public MoveDistance(double distanceToMove) {
        DISTANCE_TO_MOVE = distanceToMove;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.moveDistance(DISTANCE_TO_MOVE,gearRatio, wheelCircumfrance);
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
