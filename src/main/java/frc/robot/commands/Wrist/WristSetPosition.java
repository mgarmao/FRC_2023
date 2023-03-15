// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class WristSetPosition extends CommandBase {
    private double kP = 0.02;
    private double kI = 0.00;
    private double kD = 0.00;
    PIDController pid = new PIDController(kP, kI, kD);

    double desiredPosition;

    public WristSetPosition(double m_desiredPosition) {
        desiredPosition = m_desiredPosition;
        addRequirements(wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        double PID = pid.calculate(wrist.getPosition(), desiredPosition);
        if(PID>Constants.WRIST_MAX_POWER){
            PID=Constants.WRIST_MAX_POWER;
        }
        if(PID<-Constants.WRIST_MAX_POWER){
            PID=-Constants.WRIST_MAX_POWER;
        }
        wrist.setPower(PID);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
