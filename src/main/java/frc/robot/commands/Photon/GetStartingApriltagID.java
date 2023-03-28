// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Photon;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class GetStartingApriltagID extends CommandBase {
    public GetStartingApriltagID(int setPipline) {
        addRequirements(photon);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(photon.apriltagID()!=100){
            Constants.startingApriltag = photon.apriltagID();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if(Constants.startingApriltag==100){
            return false;
        }
        else{
            return true;
        }
    }
}