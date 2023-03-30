// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Photon;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class ChangePipline extends CommandBase {
    int pipline = 0;
    public ChangePipline(int setPipline) {
        addRequirements(photon);
        pipline = setPipline;
        photon.camera.setPipelineIndex(setPipline);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        photon.camera.setPipelineIndex(pipline);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}