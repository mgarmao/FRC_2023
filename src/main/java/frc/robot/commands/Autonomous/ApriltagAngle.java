// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Test
// Autonomous program created with help from Team 303

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;

import frc.robot.commands.Drivetrain.*;


public class ApriltagAngle extends SequentialCommandGroup {
    public ApriltagAngle() {
        addCommands(
            new RotateAlignApriltagAngle()
        );  
    }
}

