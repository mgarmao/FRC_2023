// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
// Autonomous program created with help from Team 303

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.GoingToSetpoints.ArmSetpoint;
import frc.robot.commands.GoingToSetpoints.ElSetpoint;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Wrist.WristSetPosition;

public class BackAndForth extends SequentialCommandGroup {
    public BackAndForth() {
      addCommands(
        new MoveDistance(12, 0.5, 0.5).withTimeout(2),
        new ReverseMoveDistance(100,0.45,0,false)
      );
  }
}
