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

public class RegBalance extends SequentialCommandGroup {
    public RegBalance() {
      addCommands(
        // new IntakeClose().withTimeout(0.1
        new IntakeStop().withTimeout(0.1),
        new ElSetpoint(-77).alongWith(
          new WristSetPosition(16),
          new ArmSetpoint(10)
        ).withTimeout(1.5),
        new Eject(0.99).withTimeout(0.5),
        new ElSetpoint(-1).alongWith(
          new IntakeStop()
        ).withTimeout(1),
        new ReverseMoveDistance(68,0.35,0,false).andThen(
          new ReverseBalance(true).withTimeout(15)
        )
      );
  }
}
