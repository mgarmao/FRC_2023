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

public class Balance extends SequentialCommandGroup {
    public Balance() {
      addCommands(
        // new IntakeClose().withTimeout(0.1
        new MoveDistance(12, 0.4, 0.4).withTimeout(2),
        // new ElSetpoint(-77).alongWith(
        //   new ArmSetpoint(5).withTimeout(0.3),
        //   new WristSetPosition(16).withTimeout(1)
        // ).withTimeout(1),
        // new Eject(1).withTimeout(0.5),
        // new ElSetpoint(-1).alongWith(
        //   new IntakeStop()
        // ).withTimeout(1),
        new ReverseMoveDistance(68,0.65,0,false).andThen(
          new DriveOffChargeStation().withTimeout(3),
          new ReverseMoveDistance(18, 0.5,0,true).withTimeout(2),
          new ReverseMoveDistance(12, 0,0, true).withTimeout(0.5),
          new MoveDistance(20,0.65,0).withTimeout(2).andThen(
            new ReverseBalance(true).withTimeout(15)
          )
        )
      );
  }
}
