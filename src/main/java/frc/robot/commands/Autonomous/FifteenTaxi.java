// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
// Autonomous program created with help from Team 303

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.GoingToSetpoints.ElSetpoint;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Wrist.WristSetPosition;
import frc.robot.subsystems.Elevator;

public class FifteenTaxi extends SequentialCommandGroup {
    public FifteenTaxi() {
      addCommands(
        new IntakeClose().withTimeout(0.1),
        new ElSetpoint(-70).withTimeout(2),
        new WristSetPosition(12).withTimeout(1),
        new Eject(1).withTimeout(0.5),
        new ElSetpoint(-1).withTimeout(1.5),
        new IntakeStop().withTimeout(0.1),
        new ReverseMoveDistance(170,0.45,0,false)
      );
  }
}
