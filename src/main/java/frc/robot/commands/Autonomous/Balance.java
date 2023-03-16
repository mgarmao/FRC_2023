// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
// Autonomous program created with help from Team 303

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Wrist.WristSetPosition;

public class Balance extends SequentialCommandGroup {
    public Balance() {
      addCommands(
        new IntakeClose().withTimeout(0.1),
        new WristSetPosition(14).withTimeout(2),
        new Eject(1).withTimeout(1.75),
        new IntakeStop(),
        new ReverseMoveDistance(60,0.55),
        new ReverseBalance()
        // new FowardBalance()
      );
  }
}
