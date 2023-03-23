// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
// Autonomous program created with help from Team 303

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.GoingToSetpoints.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Photon.*;
import frc.robot.commands.Wrist.*;

public class TwoCube extends SequentialCommandGroup {
    public TwoCube() {
      addCommands(
        new IntakeClose().withTimeout(0.1),
        new ElSetpoint(-50).withTimeout(2),
        new WristSetPosition(16).withTimeout(1),
        new Eject(1).withTimeout(0.5),
        new ElSetpoint(-1).withTimeout(2),
        new IntakeStop().withTimeout(0.1),
        new ReverseMoveDistance(60,0.65).andThen(
          new TrackCube(10).withTimeout(3)
        ),
        new MoveDistance(60, 0.5).withTimeout(3),
        new AlignWithApriltag(0).withTimeout(10)
      );
  }
}
