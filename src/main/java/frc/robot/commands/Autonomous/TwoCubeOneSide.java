// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
// Autonomous program created with help from Team 303

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.GoingToSetpoints.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Photon.*;
import frc.robot.commands.Wrist.*;
import frc.robot.subsystems.Wrist;

public class TwoCubeOneSide extends SequentialCommandGroup {
    public TwoCubeOneSide() {
      addCommands(
        // new IntakeClose(),
        // new ElSetpoint(-50).withTimeout(2),
        // new WristSetPosition(16).withTimeout(1),
        // new Eject(1).withTimeout(0.5),
        // new ElSetpoint(-1).withTimeout(2),
            new ChangePipline(2).withTimeout(1),
            new ReverseMoveDistance(12,0.45,false).withTimeout(3),
            new GetStartingApriltagID(0).withTimeout(1),
            new Rotate(180).withTimeout(2),
        // new ArmSetpoint(Constants.CONE_PICKUP_ARM),
        // new WristSetPosition(0),
            new MoveDistance(12,0.65).withTimeout(3).andThen(
                new TrackCube(10,0.4).withTimeout(4)
            ),
        // new ArmSetpoint(0),
            new ChangePipline(0).withTimeout(1),
            new Rotate(180).withTimeout(2),
            new MoveDistance(12, 0.65).withTimeout(3).andThen(
                new AlignWithApriltag(0).withTimeout(3)
            )
        
        // new ElSetpoint(-30).withTimeout(2),
        // new WristSetPosition(16).withTimeout(1),
        // new Eject(1).withTimeout(0.5)
      );
  }
}
