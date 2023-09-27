package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.GoingToSetpoints.ArmSetpoint;
import frc.robot.commands.GoingToSetpoints.ElSetpoint;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Wrist.WristSetPosition;

public class FifteenTaxi extends SequentialCommandGroup {
    public FifteenTaxi() {
      addCommands(
        // new IntakeClose().withTimeout(0.1),
        new IntakeStop().withTimeout(0.1),
        new ElSetpoint(-70).withTimeout(1.5),
        new ArmSetpoint(0).withTimeout(1),
        new WristSetPosition(12).withTimeout(1),
        new Eject(1).withTimeout(0.5),
        new ElSetpoint(-1).withTimeout(1.5),
        new IntakeStop().withTimeout(0.1),
        new ReverseMoveDistance(100,0.45,0,false)
      );
  }
}
