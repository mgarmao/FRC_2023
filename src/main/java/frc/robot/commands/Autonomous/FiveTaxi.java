package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.*;

public class FiveTaxi extends SequentialCommandGroup {
    public FiveTaxi() {
      addCommands(
        new ReverseMoveDistance(60,0.45,0,false).withTimeout(4)
      );
  }
}
