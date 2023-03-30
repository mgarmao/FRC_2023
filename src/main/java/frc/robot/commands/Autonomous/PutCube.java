package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoingToSetpoints.ArmSetpoint;
import frc.robot.commands.GoingToSetpoints.ElSetpoint;
import frc.robot.commands.Wrist.WristSetPosition;

public class PutCube extends SequentialCommandGroup {
    public PutCube() {
      addCommands(
        new ElSetpoint(Constants.CUBE_SCORE_HIGH_EL).withTimeout(1.5),
        new WristSetPosition(Constants.CUBE_SCORE_HIGH_WRIST).withTimeout(1),
        new ArmSetpoint(Constants.CUBE_SCORE_HIGH_ARM).withTimeout(1)
        
      );
  }
}
