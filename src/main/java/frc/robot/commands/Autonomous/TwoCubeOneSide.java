// package frc.robot.commands.Autonomous;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Drivetrain.MoveDistance;
// import frc.robot.commands.Drivetrain.MoveDistancePitchProtection;
// import frc.robot.commands.Drivetrain.ReverseMoveDistance;
// import frc.robot.commands.Drivetrain.Rotate;
// import frc.robot.commands.GoingToSetpoints.ArmSetpoint;
// import frc.robot.commands.GoingToSetpoints.ElSetpoint;
// import frc.robot.commands.Intake.Eject;
// import frc.robot.commands.Photon.AlignWithApriltag;
// import frc.robot.commands.Photon.ChangePipline;
// import frc.robot.commands.Photon.GetStartingApriltagID;
// import frc.robot.commands.Photon.TrackCube;
// import frc.robot.commands.Wrist.WristSetPosition;

// public class TwoCubeOneSide extends SequentialCommandGroup {
//     public TwoCubeOneSide() {
//       addCommands(
//         // new IntakeClose(),
//         new ElSetpoint(-50).alongWith(
//             new WristSetPosition(16)
//         ).withTimeout(1),
//         new Eject(1).withTimeout(0.5),

//         new ElSetpoint(-1).alongWith(
//             new ChangePipline(2).withTimeout(0.1),
//             new ReverseMoveDistance(16,0.45,0.4,false).withTimeout(3),
//             new GetStartingApriltagID(0).withTimeout(0.2),
//             new Rotate(180).withTimeout(2),
//             // new ArmSetpoint(Constants.CONE_PICKUP_ARM),
//             // new WristSetPosition(0),
//             new MoveDistance(2,0.65,0.5).withTimeout(3).andThen(
//                 new TrackCube(8,0.5).alongWith(
//                     new ArmSetpoint(1),/////////////////////////////////////////////////////////////////
//                     new WristSetPosition(0) ////////////////////////////////////////////////////
//                 ).withTimeout(3),
//                 new MoveDistance(5, 0.35, 0.3).alongWith(
//                     new ArmSetpoint(1),/////////////////////////////////////////////////////////////////
//                     new WristSetPosition(0) ////////////////////////////////////////////////////
//                 ).withTimeout(2)
//             ),
//             new ArmSetpoint(1).alongWith(/////////////////////////////////////////////////////////////////
//                 new WristSetPosition(0),//////////////////
//                 new Rotate(180)///////////////////////////////
//             ).withTimeout(2), ////////////////////////////////////////////////////
           
//             new MoveDistancePitchProtection(12,0.65, 0.55,true).withTimeout(3).andThen(
//                 new AlignWithApriltag(0).withTimeout(3)
//             ),
//             new ElSetpoint(-50).alongWith(
//             new WristSetPosition(16)
//             ).withTimeout(1),
//             new Eject(1).withTimeout(0.5)
//         )
//       );
//   }
// }
