// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

import frc.robot.commands.Drivetrain.*;

import frc.robot.commands.Autonomous.*;
import frc.robot.commands.Autonomous.Balance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  /** Controllers */
  private static final XboxController m_operator = new XboxController(Constants.CONTROLLER_OPERATOR);
  private static final Joystick m_driverLeft = new Joystick(Constants.JOYSTICK_LEFT);
  private static final Joystick m_driverRight = new Joystick(Constants.JOYSTICK_RIGHT);

  /** Robot Components */
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final Gyroscope gyro = new Gyroscope();
  public static final LEDs LEDs = new LEDs();
  public static final Intake Intake = new Intake();
  public static final TeleopIndicator TeleopIndicator = new TeleopIndicator();
  public static final Elevator Elevator = new Elevator();
  public static final Arm arm = new Arm();
  public static final Photon photon = new Photon();
  public static final Wrist wrist = new Wrist();
  // public static final  Shortcuts shortcuts = new Shortcuts();

  Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  private final Command WAIT_NONE = new WaitCommand(0);
  private final Command Taxi = new Taxi();
  private final Command Balance = new Balance();
  // private final Command Square = new Square();
  private SendableChooser<Command> m_auto = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /** Autonomous Chooser */
    m_auto.setDefaultOption("Taxi", Taxi);
    m_auto.addOption("Balamce", Balance);
    m_auto.addOption("Do Nothing", WAIT_NONE);
    SmartDashboard.putData("Autonomous Routine", m_auto);
    

    /** Configure the button bindings */
    configureButtonBindings();

    /** Drivetrain Controls */
    /** The Driver's joysticks control each side of the drivetrain respectively. */
    m_drivetrain.setDefaultCommand(
      new TankDrive(
        m_drivetrain,
        () -> -1 * Math.pow(m_driverRight.getY(),Constants.JOYSTICK_CURVE),
        () -> -1 * Math.pow(m_driverLeft.getY(),Constants.JOYSTICK_CURVE)
      )
    );

    // wrist.controller(m_operator.getLeftY());
    // arm.controller(m_operator.getRightY());

    // arm.setDefaultCommand(new armController(arm, () -> m_operator.getLeftTriggerAxis()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //open/close the intake with Trigger on Driver Joystick
    new JoystickButton(m_driverLeft, 1).whenPressed(() -> Intake.toggle());
    new JoystickButton(m_driverRight, 1).whenPressed(() -> Intake.toggle());

    //Raise and lower 
    new JoystickButton(m_operator, Button.kRightBumper.value)
    .whenPressed(() -> Elevator.extend())
    .whenReleased(() -> Elevator.stop());

    new JoystickButton(m_operator, Button.kLeftBumper.value)
    .whenPressed(() -> Elevator.retract())
    .whenReleased(() -> Elevator.stop());
  
    //Sets intake to run depending on left or right driver thumb button
    new JoystickButton(m_driverLeft, 2)
      .whenPressed(() -> Intake.eject(-Constants.INTAKE_SPEED))
      .whenReleased(() -> Intake.stop());
    new JoystickButton(m_driverRight, 2)
      .whenPressed(() -> Intake.retrieve())
      .whenReleased(() -> Intake.stop());

    new JoystickButton(m_operator, Button.kA.value)
    .whenPressed(()->LEDs.sendData());

    new JoystickButton(m_operator, Button.kB.value)
    .whenPressed(()->LEDs.sendData2());

    new JoystickButton(m_operator, Button.kY.value)
    .whenPressed(()->arm.extend())
    .whenReleased(()->arm.stop());
    
    new JoystickButton(m_operator, Button.kX.value)
    .whenPressed(()->arm.retract())
    .whenReleased(()->arm.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_auto.getSelected();
  }
}