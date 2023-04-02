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
  public static final Elevator elevator = new Elevator();
  public static final Arm arm = new Arm();
  public static final Photon photon = new Photon();
  public static final ArmCam armCam = new ArmCam();
  public static final Wrist wrist = new Wrist();
  // public static final  Shortcuts shortcuts = new Shortcuts();

  Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  private final Command WAIT_NONE = new WaitCommand(0);
  private final Command FifteenTaxi = new FifteenTaxi();
  private final Command FiveTaxi = new FiveTaxi();
  private final Command Balance = new Balance();
  private final Command RegBalance = new RegBalance();
  private final Command BackAndForth = new BackAndForth();
  // private final Command TwoCubeOneSide = new TwoCubeOneSide();
  // private final Command TwoCube = new TwoCube();

  // private final Command Square = new Square();
  private SendableChooser<Command> m_auto = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /** Autonomous Chooser */
    m_auto.addOption("FifteenTaxi", FifteenTaxi);
    m_auto.addOption("FiveTaxi", FiveTaxi);
    m_auto.addOption("Balamce", Balance);
    m_auto.setDefaultOption("RegBalance", RegBalance);
    m_auto.addOption("BackAndForth", BackAndForth);
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
    new JoystickButton(m_driverLeft, 1).whenPressed(() -> Intake.eject(0.99)).whenReleased(()->Intake.stop());
    new JoystickButton(m_driverRight, 1).whenPressed(() -> Intake.eject(0.99)).whenReleased(()->Intake.stop());

    //Raise and lower 
    new JoystickButton(m_operator, Button.kRightBumper.value)
    .whenPressed(() -> elevator.extend())
    .whenReleased(() -> elevator.stop());

    new JoystickButton(m_operator, Button.kLeftBumper.value)
    .whenPressed(() -> elevator.retract())
    .whenReleased(() -> elevator.stop());
  
    //Sets intake to run depending on left or right driver thumb button
    new JoystickButton(m_driverLeft, 2)
      .whenPressed(() -> Intake.eject(Constants.EJECT_SPEED))
      .whenReleased(() -> Intake.stop());
    new JoystickButton(m_driverRight, 2)
      .whenPressed(() -> Intake.retrieve())
      .whenReleased(() -> Intake.stop());

    new JoystickButton(m_operator, Button.kA.value)
    .whenPressed(()->LEDs.sendData());

    new JoystickButton(m_operator, Button.kB.value)
    .whenPressed(()->LEDs.sendData2());

    new JoystickButton(m_operator, Button.kY.value)
    .whenPressed(()->m_drivetrain.setCoast());

    new JoystickButton(m_operator, Button.kX.value)
    .whenPressed(()->m_drivetrain.setBrakeMode());
    //Brake Mode

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