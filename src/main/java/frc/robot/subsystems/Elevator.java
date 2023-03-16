package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.subsystems.HallEffect;

public class Elevator extends SubsystemBase {
    private CANSparkMax elevatorLeft,elevatorRight;
    private RelativeEncoder encoderLeft,encoderRight;
    private static final XboxController m_operator = new XboxController(Constants.CONTROLLER_OPERATOR);

    private double kP = 0.01;
    private double kI = 0.00;
    private double kD = 0.00;

    boolean goingToPosition=false;
    double elevatorCommanded = 0;

    boolean moving = false;

    HallEffect sensor0 = new HallEffect(0);
    HallEffect sensor1 = new HallEffect(1);
    
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Elevator() {
        elevatorLeft = new CANSparkMax(Constants.ELEVATOR_LEFT, MotorType.kBrushless);
        elevatorRight = new CANSparkMax(Constants.ELEVATOR_RIGHT, MotorType.kBrushless);

        encoderLeft = elevatorLeft.getEncoder();
        encoderRight= elevatorRight.getEncoder();

        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();

        elevatorLeft.follow(elevatorRight, true);

        elevatorRight.setSoftLimit(SoftLimitDirection.kForward, Constants.ELEVATOR_UPPER_LIMIT);
        elevatorRight.setSoftLimit(SoftLimitDirection.kReverse, Constants.ELEVATOR_LOWER_LIMIT);
        elevatorLeft.setSoftLimit(SoftLimitDirection.kForward, Constants.ELEVATOR_UPPER_LIMIT);
        elevatorLeft.setSoftLimit(SoftLimitDirection.kReverse, Constants.ELEVATOR_LOWER_LIMIT);

        elevatorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        elevatorLeft.setIdleMode(IdleMode.kBrake);
        elevatorRight.setIdleMode(IdleMode.kBrake);
        
        encoderLeft.setPosition(0);
        encoderRight.setPosition(0);
    }

    public void retract(){
        moving=true;
        elevatorRight.set(-Constants.ELEVATOR_POWER);
    }

    public void extend(){
        moving = true;
        elevatorRight.set(Constants.ELEVATOR_POWER); 
    }

    public void stop() {
        elevatorCommanded = encoderRight.getPosition();
        elevatorRight.stopMotor();
        moving = false;
    }

    public void zero(){
        while(sensor0.getData()||sensor1.getData()){
            elevatorRight.set(-Constants.ELEVATOR_POWER);
        }
    }

    public void setPosition(double input){
        elevatorCommanded = input;
        moving = false;
    }

    @Override
    public void periodic() {
        if(!moving){
            double elPID = pid.calculate(-encoderRight.getPosition(), -elevatorCommanded);
            SmartDashboard.putNumber("Elevator COmmanded",elevatorCommanded);
            if(elPID>Constants.ELEVATOR_POWER){
                elPID =0.5;
            }
            if(elPID<-Constants.ELEVATOR_POWER){
                elPID =0.5;
            }       
            if(elevatorCommanded-encoderRight.getPosition()<10){
                Constants.elInPosition = true;
            }     
            elevatorRight.set(-elPID);
        }

        if(m_operator.getPOV()==Constants.CONE_FRONT_PICKUP_POV){
            elevatorCommanded = Constants.CONE_FRONT_PICKUP_EL;
            Constants.elInPosition = false;
            moving = false;
        }

        // if(m_operator.getPOV()==Constants.RETRACT_POV){
        //     elevatorCommanded = Constants.RETRACT_EL;
        //     Constants.elInPosition = false;
        //     moving = false;
        // }

        SmartDashboard.putNumber("Elevator Position", encoderRight.getPosition());
        // double elPID = pid.calculate(encoderRight.getPosition(), elevatorCommanded);
        // if(elPID>0.4){
        //     elPID =0.4;
        // }
        // if(elPID<-0.4){
        //     elPID =0.4;
        // }
        // elevatorRight.set(elPID); 
        // SmartDashboard.putNumber("Elevator Left Encoder", encoderLeft.getPosition()); 
        // SmartDashboard.putNumber("Elevator Right Encoder", encoderRight.getPosition()); 
        // SmartDashboard.putNumber("CLimber Position", encoder.getPosition()); 
        // PID = pid.calculate(encoder.getPosition(), elevatorCommanded);
        // SmartDashboard.putNumber("Climber PID", PID);

        // elevator.set(PID); 
    }
}