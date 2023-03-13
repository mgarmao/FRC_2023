// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    private CANSparkMax wrist;
    private RelativeEncoder wristEncoder;

    private double kP = 0.06;
    private double kI = 0.00;
    private double kD = 0.01;
    
    private double desiredPosition = 0;
    boolean goingToPosition = false;
    private static final XboxController m_operator = new XboxController(Constants.CONTROLLER_OPERATOR);

    
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Wrist() {
        
        wrist = new CANSparkMax(Constants.WRIST, MotorType.kBrushless);
        wristEncoder = wrist.getEncoder();
        wrist.restoreFactoryDefaults();

        wrist.setSoftLimit(SoftLimitDirection.kForward, Constants.WRIST_UPPER_LIMIT);
        wrist.setSoftLimit(SoftLimitDirection.kReverse, Constants.WRIST_LOWER_LIMIT);
        
        wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
        wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        wrist.setIdleMode(IdleMode.kBrake);        
        wristEncoder.setPosition(0);    
    }
    
    public void controller(double input) {
        if(input>=Constants.WRIST_MAX_POWER){
            input = Constants.WRIST_MAX_POWER;
        }
        if(input<=-Constants.WRIST_MAX_POWER){
            input=-Constants.WRIST_MAX_POWER;
        }

        wrist.set(input);
        SmartDashboard.putNumber("Operator Left Y", input);
    }

    public void setPosition(double m_desiredPosition){
        desiredPosition = m_desiredPosition;
        goingToPosition = true;
    }

    // public void down(){
    //     wristCommanded = 0;
        
    //     wrist.set(-Constants.ELEVATOR_POWER);
    // }

    // public void WristController(){
    //     wristCommanded = -125;
        
    //     wrist.set(Constants.ELEVATOR_POWER); 
    // }

    public void stop() {
        wrist.stopMotor();
    }

    @Override
    public void periodic() {
        if(m_operator.getLeftY()!=0){
            controller(m_operator.getLeftY());
        }
        
        else{
            stop();
        }
        
        if(goingToPosition){
            if(wristEncoder.getPosition()>desiredPosition){
                double PID = pid.calculate(wristEncoder.getPosition(), desiredPosition);
                if(PID>Constants.WRIST_MAX_POWER){
                    PID=Constants.WRIST_MAX_POWER;
                }
                if(PID<-Constants.WRIST_MAX_POWER){
                    PID=-Constants.WRIST_MAX_POWER;
                }
                wrist.set(PID);
            }
            
            else{
                goingToPosition = false;
            }
        }

        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition()); 
        
        // if(XboxController.Button.kLeftStick.value>=0){
        //     extend();
        // }
        // if(XboxController.Button.kLeftStick.value<=0){
        //     retract();
        // }
        
        // SmartDashboard.putNumber("CLimber Position", encoder.getPosition()); 
        // PID = pid.calculate(encoder.getPosition(), elevatorCommanded);
        // SmartDashboard.putNumber("Climber PID", PID);

        // elevator.set(PID); 
    }
}