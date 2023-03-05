// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private CANSparkMax elevatorLeft,elevatorRight;
    private RelativeEncoder encoderLeft,encoderRight;

    private double kP = 0.06;
    private double kI = 0.00;
    private double kD = 0.01;
    private double elevatorCommanded = 0;
    
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
        elevatorCommanded = 0;
        elevatorRight.set(-Constants.ELEVATOR_POWER);
    }

    public void extend(){
        // elevatorCommanded = 160;
        elevatorRight.set(Constants.ELEVATOR_POWER); 
    }

    public void stop() {
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Elevator Left Encoder", encoderLeft.getPosition()); 
        // SmartDashboard.putNumber("Elevator Right Encoder", encoderRight.getPosition()); 
        // SmartDashboard.putNumber("CLimber Position", encoder.getPosition()); 
        // PID = pid.calculate(encoder.getPosition(), elevatorCommanded);
        // SmartDashboard.putNumber("Climber PID", PID);

        // elevator.set(PID); 
    }
}