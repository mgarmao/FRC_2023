// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private RelativeEncoder armEncoder;

    private double kP = 0.06;
    private double kI = 0.00;
    private double kD = 0.01;
    private double armCommanded = 0;
    
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Arm() {
        arm = new CANSparkMax(Constants.ARM, MotorType.kBrushless);
        armEncoder = arm.getEncoder();
        arm.restoreFactoryDefaults();

        // arm.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_UPPER_LIMIT);
        // arm.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_LOWER_LIMIT);
        
        // arm.enableSoftLimit(SoftLimitDirection.kForward, true);
        // arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        arm.setIdleMode(IdleMode.kBrake);        
        armEncoder.setPosition(0);    
    }

    public void armController(int direction){

    }

    public void retract(){
        armCommanded = 0;
        arm.set(-Constants.ARM_POWER);
    }

    public void extend(){
        // elevatorCommanded = 160;
        arm.set(Constants.ARM_POWER); 
    }

    public void stop() {
        arm.stopMotor();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition()); 
        
        // if(XboxController.Button.kLeftStick.value>=0){
        //     extend();
        // }
        // if(XboxController.Button.kLeftStick.value<=0){
        //     retract();
        // }
        // SmartDashboard.putNumber("Left Stick", XboxController.Button.kLeftStick.value); 
        
        // SmartDashboard.putNumber("CLimber Position", encoder.getPosition()); 
        // PID = pid.calculate(encoder.getPosition(), elevatorCommanded);
        // SmartDashboard.putNumber("Climber PID", PID);

        // elevator.set(PID); 
    }
}