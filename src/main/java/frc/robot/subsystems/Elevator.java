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
    private CANSparkMax elevator;
    private RelativeEncoder encoder;

    private double kP = 0.06;
    private double kI = 0.00;
    private double kD = 0.01;
    private double elevatorCommanded = 0;
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Elevator() {
        elevator = new CANSparkMax(Constants.ELEVATOR, MotorType.kBrushless);
        encoder = elevator.getEncoder();
        elevator.setIdleMode(IdleMode.kBrake);
        // elevator.setSoftLimit(SoftLimitDirection.kForward, -180);
        // elevator.setSoftLimit(SoftLimitDirection.kReverse, 0);
        // elevator.enableSoftLimit(SoftLimitDirection.kForward, false);
        // elevator.enableSoftLimit(SoftLimitDirection.kReverse, false);
        encoder.setPosition(0);
    }

    public void retract(){
        elevatorCommanded = 0;
    }

    public void extend(){
        elevatorCommanded = 160;
    }

    public void stop() {
        elevator.stopMotor();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("CLimber Position", encoder.getPosition()); 
        PID = pid.calculate(encoder.getPosition(), elevatorCommanded);
        SmartDashboard.putNumber("Climber PID", PID);

        elevator.set(PID); 
    }
}