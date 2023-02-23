// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    DoubleSolenoid intakeSolenoid;
    public Intake() {
        /** Create a new object to control the SPARK MAX motor controllers. */
        motor = new CANSparkMax(Constants.INTAKE_WHEELS, MotorType.kBrushless);
        /**
         * Restore motor controller parameters to factory default until the next controller 
         * reboot.
         */
        motor.restoreFactoryDefaults();

        /**
         * When the SPARK MAX is receiving a neutral command, the idle behavior of the motor 
         * will effectively disconnect all motor wires. This allows the motor to spin down at 
         * its own rate. 
         */
        motor.setIdleMode(IdleMode.kCoast);
        
        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID[0], Constants.INTAKE_SOLENOID[1]);
        intakeSolenoid.set(Value.kReverse);
    }

    /** Retrieve cargo for transportation. */
    public void retrieve() {
        motor.set(Constants.INTAKE_SPEED);
    }

    /** Eject cargo from the robot. */
    public void eject() {
        motor.set(-Constants.INTAKE_SPEED);
    }
    
    /** This function is called once each time the the command ends or is interrupted. */
    public void stop() {
        motor.stopMotor();
    }

    public void toggle() {
        /**
         * If the solenoid is set to forward, it'll be set to reverse. If the solenoid is set 
         * to reverse, it'll be set to forward. If the solenoid is set to off, nothing happens.
         */
        intakeSolenoid.toggle();
    }
    
    public void close() {
        intakeSolenoid.set(Value.kReverse);
    }

    /** Set the state of the intake arms to extend. */
    public void open() {
        intakeSolenoid.set(Value.kForward);
    }

    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {}

}