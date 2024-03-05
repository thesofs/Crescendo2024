// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm2 extends ProfiledPIDSubsystem {
  public final CANSparkMax arm_spark = new CANSparkMax(11,MotorType.kBrushless);
  public final CANSparkMax arm_spark2 = new CANSparkMax(12, MotorType.kBrushless);
  public final SparkAbsoluteEncoder absoluteEncoder = arm_spark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  /** Creates a new Arm2. */
  public Arm2() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.03,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(200, 200)));

      arm_spark2.follow(arm_spark, true);
      arm_spark.setSmartCurrentLimit(40);
      arm_spark2.setSmartCurrentLimit(40);
      arm_spark.setIdleMode(IdleMode.kBrake);
      arm_spark2.setIdleMode(IdleMode.kBrake);
  }

  

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    arm_spark.set(output);

    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return (absoluteEncoder.getPosition() * 360);
  }
}
