// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkAbsoluteEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

// public class ArmSubsystem extends ProfiledPIDSubsystem {
//     double kS = 0;
//     double kG = -0.6;//-0.03;//-0.06;
//     double kV = 10;//5;//2.11;
//   public final CANSparkMax m_spark = new CANSparkMax(11, MotorType.kBrushless);
//   public final CANSparkMax m_spark2 = new CANSparkMax(12, MotorType.kBrushless);
//   private ArmFeedforward armFFController = new ArmFeedforward(kS,kG,kV,0);
//   // public final RelativeEncoder encoder_arm = m_spark.getAlternateEncoder();
//   // public final CANSparkMax hand_spark = new
//   // CANSparkMax(21,MotorType.kBrushless);
//   public final SparkAbsoluteEncoder absoluteEncoder;
//   // public final DutyCycleEncoder dutyCycleEncoder;
//   // private final double ENCODER_OFFSET = -1.416992;
//   // encoder_arm.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);

//   /** Creates a new Arm. */

//   // public final CANSparkMax spark = new CANSparkMax (14, MotorType.kBrushless);
//   // public final RelativeEncoder encoderS = spark.getEncoder();

  
//   public ArmSubsystem() {
//     super(
//         // The ProfiledPIDController used by the subsystem
//         new ProfiledPIDController(
//             0, //Was 0.03 before
//             0,//0.0007, //was 0.0007
//             0,

          
//             // The motion profile constraints
//             new TrapezoidProfile.Constraints(200, 200)));

                
//   //  this.alternateEncoder = m_spark.getAbsoluteEncoder(8192);
//     this.absoluteEncoder = m_spark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

//     /**
//      * double ka
//      * The acceleration gain, in volt seconds² per radian.
//      * double kg
//      * The gravity gain, in volts.
//      * double ks
//      * The static gain, in volts.
//      * double kv
//      * The velocity gain, in volt seconds per radian.
//      */

    
//    // this.armFFController = new ArmFeedforward(kS, kG, kV, 0);

    

//     m_spark.setIdleMode(IdleMode.kBrake);
//     m_spark2.setIdleMode(IdleMode.kBrake);
//     m_spark.setSmartCurrentLimit(40);
//     m_spark2.setSmartCurrentLimit(40);

//     m_spark2.follow(m_spark, false);

//   } 

//   @Override
//   public void useOutput(double output, TrapezoidProfile.State setpoint) {
//     //SmartDashboard.putNumber("Arm Output", output);
//     double feedforward = armFFController.calculate(setpoint.position, setpoint.velocity);
//     //SmartDashboard.putNumber("FF2",armFFController.calculate(setpoint.position, setpoint.velocity));

//     //double feedforward = armFFController.calculate(setpoint.position,0); //setpoint.position
//     m_spark.setVoltage(output + feedforward);//- feedforward);
   
//   }
  

//   public double getArmEncoderPos() { // arm deg

//     return (absoluteEncoder.getPosition() * 360); // Put encoder offset back in

//   }

//   public double getVoltage(){
//     return m_spark.getAppliedOutput();
//   }

  
//   @Override
//   public double getMeasurement() {
//     // Return the process variable measurement here
//     return getArmEncoderPos();

//   }


// }
