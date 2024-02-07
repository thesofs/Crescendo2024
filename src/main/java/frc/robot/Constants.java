// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class Constants {
    public static final double THROTTLE_MIN = 0.4;
    public static final double THROTTLE_MAX = 1.0;

     public static final TrapezoidProfile.Constraints K_THETA =
        new TrapezoidProfile.Constraints(
           MAX_ANGULAR_SPEED_RADIANS, MAX_ANGULAR_SPEED_RADIANS_SQ);
}
