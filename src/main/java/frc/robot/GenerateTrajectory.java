// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.AutoLoader.AutoCommand;

/** Generates Trajectories depending on which auto mode is selected */
public class GenerateTrajectory {

    // Create a voltage constraint to ensure we don't accelerate too fast
    static DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    Constants.ksVolts,
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Generic Trajectory Config Information //
    static TrajectoryConfig config = new TrajectoryConfig(
            Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    public static Trajectory getTrajectory(AutoCommand command){
        switch(command){
            case NONE:
                return null;
            case EXAMPLE_TRAJECTORY:
                return TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(
                            new Translation2d(1, 2),
                            new Translation2d(3, 1),
                            new Translation2d(2, 0),
                            new Translation2d(3, -1),
                            new Translation2d(1, -2)
                        ),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(-180))),
                        // Pass config
                    config);
            default:
                return null;
        }
    }
}
