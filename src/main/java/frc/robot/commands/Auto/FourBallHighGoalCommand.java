// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallHighGoalCommand extends ParallelCommandGroup {
  /** Creates a new FourBallHighGoalCommand. */
  
  private Trajectory p1, p2;  // Part 1 and 2 of the trajectory

  // This command will require:
  //  - Drivetrain subsystem
  //  - Shooter subsystem
  //  - Intake subsystem
  public FourBallHighGoalCommand(DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Import Paths //
    String[] fourBallHighGoalAutoJsons = new String[4];
    for(int i = 1; i <= 4; i++){
      fourBallHighGoalAutoJsons[i-1] = "FourBallHighGoal-Pt" + i + ".wpilib.json";
    }

    // Add trajectory pt 1
    for(int i = 0; i < 2; i++){
      try {        
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fourBallHighGoalAutoJsons[i]);
        Trajectory tmpTraj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

        if(i == 0){
          p1 = tmpTraj;
        }
        else{
          p1 = p1.concatenate(tmpTraj);
        }
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
      }
    }

    // Add trajectory pt 2
    for (int i = 2; i < 4; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fourBallHighGoalAutoJsons[i]);
        Trajectory tmpTraj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

        if (i == 2) {
          p2 = tmpTraj;
        } else {
          p2 = p2.concatenate(tmpTraj);
        }
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
      }
    }    
    /**
     * Command Sequence all ran in parallel:
     *  1. Engage Shooter and Intake                      - <Parallel>
     *  2. new Sequential Command
     *    a. Move backwards, intake ball, move forward
     *    b. Shoot for x seconds
     *    c. Move backwards, intake 2 more balls, move forward
     *    d. Shoot for x seconds
     */
    addCommands(
      new SequentialCommandGroup(
        new RamseteCommand(
          p1,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0, 0))
        .beforeStarting(() -> drivetrain.resetOdometry(p1.getInitialPose())),
        new RamseteCommand(
          p2,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0, 0))
        .beforeStarting(() -> drivetrain.resetOdometry(p2.getInitialPose()))
      )
    );
  }
}
