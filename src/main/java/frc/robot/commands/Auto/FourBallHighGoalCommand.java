// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.nio.file.Path;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallHighGoalCommand extends ParallelCommandGroup {
  /** Creates a new FourBallHighGoalCommand. */
  
  // This command will require:
  //  - Drivetrain subsystem
  //  - Shooter subsystem
  //  - 
  public FourBallHighGoalCommand(DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Import Paths //
    String[] fourBallHighGoalAutoJsons = new String[4];
    for(int i = 1; i <= 4; i++){
      fourBallHighGoalAutoJsons[i] = "FourBallHighGoal-Pt" + i + ".wpilib.json";
    }

    // Add trajectory pt 1
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fourBallHighGoalAutoJsons[0]);
    

    // Add trajectory pt 2
    
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
        
      )
    );
  }
}
