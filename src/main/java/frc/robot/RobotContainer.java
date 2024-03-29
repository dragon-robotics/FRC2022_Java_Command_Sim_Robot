// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoLoader.AutoCommand;
import frc.robot.commands.Auto.FiveBallBotLowGoalCommand;
import frc.robot.commands.Auto.FourBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.OneBallBotLeftLowGoalCommand;
import frc.robot.commands.Auto.OneBallBotLowGoalCommand;
import frc.robot.commands.Auto.OneBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.OneBallTopLowGoalCommand;
import frc.robot.commands.Auto.TwoBallBotLeftLowGoalCommand;
import frc.robot.commands.Auto.TwoBallBotLowGoalCommand;
import frc.robot.commands.Auto.TwoBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.TwoBallTopLowGoalCommand;
import frc.robot.commands.Teleop.ArcadeDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Create the field //
  public final static Field2d m_field = new Field2d();
  
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_field);

  // Joysticks
  private final Joystick m_driverController = new Joystick(Constants.DRIVER);
  private final Joystick m_operatorController = new Joystick(Constants.OPERATOR);

  // Create the auto loader class to load everything for us //

  // Create SmartDashboard chooser for autonomous routines
  private final AutoLoader m_autoLoader = new AutoLoader();

  // Store our overall trajectory //
  Trajectory trajectory = new Trajectory();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set default command to arcade drive when in teleop
    m_drivetrainSubsystem.setDefaultCommand(getArcadeDriveCommand());

    // Input the field onto the SmartDashboard //
    SmartDashboard.putData("Field", m_field);

    // Load all wpilib.json trajectory files into the Roborio to speed up auto deployment //
    GenerateTrajectory.loadTrajectories();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoCommand command = m_autoLoader.getSelected();

    switch (command) {
      case NONE:
        return null;
      case EXAMPLE_TRAJECTORY:
        return getRamseteCommand();
      case ONE_BALL_TOP_LOW_GOAL:
        return new OneBallTopLowGoalCommand(
            m_drivetrainSubsystem, command);
      case ONE_BALL_TOP_LEFT_LOW_GOAL:
        return new OneBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      case ONE_BALL_BOT_LEFT_LOW_GOAL:
        return new OneBallBotLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      case ONE_BALL_BOT_LOW_GOAL:
        return new OneBallBotLowGoalCommand(
            m_drivetrainSubsystem, command);
      case TWO_BALL_TOP_LOW_GOAL:
        return new TwoBallTopLowGoalCommand(
            m_drivetrainSubsystem, command);
      case TWO_BALL_TOP_LEFT_LOW_GOAL:
        return new TwoBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      case TWO_BALL_BOT_LEFT_LOW_GOAL:
        return new TwoBallBotLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      case TWO_BALL_BOT_LOW_GOAL:
        return new TwoBallBotLowGoalCommand(
            m_drivetrainSubsystem, command);
      case FOUR_BALL_TOP_LEFT_LOW_GOAL:
        return new FourBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      case FIVE_BALL_BOT_LOW_GOAL:
        return new FiveBallBotLowGoalCommand(
            m_drivetrainSubsystem, command);
      default:
        return null;
    }
  }

  public Command getArcadeDriveCommand() {
    // Commands //
    return new ArcadeDriveCommand(
        m_drivetrainSubsystem,
        () -> -m_driverController.getRawAxis(Constants.STICK_LEFT_Y),   // speed
        () -> m_driverController.getRawAxis(Constants.STICK_RIGHT_X),   // turn
        () -> m_driverController.getRawAxis(Constants.TRIGGER_LEFT),    // throttle
        () -> m_driverController.getRawButton(Constants.BUMPER_RIGHT)   // reverse
    );
  }

  public Command getRamseteCommand() {

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = GenerateTrajectory.getTrajectory(AutoCommand.EXAMPLE_TRAJECTORY).get(0);

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    var leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    var rightController = new PIDController(Constants.kPDriveVel, 0, 0);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(exampleTrajectory);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        // trajectory,
        m_drivetrainSubsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drivetrainSubsystem::getWheelSpeeds,
        leftController,
        rightController,
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts) -> {
          m_drivetrainSubsystem.tankDriveVolts(leftVolts, rightVolts);

          leftMeasurement.setNumber(m_drivetrainSubsystem.getWheelSpeeds().leftMetersPerSecond);
          leftReference.setNumber(leftController.getSetpoint());

          rightMeasurement.setNumber(m_drivetrainSubsystem.getWheelSpeeds().rightMetersPerSecond);
          rightReference.setNumber(rightController.getSetpoint());
        },
        m_drivetrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    // m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());
    m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));
  }
}
