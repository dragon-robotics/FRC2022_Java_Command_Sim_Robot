// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  // Declare subsystem attribute/components //

  // Motor Controllers //
  WPI_TalonFX m_talonLeftLead = new WPI_TalonFX(2);
  WPI_TalonFX m_talonLeftFollow = new WPI_TalonFX(1);
  WPI_TalonFX m_talonRightLead = new WPI_TalonFX(4);
  WPI_TalonFX m_talonRightFollow = new WPI_TalonFX(3);
  DifferentialDrive m_drive = new DifferentialDrive(m_talonLeftLead, m_talonRightLead);

  // Object for simulated inputs into Talon. //
  TalonFXSimCollection m_leftDriveSim = m_talonLeftLead.getSimCollection();
  TalonFXSimCollection m_rightDriveSim = m_talonRightLead.getSimCollection();

  // Gyro - NavX //
  AHRS m_gyro = new AHRS();

  // Simulation model of the drivetrain with no measurement noise //
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2),        // 2 Falcon 500s on each side of the drivetrain.
      Constants.GEAR_RATIO,           // Standard AndyMark Gearing reduction.
      2.1,                            // MOI of 2.1 kg m^2 (from CAD model).
      26.5,                           // Mass of the robot is 26.5 kg.
      Constants.WHEEL_RADIUS_METERS,  // Robot uses 3" radius (6" diameter) wheels.
      Constants.TRACK_WIDTH_METERS,   // Distance between wheels is 0.546 meters.
      /*
       * The standard deviations for measurement noise:
       * x and y: 0.001 m
       * heading: 0.001 rad
       * l and r velocity: 0.1 m/s
       * l and r position: 0.005 m
       */
      null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this
           // line to add measurement noise.
  );

  // Create the odometry object //
  DifferentialDriveOdometry m_odometry;

  Field2d m_field;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem(Field2d field) {

    // Factory default configurations for all motors //
    m_talonLeftLead.configFactoryDefault();
    m_talonLeftFollow.configFactoryDefault();
    m_talonRightLead.configFactoryDefault();
    m_talonRightFollow.configFactoryDefault();

    // Disable all motors //
    m_talonLeftLead.set(ControlMode.PercentOutput, 0);
    m_talonLeftFollow.set(ControlMode.PercentOutput, 0);
    m_talonRightLead.set(ControlMode.PercentOutput, 0);
    m_talonRightFollow.set(ControlMode.PercentOutput, 0);

    // Set neutral mode to brake on all motors //
    m_talonLeftLead.setNeutralMode(NeutralMode.Coast);
    m_talonLeftFollow.setNeutralMode(NeutralMode.Coast);
    m_talonRightLead.setNeutralMode(NeutralMode.Coast);
    m_talonRightFollow.setNeutralMode(NeutralMode.Coast);

    // Set our followers to follow the lead motor //
    m_talonLeftFollow.follow(m_talonLeftLead);
    m_talonRightFollow.follow(m_talonRightLead);
    
    // Set our follower's inverted to be opposite of the master //
    m_talonLeftFollow.setInverted(InvertType.FollowMaster);
    m_talonRightFollow.setInverted(InvertType.FollowMaster);

    // Set our lead motor's rotation orientations //
    m_talonLeftLead.setInverted(TalonFXInvertType.CounterClockwise);
    m_talonRightLead.setInverted(TalonFXInvertType.Clockwise);

    // Configure encoder readings on the TalonFX //
    m_talonLeftLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_talonRightLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // Reset encoder //
    resetEncoders();

    // Reset gyro //
    zeroHeading();

    // Set odometry //
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    m_field = field;
  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_talonLeftLead.setVoltage(leftVolts); // Set voltage for left motor
    m_talonRightLead.setVoltage(rightVolts); // Set voltage for right motor
    m_drive.feed(); // Feed the motor safety object, stops the motor if anything goes wrong
  }  

  /**
   * Set max output of drivetrain
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){

    double leftSpeed = m_talonLeftLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;
    double rightSpeed = m_talonRightLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;  // Need to invert the results
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  /**
   * Resets the encoders for the robot
   */
  public void resetEncoders(){
    m_talonLeftLead.setSelectedSensorPosition(0.0, 0, 0);
    m_talonRightLead.setSelectedSensorPosition(0.0, 0, 0);
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading(){
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters / (2 * Math.PI * Constants.WHEEL_RADIUS_METERS);
    double motorRotations = wheelRotations * Constants.GEAR_RATIO;
    int sensorCounts = (int) (motorRotations * Constants.ENCODER_CPR);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond = velocityMetersPerSecond / (2 * Math.PI * Constants.WHEEL_RADIUS_METERS);
    double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.GEAR_RATIO;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * Constants.ENCODER_CPR);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.ENCODER_CPR;
    double wheelRotations = motorRotations / Constants.GEAR_RATIO;
    double positionMeters = wheelRotations * (2 * Math.PI * Constants.WHEEL_RADIUS_METERS);
    return positionMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(),
        nativeUnitsToDistanceMeters(m_talonLeftLead.getSelectedSensorPosition()),
        nativeUnitsToDistanceMeters(m_talonRightLead.getSelectedSensorPosition()));
    
    // Set field pose //
    m_field.setRobotPose(getPose());

    // Troubleshoot X and Y
    var translation = m_odometry.getPoseMeters().getTranslation();
    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X")
      .setNumber(translation.getX());
    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y")
      .setNumber(translation.getY());

    // Troubleshoot angle
    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Heading")
      .setNumber(m_gyro.getRotation2d().getDegrees());

    // Troubleshoot encoder values
    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left Wheel")
      .setNumber(nativeUnitsToDistanceMeters(m_talonLeftLead.getSelectedSensorPosition()));
    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right Wheel")
      .setNumber(nativeUnitsToDistanceMeters(m_talonRightLead.getSelectedSensorPosition()));

    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left Wheel Speed")
        .setNumber(m_talonLeftLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10);
    NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right Wheel Speed")
        .setNumber(m_talonRightLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10);      
  }

  @Override
  public void simulationPeriodic(){
    // This method will be called once per scheduler run during simulation
    
    /* Pass the robot battery voltage to the simulated Talon FXs */
    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    /*
     * CTRE simulation is low-level, so SimCollection inputs
     * and outputs are not affected by SetInverted(). Only
     * the regular user-level API calls are affected.
     *
     * WPILib expects +V to be forward.
     * Positive motor output lead voltage is ccw. We observe
     * on our physical robot that this is reverse for the
     * right motor, so negate it.
     *
     * We are hard-coding the negation of the values instead of
     * using getInverted() so we can catch a possible bug in the
     * robot code where the wrong value is passed to setInverted().
     */
    m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                         -m_rightDriveSim.getMotorOutputLeadVoltage());
    
    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_driveSim.update(0.02);

    /*
     * Update all of our sensors.
     *
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     */
    m_leftDriveSim.setIntegratedSensorRawPosition(
        distanceToNativeUnits(
            m_driveSim.getLeftPositionMeters()));
    m_leftDriveSim.setIntegratedSensorVelocity(
        velocityToNativeUnits(
            m_driveSim.getLeftVelocityMetersPerSecond()));
    m_rightDriveSim.setIntegratedSensorRawPosition(
        distanceToNativeUnits(
            -m_driveSim.getRightPositionMeters()));
    m_rightDriveSim.setIntegratedSensorVelocity(
        velocityToNativeUnits(
            -m_driveSim.getRightVelocityMetersPerSecond()));

    // From NavX example - Updates the NavX in sim
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-m_driveSim.getHeading().getDegrees(), 360));
    // navxSimAngle = -drivetrainSim.getHeading().getDegrees();
  }
}
