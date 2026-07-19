// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.LimelightHelpers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecanumDrivetrain extends SubsystemBase {
  private boolean isFieldCentric = true;
  private RobotConfig config;
  private SparkMax LeftFrontMotor =
      new SparkMax(ChassisConstants.FrontLeftMotorID, MotorType.kBrushless);
  private SparkMax RightFrontMotor =
      new SparkMax(ChassisConstants.FrontRightMotorID, MotorType.kBrushless);
  private SparkMax LeftBackMotor =
      new SparkMax(ChassisConstants.BackLeftMotorID, MotorType.kBrushless);
  private SparkMax RightBackMotor =
      new SparkMax(ChassisConstants.BackRightMotorID, MotorType.kBrushless);
  private MecanumDrive drive;
  private AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, NavXUpdateRate.k8Hz);
  Translation2d m_frontLeftLocation = new Translation2d(-0.5207, 0.0635);
  Translation2d m_backLeftLocation = new Translation2d(-0.5207, -0.0635);
  Translation2d m_frontRightLocation = new Translation2d(0.5207, 0.0635);
  Translation2d m_backRightLocation = new Translation2d(0.5207, -0.0635);
  MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  MecanumDrivePoseEstimator m_odometry =
      new MecanumDrivePoseEstimator(
          m_kinematics,
          gyro.getRotation2d(),
          new MecanumDriveWheelPositions(
              -LeftFrontMotor.getEncoder().getPosition(),
                  -RightFrontMotor.getEncoder().getPosition(),
              -LeftBackMotor.getEncoder().getPosition(),
                  -RightBackMotor.getEncoder().getPosition()),
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose);
  private Pose2d pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose;
  private Field2d field = new Field2d();

  private double gox, goy, goz;

  /** Creates a new ExampleSubsystem. */
  public MecanumDrivetrain() {
    SmartDashboard.putBoolean("FieldCentric", isFieldCentric);
    gyro.enableLogging(true);
    gyro.reset();

    SparkMaxConfig LeftFrontConfig = new SparkMaxConfig();
    SparkMaxConfig RightFrontConfig = new SparkMaxConfig();
    SparkMaxConfig LeftBackConfig = new SparkMaxConfig();
    SparkMaxConfig RightBackConfig = new SparkMaxConfig();

    LeftFrontConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .encoder
        .positionConversionFactor(0.10639527)
        .velocityConversionFactor(0.10639527 / 60);

    LeftFrontConfig.closedLoop.pid(0.02, 0, 0.2).feedForward.kV(1.18);
    LeftFrontMotor.configure(
        LeftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftBackConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .encoder
        .positionConversionFactor(0.10639527)
        .velocityConversionFactor(0.10639527 / 60);

    LeftBackConfig.closedLoop.pid(0.02, 0, 0.2).feedForward.kV(1.18);

    LeftBackMotor.configure(
        LeftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RightFrontConfig.idleMode(IdleMode.kBrake)
        .inverted(true)
        .encoder
        .positionConversionFactor(0.10639527)
        .velocityConversionFactor(0.10639527 / 60);

    RightFrontConfig.closedLoop.pid(0.02, 0, 0.2).feedForward.kV(1.18);

    RightFrontMotor.configure(
        RightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RightBackConfig.idleMode(IdleMode.kBrake)
        .inverted(true)
        .encoder
        .positionConversionFactor(0.10639527)
        .velocityConversionFactor(0.10639527 / 60);

    RightBackConfig.closedLoop.pid(0.02, 0, 0.2).feedForward.kV(1.18);

    RightBackMotor.configure(
        RightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drive = new MecanumDrive(LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        m_odometry
            ::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(7.0, 0.0, 10.0), // Translation PID constants
            new PIDConstants(7.0, 0.0, 10.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  private Pose2d getPose() {
    return pose;
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new MecanumDriveWheelSpeeds(
            LeftFrontMotor.getEncoder().getVelocity(), RightFrontMotor.getEncoder().getVelocity(),
            LeftBackMotor.getEncoder().getVelocity(), RightBackMotor.getEncoder().getVelocity()));
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wspeeds = new MecanumDriveWheelSpeeds();
    wspeeds = m_kinematics.toWheelSpeeds(speeds);
    // drive.driveCartesian(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond,
    // speeds.omegaRadiansPerSecond);

    drive.driveCartesian(
        speeds.vxMetersPerSecond / 5,
        speeds.vyMetersPerSecond / 5,
        speeds.omegaRadiansPerSecond / 5 / 10 / 5);

    // RightBackMotor.getClosedLoopController()
    //     .setSetpoint(wspeeds.rearRightMetersPerSecond, ControlType.kVelocity);
    // LeftBackMotor.getClosedLoopController()
    //     .setSetpoint(wspeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    // RightFrontMotor.getClosedLoopController()
    //     .setSetpoint(wspeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    // LeftFrontMotor.getClosedLoopController()
    //     .setSetpoint(wspeeds.frontLeftMetersPerSecond, ControlType.kVelocity);

    SmartDashboard.putNumber("FrontLeft", LeftFrontMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("FrontLeftGoal", wspeeds.frontLeftMetersPerSecond);

    SmartDashboard.putNumber("FrontRight", RightFrontMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("FrontRightGoal", wspeeds.frontRightMetersPerSecond);

    SmartDashboard.putNumber("BackLeft", LeftBackMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("BackLeftGoal", wspeeds.rearLeftMetersPerSecond);

    SmartDashboard.putNumber("BackRight", RightBackMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("BackRightGoal", wspeeds.rearRightMetersPerSecond);
  }

  // Drive
  public Command driveCommand(
      DoubleSupplier xsup, DoubleSupplier ysup, DoubleSupplier zsup, BooleanSupplier fast) {
    // Inline construction of command goes here.
    gyro.reset();
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          if (gyro.isCalibrating()) {
            return;
          }
          SmartDashboard.putBoolean("FieldCentric", isFieldCentric);

          double y = xsup.getAsDouble();
          double x = -ysup.getAsDouble();
          double z = zsup.getAsDouble();

          if (!fast.getAsBoolean()) {
            z = z / 6;
          }
          // ChassisConstants.deadZone
          if (x > -ChassisConstants.deadZone && x < ChassisConstants.deadZone) {
            x = 0;
          }
          if (y > -ChassisConstants.deadZone && y < ChassisConstants.deadZone) {
            y = 0;
          }
          if (z > -ChassisConstants.deadZone && z < ChassisConstants.deadZone) {
            z = 0;
          }
          // invert direction to cancel out relative direction instead of multiply
          double facing = Math.toRadians(-gyro.getYaw());
          // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
          //  facing = pose.getRotation().getRadians() + Math.PI;
          // } else {
          //  facing = pose.getRotation().getRadians();
          // }
          if (!isFieldCentric) {
            facing = Math.toRadians(180);
          }

          // math below done with assistance by AI
          double xPrime = x * Math.cos(facing) - y * Math.sin(facing);
          double yPrime = y * Math.cos(facing) + x * Math.sin(facing);
          drive.driveCartesian(
              xPrime * ChassisConstants.speedMult * 3,
              yPrime * ChassisConstants.speedMult / 2.5,
              z * ChassisConstants.speedMult);
          gox = xPrime * ChassisConstants.speedMult * 3;
          goy = yPrime * ChassisConstants.speedMult / 2.5;
          goz = z * ChassisConstants.speedMult;
        });
  }

  public Command resetGyroCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          gyro.reset();
          if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            m_odometry.resetRotation(new Rotation2d(0.5 * Math.PI));
          } else {
            m_odometry.resetRotation(new Rotation2d());
          }
        });
  }

  public Command fieldCentricCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          isFieldCentric = true;
        });
  }

  public Command robotCentricCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          isFieldCentric = false;
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // Get my wheel positions
    var wheelPositions =
        new MecanumDriveWheelPositions(
            LeftFrontMotor.getEncoder().getPosition(), RightFrontMotor.getEncoder().getPosition(),
            LeftBackMotor.getEncoder().getPosition(), RightBackMotor.getEncoder().getPosition());
    // Get the rotation of the robot from the gyro.
    var gyroAngle = gyro.getRotation2d();

    LimelightHelpers.SetRobotOrientation("", gyroAngle.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // Add it to your pose estimator
    if (limelightMeasurement.avgTagArea > 0.01) {
      m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 0));
      m_odometry.addVisionMeasurement(
          limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }
    if (limelightMeasurement.avgTagArea > 0.3
        && limelightMeasurement.tagCount > 1
        && limelightMeasurement.pose.getRotation().getDegrees() > -91
        && limelightMeasurement.pose.getRotation().getDegrees() < -89) {
      gyro.reset();
    }

    // Update the pose
    pose = m_odometry.update(gyroAngle, wheelPositions);
    field.setRobotPose(pose);
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("avgTagArea", limelightMeasurement.avgTagArea);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    MecanumDriveWheelSpeeds wheels =
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(gox * 0.10639527, goy * 0.10639527, goz * 12));
    LeftBackMotor.getEncoder()
        .setPosition(LeftBackMotor.getEncoder().getPosition() + wheels.rearLeftMetersPerSecond);
    LeftFrontMotor.getEncoder()
        .setPosition(LeftFrontMotor.getEncoder().getPosition() + wheels.frontLeftMetersPerSecond);
    RightBackMotor.getEncoder()
        .setPosition(RightBackMotor.getEncoder().getPosition() + wheels.rearRightMetersPerSecond);
    RightFrontMotor.getEncoder()
        .setPosition(RightFrontMotor.getEncoder().getPosition() + wheels.frontRightMetersPerSecond);

    // Get my wheel positions
    var wheelPositions =
        new MecanumDriveWheelPositions(
            LeftFrontMotor.getEncoder().getPosition(), RightFrontMotor.getEncoder().getPosition(),
            LeftBackMotor.getEncoder().getPosition(), RightBackMotor.getEncoder().getPosition());
    // Get the rotation of the robot from the gyro.
    var gyroAngle = pose.getRotation().rotateBy(new Rotation2d(goz * 12 / 50));
    // Update the pose
    pose = m_odometry.update(gyroAngle, wheelPositions);
    field.setRobotPose(pose);
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("avgTagArea", 0);
  }
}
