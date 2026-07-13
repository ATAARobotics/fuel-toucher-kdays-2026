// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class MecanumDrivetrain extends SubsystemBase {
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
          new Pose2d(0, 0, new Rotation2d()));
  private Pose2d pose = new Pose2d(0, 0, new Rotation2d());
  private Field2d field = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public MecanumDrivetrain() {
    gyro.enableLogging(true);
    System.out.println(gyro.getFirmwareVersion());
    gyro.reset();

    SparkMaxConfig LeftFrontConfig = new SparkMaxConfig();
    SparkMaxConfig RightFrontConfig = new SparkMaxConfig();
    SparkMaxConfig LeftBackConfig = new SparkMaxConfig();
    SparkMaxConfig RightBackConfig = new SparkMaxConfig();

    LeftFrontConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .encoder
        .positionConversionFactor(0.10639527);
    LeftFrontMotor.configure(
        LeftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftBackConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .encoder
        .positionConversionFactor(0.10639527);
    LeftBackMotor.configure(
        LeftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RightFrontConfig.idleMode(IdleMode.kBrake)
        .inverted(true)
        .encoder
        .positionConversionFactor(0.10639527);
    RightFrontMotor.configure(
        RightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RightBackConfig.idleMode(IdleMode.kBrake)
        .inverted(true)
        .encoder
        .positionConversionFactor(0.10639527);
    RightBackMotor.configure(
        RightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drive = new MecanumDrive(LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor);
  }

  // Drive
  public Command driveCommand(DoubleSupplier xsup, DoubleSupplier ysup, DoubleSupplier zsup) {
    // Inline construction of command goes here.
    gyro.reset();
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          if (gyro.isCalibrating()) {
            return;
          }

          double x = xsup.getAsDouble();
          double y = -ysup.getAsDouble();
          double z = zsup.getAsDouble() / 4;
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

          // math below done with assistance by AI
          double xPrime = x * Math.cos(facing) - y * Math.sin(facing);
          double yPrime = y * Math.cos(facing) + x * Math.sin(facing);
          drive.driveCartesian(
              xPrime * ChassisConstants.speedMult,
              yPrime * ChassisConstants.speedMult,
              z * ChassisConstants.speedMult);
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
            -LeftFrontMotor.getEncoder().getPosition(), -RightFrontMotor.getEncoder().getPosition(),
            -LeftBackMotor.getEncoder().getPosition(), -RightBackMotor.getEncoder().getPosition());
    // Get the rotation of the robot from the gyro.
    var gyroAngle = gyro.getRotation2d();

    LimelightHelpers.SetRobotOrientation("", gyroAngle.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    // Add it to your pose estimator
    m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    m_odometry.addVisionMeasurement(
        limelightMeasurement.pose, limelightMeasurement.timestampSeconds);

    // Update the pose
    pose = m_odometry.update(gyroAngle, wheelPositions);
    field.setRobotPose(pose);
    SmartDashboard.putData(field);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
