// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;


public class MecanumDrivetrain extends SubsystemBase {
  private SparkMax LeftFrontMotor, RightFrontMotor, LeftBackMotor, RightBackMotor;
  private MecanumDrive drive;
  private AHRS gyro;

  /** Creates a new ExampleSubsystem. */
  public MecanumDrivetrain() {

    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    LeftFrontMotor = new SparkMax(ChassisConstants.FrontLeftMotorID, MotorType.kBrushless);
    RightFrontMotor = new SparkMax(ChassisConstants.FrontRightMotorID, MotorType.kBrushless);
    LeftBackMotor = new SparkMax(ChassisConstants.BackLeftMotorID, MotorType.kBrushless);
    RightBackMotor = new SparkMax(ChassisConstants.BackRightMotorID, MotorType.kBrushless);

    SparkMaxConfig LeftFrontConfig = new SparkMaxConfig();
    SparkMaxConfig RightFrontConfig = new SparkMaxConfig();
    SparkMaxConfig LeftBackConfig = new SparkMaxConfig();
    SparkMaxConfig RightBackConfig = new SparkMaxConfig();

    LeftFrontConfig.idleMode(IdleMode.kBrake).inverted(false);

    LeftBackConfig.idleMode(IdleMode.kBrake).inverted(false);

    RightFrontConfig.idleMode(IdleMode.kBrake).inverted(true);

    RightBackConfig.idleMode(IdleMode.kBrake).inverted(true);
    drive = new MecanumDrive(LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor);
  }

  // Drive
  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          double facing = gyro.getYaw();
          // math below done with assistance by AI
          // invert direction to cancel out relative direction instead of multiply
          double facingrad = -Math.toRadians(facing);
          double xPrime = x.getAsDouble() * Math.cos(facingrad) - y.getAsDouble() * Math.sin(facingrad);
          double yPrime = x.getAsDouble() * Math.sin(facingrad) + y.getAsDouble() * Math.cos(facingrad);

          drive.driveCartesian(xPrime, yPrime, z.getAsDouble());
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
