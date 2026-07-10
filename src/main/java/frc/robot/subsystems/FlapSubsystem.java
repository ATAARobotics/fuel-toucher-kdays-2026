// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class FlapSubsystem extends SubsystemBase {
  private DoubleSolenoid flap =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ChassisConstants.FlapSolenoid1ID,
          ChassisConstants.FlapSolenoid2ID);

  /** Creates a new ExampleSubsystem. */
  public FlapSubsystem() {}

  public Command flapRiseCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          flap.set(DoubleSolenoid.Value.kForward);
        });
  }

  public Command flapFallCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          flap.set(DoubleSolenoid.Value.kReverse);
        });
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
