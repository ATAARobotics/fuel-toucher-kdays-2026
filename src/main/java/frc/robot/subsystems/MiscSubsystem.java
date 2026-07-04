// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import java.util.function.BooleanSupplier;

public class MiscSubsystem extends SubsystemBase {
  private DoubleSolenoid flap =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ChassisConstants.FlapSolenoid1ID,
          ChassisConstants.FlapSolenoid2ID);
  private DoubleSolenoid climb =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ChassisConstants.ClimbSolenoid1ID,
          ChassisConstants.ClimbSolenoid2ID);

  /** Creates a new ExampleSubsystem. */
  public MiscSubsystem() {}

  public Command climbCommand(
      BooleanSupplier climbRise,
      BooleanSupplier climbFall) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          int climbDelta = (climbRise.getAsBoolean() ? 1 : 0) + -(climbFall.getAsBoolean() ? 0 : 1);
          if (climbDelta > 0) {
            climb.set(DoubleSolenoid.Value.kForward);
          } else if (climbDelta < 0) {
            climb.set(DoubleSolenoid.Value.kReverse);
          }
        });
  }
  public Command flapCommand(
      BooleanSupplier flapRise,
      BooleanSupplier flapFall) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          int flapDelta = (flapRise.getAsBoolean() ? 1 : 0) + -(flapFall.getAsBoolean() ? 0 : 1);

          if (flapDelta > 0) {
            flap.set(DoubleSolenoid.Value.kForward);
          } else if (flapDelta < 0) {
            flap.set(DoubleSolenoid.Value.kReverse);
          }
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
