package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingHooks extends SubsystemBase {
  Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

  public ClimbingHooks() {
    solenoid.set(false);
  }

  @Override
  public void periodic() {
  }

  public Command extendSolenoid(boolean value) {
    return runOnce(() -> solenoid.set(value));
  }
}
