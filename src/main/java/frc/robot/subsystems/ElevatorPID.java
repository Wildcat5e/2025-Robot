package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
// import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;

public class ElevatorPID extends SubsystemBase implements Elevator {
  public static final double TOLERANCE = 0.0;
  public static final double MAX_ELEVATOR_SPEED = 12.0;
  private final TalonFX motorOne = new TalonFX(0);
  // private final TalonFX motorTwo = new TalonFX(1);
  // private final Follower follower = new Follower(0, true);
  private PIDController pidController = new PIDController(0.0, 0.0, 0.0);
  private State currentState;
  private double desiredHeight;
  private double speed;
  private double homeHeight;
  private final DoubleSubscriber pConstantSubscriber;
  private final DoubleSubscriber iConstantSubscriber;
  private final DoubleSubscriber dConstantSubscriber;
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    JOGGING_UP,
    JOGGING_DOWN,
    SYSID;
  }

  public ElevatorPID() {
    currentState = State.NOT_MOVING;
    // motorTwo.setControl(follower);
    motorOne.setPosition(0.0);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable pidConstants = inst.getTable("PID Constants");
    pConstantSubscriber = subscribeToDoubleTopic(pidConstants, "KP", 0.0);
    iConstantSubscriber = subscribeToDoubleTopic(pidConstants, "KI", 0.0);
    dConstantSubscriber = subscribeToDoubleTopic(pidConstants, "KD", 0.0);
    pidController.setTolerance(TOLERANCE);
    setCurrentPositionAsHome();
  }

  /**
   * Subscribe to a double topic. If the topic does not exist,
   * then create the topic and mark it as persistant.
   */
  private DoubleSubscriber subscribeToDoubleTopic(NetworkTable pidConstants, String topicName, double defaultValue) {
    DoubleTopic entry = pidConstants.getDoubleTopic(topicName);
    if (!entry.exists()) {
      System.out.println(String.format("Topic %s does not exist, creating it now.", topicName));
      DoublePublisher publisher = pidConstants.getDoubleTopic(topicName).publish();
      publisher.set(defaultValue);
      entry.setPersistent(true);
      publisher.close();
    } else {
      System.out.println(String.format("Topic %s EXISTS.", topicName));
    }
    return entry.subscribe(defaultValue);
  }

  private void setCurrentPositionAsHome() {
    homeHeight = getCurrentHeight();
    System.out.println("Home is currently at = " + homeHeight);
  }

  public Command setCurrentPositionAsHomeCommand() {
    return runOnce(() -> setCurrentPositionAsHome());
  }

  int counter = 0;

  @Override
  public void periodic() {
    switch (currentState) {
      case NOT_MOVING:
      case MOVING_UP:
      case MOVING_DOWN:
        speed = pidController.calculate(getCurrentHeight(), desiredHeight);
        break;
      case JOGGING_UP:
        speed = 1.0;
        break;
      case JOGGING_DOWN:
        speed = -1.0;
        break;
      case SYSID:
    }

    double clampedSpeed = Math.max(-MAX_ELEVATOR_SPEED, Math.min(MAX_ELEVATOR_SPEED, speed));

    motorOne.setVoltage(clampedSpeed);
    determineNextState(desiredHeight);
  }

  private void determineNextState(double desiredHeight) {
    double currentHeight = getCurrentHeight();
    this.desiredHeight = desiredHeight;
    pidController.setSetpoint(desiredHeight);

    if (desiredHeight > currentHeight + TOLERANCE) {
      currentState = State.MOVING_UP;
    } else if (desiredHeight < currentHeight - TOLERANCE) {
      currentState = State.MOVING_DOWN;
    } else {
      currentState = State.NOT_MOVING;
    }
  }

  private double getCurrentHeight() {
    return motorOne.getPosition().getValueAsDouble();
  }

  private void stop() {
    desiredHeight = getCurrentHeight();
    currentState = State.NOT_MOVING;
  }

  @Override
  public Command jogUpCommand() {
    return runEnd(() -> currentState = State.JOGGING_UP, () -> stop());
  }

  @Override
  public Command jogDownCommand() {
    return runEnd(() -> currentState = State.JOGGING_DOWN, () -> stop());
  }

  @Override
  public Command moveToPositionZeroCommand() {
    return runOnce(
        () -> determineNextState(Elevator.LEVEL_ZERO_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelOneCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_ONE_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_TWO_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelThreeCommand() {
    return runOnce(
        () -> determineNextState(Elevator.LEVEL_THREE_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelFourCommand() {
    return runOnce(
        () -> determineNextState(Elevator.LEVEL_FOUR_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  private void updateConfig() {
    pidController.setP(pConstantSubscriber.get());
    pidController.setI(iConstantSubscriber.get());
    pidController.setD(dConstantSubscriber.get());

    System.out.println("Kp = " + pConstantSubscriber.get() + " Ki = " + iConstantSubscriber.get() + " Kd = "
        + dConstantSubscriber.get());
  }

  public Command updateConfigCommand() {
    return runOnce(() -> updateConfig());
  }

  private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> motorOne.setControl(m_voltReq.withOutput(volts.in(Units.Volts))),
         null,
         this
      )
   );

  @Override
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    currentState = State.SYSID;
    return m_sysIdRoutine.quasistatic(direction);
  }

  @Override
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    currentState = State.SYSID;
    return m_sysIdRoutine.dynamic(direction);
  }
}