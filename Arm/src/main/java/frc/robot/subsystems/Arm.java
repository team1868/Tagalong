package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConfs;
import frc.robot.constants.ArmPositions;
import frc.robot.utils.CTREConversion;
import java.io.File;
import java.util.Scanner;

public class Arm extends SubsystemBase {
  private WPI_TalonFX _armMotor = new WPI_TalonFX(10);
  private WPI_CANCoder _armCanCoder = new WPI_CANCoder(11, "rio");
  private static TalonFXConfiguration _armMotorFXConfig =
      new TalonFXConfiguration();
  private static CANCoderConfiguration _armCanCoderConfig;
  private GenericPublisher _currentTicksEntry, _targetTicksEntry;
  private TrapezoidProfile _profile;

  private ArmFeedforward _armFF = new ArmFeedforward(
      ArmConfs.ARM_KS_VOLTS, ArmConfs.ARM_KG_VOLTS, ArmConfs.ARM_KV_VOLTS_RADPS,
      ArmConfs.ARM_KA_VOLTS_RADPSQ);
  private ArmPositions _goalPosition;

  private boolean _followingProfile = false;

  private Timer _timer = new Timer();

  private double _prevVelocityDPS;

  private double[][][] _kinematicLimits = new double[360][360][2];

  public Arm() {
    /* Arm Motor Configuration */
    _profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ArmConfs.ARM_MAX_V_DPS,
                                         ArmConfs.ARM_MAX_ACC_DPSQ),
        new TrapezoidProfile.State(0.0, 0.0));
    int counter = 0;
    while (!checkInitStatus()) {
      System.out.println("ARM Check Init Status : " + counter);
      counter++;
    }

    initCanCoderConfigs();
    configArmCanCoder();
    initArmMotor();
    configArmMotor();
    loadKinematicLimitsFromFile();
  }

  @Override
  public void periodic() {
    double falconPosTicks = _armMotor.getSelectedSensorPosition();
    if (falconPosTicks >= 65536) {
      _armMotor.setSelectedSensorPosition(falconPosTicks %= 65536);
    }
    if (falconPosTicks < 0) {
      _armMotor.setSelectedSensorPosition(falconPosTicks += 65536);
    }

    if (_followingProfile) {
      TrapezoidProfile.State goalState = _profile.calculate(_timer.get());
      double velocityError = goalState.velocity - _prevVelocityDPS;
      _prevVelocityDPS = goalState.velocity;

      double ff = _armFF.calculate(Math.toRadians(goalState.position),
                                   Math.toRadians(goalState.velocity)) /
                  12.0; // needs angle zeroed when parallel to ground

      _armMotor.set(ControlMode.Position,
                    CTREConversion.degreesToFalcon(goalState.position,
                                                   ArmConfs.ARM_GEAR_RATIO),
                    DemandType.ArbitraryFeedForward, ff);
    }
  }

  public void loadKinematicLimitsFromFile() {
    try {
      File kinematicLimitsFile =
          new File(Filesystem.getDeployDirectory().getAbsolutePath() +
                   "/armKinematicLimits.csv");
      Scanner limits = new Scanner(kinematicLimitsFile);

      // i is each row
      String[] values = limits.toString().split(",");
      for (int i = 0; i < _kinematicLimits.length; i++) {
        for (int j = 0; j < _kinematicLimits[i].length; j++) {
          //     // read 2 values (cols) for every end location
          //     // read v
          _kinematicLimits[i][j][0] = Double.valueOf(values[i * (j * 2)]);
          //     // read a
          _kinematicLimits[i][j][1] = Double.valueOf(values[i * (1 + j * 2)]);
        }
      }

      limits.close();
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public void configShuffleboard() {}
  public void updateShuffleboard() {

    _currentTicksEntry.setInteger(getCurPosition());
    _targetTicksEntry.setInteger(getTargetPosition().ticks);
  }
  public void setTargetPosition(ArmPositions pos) { _goalPosition = pos; }

  public ArmPositions getTargetPosition() { return _goalPosition; }

  public int getCurPosition() {
    return (int)_armMotor.getSelectedSensorPosition();
  }

  public void setPower(double power) {
    _followingProfile = false;
    _armMotor.set(power);
  }

  public double getPower() { return _armMotor.get(); }

  public void initArmMotor() {
    _armMotorFXConfig.slot0.kP = ArmConfs.ARM_PFAC;
    _armMotorFXConfig.slot0.kI = ArmConfs.ARM_IFAC;
    _armMotorFXConfig.slot0.kD = ArmConfs.ARM_DFAC;
    _armMotorFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        ArmConfs.ARM_SUPPLY_ENABLE_CURRENT_LIMIT,
        ArmConfs.ARM_SUPPLY_CONTINUOUS_CURRENT_LIMIT,
        ArmConfs.ARM_SUPPLY_PEAK_CURRENT_LIMIT,
        ArmConfs.ARM_SUPPLY_PEAK_CURRENT_DURATION);
    _armMotorFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        ArmConfs.ARM_STATOR_ENABLE_CURRENT_LIMIT,
        ArmConfs.ARM_STATOR_CONTINUOUS_CURRENT_LIMIT,
        ArmConfs.ARM_STATOR_PEAK_CURRENT_LIMIT,
        ArmConfs.ARM_STATOR_PEAK_CURRENT_DURATION);
    _armMotorFXConfig.initializationStrategy =
        SensorInitializationStrategy.BootToZero;
    _armMotorFXConfig.openloopRamp = ArmConfs.ARM_OPEN_LOOP_RAMP;
    _armMotorFXConfig.closedloopRamp = ArmConfs.ARM_CLOSED_LOOP_RAMP;
    _armMotorFXConfig.voltageCompSaturation =
        ArmConfs.ARM_VOLTAGE_COMP_SATURATION;
    _armMotorFXConfig.neutralDeadband = ArmConfs.ARM_MOTOR_DEADBAND;
  }

  public void configArmMotor() {
    _armMotor.configFactoryDefault();
    _armMotor.setNeutralMode(ArmConfs.ARM_NEUTRAL_MODE);
    _armMotor.configAllSettings(_armMotorFXConfig);
    resetToAbsolute();
  }

  public boolean resetMotorConfig() {
    boolean result = false;
    if (_armCanCoder.hasResetOccurred()) {
      configArmCanCoder();
      result = true;
    }
    if (_armMotor.hasResetOccurred()) {
      configArmMotor();
      result = true;
    }
    return result;
  }
  private void initCanCoderConfigs() {
    if (_armCanCoderConfig == null) {
      _armCanCoderConfig = new CANCoderConfiguration();
      _armCanCoderConfig.absoluteSensorRange =
          AbsoluteSensorRange.Unsigned_0_to_360;
      _armCanCoderConfig.sensorDirection = true;
      _armCanCoderConfig.initializationStrategy =
          SensorInitializationStrategy.BootToAbsolutePosition;
      _armCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
  }

  private void configArmCanCoder() {
    _armCanCoder.configFactoryDefault(100);
    _armCanCoder.configAllSettings(_armCanCoderConfig);
    _armCanCoder.configMagnetOffset(140);
  }

  public boolean inTolerance(int upperTol, int lowerTol) {
    double error = (CTREConversion.falconToDegrees(
        (getTargetPosition().ticks - getCurPosition()),
        ArmConfs.ARM_GEAR_RATIO));
    return (error <= upperTol) && (error >= lowerTol);
  }

  public boolean checkInitStatus() {
    ErrorCode initStatus = _armMotor.configFactoryDefault();
    return initStatus == ErrorCode.OK;
  }

  private int clamp(int target, int min, int max) {
    return Math.max(min, Math.min(max, target));
  }
  public void setArmProfile(ArmPositions goalPosition, double velocity) {
    _followingProfile = true;
    _goalPosition = goalPosition;
    double goalPosDeg = CTREConversion.falconToDegrees(_goalPosition.ticks,
                                                       ArmConfs.ARM_GEAR_RATIO);
    TrapezoidProfile.State curState = _profile.calculate(_timer.get());

    curState.position = CTREConversion.falconToDegrees(getCurPosition(),
                                                       ArmConfs.ARM_GEAR_RATIO);

    if (Math.abs(curState.position) > 360) {
      curState.position %= 360;
    }
    if (curState.position < 0) {
      curState.position += 360;
    }

    _profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ArmConfs.ARM_MAX_V_DPS,
                                         ArmConfs.ARM_MAX_ACC_DPSQ),
        // new TrapezoidProfile.Constraints(
        //     _kinematicLimits[(int) curState.position][goalPosDeg][0],
        //     _kinematicLimits[(int) curState.position][goalPosDeg][1]
        // ),
        new TrapezoidProfile.State(goalPosDeg, velocity), curState);
    _prevVelocityDPS = curState.velocity;
    _timer.restart();
  }

  public void setArmProfile(ArmPositions goalPosition) {
    setArmProfile(goalPosition, 0.0);
  }

  public void onDisable() {
    _followingProfile = false;
    _armMotor.setNeutralMode(ArmConfs.ARM_DISABLED_MODE);

    _profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ArmConfs.ARM_MAX_V_DPS,
                                         ArmConfs.ARM_MAX_ACC_DPSQ),
        new TrapezoidProfile.State(0.0, 0.0));
  }

  public void resetToAbsolute() {
    double lastAngleDeg = _armCanCoder.getAbsolutePosition();
    double absolutePosition =
        lastAngleDeg * (2048.0 / 360) * ArmConfs.ARM_GEAR_RATIO;
    _armMotor.setSelectedSensorPosition(absolutePosition);
    System.out.println("CANCODER POS" + _armCanCoder.getAbsolutePosition());
  }
  public void disabledPeriodic() {}
}
