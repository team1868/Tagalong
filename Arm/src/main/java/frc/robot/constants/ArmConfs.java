package frc.robot.constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmConfs {
  public static final double ARM_PFAC = 0.1;
  public static final double ARM_IFAC = 0.0;
  public static final double ARM_DFAC = 0.0;
  public static final double ARM_FFAC = 0.0;

  public static final boolean ARM_SUPPLY_ENABLE_CURRENT_LIMIT = true;
  public static final int ARM_SUPPLY_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int ARM_SUPPLY_PEAK_CURRENT_LIMIT = 40;
  public static final double ARM_SUPPLY_PEAK_CURRENT_DURATION = 0.1;

  public static final boolean ARM_STATOR_ENABLE_CURRENT_LIMIT = true;
  public static final int ARM_STATOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int ARM_STATOR_PEAK_CURRENT_LIMIT = 40;
  public static final double ARM_STATOR_PEAK_CURRENT_DURATION = 0.1;
  public static final double ARM_MOTOR_DEADBAND = 0.001;

  public static final double ARM_OPEN_LOOP_RAMP = 0.0;
  public static final double ARM_CLOSED_LOOP_RAMP = 0.0;
  public static final double ARM_VOLTAGE_COMP_SATURATION = 12.0;

  public static final double ARM_MAX_V_DPS = 150.0;
  public static final double ARM_MAX_ACC_DPSQ =
      300.0; 

  public static final double ARM_KS_VOLTS = 0.0;
  public static final double ARM_KG_VOLTS = 0.13; 
  public static final double ARM_KV_VOLTS_DPS = 0.53 * (Math.PI / 180);
  public static final double ARM_KA_VOLTS_DPSQ = 0.0 * (Math.PI / 180);

  public static final double ARM_GEAR_RATIO = 32; 

  public static final boolean ARM_MOTOR_INVERTED = true;
  public static final NeutralMode ARM_NEUTRAL_MODE = NeutralMode.Coast;
  public static final NeutralMode ARM_DISABLED_MODE = NeutralMode.Brake;
}
