package frc.robot.constants;

public enum ArmPositions {
  ARM_POS_90(16384),
  ARM_POS_0(0);

  public final int ticks;
  ArmPositions(int ticks) {
    this.ticks = ticks;
  }
}
