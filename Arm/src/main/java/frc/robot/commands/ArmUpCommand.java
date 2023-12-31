package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.Arm;

public class ArmUpCommand extends CommandBase {
  Arm _arm;
  ArmPositions _targetPos;
  int _upperTol;
  int _lowerTol;
  private boolean _startedMovement;
  public ArmUpCommand(Arm arm, ArmPositions targetPos, int upperTol,
                      int lowerTol) {
    _arm = arm;
    _targetPos = targetPos;
    _upperTol = upperTol;
    _lowerTol = lowerTol;
  }
  @Override
  public void initialize() {
    _startedMovement = false;
  }

  @Override
  public void execute() {
    if (!_startedMovement) { 
      _startedMovement = true;
      _arm.setArmProfile(
          _targetPos); 
    }                  
  }

  @Override
  public void end(boolean interrupted) {
    _arm.setPower(
        0.0); 
  }

  @Override
  public boolean isFinished() {
    return _arm.inTolerance(_upperTol, _lowerTol);
  }
}
