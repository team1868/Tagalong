import java.io.PrintWriter;
import java.util.Objects;

public class Solver {
  private static double _configuredMaxA = 360; // degs per second squared
  private static double _configuredMaxV = 180; // degs per second

  public static void main(String[] args) {
    double[][][] matrix = new double[360][1000][2];
    ArmFeedforward ff =
        new ArmFeedforward(0.0, 0.13, 0.53 * (Math.PI / 180), 0.0); // TODO tune

    // iterate
    for (int start = matrix.length - 1; start >= 0; start--) {
      for (int end = 0; end < matrix[start].length; end++) {
        searchV2(ff, start, end, matrix);
        System.out.println(end + " column finished");
      }
      System.out.println(start + " row finished");
    }

    try {
      // print results to file
      PrintWriter kinematicLimitsFile =
          new PrintWriter("armKinematicLimits.csv");

      for (int i = 0; i < matrix.length; i++) {
        for (int j = 0; j < matrix[i].length; j++) {
          for (int k = 0; k < matrix[i][j].length; k++) {
            kinematicLimitsFile.print(matrix[i][j][k] + ",");
          }
        }
        kinematicLimitsFile.println();
      }

      kinematicLimitsFile.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void stupidSearch(Double[][][] resultMatrix, ArmFeedforward ff,
                           int start, int end) {
    double foundA = 0;
    double foundV = 0;
    int fastestCounter = Integer.MAX_VALUE;

    for (double tempA = _configuredMaxA; tempA > 0.0;
         tempA -= 0.1) { // start from higher vals bc more efficient
      for (double tempMaxV = _configuredMaxV; tempMaxV > 0.0; tempMaxV -= 0.1) {
        double curV = 0;
        double curPos = start;
        int counter = 0;

        // simulating robot periodic while accelerating
        while (curV < tempMaxV) {
          // check if in a legal state
          if ((ff.maxAchievableAcceleration(12, curPos, curV) < tempA) ||
              (ff.maxAchievableVelocity(12, curPos, tempA) < tempMaxV)) {
            break;
          }

          // update our position and velo
          curV = Math.min(curV + (tempA * 0.02),
                          tempMaxV); // ensure doesn't go above tempMaxV
          curPos += curV * 0.02;
          counter++;
          if (counter * 2 > fastestCounter) {
            break;
          }
        }

        if (curV < tempMaxV) {
          continue;
        }

        // check when should decel
        double decelPos = mustDecelBeforePos(curV, tempA, end);
        double accelDecelDuration = counter;

        if (curPos > decelPos) {
          // never hit max velocity
          continue;
        }
        while (curPos < decelPos) {
          // check voltage constraints
          if (ff.maxAchievableVelocity(12, curPos, 0.0) < tempMaxV) {
            break;
          }
          // update our position
          curPos += curV * 0.02;
          counter++;
          if (counter + accelDecelDuration > fastestCounter) {
            break;
          }
        }
        if (curPos < decelPos) {
          continue;
        }

        // decel to 0
        while (curV > 0) {
          // check if in a legal state
          if (ff.minAchievableAcceleration(12, curPos, curV) > -tempA ||
              ff.maxAchievableVelocity(12, curPos, -tempA) < tempMaxV) {
            break;
          }

          // update our position and velo
          curV = Math.max(curV - (tempA * 0.02), 0.0);
          curPos += curV * 0.02;
          counter++;
          if (counter > fastestCounter) {
            break;
          }
        }
        if (curV > 0) {
          continue;
        }

        if (counter < fastestCounter) {
          foundA = tempA;
          foundV = tempMaxV;
        }
      }
    }

    resultMatrix[start][end][0] = foundV;
    resultMatrix[start][end][1] = foundA;
  }

  static public void searchV2(ArmFeedforward ff, int start, int end,
                              double[][][] resultMatrix) {

    double foundA = 0;
    double foundV = 0;
    double fastest = Double.MAX_VALUE;

    if (start == end) {
      resultMatrix[start][end][1] = _configuredMaxA;
      resultMatrix[start][end][0] = _configuredMaxV;
    }
    double startA = _configuredMaxA;
    double startV = _configuredMaxV;

    // use knowledge from previous path
    if (end != 0) {
      startA = resultMatrix[start][end - 1][1];
      startV = resultMatrix[start][end - 1][0];
    }
    if (start != resultMatrix.length - 1) {
      startA = Math.min(resultMatrix[start + 1][end][1], startA);
      startV = Math.min(resultMatrix[start + 1][end][0], startV);
    }

    for (double tempA = startA; tempA > 0.0; tempA -= 0.1) {
      for (double tempMaxV = startV; tempMaxV > 0.0; tempMaxV -= 0.1) {
        TrapezoidProfile.Constraints constraint =
            new TrapezoidProfile.Constraints(tempMaxV, tempA);
        TrapezoidProfile.State goal = new TrapezoidProfile.State(end, 0.0);
        TrapezoidProfile.State init = new TrapezoidProfile.State(start, 0.0);
        TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(constraint, goal, init);
        // equal to bc good if time is same but accel is lower 
        if (trapezoidProfile.totalTime() >= fastest) {
          continue;
        }
        TrapezoidProfile.State prev = trapezoidProfile.calculate(0.0);
        boolean legal = true;

        for (double time = 0.0; time <= trapezoidProfile.totalTime();
             time += 0.02) {
          TrapezoidProfile.State current = trapezoidProfile.calculate(time);
          double curA = current.velocity - prev.velocity; 

          if (ff.maxAchievableAcceleration(12, current.position,
                                           current.velocity) < curA ||
              ff.minAchievableAcceleration(12, current.position,
                                           current.velocity) > curA ||
              ff.maxAchievableVelocity(12, current.position, curA) <
                  current.velocity ||
              ff.minAchievableVelocity(12, current.position, curA) >
                  current.velocity) {
            legal = false;
            break;
          }
          prev = current;
        }

        if (legal) {
          foundA = tempA;
          foundV = tempMaxV;
          fastest = trapezoidProfile.totalTime();
        }
      }
    }
    resultMatrix[start][end][0] = foundV;
    resultMatrix[start][end][1] = foundA;
  }

  public double mustDecelBeforePos(double plateauV, double accel,
                                    double targetPos) {

    double num20MSCycles =
        Math.ceil(50.0 * plateauV / accel); // don't want partial cycle

    double dist = (0.0 + plateauV) * 0.02 * num20MSCycles /
                  2.0; // dist travelled to get to plateau

    return targetPos - dist;
  }

  public boolean canDecel(double curPos, double curV, double accel,
                           double targetPos) {
    while (true) {
      curPos += curV * 0.02;
      curV += curV + (accel * 0.02);
      if (curPos <= targetPos && curV <= 0.0) {
        return true;
      }
      if (curPos >= targetPos && curV > 0.0) {
        return false;
      }
    }
  }

  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  /**
   * A helper class that computes feedforward outputs for a simple arm (modeled
   * as a motor acting against the force of gravity on a beam suspended at an
   * angle).
   */
  public static class ArmFeedforward {
    public final double ks;
    public final double kg;
    public final double kv;
    public final double ka;

    /**
     * Creates a new ArmFeedforward with the specified gains. Units of the gain
     * values will dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     */
    public ArmFeedforward(double ks, double kg, double kv, double ka) {
      this.ks = ks;
      this.kg = kg;
      this.kv = kv;
      this.ka = ka;
    }

    /**
     * Creates a new ArmFeedforward with the specified gains. Acceleration gain
     * is defaulted to zero. Units of the gain values will dictate units of the
     * computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     */
    public ArmFeedforward(double ks, double kg, double kv) {
      this(ks, kg, kv, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians The position (angle) setpoint. This angle should
     *     be measured from the horizontal (i.e. if the provided angle is 0, the
     *     arm should be parallel with the floor). If your encoder does not
     *     follow this convention, an offset should be added.
     * @param velocityRadPerSec The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocityRadPerSec,
                            double accelRadPerSecSquared) {
      return ks * Math.signum(velocityRadPerSec) +
          kg * Math.cos(positionRadians) + kv * velocityRadPerSec +
          ka * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint
     * (acceleration is assumed to be zero).
     *
     * @param positionRadians The position (angle) setpoint. This angle should
     *     be measured from the horizontal (i.e. if the provided angle is 0, the
     *     arm should be parallel with the floor). If your encoder does not
     *     follow this convention, an offset should be added.
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity) {
      return calculate(positionRadians, velocity, 0);
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage
     * supply, a position, and an acceleration. Useful for ensuring that
     * velocity and acceleration constraints for a trapezoidal profile are
     * simultaneously achievable - enter the acceleration constraint, and this
     * will give you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be
     *     parallel with the floor). If your encoder does not follow this
     *     convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @return The maximum possible velocity at the given acceleration and
     *     angle.
     */
    public double maxAchievableVelocity(double maxVoltage, double angle,
                                        double acceleration) {
      // Assume max velocity is positive
      return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage
     * supply, a position, and an acceleration. Useful for ensuring that
     * velocity and acceleration constraints for a trapezoidal profile are
     * simultaneously achievable - enter the acceleration constraint, and this
     * will give you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be
     *     parallel with the floor). If your encoder does not follow this
     *     convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @return The minimum possible velocity at the given acceleration and
     *     angle.
     */
    public double minAchievableVelocity(double maxVoltage, double angle,
                                        double acceleration) {
      // Assume min velocity is negative, ks flips sign
      return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka) / kv;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be
     *     parallel with the floor). If your encoder does not follow this
     *     convention, an offset should be added.
     * @param velocity The velocity of the arm.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double angle,
                                            double velocity) {
      return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg -
              velocity * kv) /
          ka;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be
     *     parallel with the floor). If your encoder does not follow this
     *     convention, an offset should be added.
     * @param velocity The velocity of the arm.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double angle,
                                            double velocity) {
      return maxAchievableAcceleration(-maxVoltage, angle, velocity);
    }
  }

  /**
   * A trapezoid-shaped velocity profile.
   *
   * <p>While this class can be used for a profiled movement from start to
   * finish, the intended usage is to filter a reference's dynamics based on
   * trapezoidal velocity constraints. To compute the reference obeying this
   * constraint, do the following.
   *
   * <p>Initialization:
   *
   * <pre><code>
   * TrapezoidProfile.Constraints constraints =
   *   new TrapezoidProfile.Constraints(kMaxV, kMaxA);
   * TrapezoidProfile.State previousProfiledReference =
   *   new TrapezoidProfile.State(initialReference, 0.0);
   * </code></pre>
   *
   * <p>Run on update:
   *
   * <pre><code>
   * TrapezoidProfile profile =
   *   new TrapezoidProfile(constraints, unprofiledReference,
   * previousProfiledReference); previousProfiledReference =
   * profile.calculate(timeSincePreviousUpdate);
   * </code></pre>
   *
   * <p>where `unprofiledReference` is free to change between calls. Note that
   * when the unprofiled reference is within the constraints, `calculate()`
   * returns the unprofiled reference unchanged.
   *
   * <p>Otherwise, a timer can be started to provide monotonic values for
   * `calculate()` and to determine when the profile has completed via
   * `isFinished()`.
   */
  public static class TrapezoidProfile {
    // The direction of the profile, either 1 for forwards or -1 for inverted
    private int m_direction;

    private Constraints m_constraints;
    private State m_initial;
    private State m_goal;

    private double m_endAccel;
    private double m_endFullSpeed;
    private double m_endDecel;

    public static class Constraints {
      public final double maxVelocity;

      public final double maxAcceleration;

      /**
       * Construct constraints for a TrapezoidProfile.
       *
       * @param maxVelocity maximum velocity
       * @param maxAcceleration maximum acceleration
       */
      public Constraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
      }
    }

    public static class State {
      public double position;

      public double velocity;

      public State() {}

      public State(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
      }

      @Override
      public boolean equals(Object other) {
        if (other instanceof State) {
          State rhs = (State)other;
          return this.position == rhs.position && this.velocity == rhs.velocity;
        } else {
          return false;
        }
      }

      @Override
      public int hashCode() {
        return Objects.hash(position, velocity);
      }
    }

    /**
     * Construct a TrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param goal The desired state when the profile is complete.
     * @param initial The initial state (usually the current state).
     */
    public TrapezoidProfile(Constraints constraints, State goal,
                            State initial) {
      m_direction = shouldFlipAcceleration(initial, goal) ? -1 : 1;
      m_constraints = constraints;
      m_initial = direct(initial);
      m_goal = direct(goal);

      if (m_initial.velocity > m_constraints.maxVelocity) {
        m_initial.velocity = m_constraints.maxVelocity;
      }

      // Deal with a possibly truncated motion profile (with nonzero initial or
      // final velocity) by calculating the parameters as if the profile began
      // and ended at zero velocity
      double cutoffBegin = m_initial.velocity / m_constraints.maxAcceleration;
      double cutoffDistBegin =
          cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

      double cutoffEnd = m_goal.velocity / m_constraints.maxAcceleration;
      double cutoffDistEnd =
          cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

      // Now we can calculate the parameters as if it was a full trapezoid
      // instead of a truncated one

      double fullTrapezoidDist = cutoffDistBegin +
                                 (m_goal.position - m_initial.position) +
                                 cutoffDistEnd;
      double accelerationTime =
          m_constraints.maxVelocity / m_constraints.maxAcceleration;

      double fullSpeedDist =
          fullTrapezoidDist -
          accelerationTime * accelerationTime * m_constraints.maxAcceleration;

      // Handle the case where the profile never reaches full speed
      if (fullSpeedDist < 0) {
        accelerationTime =
            Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
        fullSpeedDist = 0;
      }

      m_endAccel = accelerationTime - cutoffBegin;
      m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
      m_endDecel = m_endFullSpeed + accelerationTime - cutoffEnd;
    }

    /**
     * Construct a TrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param goal The desired state when the profile is complete.
     */
    public TrapezoidProfile(Constraints constraints, State goal) {
      this(constraints, goal, new State(0, 0));
    }

    /**
     * Calculate the correct position and velocity for the profile at a time t
     * where the beginning of the profile was at time t = 0.
     *
     * @param t The time since the beginning of the profile.
     * @return The position and velocity of the profile at time t.
     */
    public State calculate(double t) {
      State result = new State(m_initial.position, m_initial.velocity);

      if (t < m_endAccel) {
        result.velocity += t * m_constraints.maxAcceleration;
        result.position +=
            (m_initial.velocity + t * m_constraints.maxAcceleration / 2.0) * t;
      } else if (t < m_endFullSpeed) {
        result.velocity = m_constraints.maxVelocity;
        result.position += (m_initial.velocity +
                            m_endAccel * m_constraints.maxAcceleration / 2.0) *
                               m_endAccel +
                           m_constraints.maxVelocity * (t - m_endAccel);
      } else if (t <= m_endDecel) {
        result.velocity =
            m_goal.velocity + (m_endDecel - t) * m_constraints.maxAcceleration;
        double timeLeft = m_endDecel - t;
        result.position =
            m_goal.position -
            (m_goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) *
                timeLeft;
      } else {
        result = m_goal;
      }

      return direct(result);
    }

    /**
     * Returns the time left until a target distance in the profile is reached.
     *
     * @param target The target distance.
     * @return The time left until a target distance in the profile is reached.
     */
    public double timeLeftUntil(double target) {
      double position = m_initial.position * m_direction;
      double velocity = m_initial.velocity * m_direction;

      double endAccel = m_endAccel * m_direction;
      double endFullSpeed = m_endFullSpeed * m_direction - endAccel;

      if (target < position) {
        endAccel = -endAccel;
        endFullSpeed = -endFullSpeed;
        velocity = -velocity;
      }

      endAccel = Math.max(endAccel, 0);
      endFullSpeed = Math.max(endFullSpeed, 0);

      final double acceleration = m_constraints.maxAcceleration;
      final double deceleration = -m_constraints.maxAcceleration;

      double distToTarget = Math.abs(target - position);
      if (distToTarget < 1e-6) {
        return 0;
      }

      double accelDist =
          velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

      double decelVelocity;
      if (endAccel > 0) {
        decelVelocity = Math.sqrt(
            Math.abs(velocity * velocity + 2 * acceleration * accelDist));
      } else {
        decelVelocity = velocity;
      }

      double fullSpeedDist = m_constraints.maxVelocity * endFullSpeed;
      double decelDist;

      if (accelDist > distToTarget) {
        accelDist = distToTarget;
        fullSpeedDist = 0;
        decelDist = 0;
      } else if (accelDist + fullSpeedDist > distToTarget) {
        fullSpeedDist = distToTarget - accelDist;
        decelDist = 0;
      } else {
        decelDist = distToTarget - fullSpeedDist - accelDist;
      }

      double accelTime =
          (-velocity + Math.sqrt(Math.abs(velocity * velocity +
                                          2 * acceleration * accelDist))) /
          acceleration;

      double decelTime =
          (-decelVelocity +
           Math.sqrt(Math.abs(decelVelocity * decelVelocity +
                              2 * deceleration * decelDist))) /
          deceleration;

      double fullSpeedTime = fullSpeedDist / m_constraints.maxVelocity;

      return accelTime + fullSpeedTime + decelTime;
    }

    /**
     * Returns the total time the profile takes to reach the goal.
     *
     * @return The total time the profile takes to reach the goal.
     */
    public double totalTime() { return m_endDecel; }

    /**
     * Returns true if the profile has reached the goal.
     *
     * <p>The profile has reached the goal if the time since the profile started
     * has exceeded the profile's total time.
     *
     * @param t The time since the beginning of the profile.
     * @return True if the profile has reached the goal.
     */
    public boolean isFinished(double t) { return t >= totalTime(); }

    /**
     * Returns true if the profile inverted.
     *
     * <p>The profile is inverted if goal position is less than the initial
     * position.
     *
     * @param initial The initial state (usually the current state).
     * @param goal The desired state when the profile is complete.
     */
    private static boolean shouldFlipAcceleration(State initial, State goal) {
      return initial.position > goal.position;
    }

    // Flip the sign of the velocity and position if the profile is inverted
    private State direct(State in) {
      State result = new State(in.position, in.velocity);
      result.position = result.position * m_direction;
      result.velocity = result.velocity * m_direction;
      return result;
    }
  }
}
