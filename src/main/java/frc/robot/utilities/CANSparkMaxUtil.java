package frc.robot.utilities;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
    public enum Usage {
        kAll,
        kPositionOnly,
        kVelocityOnly,
        kMinimal
    };

    /**
    * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
    * frame period of nonessential frames from 20ms to 500ms.
    *
    * See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames for a description of the status frames.
    *
    * @param motor The motor to adjust the status frame periods on.
    * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is constructed.
    * @param enableFollowing Whether to enable motor following.
    */

    public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage, boolean enableFollowing) {
        if (enableFollowing) {
            motor.setPeriodicFrameTimeout(10);
        } else {
            motor.setPeriodicFrameTimeout(500);
        }

        if (usage == Usage.kAll) {
            motor.setPeriodicFrameTimeout(20);
        } else if (usage == Usage.kPositionOnly) {
            motor.setPeriodicFrameTimeout(20);
        } else if (usage == Usage.kVelocityOnly) {
            motor.setPeriodicFrameTimeout(20);
        } else if (usage == Usage.kMinimal) {
            motor.setPeriodicFrameTimeout(20);
        }
    }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frame for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is constructed.
   */
    public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage) {
        setCANSparkMaxBusUsage(motor, usage, false);
    }
}