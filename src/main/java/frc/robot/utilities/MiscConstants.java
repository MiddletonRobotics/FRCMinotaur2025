package frc.robot.utilities;

import com.revrobotics.spark.ClosedLoopSlot;

public class MiscConstants {

    public static enum RobotType {
      REAL,
      SIM
    }
  
    public static final class SimConstants {
      public static final double loopPeriodSec = 0.02;
    }
  
    public static final class ControllerConstants {
  
      public static final double deadband = 0.05;
    }
  
    public static final class MotorControllerConstants {
      /**
       * Slot1: small position error
       * Slot2: large posiotion error
       * Slot3: slow velocity
       * Slot4: fast velcity
       */
      public static final ClosedLoopSlot[] slots = new ClosedLoopSlot[] { ClosedLoopSlot.kSlot0, ClosedLoopSlot.kSlot1,
          ClosedLoopSlot.kSlot2, ClosedLoopSlot.kSlot3 };
  
      public static final int standardStallLimit = 30;
      public static final int standardFreeLimit = 50;
  
      public static final int relaxedStallLimit = 10;
      public static final int relaxedFreeLimit = 20;
  
      public static final double maxMotorVolts = 12.0;
  
    }
  
  }
