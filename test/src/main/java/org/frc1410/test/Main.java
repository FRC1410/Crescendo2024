package org.frc1410.test;

import edu.wpi.first.wpilibj.RobotBase;

public interface Main {
  static void main(String[] args) {
      RobotBase.startRobot(Robot::new);
  }
}