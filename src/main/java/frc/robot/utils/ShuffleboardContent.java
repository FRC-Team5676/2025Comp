// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TraySubsystem;

/** Add your docs here. */
public class ShuffleboardContent {

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initRotateArm(ArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Rotate Arm");

                drLayout1.addNumber("Angle", () -> drive.getRotateArmDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getRotateArmTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getMinRotateArmDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getMaxRotateArmDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initBallScrew(BallScrewSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Ball Screw");

                drLayout1.addNumber("Distance", () -> drive.getActualUnits())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Distance", () -> drive.m_positionUnits)
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Distance", () -> drive.getUpPosition())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Distance", () -> drive.getDownPosition())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initLinearArm(ArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Linear Arm");

                drLayout1.addNumber("Angle", () -> drive.getLinearArmDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getLinearArmTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getExtendedLinearArmDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getRetractedLinearArmDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initClimber(ClimberSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Climber");

                drLayout1.addNumber("Angle", () -> drive.getActualDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getMinDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getMaxDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initWrist(ArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Wrist");

                drLayout1.addNumber("Angle", () -> drive.getWristDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getWristTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getMinWristDegrees())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getMaxWristDegrees())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initTray(TraySubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Tray");

                drLayout1.addNumber("Angle", () -> drive.getActualDegrees())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> drive.getTargetDegrees())
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> drive.getUpPosition())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> drive.getDownPosition())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

}
