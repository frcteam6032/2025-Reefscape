// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralInfeed;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class CoralManagement {
    private static CoralInfeed infeed;
    private static ElevatorSubsystem elevator;
    private static ElevatorPosition targetPosition;

    public static void init(CoralInfeed Infeed, ElevatorSubsystem Elevator) {
        CoralManagement.infeed = Infeed;
        CoralManagement.elevator = Elevator;
    }

    private static void cyclePosition() {
        switch (targetPosition) {
            case Level1:
                targetPosition = ElevatorPosition.Level2;
                break;
            case Level2:
                targetPosition = ElevatorPosition.Level3;
                break;
            case Level3:
                targetPosition = ElevatorPosition.Home;
                break;
            default:
                targetPosition = ElevatorPosition.Level1;
                break;
        }
    }

    public static Command cyclePositionsCommand() {
        return Commands.runOnce(() -> cyclePosition());
    }

    public static Command cycleAndRunToPositionCommand() {
        return cyclePositionsCommand().andThen(runToPositionCommand(targetPosition));
    }

    public static enum ElevatorPosition {
        Home(0, 0),
        FeederStation(0.0, 47),
        Level1(-1, -1),
        Level2(41.8, 150),
        Level3(57.3, 200),
        Level4(-1, -1);

        public double Height;
        public double Angle;

        // Height: Inches, Agle: Degrees
        ElevatorPosition(double height, double angle) {
            this.Height = height;
            this.Angle = angle;
        };
    }

    public static Command runToPositionCommand(ElevatorPosition desiredPosition) {
        return infeed.runToPositionCommand(desiredPosition).alongWith(elevator.runToPositionCommand(desiredPosition));
    }

    public static Command automaticElevatorCommand(boolean resolved) {
        if (resolved == true) {
            return runToPositionCommand(ElevatorPosition.Level2);
        }
        return null;
    }

}
