// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralInfeed;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class CoralManagement {
    private static CoralInfeed infeed;
    private static ElevatorSubsystem elevator;
    public static ElevatorPosition targetPosition = ElevatorPosition.Home;

    public static void init(CoralInfeed Infeed, ElevatorSubsystem Elevator) {
        CoralManagement.infeed = Infeed;
        CoralManagement.elevator = Elevator;

        DashboardStore.add("Coral State", () -> targetPosition.name());
    }

    private static void cyclePosition() {
        switch (targetPosition) {
            case Level2:
                targetPosition = ElevatorPosition.Level3;
                break;
            case Level3:
                targetPosition = ElevatorPosition.Home;
                break;
            default:
                targetPosition = ElevatorPosition.Level2;
                break;
        }
    }

    public static Command cyclePositionsCommand() {
        return Commands.runOnce(() -> cyclePosition());
    }

    public static Command cycleAndRunToPositionCommand() {
        return cyclePositionsCommand().andThen(runToPositionCommand(() -> targetPosition));
    }

    // 28.57
    public static enum ElevatorPosition {
        Home(15, 10),
        FeederStation(1.0, 62),
        Level1(5, 245),
        Level2(28.5, 245),
        Level3(80.32, 225),
        Level4(-1, -1);

        public double Height;
        public double Angle;

        // height: Elevator (Rotations)
        // angle: Coral Infeed (Degrees)
        ElevatorPosition(double height, double angle) {
            this.Height = height;
            this.Angle = angle;
        };
    }

    public static Command runToPositionCommand(Supplier<ElevatorPosition> desiredPosition) {
        return infeed.runToPositionCommand(desiredPosition).alongWith(elevator.runToPositionCommand(desiredPosition));
    }

    public static Command automaticElevatorCommand(boolean resolved) {
        if (resolved == true) {
            return runToPositionCommand(() -> targetPosition);
        }
        return null;
    }

}
