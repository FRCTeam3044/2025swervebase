package frc.robot.statemachine.states.tele;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AutoTargetUtils;
import frc.robot.util.bboard.ButtonBoard;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ScoreAlgaeNet extends State {
    private static ConfigurableParameter<Double> launchAngle = new ConfigurableParameter<>(2.2, "Net Launch Angle");
    private static ConfigurableParameter<Double> stopAngle = new ConfigurableParameter<>(2.6, "Net Stop Angle");

    public ScoreAlgaeNet(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive,
            EndEffector endEffector, LEDs LEDs, Shoulder shoulder, Elevator elevator) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, AutoTargetUtils::net));
        startWhenActive(shoulder.intakeCoral());
        startWhenActive(elevator.toNet());
        t(() -> elevator.isAtTarget() && shoulder.inSafeZone())
                .whileTrue(shoulder.climb().until(() -> shoulder.getShoulderAngle() > stopAngle.get()));
        startWhenActive(Commands.run(() -> {
            System.out.println("Shoulder Angle: " + shoulder.getShoulderAngle() + " Launch Angle: "
                    + launchAngle.get() + " Stop Angle: " + stopAngle.get());
        }));
        t(() -> shoulder.getShoulderAngle() > launchAngle.get()).onTrue(endEffector.algaeOutNet());
        t(() -> shoulder.getShoulderAngle() > stopAngle.get()).onTrue(shoulder.idle());
    }
}
