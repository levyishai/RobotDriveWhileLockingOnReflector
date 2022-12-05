package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A command that turns the robot to the given target.
 */
public class TurnToTargetCommand extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final PIDController pidController;
    private final double target;
    private final BooleanSupplier hasTargetSupplier;
    private final DoubleSupplier positionSupplier;

    /**
     * Constructs a new TurnToTargetCommand.
     *
     * @param pidController the PIDController to use
     * @param positionSupplier the target's position supplier
     * @param hasTargetSupplier a supplier that returns if the target is visible
     * @param target the target
     */
    public TurnToTargetCommand(
            PIDController pidController, double target,
            BooleanSupplier hasTargetSupplier, DoubleSupplier positionSupplier) {
        this.pidController = pidController;
        this.target = target;
        this.hasTargetSupplier = hasTargetSupplier;
        this.positionSupplier = positionSupplier;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(target);
    }

    @Override
    public void execute() {
        if (hasTargetSupplier.getAsBoolean()) {
            swerve.selfRelativeDrive(new Translation2d(), getAsRotation2d());
            return;
        }

        swerve.selfRelativeDrive(new Translation2d(), new Rotation2d(SwerveConstants.NO_TARGET_SPIN_RADIANS));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    private Rotation2d getAsRotation2d() {
        return new Rotation2d(pidController.calculate(positionSupplier.getAsDouble()));
    }
}
