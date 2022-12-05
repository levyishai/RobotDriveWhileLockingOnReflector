package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A command that drives the robot while turning to the target.
 */
public class DriveWithTurnToTargetCommand extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final PIDController pidController;
    private final DoubleSupplier xPower, yPower;
    private final double target;
    private final BooleanSupplier hasTargetSupplier;
    private final DoubleSupplier positionSupplier;

    /**
     * Constructs a new DriveWithTurnToTargetCommand.
     *
     * @param pidController the PIDController to use
     * @param positionSupplier the target's position supplier
     * @param hasTargetSupplier a supplier that returns if the target is visible
     * @param target the target
     * @param xPower the forwards power in meters per second
     * @param yPower the leftwards power in meters per second
     */
    public DriveWithTurnToTargetCommand(
            PIDController pidController, DoubleSupplier xPower, DoubleSupplier yPower, double target,
            BooleanSupplier hasTargetSupplier, DoubleSupplier positionSupplier) {
        this.pidController = pidController;
        this.xPower = xPower;
        this.yPower = yPower;
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
            swerve.selfRelativeDrive(getAsTranslation2d(), getAsRotation2d());
            return;
        }

        swerve.selfRelativeDrive(getAsTranslation2d(), new Rotation2d(SwerveConstants.NO_TARGET_SPIN_RADIANS));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    private Rotation2d getAsRotation2d() {
        return new Rotation2d(pidController.calculate(positionSupplier.getAsDouble()));
    }

    private Translation2d getAsTranslation2d() {
        return new Translation2d(xPower.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND, yPower.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    }
}
