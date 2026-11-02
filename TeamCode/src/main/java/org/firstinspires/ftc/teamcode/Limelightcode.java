package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Limelight AprilTag tracking for the turret bot.
 *
 * Each goal is a seesaw with two sides, and each side has two AprilTags. We shoot into the
 * side facing up. Each side has its own Limelight pipeline with an ID filter so it only sees
 * that side's two tags. We aim at the midpoint between them.
 *
 * Controls:
 *   During INIT:  X = blue alliance, B = red alliance
 *   INIT or PLAY: left bumper = side 1, right bumper = side 2 (pick the side facing up)
 *   During PLAY:  hold A = turret aims and flywheel spins up (only if USE_TURRET / USE_SHOOTER)
 *                 while holding A, pull right trigger = shoot (only fires when aimed and up to speed)
 *
 * Things marked TODO need real numbers from your robot / the game manual.
 */
@TeleOp(name = "Limelight", group = "Test")
public class Limelightcode extends LinearOpMode {

    // ---------------- Configuration ----------------

    // Set to true once the turret motor is on the robot and in the configuration.
    // While false, this only reads the camera and prints telemetry (no motors move).
    static final boolean USE_TURRET = false;
    static final String TURRET_MOTOR_NAME = "turret"; // TODO: match the robot configuration

    // Set to true once the shooter is on the robot and in the configuration.
    static final boolean USE_SHOOTER = false;
    static final String SHOOTER_MOTOR_NAME = "shooter"; // TODO: match the robot configuration
    static final String FEEDER_SERVO_NAME = "feeder";   // TODO: match the robot configuration
    static final double FEEDER_REST_POS = 0.0;          // TODO: servo position when not shooting
    static final double FEEDER_PUSH_POS = 0.5;          // TODO: servo position that pushes a game piece in
    static final double SHOOTER_VEL_TOLERANCE = 50;     // ticks/sec, "up to speed" when within this

    // Pipelines set up in the Limelight web UI (http://limelight.local:5801), one per goal side,
    // each with an ID filter for that side's two tags. -1 = not set up yet.
    // If side 1 and side 2 are backwards, swap the numbers.
    static final int RED_SIDE1_PIPELINE = 0;
    static final int RED_SIDE2_PIPELINE = 1;
    static final int BLUE_SIDE1_PIPELINE = -1; // TODO: set up in the Limelight web UI
    static final int BLUE_SIDE2_PIPELINE = -1; // TODO: set up in the Limelight web UI

    // TODO: Measure these on the robot / field. Used for distance estimation.
    static final double CAMERA_HEIGHT_IN = 10.0;     // lens height off the floor
    static final double CAMERA_PITCH_DEG = 20.0;     // how far the camera is tilted up
    static final double TAG_CENTER_HEIGHT_IN = 6.0;  // center of the goal tags off the floor

    // Turret aiming (simple P controller on tx). Tune kP first, then the rest.
    static final double TURRET_KP = 0.02;            // power per degree of error
    static final double TURRET_MIN_POWER = 0.05;     // overcomes friction on small errors
    static final double TURRET_MAX_POWER = 0.5;
    static final double AIM_TOLERANCE_DEG = 1.0;     // "aimed" when |error| is below this
    static final double TX_OFFSET_DEG = 0.0;         // if the shooter isn't centered on the camera

    // Ignore results older than this (camera unplugged, lag, etc.)
    static final long MAX_STALENESS_MS = 100;

    // TODO: Fill in from testing: shoot from several distances, record the
    // shooter velocity that scores. Must be sorted by distance (smallest first).
    static final double[] DISTANCE_TABLE_IN = {24, 48, 72, 96, 120};
    static final double[] SHOOTER_VEL_TABLE = {1000, 1200, 1400, 1600, 1800}; // ticks/sec

    // ---------------- Variables ----------------

    Limelight3A limelight;
    DcMotor turretMotor;
    DcMotorEx shooterMotor;
    Servo feeder;

    boolean isRed = true;
    int side = 1;              // which side of the goal is facing up (1 or 2)
    int activePipeline = -1;   // pipeline we last switched to, -1 = none

    // Updated every loop by readLimelight()
    boolean hasTarget;
    int tagsSeen;   // how many of the goal side's tags are visible (0, 1 or 2)
    double tx;      // degrees to goal center, + means right of crosshair
    double ty;      // degrees, + means above crosshair

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start(); // without this, getLatestResult() returns null

        if (USE_TURRET) {
            turretMotor = hardwareMap.get(DcMotor.class, TURRET_MOTOR_NAME);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (USE_SHOOTER) {
            shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the flywheel coast
            feeder = hardwareMap.get(Servo.class, FEEDER_SERVO_NAME);
        }

        telemetry.setMsTransmissionInterval(11);

        // Pick alliance and side during init
        while (opModeInInit()) {
            if (gamepad1.x) isRed = false;
            if (gamepad1.b) isRed = true;
            readSideButtons();
            updatePipeline();

            showAllianceAndSide();
            telemetry.addLine("X = blue, B = red. Bumpers = side. Press Play when ready.");
            telemetry.update();
        }

        if (USE_SHOOTER) {
            feeder.setPosition(FEEDER_REST_POS);
        }

        while (opModeIsActive()) {
            readSideButtons();
            updatePipeline();
            readLimelight();

            double aimError = tx - TX_OFFSET_DEG;
            boolean aimed = hasTarget && Math.abs(aimError) < AIM_TOLERANCE_DEG;
            double distance = hasTarget ? distanceFromTy(ty) : -1;
            double shooterVel = hasTarget ? interpolate(distance, DISTANCE_TABLE_IN, SHOOTER_VEL_TABLE) : 0;

            // Turret: P controller on aim error. If it turns the WRONG way, flip the sign
            // of TURRET_KP or reverse the motor.
            double turretPower = 0;
            if (hasTarget && !aimed) {
                turretPower = aimError * TURRET_KP;
                turretPower += Math.signum(turretPower) * TURRET_MIN_POWER;
                turretPower = Range.clip(turretPower, -TURRET_MAX_POWER, TURRET_MAX_POWER);
            }
            if (USE_TURRET) {
                turretMotor.setPower(gamepad1.a ? turretPower : 0);
            }

            // Shooter: holding A spins the flywheel up. Right trigger only feeds a game piece
            // when the turret is aimed and the flywheel is up to speed.
            boolean spinUp = gamepad1.a && hasTarget;
            double targetVel = spinUp ? shooterVel : 0;
            boolean atSpeed = false;
            boolean shooting = false;
            if (USE_SHOOTER) {
                shooterMotor.setVelocity(targetVel);
                atSpeed = spinUp && Math.abs(shooterMotor.getVelocity() - targetVel) < SHOOTER_VEL_TOLERANCE;
                shooting = aimed && atSpeed && gamepad1.right_trigger > 0.5;
                feeder.setPosition(shooting ? FEEDER_PUSH_POS : FEEDER_REST_POS);
            }

            showAllianceAndSide();
            telemetry.addData("Goal tags seen", tagsSeen);
            if (tagsSeen == 1) {
                telemetry.addLine("Only 1 tag visible - aim may be a little off");
            }
            if (hasTarget) {
                telemetry.addData("tx / ty", "%.1f / %.1f", tx, ty);
                telemetry.addData("Distance (in)", "%.1f", distance);
                telemetry.addData("Aimed", aimed);
                telemetry.addData("Turret power", "%.2f", turretPower);
                telemetry.addData("Shooter vel (target)", "%.0f", shooterVel);
            }
            if (USE_SHOOTER) {
                telemetry.addData("Shooter vel (actual)", "%.0f", shooterMotor.getVelocity());
                telemetry.addData("Ready to shoot", aimed && atSpeed);
                telemetry.addData("Shooting", shooting);
            }
            telemetry.update();
        }

        if (USE_SHOOTER) {
            shooterMotor.setVelocity(0);
        }
        limelight.stop();
    }

    /** Left bumper = side 1, right bumper = side 2. */
    void readSideButtons() {
        if (gamepad1.left_bumper) side = 1;
        if (gamepad1.right_bumper) side = 2;
    }

    /** The pipeline for the current alliance and side, or -1 if it isn't set up yet. */
    int wantedPipeline() {
        if (isRed) return side == 1 ? RED_SIDE1_PIPELINE : RED_SIDE2_PIPELINE;
        return side == 1 ? BLUE_SIDE1_PIPELINE : BLUE_SIDE2_PIPELINE;
    }

    /** Switches the Limelight pipeline, but only when the alliance or side changed. */
    void updatePipeline() {
        int wanted = wantedPipeline();
        if (wanted != activePipeline) {
            if (wanted >= 0) limelight.pipelineSwitch(wanted);
            activePipeline = wanted;
        }
    }

    void showAllianceAndSide() {
        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.addData("Side", side);
        if (activePipeline < 0) {
            telemetry.addLine("WARNING: no pipeline set up for this alliance/side yet");
        } else {
            telemetry.addData("Pipeline", activePipeline);
        }
    }

    /** Reads the newest Limelight result and sets hasTarget, tagsSeen, tx and ty. */
    void readLimelight() {
        hasTarget = false;
        tagsSeen = 0;

        if (activePipeline < 0) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
            return;
        }

        // Right after a switch, the Limelight may still be sending results from the old side.
        if (result.getPipelineIndex() != activePipeline) return;

        // The pipeline's ID filter means these are only this goal side's tags,
        // so with both visible the average is the midpoint between them.
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags.isEmpty()) return;

        double sumX = 0, sumY = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            sumX += tag.getTargetXDegrees();
            sumY += tag.getTargetYDegrees();
        }
        hasTarget = true;
        tagsSeen = tags.size();
        tx = sumX / tags.size();
        ty = sumY / tags.size();
    }

    /** Floor distance from camera to the goal tags in inches, from the vertical angle. */
    double distanceFromTy(double tyDeg) {
        double angleRad = Math.toRadians(CAMERA_PITCH_DEG + tyDeg);
        return (TAG_CENTER_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleRad);
    }

    /** Looks up y for x in a table, blending between the two closest entries. */
    double interpolate(double x, double[] xs, double[] ys) {
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        for (int i = 0; i < xs.length - 1; i++) {
            if (x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[ys.length - 1];
    }
}
