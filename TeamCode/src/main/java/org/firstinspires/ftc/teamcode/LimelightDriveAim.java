package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Limelight practice on last year's (non-turret) mecanum robot.
 * Same AprilTag logic as Limelightcode, but turns the WHOLE ROBOT to face
 * the midpoint between the two tags on the goal side facing up.
 *
 * Controls:
 *   During INIT:  X = blue alliance, B = red alliance
 *   INIT or PLAY: left bumper = side 1, right bumper = side 2 (pick the side facing up)
 *   During PLAY:  left stick = drive/strafe, right stick X = turn
 *                 hold A = robot auto-turns to face the goal (you can still drive/strafe)
 *
 * Things marked TODO need real numbers from your robot / the game manual.
 */
@TeleOp(name = "Limelight Drive Aim", group = "Test")
public class LimelightDriveAim extends LinearOpMode {

    // ---------------- Configuration ----------------

    // TODO: Match these to last year's robot configuration
    static final String FRONT_LEFT_NAME = "frontLeft";
    static final String FRONT_RIGHT_NAME = "frontRight";
    static final String BACK_LEFT_NAME = "backLeft";
    static final String BACK_RIGHT_NAME = "backRight";

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

    // Robot turning (simple P controller on tx). A whole robot is heavier than a turret,
    // so this usually needs different numbers than the turret version.
    static final double TURN_KP = 0.015;             // turn power per degree of error
    static final double TURN_MIN_POWER = 0.08;       // overcomes friction on small errors
    static final double TURN_MAX_POWER = 0.4;
    static final double AIM_TOLERANCE_DEG = 1.5;     // "aimed" when |error| is below this
    static final double TX_OFFSET_DEG = 0.0;         // if the camera isn't centered on the robot

    // Ignore results older than this (camera unplugged, lag, etc.)
    static final long MAX_STALENESS_MS = 100;

    // ---------------- Variables ----------------

    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight;

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

        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // TODO: If the robot drives backwards or spins when you push forward,
        // change which side is reversed (copy whatever last year's code did).
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        while (opModeIsActive()) {
            readSideButtons();
            updatePipeline();
            readLimelight();

            double aimError = tx - TX_OFFSET_DEG;
            boolean aimed = hasTarget && Math.abs(aimError) < AIM_TOLERANCE_DEG;
            double distance = hasTarget ? distanceFromTy(ty) : -1;

            // Turn power to face the goal. Positive = turn right (clockwise).
            // If the robot turns AWAY from the goal, flip the sign of TURN_KP.
            double aimTurn = 0;
            if (hasTarget && !aimed) {
                aimTurn = aimError * TURN_KP;
                aimTurn += Math.signum(aimTurn) * TURN_MIN_POWER;
                aimTurn = Range.clip(aimTurn, -TURN_MAX_POWER, TURN_MAX_POWER);
            }

            // Driver input
            double drive = -gamepad1.left_stick_y;  // forward is negative on the stick
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Holding A: the Limelight controls turning, driver still controls drive/strafe
            if (gamepad1.a) {
                turn = aimTurn;
            }

            mecanumDrive(drive, strafe, turn);

            showAllianceAndSide();
            telemetry.addData("Auto-aim", gamepad1.a ? "ON (holding A)" : "off");
            telemetry.addData("Goal tags seen", tagsSeen);
            if (tagsSeen == 1) {
                telemetry.addLine("Only 1 tag visible - aim may be a little off");
            }
            if (hasTarget) {
                telemetry.addData("tx / ty", "%.1f / %.1f", tx, ty);
                telemetry.addData("Distance (in)", "%.1f", distance);
                telemetry.addData("Aimed", aimed);
                telemetry.addData("Aim turn power", "%.2f", aimTurn);
            }
            telemetry.update();
        }

        mecanumDrive(0, 0, 0);
        limelight.stop();
    }

    /** Standard mecanum drive. turn > 0 turns right (clockwise). */
    void mecanumDrive(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double bl = drive - strafe + turn;
        double fr = drive - strafe - turn;
        double br = drive + strafe - turn;

        // Scale down so no wheel goes over full power
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                                            Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
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
}
