package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PIDController;

@Autonomous(name = "BulldogAutoPID")
public class BulldogAuto extends LinearOpMode {

    private DcMotorEx FR, BR, FL, BL;
    private PIDController leftPID, rightPID;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private double inchesToCounts(double inches) {
        return inches * DRIVE_COUNTS_PER_IN;
    }

    private void drive(double power, double leftInches, double rightInches) {
        int leftTarget = (int) inchesToCounts(leftInches);
        int rightTarget = (int) inchesToCounts(rightInches);

        // Reset encoders
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Drive using PID
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() &&
                (Math.abs(FL.getCurrentPosition()) < Math.abs(leftTarget) ||
                Math.abs(FR.getCurrentPosition()) < Math.abs(rightTarget))) {

            double leftPower = leftPID.PIDControl(leftTarget, FL.getCurrentPosition());
            double rightPower = rightPID.PIDControl(rightTarget, FR.getCurrentPosition());

            FL.setPower(leftPower * power);
            BL.setPower(leftPower * power);
            FR.setPower(rightPower * power);
            BR.setPower(rightPower * power);

            // Debugging telemetry
            telemetry.addData("Left Target", leftTarget);
            telemetry.addData("Right Target", rightTarget);
            telemetry.addData("Left Position", FL.getCurrentPosition());
            telemetry.addData("Right Position", FR.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drivetrain motors
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        // Set directions
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize PID controllers
        leftPID = new PIDController(0.5, 0.01, 0.1, 0.0);  // Tune values as needed
        rightPID = new PIDController(0.5, 0.01, 0.1, 0.0); // Tune values as needed

        waitForStart();

        if (opModeIsActive()) {
            /*Examples:
            1. Drive forward 24 inches
            2. Turn left in place
            3. Backward movement*/
            drive(0.5, 24, 24);
            drive(0.5, -12, 12);
            drive(0.5, -24, -24);
        }
    }
}
