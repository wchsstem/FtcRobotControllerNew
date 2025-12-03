package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name = "BulldogTechTeleOp")
public class BulldogOpMode extends LinearOpMode {

    private DcMotorEx FL, FR, BL, BR, launch;
    private PIDController launcherPID;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        // Motor direction configuration
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero power behavior
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Modes
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID controller
        launcherPID = new PIDController(0.5, 0.0, 0.0, 0.0);

        waitForStart();

        while (opModeIsActive()) {
            // PID-controlled launcher power
            double targetRPM = 3000; // launcher RPM
            double currentRPM = launch.getVelocity();
            double launcherPower = launcherPID.PIDControl(targetRPM, currentRPM);
            launch.setPower(launcherPower);

            // Telemetry alterations
            telemetry.addData("Launcher Target", targetRPM);
            telemetry.addData("Launcher Speed", currentRPM);
            telemetry.addData("Launcher Power", launcherPower);

            telemetry.update();
        }
    }
}
