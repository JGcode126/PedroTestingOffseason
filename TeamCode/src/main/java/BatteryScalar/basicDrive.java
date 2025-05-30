package BatteryScalar;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class basicDrive extends LinearOpMode {
    public static double timedTime = 2;
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;

    ElapsedTime timer;


    public void runOpMode(){
        timer = new ElapsedTime();

        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        timer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            if(timer.seconds() < timedTime){
                drive(0.3,0,0);
            } else {
                drive(0,0,0);
            }
            telemetry.addData("time:", timer.seconds());
            telemetry.addData("timedTime:", timedTime);

            telemetry.update();
        }
    }

    public void drive(double drive, double strafe, double turn){
        leftFrontMotor.setPower((drive+strafe+turn)*-1);
        rightFrontMotor.setPower((drive-strafe-turn)*-1);
        leftBackMotor.setPower((drive-strafe+turn)*-1);
        rightBackMotor.setPower((drive+strafe-turn)*-1);
    }
}
