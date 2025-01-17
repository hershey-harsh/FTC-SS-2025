package org.firstinspires.ftc.teamcode.parts.intake2;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake2Tasks {
    protected final Group movementTask;
    private final TimedTask autoHomeTask;
    private Intake2 intake;
    private Robot robot;
    private final TimedTask autoBucketLiftTask;
    private final TimedTask autoBucketDropperTask;
    private final TimedTask autoSpecimenPickupTask;
    private final TimedTask autoSpecimenHangTask;
    private final TimedTask autoIntakeDropTask;

    public Intake2Tasks(Intake2 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        autoBucketLiftTask = new TimedTask(TaskNames.autoBucketLift, movementTask);
        autoBucketDropperTask = new TimedTask(TaskNames.autoBucketDropper, movementTask);
        autoSpecimenPickupTask = new TimedTask(TaskNames.autoSpecimenPickup, movementTask);
        autoSpecimenHangTask = new TimedTask(TaskNames.autoSpecimenHang, movementTask);
        autoIntakeDropTask = new TimedTask(TaskNames.autoIntakeDrop);
    }

    public void constructAutoHome() {
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(this::setSlideToHomeConfig);
        autoHomeTask.addStep(() -> {
            intake.incrementIntakeUpDown(0);
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
            intake.getHardware().dropperServo.getController().pwmDisable();
        });
        autoHomeTask.addTimedStep(() -> {
            robot.opMode.telemetry.addData("homing", intake.getHardware().bucketLiftZeroSwitch.getState());
        }, () -> intake.getHardware().bucketLiftZeroSwitch.getState(), 10000);
        autoHomeTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.getHardware().bucketLiftMotor.setTargetPosition(20);
            intake.slideTargetPosition = 20;
            setMotorsToRunConfig();
        });
    }

    public void constructAutoBucketLift() {
        autoBucketLiftTask.autoStart = false;

        autoBucketLiftTask.addStep(() -> {
            intake.getHardware().tiltServo.setPosition(0.3);
            intake.getHardware().dropperServo.getController().pwmDisable();
        }, () -> intake.getHardware().tiltServo.isDone());

        autoBucketLiftTask.addStep( ()-> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().maxLiftPosition);
            intake.slideTargetPosition = intake.getSettings().maxLiftPosition;
        }, () -> intake.isLiftInTolerance());

        autoBucketLiftTask.addStep( ()->{
            intake.getHardware().dropperServo.getController().pwmEnable();
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin);
        });
    }

    public void constructAutoBucketDropper() {
        autoBucketDropperTask.autoStart = false;
        autoBucketDropperTask.addStep(() -> {
            intake.getHardware().dropperServo.getController().pwmEnable();
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMax);
        },()->intake.getHardware().dropperServo.isDone());

        autoBucketDropperTask.addStep( ()-> {
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin);
        }, () -> intake.getHardware().dropperServo.isDone());

        autoBucketDropperTask.addStep(() -> {
            intake.getHardware().dropperServo.getController().pwmDisable();
        });
        autoBucketDropperTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setPower(1);
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().minLiftPosition);
        });
    }

    public void constructAutoSpecimenPickup() {
        autoSpecimenPickupTask.autoStart = false;
        autoSpecimenPickupTask.addStep(() -> {
            intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmDefault); // default straight up position
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition);
        }, ()-> intake.getHardware().specimenServo.isDone());
        autoSpecimenPickupTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setPower(1);
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().specimenSafeHeight);
            intake.slideTargetPosition = intake.getSettings().specimenSafeHeight;
        }, () -> intake.isLiftInTolerance());
    }
    public void constructAutoSpecimenHang() {
        autoSpecimenHangTask.autoStart = false;

        autoSpecimenHangTask.addStep( ()-> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().specimenServoOpenHeight);
            intake.getHardware().bucketLiftMotor.setPower(1);
            intake.slideTargetPosition = intake.getSettings().specimenServoOpenHeight;
        }, () -> intake.isLiftInTolerance());

        autoSpecimenHangTask.addStep(() -> {
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
        }, ()->intake.getHardware().specimenServo.isDone());

        autoSpecimenHangTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setPower(1);
            intake.getHardware().bucketLiftMotor.setTargetPosition(0);
            intake.slideTargetPosition = 0;
        }, () -> intake.isLiftInTolerance());
    }

    public void constructIntakeDrop() {
        autoIntakeDropTask.autoStart = false;
        autoIntakeDropTask.addStep(() -> {
            intake.getHardware().tiltServo.setPosition(0.64);
        });
        autoIntakeDropTask.addDelay(1000);
        autoIntakeDropTask.addStep(() -> {
            intake.getHardware().intakeWheelServoRight.setPower(1);
        });
        autoIntakeDropTask.addDelay(1000);
        autoIntakeDropTask.addStep(() -> {
            intake.getHardware().tiltServo.setPosition(0);
        });
    }

    public void startAutoHome() {
        autoHomeTask.restart();
    }
    public void startAutoBucketLift() {
        autoBucketLiftTask.restart();
    }
    public void startAutoBucketDropper() {
        autoBucketDropperTask.restart();
    }
    public void startAutoSpecimenPickup() {
        autoSpecimenPickupTask.restart();
    }
    public void startAutoSpecimenHang() {
        autoSpecimenHangTask.restart();
    }
    public void startAutoIntakeDropTask() {
        autoIntakeDropTask.restart();
    }

    private void setSlideToHomeConfig() {
        double power = -0.125;
        intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.getHardware().bucketLiftMotor.setPower(power);
    }

    public void setMotorsToRunConfig() {
        intake.getHardware().bucketLiftMotor.setPower(IntakeHardware.slideHoldPower);
        intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autoHome = "auto home";
        public final static String autoBucketLift = "auto bucket lift";
        public final static String autoBucketDropper = "auto bucket drop";
        public final static String autoSpecimenPickup = "auto specimen pickup";
        public final static String autoSpecimenHang = "auto specimen hang";
        public final static String autoIntakeDrop = "drop block into bucket";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
