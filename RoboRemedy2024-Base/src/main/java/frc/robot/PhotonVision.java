// PhotonVision.java - Photonvision vision control
package frc.robot;

import org.photonvision.PhotonCamera;

public class PhotonVision {

    public static class TargetInfo {
        public boolean isFound;
        public int id;
        public double inchesInFrontOfTarget;
        public double inchesRightOfTarget;
        public double yaw;
    }

    private static LinearMapper mapper = new LinearMapper();

    // http://photonvision.local:5800/#/dashboard for right camera
    static PhotonCamera aprilTagRightCamera = new PhotonCamera("2023_Right_Camera");

    // http://photonvision2.local:5800/#/dashboard for left camera
    static PhotonCamera aprilTagLeftCamera = new PhotonCamera("2023_Left_Camera");
    static PhotonCamera cubeCamera = new PhotonCamera("3.0_USB_Camera");

    public static void PhotonVisionInit() {

        // Map Apriltag area values to Target Distance values in inches, using linear
        // mapper
        mapper.add(0.23, 120);
        mapper.add(0.25, 115.5);
        mapper.add(0.28, 109);
        mapper.add(0.29, 103);
        mapper.add(0.35, 97);
        mapper.add(0.42, 91);
        mapper.add(0.43, 85);
        mapper.add(0.52, 79);
        mapper.add(0.61, 73);
        mapper.add(0.72, 67);
        mapper.add(0.80, 61);
        mapper.add(1.08, 55);
        mapper.add(1.33, 49);
        mapper.add(1.68, 43);
        mapper.add(2.39, 37);
        mapper.add(3.17, 32);
        mapper.add(5.1, 24);
    }

    public static boolean isCubeFound() {
        if (cubeCamera == null)
            return false;
        if (!cubeCamera.isConnected())
            return false;

        var result = cubeCamera.getLatestResult();

        boolean isCubeFound = false;
        isCubeFound = result.hasTargets();
        return isCubeFound;
    }

    private static boolean isCameraConnected(PhotonCamera camera) {
        if (camera == null)
            return false;
        if (!camera.isConnected())
            return false;
        return true;
    }

    public static void setCubeCameraDriverMode(boolean driverMode) {
        if (cubeCamera != null && cubeCamera.isConnected()) {
            cubeCamera.setDriverMode(driverMode);
            System.out.println("Driver Mode:" + driverMode);
        }
    }

    public static boolean isAprilTagRightCameraAttached() {
        return isCameraConnected(aprilTagRightCamera);
    }

    public static boolean isAprilTagLeftCameraAttached() {
        return isCameraConnected(aprilTagLeftCamera);
    }

    public static boolean isCubeCameraAttached() {
        return isCameraConnected(cubeCamera);

    }

    public static double getCubeYaw() {
        if (cubeCamera == null)
            return -1;
        if (!cubeCamera.isConnected())
            return -1;

        var result = cubeCamera.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                return targetResult.getYaw();
            } else {
                return -2;
            }
        } else {
            return -3;
        }
    }

    public static double getCubePitch() {
        if (cubeCamera == null)
            return -1;
        if (!cubeCamera.isConnected())
            return -1;

        var result = cubeCamera.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                return targetResult.getPitch();
            } else {
                return -2;
            }
        } else {
            return -3;
        }
    }
    public static double getCubeArea() {
        if (cubeCamera == null)
            return -1;
        if (!cubeCamera.isConnected())
            return -1;

        var result = cubeCamera.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                return targetResult.getArea();
            } else {
                return -2;
            }
        } else {
            return -3;
        }
    }

    public static void takeCubeSnapshot() {
        if (cubeCamera != null) {
            cubeCamera.takeInputSnapshot();
            cubeCamera.takeOutputSnapshot();
        }
    }

    public static TargetInfo getBestAprilTagForCamera(double rightOffset, double frontOffset,
        double yawOffset) {

        PhotonCamera camera = aprilTagRightCamera;

        TargetInfo info = new TargetInfo();
        info.isFound = false;
        info.id = 0;
        info.inchesInFrontOfTarget = Double.MAX_VALUE;
        info.inchesRightOfTarget = Double.MAX_VALUE;
        info.yaw = Double.MAX_VALUE;

        if (camera != null && camera.isConnected()) {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var targetResult = result.getBestTarget();
                info.id = targetResult.getFiducialId();
                double area = targetResult.getArea();
                double calcDistance = mapper.calculate(area);
                if (calcDistance - frontOffset < 100) {
                    info.yaw = targetResult.getYaw() - yawOffset;
                    info.inchesRightOfTarget = -(calcDistance * Math.tan(Math.toRadians(info.yaw)))
                            - rightOffset;
                    info.inchesInFrontOfTarget = mapper.calculate(area) - frontOffset;
                    info.isFound = true;
                }
            }
        }
        return info;
    }

}