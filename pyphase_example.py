#!/usr/bin/env python3

import phase.pyphase as phase
import cv2
import os
from pathlib import Path

def main():
    device_infos = phase.stereocamera.availableDevices()
    if len(device_infos) <= 0:
        print("No devices found")
        return

    for device_info in device_infos:
        print("*****************************")
        print("Camera Name: " + device_info.getUniqueSerial())
        print("Left Serial: " + device_info.getLeftCameraSerial())
        print("Right Serial: " + device_info.getRightCameraSerial())

    # Choose first camera
    first_device = device_infos[0]
    print("Selecting first camera: " + first_device.getUniqueSerial())
    camera_name = first_device.getUniqueSerial()
    left_serial = first_device.getLeftCameraSerial()
    right_serial = first_device.getRightCameraSerial()

    device_type = phase.stereocamera.CameraDeviceType.DEVICE_TYPE_TITANIA
    interface_type = phase.stereocamera.CameraInterfaceType.INTERFACE_TYPE_USB

    script_path = os.path.dirname(os.path.realpath(__file__))
    print(script_path)

    script_dir = Path(__file__).parent.absolute()
    calibrations_dir = script_dir / "calibrations"
    chosen_cal_dir = calibrations_dir / "mycal18102023"

    l_yaml = chosen_cal_dir / "left.yaml"
    r_yaml = chosen_cal_dir / "right.yaml"
    if not l_yaml.exists() or not r_yaml.exists():
        msg = ""
        if not l_yaml.exists():
            msg += "Left calibration file not found. "
        if not r_yaml.exists():
            msg += "Right calibration file not found. "
        print(msg)
        return
    left_yaml = str(l_yaml)
    right_yaml = str(r_yaml)

    data_dir = script_dir / "pointclouds"
    out_ply = str(data_dir / "titania_out.ply")

    # Define parameters for read process
    downsample_factor = 1.0
    display_downsample = 0.25
    exposure_value = 10000

    # Check for I3DRSGM license
    license_valid = phase.stereomatcher.StereoI3DRSGM().isLicenseValid()
    if license_valid:
        print("I3DRSGM license accepted")
        stereo_params = phase.stereomatcher.StereoParams(
            phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_I3DRSGM,
            9, 0, 49, False
        )
    else:
        print("Missing or invalid I3DRSGM license. Will use StereoBM")
        stereo_params = phase.stereomatcher.StereoParams(
            phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_BM,
            11, 0, 25, False
        )

    # Load calibration
    calibration = phase.calib.StereoCameraCalibration.calibrationFromYAML(
        left_yaml, right_yaml)

    # Create stereo matcher
    matcher = phase.stereomatcher.createStereoMatcher(stereo_params)

    # Create stereo camera device information from parameters
    device_info = phase.stereocamera.CameraDeviceInfo(
        left_serial, right_serial, camera_name,
        device_type,
        interface_type)
    # Create stereo camera
    tinaniaCam = phase.stereocamera.TitaniaStereoCamera(device_info)

    # Connect camera and start data capture
    print("Connecting to camera...")
    ret = tinaniaCam.connect()
    if (ret):
        tinaniaCam.startCapture()
        # Set camera exposure value
        tinaniaCam.setExposure(exposure_value)
        print("Running camera capture...")
        print("Press 'q' to quit or 'p' to save pointcloud.")
        while tinaniaCam.isConnected():
            read_result = tinaniaCam.read()
            #print(f"Read result left:\n{read_result.left}")
            #print(f"Read result right:\n{read_result.right}")
            if read_result.valid:
                # Rectify stereo image pair
                rect_image_pair = calibration.rectify(read_result.left, read_result.right)
                rect_img_left = rect_image_pair.left
                rect_img_right = rect_image_pair.right

                match_result = matcher.compute(rect_img_left, rect_img_right)

                # Check compute is valid
                if not match_result.valid:
                    print("Failed to compute match")
                    continue

                # Find the disparity from matcher
                disparity = match_result.disparity

                # Convert disparity into 3D pointcloud
                xyz = phase.disparity2xyz(
                    disparity, calibration.getQ())

                # Display stereo and disparity images
                img_left = phase.scaleImage(
                        rect_img_left, display_downsample)
                img_right = phase.scaleImage(
                        rect_img_right, display_downsample)
                img_disp = phase.scaleImage(
                        phase.normaliseDisparity(
                            disparity), display_downsample)
                cv2.imshow("Left", img_left)
                cv2.imshow("Right", img_right)
                cv2.imshow("Disparity", img_disp)
                c = cv2.waitKey(1)

                # Save the pointcloud of current frame if 'p' is pressed
                if c == ord('p'):
                    save_success = phase.savePLY(out_ply, xyz, rect_img_left)
                    if save_success:
                        print("Pointcloud saved to " + out_ply)
                    else:
                        print("Failed to save pointcloud")
                
                # Quit data capture if 'q' is pressed
                if c == ord('q'):
                    break
            else:
                tinaniaCam.disconnect()
                raise Exception("Failed to read stereo result")
                
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()