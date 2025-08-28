import pyzed.sl as sl
import math
import time
import can
import subprocess

password = "admin"

cmd = ["sudo", "-S", "ip", "link", "set", "can1", "up", "type", "can", "bitrate", "250000"]
cmdDown = ["sudo", "-S", "ip", "link", "set", "can1", "down"]
cmdReboot = ["sudo", "reboot"]

def get_connected_cameras():
    cam_list = sl.Camera().get_device_list()
    return [cam.serial_number for cam in cam_list]

def setupcams(init_params : sl.InitParameters):
    serials = get_connected_cameras()
    cameras = []
    for serial in serials:
        cam = sl.Camera()
         # =
        params = init_params
        params.set_from_serial_number(serial)
        # init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT
        # init_params.camera_resolution = sl.RESOLUTION.HD1080
        # init_params.camera_fps = 15
        # init_params.set_from_serial_number(serial)
        #init_params.depth_mode.NEURAL_LIGHT

        status = cam.open(params)



        if status != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open camera {serial}: {status}")
            continue

        detection_parameters = sl.ObjectDetectionParameters()
        print("Object Detection: Loading Module...")

        cam.enable_positional_tracking()

        err = cam.enable_object_detection(detection_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Error {}, exit program".format(err))
            cam.close()
            exit()
            pass


        print(f"Camera {serial} opened successfully.")
        cameras.append(cam)
    return cameras

class ISOBJ:
    def __init__(self, zed : sl.Camera, confidence: float, type : sl.OBJECT_CLASS):
        self.zed = zed
        self.confidence = confidence
        self.zed.enable_positional_tracking()
        self.type = type

        detection_parameters = sl.ObjectDetectionParameters()
        # print("Object Detection: Loading Module...")

        # err = self.zed.enable_object_detection(detection_parameters)
        # if err != sl.ERROR_CODE.SUCCESS:
        #     #print("Error {}, exit program".format(err))
        #     #self.zed.close()
        #     #exit()
        #     pass

        self.detection_parameters_runtime = sl.ObjectDetectionRuntimeParameters()
        self.detection_parameters_runtime.detection_confidence_threshold = self.confidence

        self.objects = sl.Objects()

    def query(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            err = self.zed.retrieve_objects(self.objects, self.detection_parameters_runtime)

            if len(self.objects.object_list):
                bools = False
                for obj in self.objects.object_list:
                    if obj.label == self.type:
                        bools = True
                return bools
            return False
        return False



def main():
    print("Starting...")
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    init_params.camera_resolution = sl.RESOLUTION.HD1200
    init_params.coordinate_units = sl.UNIT.INCH
    init_params.camera_fps = 60
    cameras = setupcams(init_params)
    proc = subprocess.run(cmd, input=password + "\n", text=True, check=True)
    time.sleep(10)
    print("Can is up")

    try:
        isperson = ISOBJ(cameras[0], 8, sl.OBJECT_CLASS.PERSON)
    except:
        pass
        # proc = subprocess.run(cmdReboot, input=password + "\n", text=True, check=True)

    CAN0 = can.Bus(interface='socketcan', channel='can0', bitrate=250000)
    CAN1 = can.Bus(interface='socketcan', channel='can1', bitrate=250000)

    try:
        msg = can.Message(arbitration_id=0x18ff1023, data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88], is_extended_id=True)
        can0bool = 0
        can1bool = 0
        while True:

            if(isperson.query()):
                msg = can.Message(arbitration_id=0x18ff1023, data=[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff], is_extended_id=True)
                print("True")
            else:
                msg = can.Message(arbitration_id=0x18ff1023, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=True)
                print("False")
            time.sleep(0.5)
            if can0bool <3:

                try:
                    CAN0.send(msg)
                    print("Out0")
                    can0bool = 0
                except can.CanError:
                    can0bool = can0bool + 1
                    print("CAN0 error")
            if can1bool <3:

                try:
                    CAN1.send(msg)
                    can1bool = 0
                    print("Out1")
                except can.CanError:
                    can1bool = can1bool + 1
                    print("CAN1 error")
            if can1bool is False and can0bool is False:
                pass
                # proc = subprocess.run(cmdReboot, input=password + "\n", text=True, check=True)


    finally:
        CAN0.shutdown()
        CAN1.shutdown()
        for cam in cameras:
            if cam is not None and hasattr(cam, 'close'):
                cam.close()
        proc = subprocess.run(cmdDown, input=password + "\n", text=True, check=True)

        print("closed")

    pass# Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
