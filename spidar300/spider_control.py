from servo import Servo
from scservo_sdk import sms_sts, PortHandler
import serial.tools.list_ports

class Spider:
    ADDR_ID = 5
    ADDR_MIN_ANGLE_LIMIT = 9
    ADDR_MAX_ANGLE_LIMIT = 11
    DDR_MAX_TEMP_LIMIT = 13
    ADDR_MAX_VOLTAGE_LIMIT = 14
    ADDR_MIN_VOLTAGE_LIMIT = 15
    ADDR_LOCK = 55
    ADDR_MODE = 33
    ADDR_TORQUE_ENABLE = 40
    ADDR_ACCELERATION = 41
    ADDR_GOAL_POSITION = 42
    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_LOAD = 60
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63
    ADDR_SERVO_STATUS = 65
    ADDR_MOVING = 66
    ADDR_PRESENT_CURRENT = 69

    POSITION_MIN = 0
    POSITION_MAX = 4095

    def __init__(self, port = "default"):
        if port == "default":
            ports = serial.tools.list_ports.comports()
            self._port = ports[0].device
            print(f"Using port {self._port}")
        else:
            self._port = port

        self._port_handler = PortHandler(self._port)
        self._servo = sms_sts(self._port_handler)

        if self._port_handler.openPort():
            print("✓ Port opened successfully")
        else:
            print("✗ Failed to open port")

        self._id_list = [
                        111, 112, 113, 
                        211, 212, 213, 
                        121, 122, 123, 
                        221, 222, 223, 
                        131, 132, 133, 
                        231, 232, 233
                        ]
        self._default_positions = default_positions = {
                        211: 1500, 
                        221: 2048, 
                        231: 2500, 
                        111: 2500, 
                        121: 2048, 
                        131: 1500,
                        212: 2048, 
                        222: 2048, 
                        232: 2048, 
                        112: 2048, 
                        122: 2048, 
                        132: 2048,
                        213: 700,
                        223: 700,
                        233: 700,
                        113: 700,
                        123: 700,
                        133: 700,
                    }

    def ping(self, id = None):
        if id is None:
            found_servos = []    
            for _id in self._id_list:
                model_number, comm_result, error = self._servo.ping(_id)
                if comm_result == 0:  # COMM_SUCCESS
                    print(f"✓ Servo ID {_id} found!")
                    found_servos.append(_id)
            print(f"{len(found_servos)} servos available")
        else:
            model_number, comm_result, error = self._servo.ping(id)
            if comm_result == 0:  # COMM_SUCCESS
                print(f"✓ Servo ID {id} found!")

    def read1Byte(self, id, address):
            """
            Read a 1-byte value from a servo register.
    
            Args:
                address: Register address to read from.
    
            Returns:
                int: Value read from the register (0-255).
    
            Raises:
                IOError: If communication fails or servo returns an error.
            """
            read_value, comm_result, error = self._servo.read1ByteTxRx(id, address)
            if comm_result != 0:
                raise IOError(f"✗ Communication failed for servo_id: {id}, comm_result: {comm_result}")
    
            if error != 0:
                raise IOError(f"Servo {id} returned error: {error}")
    
            return read_value
    
    def read2Byte(self, id, address):
        """
        Read a 2-byte value from a servo register.

        Args:
            address: Register address to read from.

        Returns:
            int: Value read from the register (0-65535).

        Raises:
            IOError: If communication fails or servo returns an error.
        """
        read_value, comm_result, error = self._servo.read2ByteTxRx(id, address)
        if comm_result != 0:
            raise IOError(f"✗ Communication failed for servo_id: {id}, comm_result: {comm_result}")

        if error != 0:
            raise IOError(f"Servo {id} returned error: {error}")

        return read_value

    def enable_torque(self, id = None):
        ADDR_TORQUE_ENABLE = 40
        if id is None:
            for _id in self._id_list:
                self._servo.write1ByteTxRx(_id, ADDR_TORQUE_ENABLE, 1)
        else:
            self._servo.write1ByteTxRx(id, ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self, id = None):
        ADDR_TORQUE_ENABLE = 40
        if id is None:
            _id_list = self._id_list
            
        else:
            _id_list = [id]

        for _id in _id_list:
            self._servo.write1ByteTxRx(_id, ADDR_TORQUE_ENABLE, 0)

    def move_to(self, id, target, speed, acc, blocking = False, tolerance = 10):
        self._servo.WritePosEx(id, target, speed, acc)

        if blocking:
            while True:
                position = self.read2Byte(id, self.ADDR_PRESENT_POSITION)
                moving = self.read1Byte(id, self.ADDR_MOVING)
    
                if not moving and abs(position - target) <= tolerance:
                    break

    def move_to_default_position(self, id = None, speed = 100, acc = 0, blocking = False):
        if id is None:
            _id_list = self._id_list
            
        else:
            _id_list = [id]

        for _id in _id_list:
            move_to = self._default_positions[_id]
            self._servo.WritePosEx(_id, move_to, speed, acc = acc)

        if blocking:
            while True:
                for _id in _id_list:
                    position = self.read2Byte(_id, self.ADDR_PRESENT_POSITION)
                    moving = self.read1Byte(_id, self.ADDR_MOVING)

                    if moving or abs(position - self._default_positions[_id]) > 10:
                        break
                else: 
                    break

    def position(self, id):
        """
        Read the current servo position.

        Returns:
            tuple: (raw_position, degrees) where raw_position is 0-4095
                    and degrees is 0-360.
        """
        pos = self.read2Byte(id, self.ADDR_PRESENT_POSITION)

        return pos
    

    

    

    
