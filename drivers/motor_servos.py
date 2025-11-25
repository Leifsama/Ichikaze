"""
  Control of motor servos
"""
from osgar.node import Node

# === CRC8 MAXIM pro motory DDSM210 ===
def crc8_maxim(data):
    """Vypočítá 8bitový CRC checksum pro data."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


#################################################################
class MotorServos(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw')  # commands to servos

    def send_packet(self, packet):
        """Odešle příkaz motoru s přidaným CRC."""
        #full = bytearray(packet + [crc8_maxim(cmd)])
        full = bytearray(packet + [crc8_maxim(packet)])
        self.publish('raw', full)

    def on_cmd(self, data):
        assert len(data) == 4, data  # expects 4 numbers
        for i, speed in enumerate(data):
            if i % 2 == 1:
                speed = -speed
            cmd_velocity = [
                i + 1,  # motor_id
                0x64,
                speed & 0xFF, (speed >> 8) & 0xFF,  # TODO verify LSB, MSB
                0x00, 0x00,
                0x50, 0x00, 0x00
            ]
            self.send_packet(cmd_velocity)
            #self.publish(cmd_velocity)

    def run(self):
        # extra initialization step needed at the beginning
        for motor_id in [1, 2, 3, 4]:
            cmd_init = [motor_id, 0xA0, 0x02] + [0x00] * 6
            self.send_packet(cmd_init)
            #self.publish(cmd_init)

        super().run()  # just to do the basic callback logic


#################################################################
class MotorServosSelftest(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd')  # 4 numbers for all motor speeds

    def run(self):
        # send testing sequence
        self.publish('cmd', [0, 0, 0, 0])
        self.sleep(1.0)
        self.publish('cmd', [10, 10, 10, 10])  # TODO some reasonable speed of motor #1
        self.sleep(5.0)
        self.publish('cmd', [0, 0, 0, 0])
        self.sleep(1.0)
