from devices.interface import Device
from pymodbus.constants import Defaults
from pymodbus.client.sync import ModbusTcpClient 

class ModbusDevice(Device):

    def __init__(self, host, port, timeout = 0.5):
        Defaults.Timeout = timeout
        self.plc_device = ModbusTcpClient(host = host, port = port, timeout = timeout, auto_open = True)
        
    def get_data(self, register):
        return self.plc_device.read_holding_registers(register).registers[0]        

    def get_digital_input(self, input: list):
        # return self.plc_device.read_coils(input_pin).bits[0]
        regitster = input[0]
        register_bit = input[1]

        return self.get_data(register=regitster) >> register_bit & 1
           
    def set_digital_output(self, register, mask, state: bool):
        value = self.get_data(register)
        if state:    
            new_value = value | mask
        else:
            new_value = (value & ~mask) & 0xFFFF
        return self.plc_device.write_register(register, new_value)             
    
    def send_data(self, register, value):
        return self.plc_device.write_register(register, value)
        
    def set_true(c, register, mask):
        value = c.get_data(register)
        new_value = value | mask
        return c.send_data(register, new_value)

    def set_false(c, register, mask):
        value = c.get_data(register)
        new_value = (value & ~mask) & 0xFFFF
        return c.send_data(register, new_value)
      
