from abc import ABC, abstractmethod

class Device(ABC):

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def get_digital_input(self):
        pass

    @abstractmethod
    def set_digital_output(self):
        pass

    @abstractmethod
    def get_data(self):
        pass     

    @abstractmethod
    def send_data(self):
        pass    
        
    @abstractmethod
    def set_true(self):
        pass
        
    @abstractmethod
    def set_false(self):
        pass                   