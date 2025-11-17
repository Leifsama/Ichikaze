"""
   RasPI OSGAR node handling GPIO - sonars and pause button
"""
from gpiozero import DistanceSensor, Button # <-- Přidáno Button
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero.exc import DistanceSensorNoEcho  # not used yet

from osgar.node import Node


class Sonars(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('sonars', 'pause')

        # Použijeme Pigpio pro přesnější měření času.
        # Důležité: Služba 'pigpiod' musí běžet (sudo systemctl start pigpiod).
        factory = PiGPIOFactory()
        self.sensor1 = DistanceSensor(echo=23, trigger=18, pin_factory=factory)
        self.sensor2 = DistanceSensor(echo=16, trigger=17, pin_factory=factory)

        # Tlačítko: GPIO 5 (Pin 29), aktivujeme interní pull-up rezistor
        self.button = Button(5, pull_up=True)
        self.prev_button_status = None  # not defined

    def on_trigger(self, data):
        distance1 = self.sensor1.distance
        distance2 = self.sensor2.distance
        button_status = button.is_pressed
        self.publish('sonars', [distance1, distance2])  # could this be None as non defined?
        if self.prev_button_status != button_status:
            self.publish('pause', button_status)
            self.prev_button_status = button_status
