"""
Controlador para la conexión BLE
"""

from bluepy.btle import Peripheral, UUID, Scanner
import threading
import struct
import time

class BLEController:
    def __init__(self, mac_address, service_uuid, characteristic_uuid):
        self.mac_address = mac_address
        self.service_uuid = service_uuid
        self.characteristic_uuid = characteristic_uuid
        self.connected = False
        self.characteristic = None
        self.connection_thread = None
        self.peripheral = None

    def connect(self):
        try:
            self.peripheral = Peripheral(self.mac_address)
            service = self.peripheral.getServiceByUUID(self.service_uuid)
            self.characteristic = service.getCharacteristics(self.characteristic_uuid)[0]
            self.connected = True
            print("BLE connection established")
            print(f"Services: {self.peripheral.getServices()}")
        except Exception as e:
            print("Connection error:", e)
            self.connected = False

    def send_command(self, command):
        if self.connected and self.characteristic:
            try:
            #command_bytes = struct.pack('B', command)
                data = command.encode('utf-8')
                self.characteristic.write(data)
            except Exception as e:
                print("Error sending command:", e)

    def start_sending(self, command):
        #while self.connected:
        self.send_command(command)
            #time.sleep(1)  # Interval

    def start_sending_continuous(self, command):
        self.connection_thread = threading.Thread(target=self.start_sending, args=(command,))
        self.connection_thread.start()

    def stop_sending_continuous(self):
        self.connected = False
        if self.connection_thread:
            self.connection_thread.join()
        self.peripheral.disconnect()

    def disconnect(self):
        try:
            if self.connected:
                self.stop_sending_continuous()  # Detener el envío continuo antes de la desconexión si es necesario
                self.peripheral.disconnect()  # Desconectar el dispositivo BLE
                self.connected = False
                print("BLE disconnected")
            else:
                print("There's no connection")
        except Exception as e:
            print("Error disconnecting BLE:", e)


    def scan_devices(self, timeout=10):
        """
        Realiza un escaneo de dispositivos BLE disponibles.
        
        Args:
        - timeout: Tiempo de duración del escaneo en segundos (predeterminado: 10 segundos).
        
        Returns:
        - Lista de dispositivos encontrados con sus direcciones MAC y nombres.
        """
        try:
            scanner = Scanner()
            devices = scanner.scan(timeout)

            found_devices = []
            for dev in devices:
                found_devices.append({"address": dev.addr, "name": dev.getValueText(9)})  # 9 representa el nombre del dispositivo
                
            return found_devices
        except Exception as e:
            print("Error en el escaneo de dispositivos:", e)
            return []

