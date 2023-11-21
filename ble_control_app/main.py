"""
BLE Client Application
"""


import tkinter as tk

from BLEcontroller import *

# Directions
forward = "1010"
backward = "0101"
right = "1000"
left = "0010"
stop = "0000"

# MAC Address from the ESP32
esp32_mac_address = "40:4c:ca:40:8e:1a" # "78:64:C0:C8:01:5B" # 

# Service UUID and characteristics from the ESP32 to send data
service_uuid = UUID("180")  # Service UUID ESP32
characteristic_uuid = UUID("DEAD")  # Write characteristic


def scan():
    # Realizar un escaneo de dispositivos
    devices_found = ble_controller.scan_devices()

    # Mostrar los dispositivos encontrados
    for device in devices_found:
        print("Dirección MAC:", device["address"])
        print("Nombre:", device["name"])
        print()

# Funciones para controlar el carrito usando botones de la interfaz
def move_continuous(event, direction):
    if event == "press":
        ble_controller.start_sending_continuous(direction)
        print(f"Sending: {direction}")    
    #elif event == "release":
        #ble_controller.start_sending_continuous("0000")
        #ble_controller.stop_sending_continuous()
        print(f"Stoppped sending {direction}")

# Funciones de callback para los botones físicos
def button_forward_pressed():
    move_continuous("press", forward)
    button_released()

def button_backward_pressed():
    move_continuous("press", backward)
    button_released()

def button_left_pressed():
    move_continuous("press", left)
    button_released()

def button_right_pressed():
    move_continuous("press", right)
    button_released()

def button_stop_pressed():
    move_continuous("press", stop)
    button_released()

def button_released():
    move_continuous("release", None)

# Función para desconectar y terminar el programa
def disconnect_and_exit():
    ble_controller.disconnect()
    root.quit()
    print("Program ended")

# BLEController instance
ble_controller = BLEController(esp32_mac_address, service_uuid, characteristic_uuid)
#scan()
ble_controller.connect()
#a = input("cont")


# Crear ventana de tkinter
root = tk.Tk()
root.title("ESP32 BLE Client Application")

# Control buttons on root
button_font = ('Helvetica', 20, 'bold')  # Definir la fuente y tamaño del texto
button_height = 1
button_width = 3

button_forward = tk.Button(root, text="↑", command=button_forward_pressed, height=button_height, width=button_width, font=button_font)
button_forward.grid(row=0, column=1, pady=10)

button_left = tk.Button(root, text="←", command=button_left_pressed, height=button_height, width=button_width, font=button_font)
button_left.grid(row=1, column=0, padx=10)

button_right = tk.Button(root, text="→", command=button_right_pressed, height=button_height, width=button_width, font=button_font)
button_right.grid(row=1, column=2, padx=10)

button_backward = tk.Button(root, text="↓", command=button_backward_pressed, height=button_height, width=button_width, font=button_font)
button_backward.grid(row=1, column=1)

button_backward = tk.Button(root, text="Stop", command=button_stop_pressed, height=button_height, width=button_width, font=button_font)
button_backward.grid(row=2, column=1, pady=10)


# End button
button_disconnect = tk.Button(root, text="End", command=disconnect_and_exit, height=button_height, width=button_width)
button_disconnect.grid(row=2, column=2, pady=10)  # Ubicar abajo con un margen superior


# 'q' key ends also the program
root.bind('q', lambda event: disconnect_and_exit())

# Asociar presión y liberación de teclas a las funciones de movimiento continuo
root.bind('<KeyPress-Up>', lambda event: move_continuous("press", forward))
root.bind('<KeyRelease-Up>', lambda event: move_continuous("release", None))

root.bind('<KeyPress-Down>', lambda event: move_continuous("press", backward))
root.bind('<KeyRelease-Down>', lambda event: move_continuous("release", None))

root.bind('<KeyPress-Left>', lambda event: move_continuous("press", left))
root.bind('<KeyRelease-Left>', lambda event: move_continuous("release", None))

root.bind('<KeyPress-Right>', lambda event: move_continuous("press", right))
root.bind('<KeyRelease-Right>', lambda event: move_continuous("release", None))

root.bind('s ', lambda event: move_continuous("press", stop))
#root.bind('<space>', lambda event: move_continuous("release", None))

# Ejecutar la ventana
root.mainloop()

