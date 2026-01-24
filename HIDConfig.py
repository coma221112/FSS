import hid
import struct
import tkinter as tk
from tkinter import messagebox
import threading
import time

class JoystickMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("USB HID Joystick Monitor")
        self.root.geometry("700x550")
        
        self.device = None
        self.running = False
        
        self.setup_ui()
        self.connect_device()
        
        # Start reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
    def setup_ui(self):
        # Status bar
        status_frame = tk.Frame(self.root, bg='lightgray', height=30)
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        self.status_label = tk.Label(status_frame, text="Status: Disconnected", 
                                     bg='lightgray', font=('Arial', 10, 'bold'))
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # Buttons section (Report ID 1)
        buttons_frame = tk.LabelFrame(self.root, text="Buttons (Report ID 1)", 
                                     font=('Arial', 10, 'bold'))
        buttons_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.button_labels = {}
        for i in range(24):
            row = i // 8
            col = i % 8
            label = tk.Label(buttons_frame, text=f"B{i+1}: OFF", 
                           relief=tk.RIDGE, width=10, bg='lightgray')
            label.grid(row=row, column=col, padx=2, pady=2)
            self.button_labels[i] = label
            
        # Axes section (Report ID 1)
        axes_frame = tk.LabelFrame(self.root, text="Axes (Report ID 1)", 
                                  font=('Arial', 10, 'bold'))
        axes_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.axis_labels = {}
        axis_names = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        for i, name in enumerate(axis_names):
            row = i // 3
            col = (i % 3) * 2
            
            tk.Label(axes_frame, text=f"{name}:", font=('Arial', 9)).grid(
                row=row, column=col, padx=5, pady=3, sticky=tk.E)
            
            label = tk.Label(axes_frame, text="0", font=('Courier', 9), 
                           relief=tk.SUNKEN, width=12, bg='white')
            label.grid(row=row, column=col+1, padx=5, pady=3, sticky=tk.W)
            self.axis_labels[name.lower()] = label
            
        # Configuration section (Report ID 2)
        config_frame = tk.LabelFrame(self.root, text="Configuration (Report ID 2)", 
                                    font=('Arial', 10, 'bold'))
        config_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Max Range
        tk.Label(config_frame, text="Max Range:").grid(
            row=0, column=0, padx=5, pady=3, sticky=tk.E)
        self.max_range_entry = tk.Entry(config_frame, width=15)
        self.max_range_entry.insert(0, "800000")
        self.max_range_entry.grid(row=0, column=1, padx=5, pady=3, sticky=tk.W)
        
        # Calib Long Press Ms
        tk.Label(config_frame, text="Calib Long Press (ms):").grid(
            row=1, column=0, padx=5, pady=3, sticky=tk.E)
        self.calib_press_entry = tk.Entry(config_frame, width=15)
        self.calib_press_entry.insert(0, "2000")
        self.calib_press_entry.grid(row=1, column=1, padx=5, pady=3, sticky=tk.W)
        
        # Deadzone X
        tk.Label(config_frame, text="Deadzone X:").grid(
            row=2, column=0, padx=5, pady=3, sticky=tk.E)
        self.deadzone_x_entry = tk.Entry(config_frame, width=15)
        self.deadzone_x_entry.insert(0, "3000")
        self.deadzone_x_entry.grid(row=2, column=1, padx=5, pady=3, sticky=tk.W)
        
        # Deadzone Y
        tk.Label(config_frame, text="Deadzone Y:").grid(
            row=3, column=0, padx=5, pady=3, sticky=tk.E)
        self.deadzone_y_entry = tk.Entry(config_frame, width=15)
        self.deadzone_y_entry.insert(0, "1000")
        self.deadzone_y_entry.grid(row=3, column=1, padx=5, pady=3, sticky=tk.W)
        
        # Deadzone Z
        tk.Label(config_frame, text="Deadzone Z:").grid(
            row=4, column=0, padx=5, pady=3, sticky=tk.E)
        self.deadzone_z_entry = tk.Entry(config_frame, width=15)
        self.deadzone_z_entry.insert(0, "1000")
        self.deadzone_z_entry.grid(row=4, column=1, padx=5, pady=3, sticky=tk.W)
        
        # Buttons frame
        btn_frame = tk.Frame(config_frame)
        btn_frame.grid(row=5, column=0, columnspan=2, pady=10)
        
        # Send config button
        self.send_config_btn = tk.Button(btn_frame, 
                                        text="Send Configuration to Device",
                                        command=self.send_configuration,
                                        bg='lightgreen',
                                        font=('Arial', 9, 'bold'))
        self.send_config_btn.pack(side=tk.LEFT, padx=5)
        
        # Request config button
        self.request_config_btn = tk.Button(btn_frame, 
                                           text="Request Configuration from Device",
                                           command=self.request_configuration,
                                           bg='lightblue')
        self.request_config_btn.pack(side=tk.LEFT, padx=5)
        
        # Status message
        self.config_status = tk.Label(config_frame, text="", 
                                     font=('Arial', 8), fg='blue')
        self.config_status.grid(row=6, column=0, columnspan=2, pady=5)
        
    def connect_device(self):
        """Connect to the HID device - UPDATE VID/PID to match your device"""
        try:
            # TODO: Replace with your actual VID and PID
            VID = 0x0483  # STM32 default VID (update this!)
            PID = 0x5750  # Custom HID PID (update this!)
            
            self.device = hid.device()
            self.device.open(VID, PID)
            self.device.set_nonblocking(1)
            
            manufacturer = self.device.get_manufacturer_string()
            product = self.device.get_product_string()
            
            self.status_label.config(text=f"Status: Connected - {manufacturer} {product}")
            
        except Exception as e:
            self.status_label.config(text=f"Status: Connection Failed")
            messagebox.showerror("Connection Error", 
                               f"Could not connect to device.\n{e}\n\n"
                               "Please update VID/PID in the code.")
            
    def read_loop(self):
        """Background thread to read HID reports"""
        while self.running:
            if self.device:
                try:
                    data = self.device.read(64, timeout_ms=10)
                    if data:
                        self.process_report(data)
                except Exception as e:
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
                
    def process_report(self, data):
        """Process incoming HID report based on Report ID"""
        if not data or len(data) == 0:
            return
            
        report_id = data[0]
        
        if report_id == 1:
            self.process_joystick_report(data)
        elif report_id == 2:
            self.process_config_report(data)
            
    def process_joystick_report(self, data):
        """Process Report ID 1 - Joystick data"""
        try:
            # Report format: 1 byte report ID + 4 bytes buttons + 12 bytes axes (6x int16)
            if len(data) < 17:
                return
                
            # Extract button data (4 bytes = 32 bits)
            buttons = struct.unpack('<I', bytes(data[1:5]))[0]
            
            # Update button display
            for i in range(24):
                state = (buttons >> i) & 1
                self.button_labels[i].config(
                    text=f"B{i+1}: {'ON' if state else 'OFF'}",
                    bg='lightgreen' if state else 'lightgray'
                )
            
            # Extract axis data (6x int16)
            axes = struct.unpack('<hhhhhh', bytes(data[5:17]))
            axis_names = ['x', 'y', 'z', 'rx', 'ry', 'rz']
            
            for i, name in enumerate(axis_names):
                self.axis_labels[name].config(text=f"{axes[i]:6d}")
            
        except Exception as e:
            pass
            
    def process_config_report(self, data):
        """Process Report ID 2 - Configuration data received from device"""
        try:
            # Report format: 1 byte report ID + 4 bytes maxRange + 4 bytes calibMs + 
            #                3x 2 bytes deadzones + 49 bytes reserved
            if len(data) < 64:
                return
                
            # Parse configuration
            max_range = struct.unpack('<I', bytes(data[1:5]))[0]
            calib_ms = struct.unpack('<I', bytes(data[5:9]))[0]
            deadzone_x = struct.unpack('<H', bytes(data[9:11]))[0]
            deadzone_y = struct.unpack('<H', bytes(data[11:13]))[0]
            deadzone_z = struct.unpack('<H', bytes(data[13:15]))[0]
            
            # Update entry fields with received values
            self.max_range_entry.delete(0, tk.END)
            self.max_range_entry.insert(0, str(max_range))
            
            self.calib_press_entry.delete(0, tk.END)
            self.calib_press_entry.insert(0, str(calib_ms))
            
            self.deadzone_x_entry.delete(0, tk.END)
            self.deadzone_x_entry.insert(0, str(deadzone_x))
            
            self.deadzone_y_entry.delete(0, tk.END)
            self.deadzone_y_entry.insert(0, str(deadzone_y))
            
            self.deadzone_z_entry.delete(0, tk.END)
            self.deadzone_z_entry.insert(0, str(deadzone_z))
            
            self.config_status.config(text="✓ Configuration received from device", fg='green')
            
        except Exception as e:
            self.config_status.config(text=f"✗ Error parsing config: {e}", fg='red')
            
    def send_configuration(self):
        """Send configuration to device (Report ID 2)"""
        if not self.device:
            messagebox.showwarning("Not Connected", "Device not connected")
            return
            
        try:
            # Parse values from entry fields
            max_range = int(self.max_range_entry.get())
            calib_ms = int(self.calib_press_entry.get())
            deadzone_x = int(self.deadzone_x_entry.get())
            deadzone_y = int(self.deadzone_y_entry.get())
            deadzone_z = int(self.deadzone_z_entry.get())
            
            # Validate ranges
            if max_range < 0 or max_range > 0xFFFFFFFF:
                raise ValueError("Max Range must be 0-4294967295")
            if calib_ms < 0 or calib_ms > 0xFFFFFFFF:
                raise ValueError("Calib Long Press must be 0-4294967295")
            if deadzone_x < 0 or deadzone_x > 0xFFFF:
                raise ValueError("Deadzone X must be 0-65535")
            if deadzone_y < 0 or deadzone_y > 0xFFFF:
                raise ValueError("Deadzone Y must be 0-65535")
            if deadzone_z < 0 or deadzone_z > 0xFFFF:
                raise ValueError("Deadzone Z must be 0-65535")
            
            # Build the report (64 bytes total)
            report = bytearray(64)
            report[0] = 0x02  # Report ID
            
            # Pack configuration values
            struct.pack_into('<I', report, 1, max_range)      # bytes 1-4
            struct.pack_into('<I', report, 5, calib_ms)       # bytes 5-8
            struct.pack_into('<H', report, 9, deadzone_x)     # bytes 9-10
            struct.pack_into('<H', report, 11, deadzone_y)    # bytes 11-12
            struct.pack_into('<H', report, 13, deadzone_z)    # bytes 13-14
            # bytes 15-63 are reserved (already 0)
            
            # Send to device
            self.device.write(list(report))
            
            self.config_status.config(text="✓ Configuration sent to device", fg='green')
            
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))
            self.config_status.config(text=f"✗ Invalid input: {e}", fg='red')
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send config:\n{e}")
            self.config_status.config(text=f"✗ Send failed: {e}", fg='red')
            
    def request_configuration(self):
        """Request configuration from device (send empty Report ID 2)"""
        if not self.device:
            messagebox.showwarning("Not Connected", "Device not connected")
            return
            
        try:
            # Send a request with Report ID 2 and all zeros
            # Device should respond by sending its current config
            request = [0x02] + [0xFF] * 63  # Report ID 2 + 63 0xFF bytes as request marker
            self.device.write(request)
            self.config_status.config(text="Configuration request sent...", fg='blue')
        except Exception as e:
            messagebox.showerror("Error", f"Failed to request config:\n{e}")
            self.config_status.config(text=f"✗ Request failed: {e}", fg='red')
            
    def on_close(self):
        """Clean up on window close"""
        self.running = False
        if self.device:
            self.device.close()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = JoystickMonitor(root)
    root.mainloop()

if __name__ == "__main__":
    main()