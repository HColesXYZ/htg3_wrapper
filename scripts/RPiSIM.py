import socket
import curses

def get_key():
    # Set up curses
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)

    stdscr.clear()
    stdscr.addstr("Press a key: ")
    stdscr.refresh()

    # Get a character
    key = stdscr.getch()

    # Convert the key code to a character
    key_char = chr(key)

    curses.endwin()  # End curses mode

    return key_char

def get_local_ipv4():
    # Create a socket to get local IPv4 address
    temp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Connect to any address (doesn't send any packets)
        temp_sock.connect(('8.8.8.8', 80))
        local_ip = temp_sock.getsockname()[0]
        print(f"Local IPv4 Address: {local_ip}")
    except Exception as e:
        print("Error getting local IPv4 address:", e)
    finally:
        temp_sock.close()

class RPIComms:
    def __init__(self):
        self.sockfd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.sockfd.fileno() == -1:
            print("Comms - socket creation failed.")
            exit(0)
        self.servaddr = ('', 0)  # Initialize servaddr with desired values
        self.cliaddr = ('', 0)  # Initialize cliaddr with desired values
        self.ack = 'ACK\n'
        self.buffer = [256]
    
    def init_server(self, port):
        self.servaddr = ('', port)
        try:
            self.sockfd.bind(self.servaddr)
            print("Comms - socket successfully binded.")
        except socket.error as e:
            print(f"Comms - socket bind failed: {e}")
            exit(0)
        
        self.sockfd.listen(5)
        print("Comms - server listening.")
        
        self.conn, self.cliaddr = self.sockfd.accept()
        print("Comms - server accept the client.")
    
    def closeComms(self):
        self.sockfd.close()

    def write_data(self):
        try:
            if self.buffer[0] is not None:
                data_to_send = str(self.buffer[0]).encode('utf-8')  # Convert to bytes
                self.conn.sendall(data_to_send)
                print(f"Comms - Sent data: {self.buffer[0]}")
        except socket.error as e:
            print(f"Comms - Error while sending data: {e}")

if __name__ == "__main__":
    
    get_local_ipv4() 
    comms = RPIComms()
    comms.init_server(52700) # Change this to the desired port number
    
    while(True):
        pressed_key = get_key()
        comms.buffer[0] = pressed_key
        comms.write_data()
        
        if pressed_key == 'x':
            comms.closeComms()
            exit(1)

# python3 RPiSIM.py
