# Custom Module Imports
from .reader import *

# Standard Library Imports
from socket import *
from select import select
import errno

def ConnectionFailed(code : int, msg : str = '') -> None:
    if (not msg):
        msg = errors[str(code)]
    raise Exception(f"Cannot connect to Pluto, Please Try Again (Error Code {code} : {msg})")

#prot = plutoGET()

class plutoSock:
    def __init__(self, IP_ADDRESS, PORT, buffer : plutoBuffer, responseState : plutoState):
        self.sock = None
        self.sockList = list()
        self.socketSyckLock = 0
        self.c_state = IDLE
        self.offset = 0
        self.dataSize = 0
        self.err_rcvd = False
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = PORT

        self.buffer = buffer

        self.responseState = responseState
        self.response = plutoGET(self.buffer, self.responseState)

    def connect(self) -> bool:
        timeOut = 7 # Seconds
        retry_attempts = 10

        print("Connecting to Pluto......")

        self.sock = socket(AF_INET, SOCK_STREAM)
        if (not self.sock):
            ConnectionFailed(1)

        addr = (self.IP_ADDRESS, self.PORT)

        # Setting Socket to Non-Blocking
        try:
            self.sock.setblocking(False)
        except:
            ConnectionFailed(2)
        if (self.sock.getblocking()):
            ConnectionFailed(2)

        res = self.sock.connect_ex(addr)
        if (res != 0):
            if (res in [errno.EWOULDBLOCK, errno.EINPROGRESS]): # res == errno.EINPROGRESS
                for attempt in range(1,retry_attempts+1):
                    try:
                        rr, rw, re = select([], [self.sock], [], timeOut)
                    except IOError as err:
                        if (err.errno != errno.EINTR):
                            ConnectionFailed(3)
                        else:
                            print(f'Trying to Write to Socket, Attempt ({attempt}/{retry_attempts})')
                            continue
                    if (rw):
                        try:
                            res = self.sock.getsockopt(SOL_SOCKET, SO_ERROR)
                        except:
                            ConnectionFailed(4)
                        if (res):
                            ConnectionFailed(5)
                        else:
                            break # Success
                    else:
                        ConnectionFailed(6)
                else:
                    ConnectionFailed(3, 'Unable to Write to Socket, Terminating.')
            else:
                ConnectionFailed(4)
        else:
            pass # Success
        
        # Setting Socket to Blocking
        try:
            self.sock.setblocking(True)
        except:
            ConnectionFailed(2)
        if (not self.sock.getblocking()):
            ConnectionFailed(2)

        # Checking Status for KeepAlive Option
        try:
            self.sock.getsockopt(SOL_SOCKET, SO_KEEPALIVE)
        except:
            self.sock.close()
            ConnectionFailed(7)

        # Setting the Option Active
        try:
            self.sock.setsockopt(SOL_SOCKET, SO_KEEPALIVE, 1)
        except:
            self.sock.close()
            ConnectionFailed(7)

        # Checking Status AGAIN for KeepAlive Option
        try:
            self.sock.getsockopt(SOL_SOCKET, SO_KEEPALIVE)
        except:
            self.sock.close()
            ConnectionFailed(7)       

        try:
            err = self.sock.getsockopt(SOL_SOCKET, SO_ERROR)
        except:
            ConnectionFailed(4)
        if (err != 0):
            ConnectionFailed(8, 'Socket Failed ({err})')

        print('Pluto Connected!')
        return True

    def disconnect(self):
        self.sock.close()

    def write(self, data : "list[int]") -> int:
        data = bytes(data)
        try:
            sent = self.sock.send(data)
        except WindowsError as winerror:
            print(winerror)
            if winerror.errno == 10053:
                self.connect()
                return None
            else:
                raise winerror
        if (sent == 0):
            # If No Data is sent, fail the connection
            self.sock.close()
            ConnectionError(3)
        elif (sent < len(data)):
            print('Low Socket Connection Speed, There might be lag.')
            while (sent < len(data)):
                t_sent = self.sock.send(data[sent:])
                if (t_sent == 0):
                    self.sock.close()
                    ConnectionError(3)
                sent += t_sent

    def read(self, bufsize : int):
        try:
            c = self.sock.recv(bufsize)[0]
        except:
            return -1
        else:
            return c

    def readResponseMSP(self):
        c = chr(int(self.read(1)))
        if (self.c_state == IDLE):
            self.c_state = (HEADER_START if (c == '$') else IDLE)
        elif (self.c_state == HEADER_START):
            self.c_state = (HEADER_M if (c == 'M') else IDLE)
        elif (self.c_state == HEADER_M):
            if (c == '>'):
                self.c_state = HEADER_ARROW
            elif (c == '!'):
                self.c_state = HEADER_ERR
            else:
                self.c_state = IDLE
        elif ((self.c_state == HEADER_ARROW) or (self.c_state == HEADER_ERR)):
            self.err_rcvd = (self.c_state == HEADER_ERR)
            self.dataSize = ord(c) & 0xFF
            self.offset = 0
            self.checksum = 0
            self.checksum ^= self.dataSize
            # (?) the command is to follow (?)
            self.c_state = HEADER_SIZE
        elif (self.c_state == HEADER_SIZE):
            t_c = (ord(c) & 0xFF)
            self.cmd = int(t_c)
            self.checksum ^= (t_c)
            self.c_state = HEADER_CMD
        elif (self.c_state == HEADER_CMD and self.offset < self.dataSize):
            t_c = (ord(c) & 0xFF)
            self.checksum ^= (t_c)
            self.buffer.inputBuffer[self.offset] = int(t_c)
            self.offset = self.offset + 1
        elif (self.c_state == HEADER_CMD and self.offset >= self.dataSize):
            # Comparing Calculated and Transferred Checksum
            t_c = (ord(c) & 0xFF)
            if ((self.checksum & 0xFF) == int(t_c)):
                if (self.err_rcvd):
                    pass
                else:
                    self.buffer.bufferIndex = 0
                    self.response.evaluateCommand(self.cmd)
            else:
                pass
            self.c_state = IDLE