from .plutosock import *

class plutoMSP:
    def __init__(self, sock : plutoSock):
        self.MSP_HEADER = MSP_HEADER
        self.socket = sock

    def sendRequestMSP(self, data : "list[int]") -> None:
        self.socket.write(data)

    '''
        def sendMulRequestMSP(self, data : list[int], droneIndex : int) -> None:
        com.writeMulSock(data, droneIndex)
    '''

    def createPacketMSP(self, msp : int, payload : "list[int]") -> "list[int]":
        bf = []
        for it in MSP_HEADER:
            bf.append(int(ord(it) & 0xFF))
        
        checksum : int = 0
        pl_size = int(len(payload) & 0xFF)
        bf.append(pl_size)
        checksum ^= (pl_size & 0xFF)

        bf.append(int(msp & 0xFF))
        checksum ^= (msp & 0xFF)

        if payload:
            for it in payload:
                bf.append(int(it & 0xFF))
                checksum ^= (it & 0xFF)
        
        bf.append(checksum)
        return bf


    def sendRequestMSP_SET_RAW_RC(self, channels : "list[int]") -> None:
        rc_signals = [0]*16
        for i in range(8):
            rc_signals[2*i] = int(channels[i] & 0xFF)
            rc_signals[2*i + 1] = int((channels[i] >> 8) & 0xFF)
        
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_RAW_RC, rc_signals))

    def sendMulRequestMSP_SET_RAW_RC(self, channels : "list[int]") -> None:
        droneIndex = channels[8]
        rc_signals = [0]*16
        for i in range(8):
            rc_signals[2*i] = int(channels[i] & 0xFF)
            rc_signals[2*i + 1] = int((channels[i] >> 8) & 0xFF)
        
        self.sendMulRequestMSP(self.createPacketMSP(MSP_SET_RAW_RC, rc_signals), droneIndex)     


    def sendRequestMSP_SET_POS(self, posArray : "list[int]") -> None:
        posData = [0]*8
        for i in range(4):
            posData[2*i] = int(posArray[i] & 0xFF)
            posData[2*i + 1] = int((posArray[i] >> 8) & 0xFF)
        
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_POS, posData))


    def sendRequestMSP_SET_COMMAND(self, commandType : int) -> None:
        payload = [0]*1
        payload[0] = int(commandType & 0xFF)
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_COMMAND, payload))


    def sendRequestMSP_GET_DEBUG(self, requests : "list[int]") -> None:
        for i in range(len(requests)):
            self.sendRequestMSP(self.createPacketMSP(requests[i], list()))
    
    def sendMulRequestMSP_GET_DEBUG(self, requests : "list[int]", droneIndex : int) -> None:
        for i in range(len(requests)):
            self.sendMulRequestMSP(self.createPacketMSP(requests[i], list()), droneIndex)


    def sendRequestMSP_SET_ACC_TRIM(self, trim_roll : int, trim_pitch : int) -> None:
        payload = [0]*4
        payload[0] = int(trim_pitch & 0xFF)
        payload[1] = int((trim_pitch >> 8) & 0xFF)
        payload[2] = int(trim_roll & 0xFF)
        payload[3] = int((trim_roll >> 8) & 0xFF)
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_ACC_TRIM, payload))

    def sendRequestMSP_ACC_TRIM(self) -> None:
        self.sendRequestMSP(self.createPacketMSP(MSP_ACC_TRIM, list()))


    def sendRequestMSP_EEPROM_WRITE(self) -> None:
        self.sendRequestMSP(self.createPacketMSP(MSP_EEPROM_WRITE, list()))