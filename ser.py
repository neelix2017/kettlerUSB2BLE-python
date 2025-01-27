import asyncio
import serial_asyncio
import logging
from typing import Any, Dict, Union
import struct
import subprocess
from bless import (  # type: ignore
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)
import bleConstants         as bc
import datetime,time
import io

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(name=__name__)
_data = []
queue = []
speed = 0
hr    = 0
power = 25
rpm = 0
time = 0
gear = 4
_rpm = []
serial_connected = False
session_data = []
    
class Kettler(asyncio.Protocol):
    
   
    def connection_made(self, transport):
        self.transport = transport
        logger.debug('port opened', transport)
        transport.serial.rts = True  # You can manipulate Serial object via transport
        #transport.write(b'ST\r\n')  # Write serial data via transport
    
    def translateData(self,segments):
        global speed
        global rpm
        global hr
        global power
        global time
        # heartRate cadence speed distanceInFunnyUnits destPower energy timeElapsed realPower
        # 000 052 095 000 030 0001 00:12 030
        hr = int(segments[0])
        rpm = int(segments[1])
        speed = int(segments[2])*0.1
        distance = (segments[3])
        #if (power != Normalize(int(segments[4]))):
        #   queue.append(['s',"PW", power])
        power = int(segments[4])
        energy = int(segments[5])
        t = segments[6].split(':')
        time = int(t[0])*60+int(t[1])
        realPower = int(segments[7])
        logger.info("time: %s, cadence: %s, power: %s  realPower: %s, HR: %s" % (time, rpm, power, realPower, hr))
        
        
    def data_received(self, data):
        global _data
        logger.debug('data received', repr(data))
        _data.append(data.decode("utf-8"))
        logger.debug(str(_data))
        if b'\n' in data:
            x = "".join(_data)
            #logger.info(x)
            segments = x.split('\t')
            if len(segments)>=8 :
                _data.clear()
                self.translateData(segments)
                #self.transport.close()
        
    def getstatus(self):
        self.transport.write(b'ST\r\n')
        
    def reset(self):
        self.transport.write(b'CM\r\n')
        
    def setPower(self, power):
        self.transport.write(bytes('PW'+str(power)+'\r\n','utf-8'))
        
    def askState(self):
        global power
        if (False) :
            self.setPower(power)
        else :
            self.getstatus()



    def connection_lost(self, exc):
        print('port closed')
        self.transport.loop.stop()

    def pause_writing(self):
        print('pause writing')
        print(self.transport.get_write_buffer_size())

    def resume_reading(self):
        # This will start the callbacks to data_received again with all data that has been received in the meantime.
        self.transport.resume_reading()
        
     


async def reader():
    global serial_connected
    try:
        transport, protocol = await serial_asyncio.create_serial_connection(loop, Kettler, 'COM3', baudrate=9600)
        serial_connected = True
    except Exception as e:
        serial_connected = False
        logger.info(f"Serial not connected")
    global queue
    queue.append(['c',bc.cFitnessMachineControlPointUUID, b'\x80\x05\x01'])
    queue.append(['s','CM'])
    if not serial_connected: return
    while True:
        if len(queue) > 0:
            if (queue[0][0]=='s'):
                logger.info(queue)
                if (queue[0][1]=='PW'):
                    protocol.setPower(queue[0][2])
                if (queue[0][1]=='ST'):
                    protocol.askState()
                if (queue[0][1]=='CM'):
                    protocol.reset()
                queue = queue[1:]
        await asyncio.sleep(1)
        #protocol.askState()
        


def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    logger.info(f" [{characteristic.uuid}] Read request recieved {characteristic.value}")
    # b'\x02@\x00\x00\x08 \x00\x00'
    
    return characteristic.value


def write_request(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    characteristic.value = value
    global speed
    global rpm
    global hr
    global power
    global serial_connected
    logger.info(f"[{characteristic.uuid}] Char value set to {characteristic.value}")
    if characteristic.value == b"\x00":  #request control
        logger.debug("request control")
        response = b'\x80\x00\x01'
        queue.append(['c',bc.cFitnessMachineControlPointUUID, response])
    elif characteristic.value == b"\x01":  #resetControl
        logger.debug("resetControl")
        response = b'\x80\x01\x01'
        queue.append(['c',bc.cFitnessMachineControlPointUUID, response])
    elif characteristic.value == b"\x07":  #startOrResume
        logger.debug("startOrResume")
        response = b'\x80\x07\x01'
        queue.append(['c',bc.cFitnessMachineControlPointUUID, response])
    elif characteristic.value[0] == 17:  #setIndoorBikeSimulationParameters
        logger.debug("setIndoorBikeSimulationParameters")
        response = b'\x80\x11\x01'
        try:
            tuple  = struct.unpack (bc.little_endian + bc.unsigned_char + bc.short + bc.short + bc.unsigned_char + bc.unsigned_char, value)
        except Exception as e:
            logger.debug("bleBless error: unpack SetIndoorBikeSimulation %s" % e)
        #windspeed = struct.unpack("<H",io.BytesIO(bytearray(value)).read(2)) * 0.001
        #grade = struct.unpack("<H",io.BytesIO(bytearray(value)).read(2)) * 0.01
        #crr = struct.unpack("<H",io.BytesIO(bytearray(value)).read(1)) * 0.0001
        #w = struct.unpack("<H",io.BytesIO(bytearray(value)).read(1)) * 0.01
        #logger.info(f"{windspeed},{grade},{crr},{w}")
        wind  = round(tuple[1] * 0.001,  3)
        grade = round(tuple[2] * 0.01,   2)
        crr   = round(tuple[3] * 0.0001, 4)
        w     = round(tuple[4] * 0.01,   2)
        #simpower = 170 * (1 + 1.15 * (rpm - 80.0) / 80.0) * (1.0 + 3 * (grade)/ 100.0)
        #gear = 5
        #simpower = Normalize(simpower * (1.0 + 0.1 * (gear - 5)))
        simpower = Normalize(makePower(rpm,grade,crr,autoGear(rpm)))
        logger.info(f"simpower={simpower}")
        queue.append(['c',bc.cFitnessMachineControlPointUUID, response])  
        if (abs(simpower-power)>5):
            power = simpower
            if serial_connected: 
                queue.append(['s',"PW", power])
    elif characteristic.value[0] == 5:  #set target power
        logger.debug("set target power")
        response = b'\x80\x05\x01'
        pw = io.BytesIO(bytearray(value)).read(2)
        power = struct.unpack("<H",  pw)  
        logger.info(pw)
        if serial_connected: 
            queue.append(['s',"PW", Normalize(140)])    

def avg(x):
    z=0
    for _x in x:
        z+=_x
    return (z/len(x))

def autoGear(rpm):
    global gear
    global _rpm
    #           50x11  50x13  50x15  50x17  50x19  34x15  34x17  34x19  34x21  34x23  34x25  34x27  34x30  34x34
    gearbox = [  4.55,  3.85,  3.33,  2.94,  2.63,  2.27,  2.00,  1.79,  1.62,  1.48,  1.36,  1.26,  1.13,  1.06]    
    if (20<rpm<60):
        _rpm.append(rpm)
        if (len(_rpm)>=5):
            _rpm = _rpm[1:]
            if (40<avg(_rpm)<60):
                gear+=1
                if (gear==14):gear = 13
                _rpm = []
    elif (rpm>100):
        _rpm.append(rpm)
        if (len(_rpm)>=5):
            _rpm = _rpm[1:]
            if (avg(_rpm)>100):
                gear-=1
                if (gear==-1):gear = 0
                _rpm = []
    else:
        _rpm = []
    return gear

def makePower(rpm,grade,crr,gear):
    wheel = 645.0                       #609.6
    speed = int(3.6*gear*0.0166667*wheel*0.001*rpm*10/3600.0*1000.0)
    
    # formula https://www.fiets.nl/training/de-natuurkunde-van-het-fietsen/
    
    c     = crr     #0.004                  # roll-resistance constant
    mm    = 93                              # riders weight kg
    mb    = 8.8                             # bike weight kg
    m     = mb + mm
    g     = 9.7803184                       # m/s2
    v     = speed / 3.6                     # m/s       km/hr * 1000 / 3600
    # kotalni upor
    Proll = c * m * g * v                   # Watt
    p     = 1.205                           # air-density
    cdA   = 0.3                             # resistance factor
                                            # p_cdA = 0.375
    w     =  0                              # wind-speed
    # zračni upor
    Pair  = 0.5 * p * cdA * (v+w)*(v+w)* v  # Watt
    i     = grade                           # Percentage 0...100
    # upor strmine
    Pslope= i/100 * m * g * v               # Watt
    # mehanski upor (veriga, pesto)
    Pbike = 37                              # bike effi
    return Normalize(Proll + Pair + Pslope + Pbike)
    

def Normalize(x, base=5,max = 400):
    if (x<0): x = 0
    if (x>max): x = max
    return base * round(x/base)

async def run(loop):
    # Instantiate the server
    gatt: Dict = {
    bc.sFitnessMachineUUID: {
        bc.cFitnessMachineFeatureUUID: {
            "Properties":   (GATTCharacteristicProperties.read),
            "Permissions":  (GATTAttributePermissions.readable | GATTAttributePermissions.writeable),
            "Value":        bc.fmf_Info,                        # b'\x02\x40\x00\x00\x08\x20\x00\x00',
            "value":        bc.fmf_Info,
            "Description":  bc.cFitnessMachineFeatureName
        },
        bc.cIndoorBikeDataUUID: {
            "Properties":   (GATTCharacteristicProperties.notify),
            "Permissions":  (GATTAttributePermissions.readable | GATTAttributePermissions.writeable),
            "Value":        bc.ibd_Info,                        # Instantaneous Cadence, Power, HeartRate
            "value":        bc.ibd_Info,
            "Description":  bc.cIndoorBikeDataName
        },
        bc.cFitnessMachineStatusUUID: {
            "Properties":   (GATTCharacteristicProperties.notify),
            "Permissions":  (GATTAttributePermissions.readable | GATTAttributePermissions.writeable),
            "value":        b'\x00\x00',                        # Status as "sent" to Cycling Training Program
            "Value":        b'\x00\x00',
            "Description":  bc.cFitnessMachineStatusName
        },
        bc.cFitnessMachineControlPointUUID: {
            "Properties":   (GATTCharacteristicProperties.write | GATTCharacteristicProperties.indicate),
            "Permissions":  (GATTAttributePermissions.readable | GATTAttributePermissions.writeable),
            "Value":        b'\x00\x00',                        # Commands as received from Cycling Training Program
            "value":        b'\x00\x00',
            "Description":  bc.cFitnessMachineControlPointName
        },
        bc.cSupportedPowerRangeUUID: {
            "Properties":   (GATTCharacteristicProperties.read),
            "Permissions":  (GATTAttributePermissions.readable | GATTAttributePermissions.writeable),
            "Value":        bc.spr_Info,                        # Static additional properties of the FTMS
                                                                # b'\x00\x00\xe8\x03\x01\x00'
                                                                # min=0, max=1000, incr=1 
                                                                # ==> 0x0000 0x03e8 0x0001 ==> 0x0000 0xe803 0x0100
            "value":        bc.spr_Info,
            "Description":  bc.cSupportedPowerRangeName
        }
    },
    bc.sHeartRateUUID: {
        bc.cHeartRateMeasurementUUID: {
            "Properties":   (GATTCharacteristicProperties.notify),
            "Permissions":  (GATTAttributePermissions.readable | GATTAttributePermissions.writeable),
            "Value":        bc.hrm_Info,
            "value":        bc.hrm_Info,
            "Description":  bc.cHeartRateMeasurementName
        }
    }
}
    global queue
    global serial_connected
    subprocess.call("powercfg -change -monitor-timeout-ac 180")
    subprocess.call("powercfg -change -standby-timeout-ac 180")
    my_service_name = "Kettler-APP"
    server = BlessServer(name=my_service_name, loop=loop)
    server.read_request_func = read_request
    server.write_request_func = write_request
    await server.add_gatt(gatt)
    await server.start()
    logger.debug(server.get_characteristic(bc.cFitnessMachineFeatureUUID))
    logger.debug("Advertising")
    await asyncio.sleep(3)
    #84-24-FA-00-00-00-00-00-00-00-00-00-00-00-00-00-00
    while (True):
        global speed
        global rpm
        global hr
        global power
        global time
        global session_data
        #send = ( b'\x44\x02\xaa\xaa\xbb\xbb\xcc\xcc\xdd\xdd' )
        #_speed = struct.pack('<h',int(speed)*100)
        #_rpm   = struct.pack('<h',rpm)
        #_power = struct.pack('<h',power)
        #_hr    = struct.pack('<h',hr)
        #send=send.replace(b'\xaa\xaa', _speed)
        #send=send.replace(b'\xbb\xbb', _rpm)
        #send=send.replace(b'\xcc\xcc', _power)
        #send=send.replace(b'\xdd\xdd', _hr)
        flags = (bc.ibd_InstantaneousCadencePresent | bc.ibd_InstantaneousPowerPresent)
        s     = int(speed * 100) & 0xffff      # Avoid value anomalities
        s = 0xffff
        c     = int(rpm*2 )      & 0xffff      # Avoid value anomalities
        p     = int(power)       & 0xffff      # Avoid value anomalities
        info  = struct.pack (bc.little_endian + bc.unsigned_short * 4, flags, s, c, p)
        logger.debug("%s", info)
        logger.info("cadence: %s, power: %s  speed: %s, HR: %s" % (rpm, power, speed, hr))
        server.get_characteristic(bc.cIndoorBikeDataUUID).value =  bytearray (info)  
        server.update_value(     bc.sFitnessMachineUUID, bc.cIndoorBikeDataUUID    )
        #import random
        #power = random.randrange(100, 300, 5)
        #rpm = random.randrange(50, 70, 2)
        session_data.append([time,power,rpm,hr,speed])
        if len(queue) > 0:
            if (queue[0][0]=='c'):
                logger.info(queue)
                server.get_characteristic(queue[0][1]).value =  bytearray (queue[0][2])  
                server.update_value(     bc.sFitnessMachineUUID, queue[0][1]    )
                queue = queue[1:]
        if serial_connected:
            if (len(queue) ==0):
                queue.append(['s',"ST"])
        await asyncio.sleep(1)
    await server.stop()


def createCSV(session_data):
    import csv
    filename = str('out-'+datetime.datetime.now().strftime("%Y%m%d-%H%M%S")+'.csv' )
    print('SAVING ......')
    with open (filename, 'w') as cvsfile:
        wtr = csv.writer(cvsfile, delimiter=';', lineterminator='\n')
        for line in session_data:
            print (line)
            wtr.writerow([line])
    print(filename+' SAVED')

def createTCX(session_data):
    import TCXexport
    tcx = TCXexport.clsTcxExport()
    tcx.Start()   
    now = datetime.datetime.utcnow()
    j = len (session_data)
    for line in session_data:
        tcx.Trackpoint(Time = now - datetime.timedelta(0,j), HeartRate=line[3], Cadence=line[2], Watts=line[1], SpeedKmh= line[4])
        j -= 1
    tcx.Stop()
    
    
    
async def repeater(): # Here
    loop = asyncio.get_running_loop()
    loop.create_task(reader()) # Here
    await loop.create_task(run(loop)) # "await" is needed


loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
try:
    loop.run_until_complete(repeater()) 
except KeyboardInterrupt: 
    subprocess.call("powercfg -change -monitor-timeout-ac 10")
    subprocess.call("powercfg -change -standby-timeout-ac 30")
    #createCSV(session_data)
    createTCX(session_data)
    
