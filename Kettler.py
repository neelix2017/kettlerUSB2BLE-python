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
import bikeConstants        as bike
import datetime,time
import io
import winsound
import random
import keyboard

#logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(filename="Kettler.log",
                    filemode='a',
                    format='%(asctime)s,%(msecs)03d %(name)s %(levelname)s %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    level=logging.DEBUG)
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
gained_control = False
running = True
session_data = []
_uuid = []
    
class Kettler(asyncio.Protocol):
    
   
    def connection_made(self, transport):
        self.transport = transport
        logger.debug('port opened', transport)
        winsound.PlaySound('Kettler_gears\\connected.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
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
        try:
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
            logger.debug("Kettler responds =   time: %s, cadence: %s, power: %s  realPower: %s, HR: %s" % (time, rpm, power, realPower, hr))
        except Exception as e:
            logger.debug(f"Unknown response "+str(segments))

        
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
        self.transport.write(b'RS\r\n')
        
    def init(self):
        self.transport.write(b'CM\r\n')
        
    def setPower(self, p):
        global power
        power = p
        p = bytes('PW '+str(p)+'\n','UTF-8')
        logger.debug("power "+str(p))
        self.transport.write(p)
        
    def askState(self, setpower):
        if (setpower!=0) :
            self.setPower(setpower)
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
    global gained_control
    global running
    try:
        transport, protocol = await serial_asyncio.create_serial_connection(loop, Kettler, 'COM3', baudrate=9600)
        serial_connected = True
    except Exception as e:
        serial_connected = False
        logger.debug(f"Serial not connected")
    global queue
    queue.append(['c',bc.cFitnessMachineControlPointUUID, b'\x80\x05\x01'])
    queue.append(['s','CM'])
    if not serial_connected: return
    while running:
        if len(queue) > 0:
            if (queue[0][0]=='s'):
                logger.debug(queue)
                if (queue[0][1]=='PW'):
                    protocol.askState(queue[0][2])
                if (queue[0][1]=='ST'):
                    protocol.askState(0)
                if (queue[0][1]=='CM'):
                    protocol.reset()
                    await asyncio.sleep(5)
                    protocol.init()
                    await asyncio.sleep(1)
                    gained_control = True
                    queue.append(['s','PW',100])
                queue = queue[1:]
        await asyncio.sleep(1)
        #protocol.askState()
        


def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    logger.debug(f" [{characteristic.uuid}] Read request recieved {characteristic.value}")
    if (characteristic.uuid not in _uuid):
        winsound.PlaySound('Kettler_gears\\device_subscribed.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
        _uuid.append(characteristic.uuid)
    # b'\x02@\x00\x00\x08 \x00\x00'
    
    return characteristic.value


def write_request(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    characteristic.value = value
    global gear
    global speed
    global rpm
    global hr
    global power
    global serial_connected
    logger.debug(f"[{characteristic.uuid}] Char value set to {characteristic.value}")
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
        simpower = makePower(rpm,grade,crr,w,wind,autoGear(rpm))
        logger.debug(f"BLE notified info   =   wind:{wind}, grade:{grade},crr:{crr},w:{w}            calculate simpower={simpower}, gear={gear}")
        queue.append(['c',bc.cFitnessMachineControlPointUUID, response])  
        if (abs(simpower-power)>5):
            if serial_connected: 
                #winsound.Beep(4000, 100)
                queue.append(['s',"PW", simpower])
    elif characteristic.value[0] == 5:  #set target power
        logger.debug("set target power")
        response = b'\x80\x05\x01'
        pw = io.BytesIO(bytearray(value)).read(2)
        power = struct.unpack("<H",  pw)  
        logger.debug(pw)
        if serial_connected: 
            queue.append(['s',"PW", Normalize(100)])    

def avg(x):
    z=0
    for _x in x:
        z+=_x
    return (z/len(x))
    
    
def autoGear(rpm):
    global gear
    global _rpm
    if (20<rpm<60):
        _rpm.append(rpm)
        if (len(_rpm)>=5):
            #_rpm = _rpm[1:]
            #if (20<avg(_rpm)<60):
            gear+=1
            #winsound.Beep(2500, 200)
            if (gear>=14):gear = 13
            winsound.PlaySound('Kettler_gears\\'+str(gear+1)+'.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
            _rpm = []
    elif (rpm>94):
        _rpm.append(rpm)
        if (len(_rpm)>=3):
            #_rpm = _rpm[1:]
            #if (avg(_rpm)>94):
            gear-=1
            #winsound.Beep(4000, 200)
            if (gear<=-1):gear = 0
            winsound.PlaySound('Kettler_gears\\'+str(gear+1)+'.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
            _rpm = []
    else:
        _rpm = []
    return bike.ratio(gear)

def makePower(rpm,grade,crr,p_cdA_d,wind,gear):
    pi              =  3.141592653
    circ            = (bike.wheel + 28 * 2) * pi
    speed           = int(circ * rpm * gear * 60 / 1000000)

    # formula https://www.fiets.nl/training/de-natuurkunde-van-het-fietsen/  
    c               = crr     #0.004                  # roll-resistance constant
    mm              = 93                              # riders weight kg
    m               = bike.weight + mm
    g               = 9.7803184                       # m/s2
    v               = speed / 3.6                     # m/s       km/hr * 1000 / 3600
    # kotalni upor
    Proll           = c * m * g * v                   # Watt
    p               = 1.205                           # air-density
    cdA             = 0.3                             # resistance factor
                                                      # p_cdA = 0.375
    w               =  wind/3.6                       # wind-speed
    
    # zračni upor
    Pair            = 0.5 * p_cdA_d * (v+w)*abs(v+w)* v  # Watt
    # upor strmine
    i               = grade/100                       # Percentage 0...100
    Pslope          = i * m * g * v                   # Watt
    # mehanski upor (veriga, pesto)
    #Pbike           = 37                              # bike effi
    Pbike = 1.015
    return Normalize((Proll + Pair + Pslope) * Pbike)
    

def Normalize(x, base=5,max = 400):
    if (x<25): x = 25
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
    global speed
    global rpm
    global hr
    global power
    global time
    global session_data
    global running
    global queue
    global serial_connected
    global gear
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
    while (running):
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
        logger.debug("Sent to BLE      =    cadence: %s, power: %s  speed: %s, HR: %s" % (rpm, power, speed, hr))
        print("cadence: {:03d}, power: {:03d}  speed: {:03d}, HR: {:03d},       gear: {:02d}".format (rpm, power, speed, hr, gear+1),end='\r')
        server.get_characteristic(bc.cIndoorBikeDataUUID).value =  bytearray (info)  
        server.update_value(     bc.sFitnessMachineUUID, bc.cIndoorBikeDataUUID    )
        
        if not serial_connected :
            _min = rpm-15
            _max = _min+15
            if (power>350):
                _min-=40
                _max-=40
            elif (power>240):
                _min-=20
                _max-=20
            elif (power<100):
                _min=90
                _max=110
            rpm = random.randrange(_min, _max, 2)
            power = makePower(rpm,0,0.004,0.4,0,autoGear(rpm))
            #logger.info(f"gear={gear}")
        
        session_data.append([time,power,rpm,hr,speed])
        if len(queue) > 0:
            if (queue[0][0]=='c'):
                logger.debug(queue)
                server.get_characteristic(queue[0][1]).value =  bytearray (queue[0][2])  
                server.update_value(     bc.sFitnessMachineUUID, queue[0][1]    )
                queue = queue[1:]
        if serial_connected:
            if (len(queue) ==0 and gained_control):
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
    if (len (session_data)<60):return
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


def pressed_keys(e):
    global gear
    global running
    if (e.event_type=='down'):
        if (e.name=='shift'):
            gear += 1
            if (gear>=14):gear = 13
            winsound.PlaySound('Kettler_gears\\'+str(gear+1)+'.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
        if (e.name=='ctrl') :
            gear -= 1
            if (gear<=-1):gear = 0
            winsound.PlaySound('Kettler_gears\\'+str(gear+1)+'.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
        if (e.name== 'space'):winsound.PlaySound('Kettler_gears\\'+str(gear+1)+'.wav',  winsound.SND_FILENAME | winsound.SND_ASYNC)
        if (e.name=='esc'):
            running=False


if __name__ == "__main__": 
    keyboard.hook(pressed_keys)
    print("                                      ")
    print("                                      ")
    print("                                      ")
    print("                    START             ")
    print("                                      ")
    print("                                      ")
    print("shift = gear up")
    print("ctrl  = gear down")
    print("esc   = stop")
    print("                                      ")
    print("                                      ")
    subprocess.call("powercfg -change -monitor-timeout-ac 180")
    subprocess.call("powercfg -change -standby-timeout-ac 180")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(repeater()) 
    except KeyboardInterrupt: 
        print('\r\n\r\nbreak')
    finally:
        subprocess.call("powercfg -change -monitor-timeout-ac 10")
        subprocess.call("powercfg -change -standby-timeout-ac 30")
        #createCSV(session_data)
        createTCX(session_data)
        winsound.PlaySound('Kettler_gears\\disconnected.wav',  winsound.SND_FILENAME)
        keyboard.unhook_all()
        