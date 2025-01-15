#-------------------------------------------------------------------------------
# Version info
#-------------------------------------------------------------------------------
__version__ = "2023-03-17"
# 2023-03-17    FortiusANT --> FortiusAnt
# 2021-04-28    If paused (> 5 minutes), close and start new TCX file.
#               Do not write is empty
# 2020-12-20    Constants used from constants.py
# 2020-11-15    Distance added to produce a valid TCX
# 2020-11-05    First version
#-------------------------------------------------------------------------------
import time
from   datetime         import datetime

from   constants                    import mode_Power, mode_Grade
#-------------------------------------------------------------------------------
# TCX template
#-------------------------------------------------------------------------------
TcxHeader =     '<?xml version="1.0" encoding="utf-8"?>\n' \
                '<TrainingCenterDatabase \n' \
                '    xsi:schemaLocation="http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2\n' \
                '        http://www.garmin.com/xmlschemas/TrainingCenterDatabasev2.xsd\n' \
                '        http://www.garmin.com/xmlschemas/ActivityExtension/v2\n' \
                '        http://www8.garmin.com/xmlschemas/ActivityExtensionv2.xsd"\n' \
                '        xmlns:ns5="http://www.garmin.com/xmlschemas/ActivityGoals/v1"\n' \
                '        xmlns:ns3="http://www.garmin.com/xmlschemas/ActivityExtension/v2"\n' \
                '        xmlns:ns2="http://www.garmin.com/xmlschemas/UserProfile/v2"\n' \
                '        xmlns="http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2"\n' \
                '        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n' \
                '>\n'

                #---------------------------------------------------------------
                # parameter 1  = id                 = string    Name of the activity
                #           2  = StartTime          = string    2020-11-02T18:31:34.796Z
                #           3  = TotalTimeSeconds   = integer   3321
                #           4  = DistanceMeters     = integer   28972
                #           5  = Calories           = integer   574
                #           6  = Intensity          = string    Active
                #           7  = Cadence            = integer   78
                #           8  = TriggerMethod      = string    Manual
                #           9  = AverageHeartRateBpm= integer   60
                #          10  = MaximumHeartRateBpm= integer   60
                #---------------------------------------------------------------
TcxActivities = '<Activities>\n' \
                '   <Activity Sport="Biking">\n' \
                '       <Id>%s</Id>\n' \
                '       <Notes>Generated by FortiusAnt</Notes>\n' \
                '       <Lap StartTime="%s">\n' \
                '           <TotalTimeSeconds>%s</TotalTimeSeconds>\n' \
                '           <DistanceMeters>%s</DistanceMeters>\n' \
                '           <Calories>%s</Calories>\n' \
                '           <Intensity>%s</Intensity>\n' \
                '           <Cadence>%s</Cadence>\n' \
                '           <TriggerMethod>%s</TriggerMethod>\n' \
                '           <AverageHeartRateBpm xsi:type="HeartRateInBeatsPerMinute_t"><Value>%s</Value></AverageHeartRateBpm>\n' \
                '           <MaximumHeartRateBpm xsi:type="HeartRateInBeatsPerMinute_t"><Value>%s</Value></MaximumHeartRateBpm>\n' \
                '           <Track>\n'

                #---------------------------------------------------------------
                # parameter 1  = Time               = string    2020-11-02T18:31:34.796Z
                #---------------------------------------------------------------
TcxTrackpoint = '               <Trackpoint>\n' \
                '                   <Time>%s</Time>\n'

                #---------------------------------------------------------------
                # parameter 1  = LatitudeDegrees    = float     51.909076690674
                # parameter 2  = LongitudeDegrees   = float     5.8606972694397
                #---------------------------------------------------------------
TcxPosition =   '                   <Position>\n' \
                '                       <LatitudeDegrees>%s</LatitudeDegrees>\n' \
                '                       <LongitudeDegrees>%s</LongitudeDegrees>\n' \
                '                   </Position>\n'

                #---------------------------------------------------------------
                # parameter 1  = AltitudeMeters     = float     6.123
                #---------------------------------------------------------------
TcxAltitude =   '                   <AltitudeMeters>%s</AltitudeMeters>\n'

                #---------------------------------------------------------------
                # parameter 1  = DistanceMeters     = float     010.987
                #---------------------------------------------------------------
TcxDistance =   '                   <DistanceMeters>%s</DistanceMeters>\n'

                #---------------------------------------------------------------
                # parameter 1  = HeartRateBpm       = integer   60
                #---------------------------------------------------------------
TcxHeartRate =  '                   <HeartRateBpm>\n' \
                '                       <Value>%s</Value>\n' \
                '                   </HeartRateBpm>\n'
                #---------------------------------------------------------------
                # parameter 1  = Cadence            = integer   70
                #---------------------------------------------------------------
TcxCadence =    '                   <Cadence>%s</Cadence>\n'

                #---------------------------------------------------------------
                # parameter 1  = Watts              = integer   71
                # parameter 2  = SPeed              = float     7.777
                #---------------------------------------------------------------
TcxWattSpeed =  '                   <Extensions>\n' \
                '                       <ns3:TPX>\n' \
                '                           <ns3:Watts>%s</ns3:Watts>\n' \
                '                           <ns3:Speed>%s</ns3:Speed>\n' \
                '                       </ns3:TPX>\n' \
                '                   </Extensions>\n'
                #---------------------------------------------------------------
TcxTpEnd =      '               </Trackpoint>\n'
                #---------------------------------------------------------------
TcxFooter =	    '           </Track>\n' \
                '       </Lap>\n' \
                '   </Activity>\n' \
                '</Activities>\n' \
                '</TrainingCenterDatabase>\n'

class clsTcxExport():
    def __init__(self):
        self.Start()

    def TcxTime(self, dt):
        #-----------------------------------------------------------------------
        # Return a time string in TCX format
        #-----------------------------------------------------------------------
        return dt.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"

    #---------------------------------------------------------------------------
    # S t a r t
    #---------------------------------------------------------------------------
    # Function      Initialize all variables so that trackpoints can be added.
    #
    # Output        self.variables
    #
    # Returns       none
    #---------------------------------------------------------------------------
    def Start(self):
        self.tcx                    = ''                # Contents for TCX file
        self.StartTime              = datetime.utcnow() # Start time of the track
        self.StartTimeSeconds       = time.time()
        self.TotalTimeSeconds       = 0
        self.TotalDistance          = 0
        self.TotalCalories          = 0
        self.SumCadence             = 0                 # To be averaged!
        self.NrCadence              = 1
        self.SumHeartRate           = 0                 # To be averaged!
        self.NrHeartRate            = 1
        self.HeartRateMax           = 0                 # Max of HeartRate
        self.NrTrackpoints          = 0                 # Count

        self.TrackpointXcalled      = 0                 # Last call to avoid too
                                                        #       many track points
        self.TrackpointXwritten     = 0                 # Last trackpoint written
                                                        #       to detect long break

        self.Distance               = 0                 # Temp field
        self.ElapsedTime            = 0

        self.TrackpointTime         = ''
        self.TrackpointDistance     = 0                 # Calculated on Tacx speed
        self.TrackpointAltitude     = 0                 # idem
        self.TrackpointSpeedKmh     = 0

        self.TrackpointHeartRate    = 0                 # Provided by Tacx
        self.TrackpointCadence      = 0
        self.TrackpointCurrentPower = 0

        #-----------------------------------------------------------------------
        # Note that we continue where we stopped!
        #-----------------------------------------------------------------------

    #---------------------------------------------------------------------------
    # T r a c k p o i n t X
    #---------------------------------------------------------------------------
    # Input         Time of previous call, CurrentPower
    #               In GradeMode: TargetGrade
    #
    # Function      A TacxTrainer knows about power, not altitude, position or distance.
    #               Using power and grade, the speed can be calculated
    #               Using the speed, the distance can be calculated
    #
    #               And THEN the trackpoint can be created
    #
    # Output        self.variables
    #
    # Returns       none
    #---------------------------------------------------------------------------
    def TrackpointX(self, TacxTrainer, HeartRate):
        TrackpointXcalled = time.time()
        self.ElapsedTime = TrackpointXcalled - self.TrackpointXcalled
        #-----------------------------------------------------------------------
        # Skip first call; without previous trackpoint no data
        #-----------------------------------------------------------------------
        if self.TrackpointXcalled == 0:
            self.TrackpointXcalled = TrackpointXcalled

        #-----------------------------------------------------------------------
        # Ignore if called within a second
        #-----------------------------------------------------------------------
        elif self.ElapsedTime < 1:
            pass

        #-----------------------------------------------------------------------
        # Create trackpoint
        #-----------------------------------------------------------------------
        else:
            self.TrackpointXcalled = TrackpointXcalled
            #-------------------------------------------------------------------
            # Calculate speed, based upon the current power and grade
            # (provided) Grade is used only in Power mode
            # TargetGrade is used in Grade mode anyway
            #-------------------------------------------------------------------
            TacxTrainer.Power2Speed(0)          # Assume flat ride (power mode)

            #-------------------------------------------------------------------
            # Calculate distance since previous call of TrackpointX
            # Note that distance is accumulated as long as trainer-fields are
            # equal to reduce number of trackpoints
            #-------------------------------------------------------------------
            d = TacxTrainer.CalculatedSpeedKmh * 1000 / 3600 * self.ElapsedTime
            self.Distance       += d

            #-------------------------------------------------------------------
            # Calculate altitude
            #-------------------------------------------------------------------
            if TacxTrainer.TargetMode == mode_Grade:
                self.TrackpointAltitude += d * TacxTrainer.TargetGrade / 100
            else:
                self.TrackpointAltitude = 0

            #-------------------------------------------------------------------
            # Write this trackpoint to the exportTCX file
            # (but avoid duplicate trainer data to reduce #trackpoints)
            #-------------------------------------------------------------------
            if     HeartRate                != self.TrackpointHeartRate        \
                or TacxTrainer.Cadence      != self.TrackpointCadence          \
                or TacxTrainer.CurrentPower != self.TrackpointCurrentPower:

                self.TrackpointDistance     = self.Distance
                self.TrackpointHeartRate    = HeartRate
                self.TrackpointCadence      = TacxTrainer.Cadence
                self.TrackpointCurrentPower = TacxTrainer.CurrentPower
                self.TrackpointSpeedKmh     = TacxTrainer.CalculatedSpeedKmh

                self.Trackpoint(None,                           \
                                None,                           \
                                self.TrackpointAltitude,        \
                                self.TrackpointDistance,        \
                                self.TrackpointHeartRate,       \
                                self.TrackpointCadence,         \
                                self.TrackpointCurrentPower,    \
                                self.TrackpointSpeedKmh)
                self.Distance = 0
                self.TrackpointXwritten = time.time()
            else:
                #---------------------------------------------------------------
                # If trackpoints are written and there is 5 minutes of 
                # inactivity, close and start new TCX file.
                #---------------------------------------------------------------
                if self.TrackpointXwritten > 0 and self.TrackpointXwritten < time.time() - 300:
                    self.Stop()     # Writes the TCX file
                                    # AND executes self.Start() to reinitialize

    #---------------------------------------------------------------------------
    # T r a c k p o i n t
    #---------------------------------------------------------------------------
    # Input         Parameters at current trackpoint
    #
    # Function      Add trackpoint
    #               Note that DISTANCE is the TotalDistance, not the distance
    #               since previous as expected.
    #               The speed (in tool reading the TCX) is calculated, using the
    #               elapsed time since previous trackpoint.
    #
    #               SpeedKmh is written in the trackpoint but seems unused (?)
    #
    # Output        self.tcx; trackpoint added
    #               self.variables incremented with trackpoint data
    #
    # Returns       none
    #---------------------------------------------------------------------------
    def Trackpoint(self, Latitude=None, Longitude=None, \
                         Altitude=None, Distance=None, HeartRate=None, \
                         Cadence=None,  Watts=None,    SpeedKmh=0):
        #-----------------------------------------------------------------------
        # Trackpoint calculations
        #-----------------------------------------------------------------------
        self.NrTrackpoints  += 1
        self.TrackpointTime  = self.TcxTime(datetime.utcnow())
        #-----------------------------------------------------------------------
        # Add trackpoint
        #-----------------------------------------------------------------------
        s = ''
        s += TcxTrackpoint % (self.TrackpointTime)
        if Latitude != None:    s += TcxPosition   % (float(Latitude), float(Longitude))
        if Altitude != None:    s += TcxAltitude   % (float(Altitude))

        if Distance != None:
                                self.TotalDistance += Distance
                                s += TcxDistance   % (float(self.TotalDistance))
        if HeartRate != None:
                                s += TcxHeartRate  % (int(HeartRate))
                                self.SumHeartRate += HeartRate
                                self.NrHeartRate  += 1
        if Cadence != None:     
                                s += TcxCadence    % (int(Cadence))
                                self.SumCadence += Cadence
                                self.NrCadence  += 1

        if Watts    != None:    s += TcxWattSpeed  % (int(Watts), float(SpeedKmh))

        if HeartRate != None and HeartRate > self.HeartRateMax:
                                self.HeartRateMax = HeartRate
        s += TcxTpEnd

        self.tcx += s.replace('   ', '\t')

    #---------------------------------------------------------------------------
    # S t o p
    #---------------------------------------------------------------------------
    # Input         self.tcx
    #
    # Function      Add the last pending trackpoint.
    #               write the *.tcx file
    #               reset all variables
    #
    # Output        self.variables
    #
    # Returns       none
    #---------------------------------------------------------------------------
    def Stop(self):
        #-----------------------------------------------------------------------
        # Write the last trackpoint to the exportTCX file
        #-----------------------------------------------------------------------
        if  self.Distance:
            self.Trackpoint(None,                           \
                            None,                           \
                            self.TrackpointAltitude,        \
                            self.TrackpointDistance,        \
                            self.TrackpointHeartRate,       \
                            self.TrackpointCadence,         \
                            self.TrackpointCurrentPower,    \
                            self.TrackpointSpeedKmh)

        #-----------------------------------------------------------------------
        # Track calculations
        #-----------------------------------------------------------------------
        self.TotalTimeSeconds = time.time() - self.StartTimeSeconds

        #-----------------------------------------------------------------------
        # Pre-pend the Activity totals
        #-----------------------------------------------------------------------
        self.tcx = TcxActivities % (self.TcxTime(self.StartTime) + ' @ FortiusAnt ' , \
                                    self.TcxTime(self.StartTime), \
                                    int(self.TotalTimeSeconds), \
                                    int(self.TotalDistance), \
                                    int(self.TotalCalories), \
                                    'Active', \
                                    int(self.SumCadence   / self.NrCadence), \
                                    'Manual', \
                                    int(self.SumHeartRate / self.NrHeartRate), \
                                    int(self.HeartRateMax) \
                                   ) + \
                    self.tcx

        #-----------------------------------------------------------------------
        # Pre-pend header and append footer
        #-----------------------------------------------------------------------
        self.tcx = TcxHeader + self.tcx + TcxFooter

        #-----------------------------------------------------------------------
        # Write tcx file
        #-----------------------------------------------------------------------
        if True:
            filename = 'Kettler.' + self.StartTime.strftime('%Y-%m-%d %H-%M-%S') + ".tcx"
            tcxFile = open(filename,"w+")
            tcxFile.write(self.tcx)
            tcxFile.close

        #-----------------------------------------------------------------------
        # Cleanup
        #-----------------------------------------------------------------------
        self.Start()

if __name__ == "__main__":
    tcx = clsTcxExport()
    tcx.Start()                                             # Optional
    tcx.Trackpoint(HeartRate=78, Cadence=123, Watts=456)
    tcx.Trackpoint(HeartRate=78, Cadence=123, Watts=456)
    tcx.Stop()