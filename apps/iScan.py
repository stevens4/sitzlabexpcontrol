#this must be run first before importing the alternate reactor to avoid a 
#conflict between the qt and twisted reactors
import sys
from PySide import QtGui, QtCore
if QtCore.QCoreApplication.instance() is None:
    app = QtGui.QApplication(sys.argv)
    import qt4reactor
    qt4reactor.install() 

# for server calls
from twisted.internet.defer import inlineCallbacks, Deferred, returnValue
from ab.abclient import getProtocol

# configuration parameters
from config.serverURLs import *

# some utility widgets
from qtutils.dictcombobox import DictComboBox
from qtutils.layout import SqueezeRow
from qtutils.label import LabelWidget

# fun
from functools import partial

# saving scan data
from filecreationmethods import saveCSV, filenameGen, checkPath
from config.filecreation import POOHDATAPATH

# ID info for different apps
from daqmx.task.ai import VoltMeter

# clients
from steppermotor.steppermotorclient import ChunkedStepperMotorClient
from voltmeter.voltmeterclient import VoltMeterClient
from steppermotor.wavelengthclient import WavelengthClient
from delaygenerator.delaygeneratorclient import DelayGeneratorClient
from steppermotor.polarizerclient import PolarizerClient

# math
import numpy as np

# plotting
from pyqtgraph import PlotWidget, ErrorBarItem, mkPen, LegendItem, ScatterPlotItem

# waiting during a pause
from time import sleep

# for preserving data order and integrity
from collections import OrderedDict

# for making directory manipulations be OS indep.
from os.path import join


DEBUG = len(sys.argv) > 1 and sys.argv[1] == 'debug'


DEFAULT_SPINBOX_SETUP = {
    'rangeMin':-100000,
    'rangeMax':100000,
    'stepMin':1,
    'stepMax':100000,
    'startInit':0,
    'stopInit':10,
    'stepSizeInit':1,
    'stepMax':10000
}

'''
implements InputWidget interface.
__init__(scan_input_generator,next_agent,cancel_agent)

'''
class InputWidget(QtGui.QWidget):
    def __init__(
        self,
        getPositionCommand,
        setPositionCommand,
        cancelCommand = lambda:None,
        enableSpinBoxes = True,
        spinBoxProperties = DEFAULT_SPINBOX_SETUP
        ):
                
        QtGui.QWidget.__init__(self)
        self.getPosition = getPositionCommand
        self.setPosition = setPositionCommand
        self.cancelCommand = cancelCommand
        self.enableSpinBoxes = enableSpinBoxes
        self.spinBoxProperties = spinBoxProperties
        
        self.scanValues = []
        
        self.initWidget()
        
    def initWidget(self):
        # handy function to test if getPosition is a lambda function (i.e. doesn't actually step anything; e.g. manual scan)
        def isALambda(v):
            LAMBDA = lambda:0
            return isinstance(v, type(LAMBDA)) and v.__name__ == LAMBDA.__name__
        
        # set up the layout for a general input tab
        layout = QtGui.QFormLayout()
        self.setLayout(layout)
        
        # set up the center scan button
        @inlineCallbacks
        def centerScanClicked():
            # get current position
            currPosition = yield self.getPosition()
            
            # find width of scan
            oldStart = self.startSpin.value()
            oldStop = self.stopSpin.value()
            width = oldStop - oldStart
            
            newStart = currPosition - .5*width
            newStop = currPosition + .5*width
            
            self.startSpin.setValue(newStart)
            self.stopSpin.setValue(newStop)
            
        centerButton = QtGui.QPushButton('center scan')        
        centerButton.clicked.connect(centerScanClicked)
        layout.addRow(centerButton)
        
        # disable the center button if there isn't a setPosition command (e.g. manual scan)
        if isALambda(self.getPosition):
            centerButton.setDisabled(True)
        
        # set up spinboxes for start, stop, stepSize
        if self.spinBoxProperties['stepMin'] < 1:
            self.startSpin = QtGui.QDoubleSpinBox()
            self.stopSpin = QtGui.QDoubleSpinBox()
            self.stepSizeSpin = QtGui.QDoubleSpinBox()
        else:
            self.startSpin = QtGui.QSpinBox()
            self.stopSpin = QtGui.QSpinBox()
            self.stepSizeSpin = QtGui.QSpinBox()
        
        self.startSpin.setMinimum(self.spinBoxProperties['rangeMin'])
        self.stopSpin.setMinimum(self.spinBoxProperties['rangeMin'])
        self.stepSizeSpin.setMinimum(self.spinBoxProperties['stepMin'])
        self.stepSizeSpin.setMaximum(self.spinBoxProperties['stepMax'])
        self.startSpin.setMaximum(self.spinBoxProperties['rangeMax'])
        self.stopSpin.setMaximum(self.spinBoxProperties['rangeMax'])
        
        # set initial spinbox values
        self.startSpin.setValue(self.spinBoxProperties['startInit'])
        self.stopSpin.setValue(self.spinBoxProperties['stopInit'])
        self.stepSizeSpin.setValue(self.spinBoxProperties['stepSizeInit'])
        
        # add the spinboxes to the layout
        if self.enableSpinBoxes:
            layout.addRow('start', self.startSpin)
            layout.addRow('stop', self.stopSpin)
            layout.addRow('step size', self.stepSizeSpin)

    def initScan(self): 
        startValue = self.startSpin.value()
        stopValue = self.stopSpin.value()
        stepSizeValue = self.stepSizeSpin.value()
        
        if startValue > stopValue:
            stepSizeValue = -1*stepSizeValue
        
        self.scanValues = np.arange(startValue,stopValue,stepSizeValue)
        self.scanValuesArray = np.append(self.scanValues,stopValue)
        self.scanValues = self.scanValuesArray.tolist()
    
    def initFlipScan(self):
        self.scanValues = self.scanValuesArray.tolist()
        self.scanValues = self.scanValues[::-1]
    
    def checkIfDone(self):
        self.done = len(self.scanValues) == 0
        
    @inlineCallbacks
    def next(self):
        self.checkIfDone()
        if not self.done:
            newPosition = self.scanValues.pop(0)
            result = yield self.setPosition(newPosition)
            if result == True:  #workaround for DG which returns True if successful
                result = newPosition
            returnValue(result)
        if self.done:
            returnValue(None)
        
    @inlineCallbacks
    def cancel(self):
        for i in range(len(self.scanValues)):
            self.scanValues.pop()
        yield self.cancelCommand()
        return


        

'''
-> __init__(voltmeterClient)
takes in client to voltmeter server, returns object that implements \
the ScanOutput interface. Can specify channel to read from, and number \
of shots to read for each output. Returns mean value over shots as well \
as estimator of error on the mean.

'''
class VoltMeterOutputWidget(QtGui.QWidget):
    def __init__(self,voltmeterClient):                         
        QtGui.QWidget.__init__(self)
        layout = QtGui.QFormLayout()
        self.setLayout(layout)            

        # let user pick which channel to measure
        channels_dict = {}
        channels_combo = DictComboBox()        
        layout.addRow('channel',channels_combo)        

        # let user select number of shots to acquire
        shots_spin = QtGui.QSpinBox()
        shots_spin.setRange(1,1000)
        shots_spin.setValue(10)
        layout.addRow('shots',shots_spin)

        # populate combo box with available channels
        def on_channels(channels):
            if not channels: return
            channel = channels.pop()           
            def on_description(description):
                channels_dict[channel] = '%s\t(%s)' % (description,channel)
                if channels:
                    on_channels(channels)
                else:
                    channels_combo.updateCombo(channels_dict)
                    self.setEnabled(True)
            voltmeterClient.getChannelParameter(
                channel,VoltMeter.DESCRIPTION                
            ).addCallback(on_description)
        voltmeterClient.getChannels().addCallback(on_channels)

        # don't enable widget until we get all the channels
        self.setEnabled(False)
        
        self.voltmeterClient = voltmeterClient
        self.channels_combo = channels_combo
        self.shots_spin = shots_spin
        self._cancel = False
        self.plot = None
        self.plotterXVal = 0
    
    def setPlotterXVal(self,val):
        self.plotterXVal = val
        
    def initScan(self):
        self._cancel = False
    
    def startAcquisition(self):
        # measure current selected channel
        channel = self.channels_combo.getCurrentKey()

        # measure for specified shot number
        shots = self.shots_spin.value()
        
        voltagesList = []
        d = Deferred()
        def onVoltages(voltagesDict):
            # push acquired value to list
            thisVoltage = voltagesDict[channel]
            
            # every 2 voltages, put spots down on the plot for this measurement
            if len(voltagesList) % 2 == 1 and len(voltagesList) != 0:
                xVals = [self.plotterXVal,self.plotterXVal]
                yVals = [voltagesList[len(voltagesList)-1],thisVoltage]
                self.plot.addItem(ScatterPlotItem(x=xVals,y=yVals,pen='r',size=12))
            
            
            voltagesList.append(thisVoltage)
            
            
            # did someone call cancel? if so, unsubscribe and return none
            if self._cancel:
                self.voltmeterClient.removeListener(onVoltages)
                d.callback(None)
            
            # if we're done unsubscribe and send back the voltage list
            if len(voltagesList) is shots:
                self.voltmeterClient.removeListener(onVoltages)
                d.callback(voltagesList)
        # sign up for messages about new acquisitions
        self.voltmeterClient.addListener(onVoltages)
        return d
    
    def cancel(self):
        # if acquiring, quit on next acquisition
        #self.voltmeterClient.removeListener(onVoltages)
        self._cancel = True


# put together the interface
class SmartScanGUI(QtGui.QWidget):
    # oh god i'm so sorry. don't listen to him; he's never sorry.
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.setLayout(QtGui.QHBoxLayout())
        
        # a dict to store data
        self.dataMatrix = {}
        self.refData = {}
        
        ########################################## create a plot and associated widget ###########################################################
        self.plotWidget = PlotWidget()
        self.layout().addWidget(self.plotWidget,1)
        
        def xYPlot(plotWidget,x,y,yerr=None,xerr=None,color='w',name='Current'):
            thisPlot = plotWidget.plot(x,y,pen=mkPen(color,width=2))
            plotWidget.addItem(
                ErrorBarItem(
                    x=np.asarray(x),
                    y=np.asarray(y),
                    top=np.asarray(yerr) if yerr is not None else None,
                    bottom=np.asarray(yerr) if yerr is not None else None,
                    left=np.asarray(xerr) if xerr is not None else None,
                    right=np.asarray(xerr) if xerr is not None else None,
                    beam=.05,
                    pen=mkPen(color)
                )
            )
            
        # method for updating plot with new data (plot has to be cleared first)
        def updatePlot():
            self.plotWidget.clear()
            for name, rData in self.refData.iteritems():
                xYPlot(
                    self.plotWidget,
                    rData['data'][0],
                    rData['data'][1],
                    yerr=rData['data'][2],
                    color=rData['color'],
                    name=name
                )
            if len(self.dataMatrix.keys()) >= 1:
                for xVal, yValues in self.dataMatrix.items():
                    if xVal in self.xVals:
                        oldMean = self.yVals[self.xVals.index(xVal)]
                        thisMean = np.mean(yValues)
                        newMean = (oldMean+thisMean)/2.
                        self.yVals[self.xVals.index(xVal)] = newMean
                        newErr = np.std(yValues)/np.sqrt(len(yValues))
                        self.errVals[self.xVals.index(xVal)] = newErr
                    else:
                        self.xVals.append(xVal)
                        mean = np.mean(yValues)
                        self.yVals.append(mean)
                        err = np.std(yValues)/np.sqrt(len(yValues))
                        self.errVals.append(err)
                xYPlot(self.plotWidget,self.xVals,self.yVals,yerr=self.errVals) 

        
        ############################################## configure a control panel layout ########################################################
        cpLayout = QtGui.QVBoxLayout()
        self.layout().addLayout(cpLayout)

        # configure the output widget
        outputPane = QtGui.QTabWidget()
        cpLayout.addWidget(LabelWidget('output',outputPane))


            
        @inlineCallbacks
        def onInit():
            ############################################################# VOLTMETER OUTPUT ###########################################################
            # add volt meter to scan output
            if DEBUG:
                vmURL = TEST_VOLTMETER_SERVER
            else:
                vmURL = VOLTMETER_SERVER
            vmProtocol = yield getProtocol(vmURL)
            vmClient = VoltMeterClient(vmProtocol)    
            vmWidget = VoltMeterOutputWidget(vmClient)
            outputPane.addTab(vmWidget,'voltmeter')


            ############################################################# BEGIN INPUTS ###########################################################

            # configure the input widget
            inputPane = QtGui.QTabWidget()
            inputPane.setTabPosition(inputPane.West)
            cpLayout.addWidget(LabelWidget('input',inputPane),1)    

            
            # algorithm for scan inputs is:
            # 0. check to see if input is disabled
            # 1. create client for server from protocol object
            # 2. create combo widget to hold interval and list widgets
            # 3. create interval widget using client object, add to combo
            # 4. same for list widget
            # 5. add combo widget to base combo widget (resulting in 2-D tab widget)

            
            ############################################################# MANUAL INPUT ###########################################################
            class ManualInputWidget(QtGui.QWidget):
                def __init__(self,parentWidget):
                    QtGui.QWidget.__init__(self)
                    self.parentWidget = parentWidget
                    self.done = False
                
                def initScan(self):
                    return
                
                def checkIfDone(self):
                    return
                
                def next(self):
                    result, valid = QtGui.QInputDialog.getDouble(
                        self.parentWidget, 
                        'next x value', 
                        'enter next x value',
                        decimals=6
                    )
                    if valid:
                        return result
                    else:
                        return None
                
                def cancel(self):
                    return

            inputPane.addTab(
                ManualInputWidget(self),
                'manual'
            )
            
            
            
            ############################################################# MANUAL SCAN INPUT ###########################################################
            
            class ManualScanInputWidget(InputWidget):
                def __init__(self,parent):
                    spinBoxProps = {
                        'rangeMin':-100000,
                        'rangeMax':100000,
                        'stepMin':.000001,
                        'stepMax':100000,
                        'startInit':0,
                        'stopInit':1,
                        'stepSizeInit':.1
                    }
                    
                    def setPosition(position):
                        msgBox = QtGui.QMessageBox()
                        msgBox.setText("next position:\t"+str(position))
                        msgBox.setStandardButtons(QtGui.QMessageBox.Ok | QtGui.QMessageBox.Cancel)
                        msgBox.setDefaultButton(QtGui.QMessageBox.Ok)
                        ret = msgBox.exec_()
                        if ret == QtGui.QMessageBox.Ok:
                            return position
                        elif ret == QtGui.QMessageBox.Cancel:
                            return None
                    
                    InputWidget.__init__(self,lambda(x):None,setPosition,spinBoxProperties = spinBoxProps)
                    
            inputPane.addTab(
                ManualScanInputWidget(self),
                'manual scan'
            )

            
            ############################################################# STEPPER MOTOR INPUTS ###########################################################
            
            # load in the stepper motor names
            from config.steppermotor import KDP, BBO, PDL, LID, POL 
            
            # get stepper motor protocol (how to communicate to stepper motor server)
            smProtocol = yield getProtocol(
                TEST_STEPPER_MOTOR_SERVER if DEBUG else STEPPER_MOTOR_SERVER 
            )
            
            # define a chunked (cancellable) client for each stepper motor
            stepperMotorsClients = {}
            for stepperMotorName in (KDP,BBO,PDL,LID,POL):
                stepperMotorsClients[stepperMotorName] = ChunkedStepperMotorClient(smProtocol,stepperMotorName)

            # define an input widget for each stepper motor, each add to input pane
            for smID,smClient in stepperMotorsClients.items():
                spinBoxProps = {
                    'rangeMin':-100000,
                    'rangeMax':100000,
                    'stepMin':1,
                    'stepMax':100000,
                    'startInit':0,
                    'stopInit':100,
                    'stepSizeInit':1
                }
                
                thisInputWidget = InputWidget(
                    smClient.getPosition,
                    smClient.setPosition,
                    cancelCommand = smClient.cancel,
                    spinBoxProperties = spinBoxProps
                )
            
                inputPane.addTab(
                    thisInputWidget,
                    smID
                )
            
            
            
            ############################################################# WAVELENGTH SERVER INPUT ###########################################################

            '''
            # add wavelength client to scan input
            wlProtocol = yield getProtocol(
                TEST_WAVELENGTH_SERVER if DEBUG else WAVELENGTH_SERVER
            )
            wlClient = WavelengthClient(wlProtocol)
            
            spinBoxProps = {
                'rangeMin':24100,
                'rangeMax':25000,
                'stepMin':.01,
                'stepMax':900,
                'startInit':24200,
                'stopInit':24220,
                'stepSizeInit':1
            }
           
            wlInputWidget = InputWidget(
                    wlClient.getWavelength,
                    wlClient.setWavelength,
                    cancelCommand = wlClient.cancelWavelengthSet,
                    spinBoxProperties = spinBoxProps
            )

            inputPane.addTab(
                wlInputWidget,
                'wl'
            )
            '''
            
            
            
            ############################################################# POLARIZER SERVER INPUT ###########################################################
            '''
            # get protocol
            polProtocol = yield getProtocol(
                TEST_POLARIZER_SERVER if DEBUG else POLARIZER_SERVER
            )
            
            # define the client
            polClient = PolarizerClient(polProtocol)
            
            # set limits on spinboxes
            spinBoxProps = {
                'rangeMin':-720,
                'rangeMax':720,
                'stepMin':.01,
                'stepMax':720,
                'startInit':0,
                'stopInit':90,
                'stepSizeInit':5
            }
            
            polInputWidget = InputWidget(
                    polClient.getAngle,
                    polClient.setAngle,
                    cancelCommand = polClient.cancelAngleSet,
                    spinBoxProperties = spinBoxProps
                )

            inputPane.addTab(
                polInputWidget,
                'pol'
            )
            '''
            
            
            
            ############################################################# DDG INPUTS ###########################################################
            
            # load in the delay generator names
            from config.delaygenerator import MAV_PUMP_LAMP, MAV_PUMP_QSW, MAV_PROBE_LAMP, MAV_PROBE_QSW, MAV_NOZZLE
            
            # load in the delay generator limits
            from config.delaygenerator import MIN_DELAY, MAX_DELAY, DELAY_RES
            
            # get the delay generator protocol
            dgProtocol = yield getProtocol(
                TEST_DELAY_GENERATOR_SERVER if DEBUG else DELAY_GENERATOR_SERVER
            )
            dgClient = DelayGeneratorClient(dgProtocol)
            
            # define an input widget for each delay generator, each add to input pane 
            for dgName in (MAV_PUMP_LAMP, MAV_PUMP_QSW, MAV_PROBE_LAMP, MAV_PROBE_QSW, MAV_NOZZLE):
                
                spinBoxProps = {
                    'rangeMin':MIN_DELAY,
                    'rangeMax':MAX_DELAY,
                    'stepMin':DELAY_RES,
                    'stepMax':MAX_DELAY-MIN_DELAY,
                    'startInit':1,
                    'stopInit':1001,
                    'stepSizeInit':10
                }

                # because there is one DG client for all DGs (unlike SM client), we use partial to map the client\
                # commands to individual DGs so that we preserve as much of the same structure as possible.
                thisInputWidget = InputWidget(
                    partial(dgClient.getDelay,dgName),
                    partial(dgClient.setPartnerDelay,dgName),
                    spinBoxProperties = spinBoxProps
                )
            
                inputPane.addTab(
                    thisInputWidget,
                    dgName
                )
                
            ############################################################# END INPUTS ###########################################################


            ############################################################# SCANNING ###########################################################

            #define fundamental scan logic:
            #   1: ask independent variable to change, wait
            #   2: measure dependent
            #   3: perform onStep task with x-y pair (e.g. update plot)
            
            class Scan:
                def __init__(self,inputWidget,outputWidget,onStepFunct,repeats=1,plotWidget=None):
                    self.input = inputWidget
                    self.output = outputWidget
                    self.onStep = onStepFunct
                    self.output.plot = plotWidget
                    self.repeatTotal = repeats
                    self.activeRepeat = 1
                    self.startScan()
                    
                def startScan(self):
                    if len(self.input.scanValues) == 0: self.input.initScan()
                    self.output.initScan()
                    self.paused = False
                    self.done = False
                    self.loop()

                @inlineCallbacks
                def loop(self):
                    # pause if paused
                    if self.paused:
                        resumeCancelDialog = QtGui.QMessageBox()
                        resumeCancelDialog.setText("the scan has been paused.")
                        resumeCancelDialog.setInformativeText("do you want to cancel the scan?")
                        resumeCancelDialog.setStandardButtons(QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
                        resumeCancelDialog.setDefaultButton(QtGui.QMessageBox.No)
                    
                        option = resumeCancelDialog.exec_()
                        
                        if option == QtGui.QMessageBox.Yes:
                            #quit the scan
                            self.cancel()
                            return
                            
                        if option == QtGui.QMessageBox.No:
                            #resume the scan
                            self.resume()
                    
                        while self.paused:
                            QtGui.QApplication.processEvents()
                            sleep(.1)
                        
                    # check if done by asking input
                    self.input.checkIfDone()
                    inputDone = self.input.done
                    
                    # if done, finish to clean up
                    if inputDone:
                        self.finish()
                    
                    # if not done, continue onto next point & measure
                    if not inputDone:
                        inputData = yield self.input.next()
                        if inputData == None:
                            yield self.cancel()
                            return
                        else:
                            self.output.setPlotterXVal(inputData)
                            outputDataDefer = self.output.startAcquisition()
                            outputData = yield outputDataDefer
                            yield self.onStep(inputData,outputData)
                            self.loop()
                    
                def pause(self):
                    self.paused = True
                
                def resume(self):
                    self.paused = False
                    
                def cancel(self):
                    self.done = True
                    self.input.cancel()
                    self.output.cancel()
                    scanToggleClicked()
                    
                @inlineCallbacks    
                def finish(self):
                    if self.activeRepeat == self.repeatTotal:
                        self.done = True
                        yield self.input.cancel()
                        self.output.cancel()
                        scanToggleClicked()
                    else:
                        self.activeRepeat += 1
                        if self.activeRepeat % 2 == 0:
                            self.input.initFlipScan()
                        else:
                            self.input.initScan()
                        self.startScan()

            # define what to do after values are acquired at a position
            def onStepped(input,output):
                # unpack scan step data
                position, output = input, output
                if position == None or output == None: 
                    return
                else:
                    # update data array
                    if position not in self.dataMatrix.keys():
                        self.dataMatrix[position] = []
                    for value in output:
                        self.dataMatrix[position].append(value)

                    # update plot
                    updatePlot()
               
            # define scanning start/pause/cancel logic
            def scanToggleClicked():
                if self.scanning:
                    #currently scanning so check if the scan is done, if not this was a pause
                    if not self.thisScan.done:
                        #pause the scan, pop resume/cancel dialog
                        self.thisScan.pause()

                    else:
                        self.scanning = False
                        scanToggleButton.setText("start")
                        return
                    
                if not self.scanning:
                    #not currently scanning, so start the scan
                    self.scanning = True
                    
                    #gather the agents (classes with specific methods) that progress scan and measure values
                    inputAgent = inputPane.currentWidget()
                    outputAgent = outputPane.currentWidget()
                    updateAgent = onStepped
                    
                    #dump whatever data is in the matrix, prepare the plot
                    for xVal in self.dataMatrix.keys():
                        del self.dataMatrix[xVal]
                    
                    # clear the lists that the plot uses to plot
                    self.xVals = []
                    self.yVals = []
                    self.errVals = []

                    updatePlot()
                    self.plotWidget.enableAutoRange()
                    
                    #define the scan, which automatically starts it
                    numToRepeat = self.repeatSpinBox.value()
                    self.thisScan = Scan(inputAgent, outputAgent, updateAgent, numToRepeat, self.plotWidget)
                    
                    #rename our button so users know about the other half of this function
                    scanToggleButton.setText("pause/cancel")
                
                return
            
            self.scanning = False
            
            # set up the GUI to have the scan start/pause/cancel button and repeat spinbox    
            scanPane = QtGui.QHBoxLayout()
            scanToggleButton = QtGui.QPushButton("start")
            scanToggleButton.clicked.connect(scanToggleClicked)
            scanPane.addWidget(scanToggleButton)
            
            self.repeatSpinBox = QtGui.QSpinBox()
            self.repeatSpinBox.setRange(1,10000)
            self.repeatSpinBox.setValue(1)
            scanPane.addWidget(self.repeatSpinBox)
            
            cpLayout.addWidget(LabelWidget('scan',scanPane))


            ############################################################# LOAD FUNCTIONS ###########################################################

            refLayout = QtGui.QHBoxLayout()
            
            def onLoadClicked():
                dir, filePrefix = filenameGen()
                dir = join(POOHDATAPATH,dir)
                
                refFileName = QtGui.QFileDialog.getOpenFileName(self,'select file', dir,"CSV Files (*.csv)")
                
                rData = np.loadtxt(open(refFileName[0],"rb"),delimiter=",")
                name = refFileName[0].rpartition('/')[2]
                
                color = QtGui.QColorDialog.getColor()
                
                if 'matrix' in refFileName[0]:
                    xVals = rData[:,0]
                    yVals = []
                    errVals = []
                    for rowNum in range(len(xVals)):
                        thisYList = rData[rowNum,1:]
                        yVals.append(np.mean(thisYList))
                        errVals.append(np.std(thisYList)/np.sqrt(len(thisYList)))
                    self.refData[name] = {
                        'color': color,
                        'data': [xVals, yVals, errVals]
                    }
                else:
                    self.refData[name] = {
                        'color': color,
                        'data': [rData[:,0], rData[:,1], rData[:,2]]
                    }
                
                updatePlot()
            
            loadButton = QtGui.QPushButton('load')
            loadButton.clicked.connect(onLoadClicked)
            refLayout.addWidget(SqueezeRow(loadButton))

            def onClearClicked():
                for refs in self.refData.keys():
                    del self.refData[refs]
                    
                updatePlot()

            clearButton = QtGui.QPushButton('clear all')
            clearButton.clicked.connect(onClearClicked)
            refLayout.addWidget(SqueezeRow(clearButton))

            cpLayout.addWidget(LabelWidget('reference',refLayout))    

            
            ############################################################# SAVE FUNCTIONS ###########################################################
            
            saveLayout = QtGui.QHBoxLayout()
            
            def onSaveRawClicked():
                dataType = np.dtype(np.float32)
                orderedDataDict = OrderedDict(sorted(self.dataMatrix.items()))
                data = np.reshape(np.asarray(orderedDataDict.keys(),dtype=dataType),(len(orderedDataDict.keys()),1)) #just x values as a column
                yVals = np.asarray(orderedDataDict.values(),dtype=dataType)
                data = np.hstack((data,yVals))
                saveFile(data,'matrix')
            saveRawButton = QtGui.QPushButton('save (raw)')
            saveRawButton.clicked.connect(onSaveRawClicked)
            saveLayout.addWidget(SqueezeRow(saveRawButton))
            
            def onSaveStatsClicked():
                xData = self.dataMatrix.keys()
                yData = []
                errData = []
                for rawValues in self.dataMatrix.values():
                    yData.append(np.mean(rawValues))
                    errData.append(np.std(rawValues)/np.sqrt(len(rawValues)))
                data = np.asarray([xData, yData, errData], dtype=np.dtype(np.float32))
                saveFile(np.transpose(data),'stats')
            saveStatsButton = QtGui.QPushButton('save (stats)')
            saveStatsButton.clicked.connect(onSaveStatsClicked)
            saveLayout.addWidget(SqueezeRow(saveStatsButton))
                
            def saveFile(dataToSave,prefix):
                dir, filePrefix = filenameGen()
                dir = join(POOHDATAPATH,dir)
                checkPath(dir)
                subDir = QtGui.QFileDialog.getExistingDirectory(self,'select folder', dir)
                desc, valid = QtGui.QInputDialog.getText(self, 'enter file description','description' )
                
                if not valid:
                    desc = None
                else:
                    saveCSV(dataToSave, subDir=subDir, description=prefix+'_'+desc)
            
            cpLayout.addWidget(LabelWidget('save',saveLayout))    

        onInit()

    def closeEvent(self, event):
        if reactor.running: reactor.stop()
        event.accept()

if __name__ == '__main__':
    from twisted.internet import reactor
    container = []
    ssWidget = SmartScanGUI()
    ssWidget.show()
    container.append(ssWidget)
    reactor.run()