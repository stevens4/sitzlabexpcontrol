#this must be run first before importing the alternate reactor to avoid a 
#conflict between the qt and twisted reactors
import sys
from PySide import QtGui, QtCore
if QtCore.QCoreApplication.instance() is None:
    app = QtGui.QApplication(sys.argv)
    import qt4reactor
    qt4reactor.install() 

from twisted.internet.defer import inlineCallbacks, Deferred, returnValue

from qtutils.toggle import ToggleObject, ClosedToggle
from qtutils.dictcombobox import DictComboBox

from functools import partial

from sitz import compose, STEPPER_MOTOR_SERVER, TEST_STEPPER_MOTOR_SERVER, WAVELENGTH_SERVER

from filecreationmethods import saveCSV

from scan import Scan

from scan.widget import ScanToggleObject, IntervalScanInputWidget, ListScanInputWidget
from scan.input import IntervalScanInput, ListScanInput

from voltmeter.voltmeterclient import VoltMeterClient

from ab.abclient import getProtocol    

from config.steppermotor import SM_CONFIG, KDP, BBO, PDL
from config.voltmeter import VM_SERVER_CONFIG, VM_DEBUG_SERVER_CONFIG
from config.scantypes import SCAN_TYPES
from config.filecreation import POOHDATAPATH

from steppermotor.steppermotorclient import ChunkedStepperMotorClient

from math import pow
from numpy import asarray

DEBUG = len(sys.argv) > 1 and sys.argv[1] == 'debug'

'''
created by stevens4 on 2013/06/27

provides a gui built out of other components for scanning specific 
positions on a stepper motor, eg. SmartScan of PDL while observing
ion signal

'''
@inlineCallbacks
def SmartScanGUI():
    
    vmProtocol = yield getProtocol(
        (VM_DEBUG_SERVER_CONFIG if DEBUG else VM_SERVER_CONFIG)['url']
    )
    smProtocol = yield getProtocol(
        TEST_STEPPER_MOTOR_SERVER if DEBUG else STEPPER_MOTOR_SERVER 
    )
    wlProtocol = yield getProtocol(WAVELENGTH_SERVER)
    
    #configure a layout for the plot widget & controls to go side by side on
    widget = QtGui.QWidget()
    container.append(widget)
    widget.show()
    layout = QtGui.QHBoxLayout()
    widget.setLayout(layout)
    
    # create a plot and associated widget
    from pyqtgraph import PlotWidget, ErrorBarItem
    plotWidget = PlotWidget()
    plot = plotWidget.plot()
    layout.addWidget(plotWidget,1)
    
    # configure a control panel layout
    cpLayout = QtGui.QFormLayout()
    layout.addLayout(cpLayout)


    #add a combobox for the voltmeters populated by a server request result
    vmClient = VoltMeterClient(vmProtocol)
    channels = yield vmClient.getChannels()
    vmCombo = DictComboBox({channel:channel for channel in channels})
    cpLayout.addRow('voltmeter',vmCombo)

    # create dictionary of agents
    # <BLACK BOX>
    A_KDP, A_BBO, A_PDL, A_SURF = 0,1,2,3
    AGENTS = (A_KDP,A_BBO,A_PDL,A_SURF)
    SET_POSITION, GET_POSITION, CANCEL, PARAMETERS, NAME = 0,1,2,3,4
    ISIW = IntervalScanInputWidget
    agents = {
                 {
                     KDP:A_KDP,
                     BBO:A_BBO,
                     PDL:A_PDL
                 }[smID]:{
                     SET_POSITION:compose(
                         smClient.setPosition,
                         compose(
                             int,
                             round
                         )
                     ),
                     GET_POSITION:smClient.getPosition,
                     CANCEL:smClient.cancel,
                     PARAMETERS:{
                         ISIW.MIN:-99999,
                         ISIW.MAX:99999,
                         ISIW.PRECISION:0
                     },
                     NAME:SM_CONFIG[smID]['name']
                 }
        for smID, smClient in
        (
            (smID,ChunkedStepperMotorClient(smProtocol,smID))
            for smID in
            SM_CONFIG.keys()
        )
    }
    @inlineCallbacks
    def wavelengthAgent(position):
        yield wlProtocol.sendCommand('set-wavelength',position)
        wavelength = yield wlProtocol.sendCommand('get-wavelength')
        returnValue(wavelength)
    agents.update(
        {
            A_SURF:{ 
                SET_POSITION:wavelengthAgent,
                GET_POSITION:partial(
                    wlProtocol.sendCommand,
                    'get-wavelength'
                ),
                CANCEL:partial(
                    wlProtocol.sendCommand,
                    'cancel-wavelength-set'
                ),
                PARAMETERS:{
                    ISIW.MIN:24100.0,
                    ISIW.MAX:24500.0,
                    ISIW.PRECISION:2
                },
                NAME:'surf'
            }
        }
    )
    # </BLACK BOX>    
    
    #create a tab widget for list scan & interval scan to go on
    agentTabs = QtGui.QTabWidget()
    agentTabs.setTabPosition(QtGui.QTabWidget.West)
    cpLayout.addRow('',agentTabs)
    for agentKey in AGENTS:
        inputTabs = QtGui.QTabWidget()
        intervalWidget = IntervalScanInputWidget()
        for spin in ISIW.PARAMETERS:
            intervalWidget.setParameter(spin,ISIW.PRECISION,agents[agentKey][PARAMETERS][ISIW.PRECISION])
            if spin in (ISIW.START,ISIW.STOP):
                for param in (ISIW.MIN,ISIW.MAX):
                    intervalWidget.setParameter(spin,param,agents[agentKey][PARAMETERS][param])
        listWidget = ListScanInputWidget()
        INTERVAL, LIST = 0, 1
        INPUTS = (INTERVAL,LIST)
        for input in INPUTS:
            inputTabs.addTab(
                *{
                    INTERVAL:(intervalWidget,'interval'),
                    LIST:(listWidget,'list')
                }[input]
            )
        agentTabs.addTab(inputTabs,agents[agentKey][NAME])

    #create a scan toggle
    scanToggle = ScanToggleObject()
    
    #on start button click, clear data arrays & toggle scan
    x, y, yerr = [], [], []
    def onActivationRequested(x,y):
        while x: x.pop()
        while y: y.pop()
        while yerr: yerr.pop()
        agent = agentTabs.currentWidget().currentWidget().getInput(
            agents[
                AGENTS[
                    agentTabs.currentIndex()
                ]
            ][SET_POSITION]
        ).next
        scanToggle.setInput(agent)

        def output(channel,total):
            output.count = 0
            output.sum = 0
            output.squaresSum = 0
            d = Deferred()
            def onVoltages(voltages):
                output.sum += voltages[channel]
                output.squaresSum += voltages[channel]**2
                output.count += 1
                if output.count is total:
                    average = output.sum / total
                    yerr.append(pow(output.squaresSum / total - average**2, .5)/pow(total,.5)) 
                    vmClient.removeListener(onVoltages)
                    d.callback(average)
                    print 'output'
            vmClient.addListener(onVoltages)
            return d
        scanToggle.setOutput(partial(output,vmCombo.getCurrentKey(),shotsSpin.value()))
        scanToggle.toggle()    
    scanToggle.activationRequested.connect(partial(onActivationRequested,x,y))
    def onDeactivationRequested():
        agents[
            AGENTS[
                agentTabs.currentIndex()
            ]
        ][CANCEL]()
    scanToggle.deactivationRequested.connect(onDeactivationRequested)

    #create a spinbox for the shots to average parameter
    shotsSpin = QtGui.QSpinBox()
    shotsSpin.setRange(1,10000)
    cpLayout.addRow('shots to average', shotsSpin)
    
    # create a toggle widget
    from qtutils.toggle import ToggleWidget
    cpLayout.addWidget(ToggleWidget(scanToggle))
    
    # plot on step completion
    def onStepped(data):
        print 'step'
        input, output = data
        x.append(input)
        y.append(output)
        plotWidget.clear()
        plotWidget.plot(x,y)
        errorBars = ErrorBarItem(x=asarray(x),y=asarray(y),top=asarray(yerr),bottom=asarray(yerr),beam=.05)
        plotWidget.addItem(errorBars)
        scanToggle.completeStep()
        
    scanToggle.stepped.connect(onStepped)

    #dropdown box for measurementType so saveCSV (below) uses correct directory
    measureList = SCAN_TYPES.keys()
    measureCombo = QtGui.QComboBox()
    measureCombo.addItems(measureList)
    cpLayout.addRow('measurement',measureCombo)

    #save button for use on plots with errorbars
    import numpy as np
    def saveCSVButFunc():
        measure = measureCombo.currentText()
        dataArray = asarray([x,y,yerr],dtype=np.dtype(np.float32))
        saveCSV(measure,dataArray.T,POOHDATAPATH)
    saveCSVButton = QtGui.QPushButton('save (csv)')
    saveCSVButton.clicked.connect(saveCSVButFunc)
    cpLayout.addWidget(saveCSVButton)
    

if __name__ == '__main__':
    from twisted.internet import reactor
    container = []
    SmartScanGUI()
    reactor.run()
    
    
