#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import sys
import threading  # Timer für den DCM-Controller
import time
import helper_calc as calc
from epics import caget, caput, camonitor, camonitor_clear
from PySide2 import QtWidgets, QtUiTools, QtCore, QtGui
from PySide2.QtCore import QRunnable, Slot, QThreadPool, QObject, Signal
import pyqtgraph as pg
#pg.setConfigOption('background', 'w')  # Plothintergrund weiß (2D)
#pg.setConfigOption('foreground', 'k')  # Plotvordergrund schwarz (2D)
#pg.setConfigOptions(antialias=True)  # Enable antialiasing for prettier plots

# to use pyqtgraph with PySide2, see also:
# https://stackoverflow.com/questions/60580391/pyqtgraph-with-pyside-2-and-qtdesigner
DIR_PATH = os.path.dirname(os.path.realpath(__file__))


class UiLoader(QtUiTools.QUiLoader):
    def createWidget(self, className, parent=None, name=""):
        if className == "PlotWidget":
            return pg.PlotWidget(parent=parent)
        return super().createWidget(className, parent, name)


def load_ui(fname):
    fd = QtCore.QFile(fname)
    if fd.open(QtCore.QFile.ReadOnly):
        loader = UiLoader()
        window = loader.load(fd)
        fd.close()
        return window


class Worker(QRunnable):
    """
    Worker thread, for more info see:
    https://www.learnpyqt.com/tutorials/multithreading-pyqt-applications-qthreadpool/

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    """

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress

    @Slot()
    def run(self):
        """
        Initialise the runner function with passed args, kwargs.
        """
        self.fn(*self.args, **self.kwargs)
        # # Retrieve args/kwargs here; and fire processing using them
        # try:
        #     result = self.fn(*self.args, **self.kwargs)
        # except:
        #     traceback.print_exc()
        #     exctype, value = sys.exc_info()[:2]
        #     self.signals.error.emit((exctype, value, traceback.format_exc()))
        # else:
        #     self.signals.result.emit(result)  # Return the result of the processing
        # finally:
        #     self.signals.finished.emit()  # Done
        self.signals.finished.emit()  # Done


class WorkerSignals(QObject):
    """
    Defines the signals available from a running worker thread.

    Supported signals are:

    finished
        No data

    error
        tuple (exctype, value, traceback.format_exc() )

    result
        object data returned from processing, anything

    progress
        int indicating % progress or 0/1 fore not done/done

    """
    finished = Signal()
    # error = Signal(tuple)
    # result = Signal(object)
    progress = Signal(int)


class DCM(QtCore.QObject):
    def __init__(self):
        super(DCM, self).__init__()
        self.window = load_ui(os.path.join(DIR_PATH, 'dcmController.ui'))
        self.window.installEventFilter(self)

        self.threadpool = QThreadPool()

        self.ioni_pv = self.window.ioniPv.text()
        camonitor(self.ioni_pv, writer=self.pv_monitor)
        camonitor("bIICurrent:Mnt1", writer=self.pv_monitor)

        self.control = None
        self.window.getValues.clicked.connect(self.calc_hold_values)
        self.window.getValues_topo.clicked.connect(self.topo_hold_values)
        self.window.control_activate.toggled.connect(self.dcm_controller)
        self.window.manually.toggled.connect(self.monitor_manually)
        self.window.ioniPv.returnPressed.connect(self.clear_and_set_monitor)
        self.window.add_topo.toggled.connect(self.monitor_pco1600)
        self.window.triggered_topo.toggled.connect(self.monitor_topo)
        self.window.manually_status.textChanged.connect(self.dcm_controller_manually_thread)
        self.window.topo_status.textChanged.connect(self.topo_controller_thread)
        self.window.IoniSignal.textChanged.connect(self.calc_norm)
        self.window.ringcurrent.textChanged.connect(self.calc_norm)
        self.window.IoniSignal_hold.textChanged.connect(self.allow_to_activate)
        self.window.normSignal_hold.textChanged.connect(self.allow_to_activate)
        self.window.mean_hold.textChanged.connect(self.allow_to_activate_topo)

        home_dir = os.path.expanduser("~")
        dcm_log_dir = home_dir + "/DCMController-logs"
        if not os.path.exists(dcm_log_dir):
            os.makedirs(dcm_log_dir)

        self.log = open("%s/dcmController-log_%s.txt" % (dcm_log_dir, time.ctime()), "w")
        print('%s DCM - controller started' % time.ctime())
        print("%s/dcmController-log_%s.txt" % (dcm_log_dir, time.ctime()))
        self.log.write('%s DCM - controller started' % time.ctime())
        self.log.flush()

    def pv_monitor(self, pv_value):

        pv, date, zeit, value = pv_value.split()
        if pv == self.ioni_pv:
            self.window.IoniSignal.setText(value)
        if pv == "bIICurrent:Mnt1":
            self.window.ringcurrent.setText("{:.6s}".format(value))
        if pv == "PCO1600:Stats1:MeanValue_RBV":
            self.window.pco1600_mean.setText(value)
        if pv == "DCM:Controller":
            if value == 'Check':
                self.window.manually_status.setText('triggered Status: Checking')
            else:
                self.window.manually_status.setText('triggered Status: Idle')
        if pv == "TOPO:Controller":
            if value == 'Check':
                self.window.topo_status.setText('Topo Status: Checking')
            else:
                self.window.topo_status.setText('Topo Status: Idle')

    def clear_and_set_monitor(self):

        self.log.write('\n%s cleared' % self.ioni_pv)
        print('%s cleared' % self.ioni_pv)
        camonitor_clear(self.ioni_pv)
        self.window.IoniSignal.setText('0')

        self.ioni_pv = self.window.ioniPv.text()
        camonitor(self.ioni_pv, writer=self.pv_monitor)
        self.log.write('\n%s entered' % self.ioni_pv)
        print('%s entered' % self.ioni_pv)

    def monitor_pco1600(self):

        if self.window.add_topo.isChecked():
            camonitor("PCO1600:Stats1:MeanValue_RBV", writer=self.pv_monitor)
        else:
            camonitor_clear("PCO1600:Stats1:MeanValue_RBV")

    def monitor_manually(self):

        if self.window.manually.isChecked():
            camonitor("DCM:Controller", writer=self.pv_monitor)
            self.window.manually_status.setText('triggered Status: Idle')
            # turn off the automatic mode
            self.window.control_activate.setChecked(0)
        else:
            camonitor_clear("DCM:Controller")
            self.window.manually_status.setText('triggered Status: off')

    def monitor_topo(self):

        if self.window.triggered_topo.isChecked():
            camonitor("TOPO:Controller", writer=self.pv_monitor)
            self.window.topo_status.setText('Topo Status: Idle')
        else:
            camonitor_clear("TOPO:Controller")
            self.window.topo_status.setText('Topo Status: off')

    def calc_norm(self):

        if self.window.IoniSignal.text():  # when calc_norm is called before k_signal has a value
            k_value = float(self.window.IoniSignal.text())
        else:
            k_value = 0
        if self.window.ringcurrent.text():  # when calc_norm is called before ringcurrent has a value
            ring_value = float(self.window.ringcurrent.text()) * 1e-3
            if ring_value == 0:
                ring_value = 1
        else:
            ring_value = 1

        norm_signal = k_value / ring_value
        norm_signal = str('{:0.5e}'.format(norm_signal))
        self.window.normSignal.setText(norm_signal)

    def calc_hold_values(self):

        self.window.IoniSignal_hold.setText(self.window.IoniSignal.text())
        self.window.normSignal_hold.setText(self.window.normSignal.text())

    def topo_hold_values(self):

        # get the current camera state, so we can restore it after the procedure
        image_mode_ini = caget("PCO1600:cam1:ImageMode")
        auto_save_ini = caget("PCO1600:TIFF1:AutoSave")

        caput("PCO1600:cam1:ImageMode", 'Continuous')
        caput("PCO1600:TIFF1:AutoSave", 'No')
        time.sleep(0.2)
        caput("PCO1600:cam1:Acquire", 1)

        exp_time = caget("PCO1600:cam1:AcquireTime_RBV")
        wait_time = exp_time + 1.

        time.sleep(wait_time)

        self.window.mean_hold.setText(self.window.pco1600_mean.text())

        # stop the camera and put it back to the state it was before
        caput("PCO1600:cam1:Acquire", 0)
        time.sleep(0.2)
        caput("PCO1600:cam1:ImageMode", image_mode_ini)
        caput("PCO1600:TIFF1:AutoSave", auto_save_ini)

    def allow_to_activate(self):

        if self.window.monitor_normSignal.isChecked() and calc.isfloat(self.window.normSignal_hold.text()) is True or \
                self.window.monitor_IoniSignal.isChecked() and calc.isfloat(self.window.IoniSignal_hold.text()) is True:
            self.window.control_activate.setEnabled(1)
            self.window.manually.setEnabled(1)
        else:
            self.window.control_activate.setChecked(0)
            self.window.control_activate.setEnabled(0)
            self.window.manually.setChecked(0)
            self.window.manually.setEnabled(0)

    def allow_to_activate_topo(self):

        if calc.isfloat(self.window.mean_hold.text()) is True:
            self.window.triggered_topo.setEnabled(1)
        else:
            self.window.triggered_topo.setChecked(0)
            self.window.triggered_topo.setEnabled(0)

    def dcm_controller(self):

        self.log.flush()

        if self.window.control_activate.isChecked() is True:

            # uncheck the manual mode
            self.window.manually.blockSignals(True)
            self.window.manually.setChecked(0)
            self.window.manually.blockSignals(False)
            self.monitor_manually()
            self.window.t_pruef.setEnabled(1)

            if self.control is None:  # in case the controller has been activated
                self.log.write('\n%s DCM - controller activated' % time.ctime())
                print('%s DCM - controller activated' % time.ctime())
                if self.window.monitor_normSignal.isChecked():
                    hold_signal = float(self.window.normSignal_hold.text())
                    self.log.write('\nnormed Signal to hold %.5e' % hold_signal)
                    print('normed Signal to hold %.5e' % hold_signal)
                else:
                    hold_signal = float(self.window.IoniSignal_hold.text())
                    self.log.write('\nIoni signal to hold %.5e' % hold_signal)
                    print('Ioni signal to hold %.5e' % hold_signal)

            self.control = threading.Timer(self.window.t_pruef.value(), self.dcm_controller)
            self.control.start()

            # calc the difference in percent
            if self.window.monitor_normSignal.isChecked():
                hold_signal = float(self.window.normSignal_hold.text())
                act_signal = float(self.window.normSignal.text())
            else:
                hold_signal = float(self.window.IoniSignal_hold.text())
                act_signal = float(self.window.IoniSignal.text())

            diff = act_signal / hold_signal * 100
            threshold = self.window.threshold.value()

            # when nb_state == 2 the Neben-BS is opened
            nb_state = caget("BS02R02U102L:State")
            # only valid if dcm_e is between 6 and 60keV
            dcm_e = caget("Energ:25002000rbv")
            # only if the beamstop is lower than the dcm- + dmm-offset and done moving
            # dmm-offset determined via dcm-y
            dcm_offset = caget("Energ:25002000z2.B")
            dcm_y = caget("OMS58:25001007")
            beamstop = caget("OMS58:25003001.RBV")
            beamstop_offset = (dcm_offset + dcm_y) - beamstop
            beamstop_done_moving = caget("OMS58:25003001.DMOV")

            # take action (there is something wrong if diff is less than 10% signal...)
            if threshold > diff > 10 and nb_state == 2 and 6 < dcm_e < 60 and beamstop_offset > 0 and \
                    beamstop_done_moving == 1:
                tweak_forward = False
                tweak_back = False
                self.log.write('\n%s low signal detected: %.2f%%' % (time.ctime(), diff))
                print('%s low signal detected: %.2f%%' % (time.ctime(), diff))
                tweak_step = self.window.piezo_tweak.value()
                tweak_value = caget("PI662:Piezo1.TWV")
                if tweak_step != tweak_value:
                    self.log.write('\n%s putting tweak-value to %.3f' % (time.ctime(), tweak_step))
                    print('%s putting tweak-value to %.3f' % (time.ctime(), tweak_step))
                    caput("PI662:Piezo1.TWV", tweak_step)

                # tweak once to get direction, wait 2s for keithely
                self.log.write('\n%s tweaking once forward to get the right direction...' % time.ctime())
                print('%s tweaking once forward to get the right direction...' % time.ctime())
                caput("PI662:Piezo1.TWF", 1)
                time.sleep(2)

                if self.window.monitor_normSignal.isChecked():
                    new_signal = float(self.window.normSignal.text())
                else:
                    new_signal = float(self.window.IoniSignal.text())

                if new_signal > act_signal:
                    self.log.write('\n%s yes, tweaking forward is the right direction!' % time.ctime())
                    print('%s yes, tweaking forward is the right direction!' % time.ctime())
                    tweak_forward = True
                else:
                    self.log.write('\n%s nope, now tweaking twice reverse...' % time.ctime())
                    print('%s nope, now tweaking twice reverse...' % time.ctime())
                    caput("PI662:Piezo1.TWR", 1)
                    time.sleep(1)
                    caput("PI662:Piezo1.TWR", 1)
                    time.sleep(2)

                    if self.window.monitor_normSignal.isChecked():
                        new_signal = float(self.window.normSignal.text())
                    else:
                        new_signal = float(self.window.IoniSignal.text())

                    if new_signal > act_signal:
                        self.log.write('\n%s yes, tweaking reverse is the right direction!' % time.ctime())
                        print('%s yes, tweaking reverse is the right direction!' % time.ctime())
                        tweak_back = True
                    else:
                        caput("PI662:Piezo1.TWF", 1)
                        time.sleep(2)
                        self.log.write('\n%s nope, signal is not rising... i do nothing' % time.ctime())
                        print('%s nope, signal is not rising... i do nothing' % time.ctime())
                        # put the original user-tweak-value
                        caput("PI662:Piezo1.TWV", tweak_value)

                if tweak_forward:
                    if self.window.monitor_normSignal.isChecked():
                        while new_signal > act_signal:
                            act_signal = float(self.window.normSignal.text())
                            self.log.write('\n%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                            self.log.write('\ntweaking forward')
                            print('%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                            print('tweaking forward')
                            caput("PI662:Piezo1.TWF", 1)
                            time.sleep(2)
                            new_signal = float(self.window.normSignal.text())
                            self.log.write('\n%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                            print('%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                    else:
                        while new_signal > act_signal:
                            act_signal = float(self.window.IoniSignal.text())
                            self.log.write('\n%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                            self.log.write('\ntweaking forward')
                            print('%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                            print('tweaking forward')
                            caput("PI662:Piezo1.TWF", 1)
                            time.sleep(2)
                            new_signal = float(self.window.IoniSignal.text())
                            self.log.write('\n%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))
                            print('%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))

                    self.log.write('\ntweaking once back...')
                    print('tweaking once back...')
                    caput("PI662:Piezo1.TWR", 1)
                    time.sleep(2)

                if tweak_back:
                    if self.window.monitor_normSignal.isChecked():
                        while new_signal > act_signal:
                            act_signal = float(self.window.normSignal.text())
                            self.log.write('\n%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                            self.log.write('\ntweaking reverse')
                            print('%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                            print('tweaking reverse')
                            caput("PI662:Piezo1.TWR", 1)
                            time.sleep(2)
                            new_signal = float(self.window.normSignal.text())
                            self.log.write('\n%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                            print('%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                    else:
                        while new_signal > act_signal:
                            act_signal = float(self.window.IoniSignal.text())
                            self.log.write('\n%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                            self.log.write('\ntweaking reverse')
                            print('%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                            print('tweaking reverse')
                            caput("PI662:Piezo1.TWR", 1)
                            time.sleep(2)
                            new_signal = float(self.window.IoniSignal.text())
                            self.log.write('\n%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))
                            print('%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))

                    self.log.write('\ntweaking once forward...')
                    print('tweaking once forward...')
                    caput("PI662:Piezo1.TWF", 1)
                    time.sleep(2)

                if tweak_forward or tweak_back:
                    if self.window.monitor_normSignal.isChecked():
                        act_signal = self.window.normSignal.text()
                        self.log.write('\n%s the new normed signal to hold is: %s' % (time.ctime(), act_signal))
                        print('%s the new normed signal to hold is: %s' % (time.ctime(), act_signal))
                        self.window.normSignal_hold.setText(self.window.normSignal.text())
                        time.sleep(1)
                    else:
                        act_signal = self.window.IoniSignal.text()
                        self.log.write('\n%s the new Ioni signal to hold is: %s' % (time.ctime(), act_signal))
                        print('%s the new Ioni signal to hold is: %s' % (time.ctime(), act_signal))
                        self.window.IoniSignal_hold.setText(self.window.IoniSignal.text())
                        time.sleep(1)

                    # put the original user-tweak-value
                    caput("PI662:Piezo1.TWV", tweak_value)

        else:
            self.control.cancel()  # terminates the controller
            self.control = None  # sets back the Controller-Status
            self.log.write('\n%s DCM - controller terminated' % time.ctime())
            print('%s DCM - controller terminated' % time.ctime())

    def dcm_controller_manually_thread(self):

        """Run the dcm-controller in a separate thread in order to not freeze the GUI."""

        worker = Worker(self.dcm_controller_manually)  # Any other args, kwargs are passed to the run function
        #worker.signals.progress.connect(self.progress_bar)
        #worker.signals.finished.connect(self.bl_spectrum)

        # Execute
        self.threadpool.start(worker)

    def dcm_controller_manually(self, progress_callback):

        #done = False
        #progress_callback.emit(done)

        if self.window.manually_status.text() != 'triggered Status: Checking':
            return

        self.log.flush()

        self.log.write('\n%s DCM - controller manually triggered' % time.ctime())
        print('%s DCM - controller manually triggered' % time.ctime())
        if self.window.monitor_normSignal.isChecked():
            hold_signal = float(self.window.normSignal_hold.text())
            self.log.write('\nnormed Signal to hold %.5e' % hold_signal)
            print('normed Signal to hold %.5e' % hold_signal)
        else:
            hold_signal = float(self.window.IoniSignal_hold.text())
            self.log.write('\nIoni signal to hold %.5e' % hold_signal)
            print('Ioni signal to hold %.5e' % hold_signal)

        # calc the difference in percent
        if self.window.monitor_normSignal.isChecked():
            hold_signal = float(self.window.normSignal_hold.text())
            act_signal = float(self.window.normSignal.text())
        else:
            hold_signal = float(self.window.IoniSignal_hold.text())
            act_signal = float(self.window.IoniSignal.text())

        diff = act_signal / hold_signal * 100
        threshold = self.window.threshold.value()

        # when nb_state == 2 the Neben-BS is opened
        nb_state = caget("BS02R02U102L:State")
        # only valid if dcm_e is between 6 and 60keV
        dcm_e = caget("Energ:25002000rbv")
        # only if the beamstop is lower than the dcm-beamoffset and done moving
        dcm_offset = caget("Energ:25002000z2.B")
        beamstop = caget("OMS58:25003001.RBV")
        beamstop_offset = dcm_offset - beamstop
        beamstop_done_moving = caget("OMS58:25003001.DMOV")

        # take action (there is something wrong if diff is less than 10% signal...)
        if threshold > diff > 10 and nb_state == 2 and 6 < dcm_e < 60 and beamstop_offset > 0 and \
                beamstop_done_moving == 1:
            tweak_forward = False
            tweak_back = False
            self.log.write('\n%s low signal detected: %.2f%%' % (time.ctime(), diff))
            print('%s low signal detected: %.2f%%' % (time.ctime(), diff))
            tweak_step = self.window.piezo_tweak.value()
            tweak_value = caget("PI662:Piezo1.TWV")
            if tweak_step != tweak_value:
                self.log.write('\n%s putting tweak-value to %.3f' % (time.ctime(), tweak_step))
                print('%s putting tweak-value to %.3f' % (time.ctime(), tweak_step))
                caput("PI662:Piezo1.TWV", tweak_step)

            # tweak once to get direction, wait 2s for keithely
            self.log.write('\n%s tweaking once forward to get the right direction...' % time.ctime())
            print('%s tweaking once forward to get the right direction...' % time.ctime())
            caput("PI662:Piezo1.TWF", 1)
            time.sleep(2)

            if self.window.monitor_normSignal.isChecked():
                new_signal = float(self.window.normSignal.text())
            else:
                new_signal = float(self.window.IoniSignal.text())

            if new_signal > act_signal:
                self.log.write('\n%s yes, tweaking forward is the right direction!' % time.ctime())
                print('%s yes, tweaking forward is the right direction!' % time.ctime())
                tweak_forward = True
            else:
                self.log.write('\n%s nope, now tweaking twice reverse...' % time.ctime())
                print('%s nope, now tweaking twice reverse...' % time.ctime())
                caput("PI662:Piezo1.TWR", 1)
                time.sleep(1)
                caput("PI662:Piezo1.TWR", 1)
                time.sleep(2)

                if self.window.monitor_normSignal.isChecked():
                    new_signal = float(self.window.normSignal.text())
                else:
                    new_signal = float(self.window.IoniSignal.text())

                if new_signal > act_signal:
                    self.log.write('\n%s yes, tweaking reverse is the right direction!' % time.ctime())
                    print('%s yes, tweaking reverse is the right direction!' % time.ctime())
                    tweak_back = True
                else:
                    caput("PI662:Piezo1.TWF", 1)
                    time.sleep(2)
                    self.log.write('\n%s nope, signal is not rising... i do nothing' % time.ctime())
                    print('%s nope, signal is not rising... i do nothing' % time.ctime())
                    # put the original user-tweak-value
                    caput("PI662:Piezo1.TWV", tweak_value)

            if tweak_forward:
                if self.window.monitor_normSignal.isChecked():
                    while new_signal > act_signal:
                        act_signal = float(self.window.normSignal.text())
                        self.log.write('\n%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                        self.log.write('\ntweaking forward')
                        print('%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                        print('tweaking forward')
                        caput("PI662:Piezo1.TWF", 1)
                        time.sleep(2)
                        new_signal = float(self.window.normSignal.text())
                        self.log.write('\n%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                        print('%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                else:
                    while new_signal > act_signal:
                        act_signal = float(self.window.IoniSignal.text())
                        self.log.write('\n%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                        self.log.write('\ntweaking forward')
                        print('%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                        print('tweaking forward')
                        caput("PI662:Piezo1.TWF", 1)
                        time.sleep(2)
                        new_signal = float(self.window.IoniSignal.text())
                        self.log.write('\n%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))
                        print('%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))

                self.log.write('\ntweaking once back...')
                print('tweaking once back...')
                caput("PI662:Piezo1.TWR", 1)
                time.sleep(2)

            if tweak_back:
                if self.window.monitor_normSignal.isChecked():
                    while new_signal > act_signal:
                        act_signal = float(self.window.normSignal.text())
                        self.log.write('\n%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                        self.log.write('\ntweaking reverse')
                        print('%s the normed signal is: %.5e' % (time.ctime(), act_signal))
                        print('tweaking reverse')
                        caput("PI662:Piezo1.TWR", 1)
                        time.sleep(2)
                        new_signal = float(self.window.normSignal.text())
                        self.log.write('\n%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                        print('%s the new normed signal is: %.5e' % (time.ctime(), new_signal))
                else:
                    while new_signal > act_signal:
                        act_signal = float(self.window.IoniSignal.text())
                        self.log.write('\n%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                        self.log.write('\ntweaking reverse')
                        print('%s the Ioni signal is: %.5e' % (time.ctime(), act_signal))
                        print('tweaking reverse')
                        caput("PI662:Piezo1.TWR", 1)
                        time.sleep(2)
                        new_signal = float(self.window.IoniSignal.text())
                        self.log.write('\n%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))
                        print('%s the new Ioni signal is: %.5e' % (time.ctime(), new_signal))

                self.log.write('\ntweaking once forward...')
                print('tweaking once forward...')
                caput("PI662:Piezo1.TWF", 1)
                time.sleep(2)

            if tweak_forward or tweak_back:
                if self.window.monitor_normSignal.isChecked():
                    act_signal = self.window.normSignal.text()
                    self.log.write('\n%s the new normed signal to hold is: %s' % (time.ctime(), act_signal))
                    print('%s the new normed signal to hold is: %s' % (time.ctime(), act_signal))
                    self.window.normSignal_hold.setText(self.window.normSignal.text())
                    time.sleep(1)
                else:
                    act_signal = self.window.IoniSignal.text()
                    self.log.write('\n%s the new Ioni signal to hold is: %s' % (time.ctime(), act_signal))
                    print('%s the new Ioni signal to hold is: %s' % (time.ctime(), act_signal))
                    self.window.IoniSignal_hold.setText(self.window.IoniSignal.text())
                    time.sleep(1)

                # put the original user-tweak-value
                caput("PI662:Piezo1.TWV", tweak_value)
        else:
            self.log.write('\n%s no adjustment, if-condition not met' % time.ctime())
            print('%s no adjustment, if-condition not met' % time.ctime())

        # set back the trigger PV
        caput("DCM:Controller", "Idle")

        #done = True
        #progress_callback.emit(done)

    def topo_controller_thread(self):

        """Run the dcm-controller in a separate thread in order to not freeze the GUI."""

        worker = Worker(self.topo_controller)  # Any other args, kwargs are passed to the run function
        #worker.signals.progress.connect(self.progress_bar)
        #worker.signals.finished.connect(self.bl_spectrum)

        # Execute
        self.threadpool.start(worker)

    def topo_controller(self, progress_callback):

        if self.window.topo_status.text() != 'Topo Status: Checking':
            return

        self.log.flush()

        self.log.write('\n%s TOPO - controller triggered' % time.ctime())
        print('%s TOPO - controller triggered' % time.ctime())
        hold_signal = float(self.window.mean_hold.text())
        self.log.write('\nPCO1600-mean to hold %.2f' % hold_signal)
        print('PCO1600-mean to hold %.2f' % hold_signal)

        # get the current camera state, so we can restore it after the procedure
        image_mode_ini = caget("PCO1600:cam1:ImageMode")
        auto_save_ini = caget("PCO1600:TIFF1:AutoSave")

        caput("PCO1600:cam1:ImageMode", 'Continuous')
        caput("PCO1600:TIFF1:AutoSave", 'No')
        time.sleep(0.2)
        caput("PCO1600:cam1:Acquire", 1)

        exp_time = caget("PCO1600:cam1:AcquireTime_RBV")
        wait_time = exp_time + 2.

        time.sleep(wait_time)
        # calc the difference in percent
        hold_signal = float(self.window.mean_hold.text())
        act_signal = float(self.window.pco1600_mean.text())

        diff = act_signal / hold_signal * 100
        threshold = self.window.mean_threshold.value()

        # when nb_state == 2 the Neben-BS is opened
        nb_state = caget("BS02R02U102L:State")
        # only valid if dcm_e is between 6 and 60keV
        dcm_e = caget("Energ:25002000rbv")
        # only if the beamstop is lower than the dcm-beamoffset and done moving
        dcm_offset = caget("Energ:25002000z2.B")
        beamstop = caget("OMS58:25003001.RBV")
        beamstop_offset = dcm_offset - beamstop
        beamstop_done_moving = caget("OMS58:25003001.DMOV")

        # take action (there is something wrong if diff is less than 10% signal...)
        if threshold > diff > 10 and nb_state == 2 and 6 < dcm_e < 60 and beamstop_offset > 0 and \
                beamstop_done_moving == 1:
            tweak_forward = False
            tweak_back = False
            self.log.write('\n%s low signal detected: %.2f%%' % (time.ctime(), diff))
            print('%s low signal detected: %.2f%%' % (time.ctime(), diff))
            tweak_value = self.window.piezo_tweak.value()
            #tweak_value = caget("PIE665:Piezo1.TWV")
            #if tweak_step != tweak_value:
                #self.log.write('\n%s putting tweak-value to %.3f' % (time.ctime(), tweak_step))
                #print('%s putting tweak-value to %.3f' % (time.ctime(), tweak_step))
                #caput("PIE665:Piezo1.TWV", tweak_step)

            # tweak once to get direction, wait exp_time for camera
            self.log.write('\n%s tweaking once forward to get the right direction...' % time.ctime())
            print('%s tweaking once forward to get the right direction...' % time.ctime())
            caput("PIE665:Piezo1.TWF", 1)
            time.sleep(wait_time)

            new_signal = float(self.window.pco1600_mean.text())

            if new_signal > act_signal:
                self.log.write('\n%s yes, tweaking forward is the right direction!' % time.ctime())
                print('%s yes, tweaking forward is the right direction!' % time.ctime())
                tweak_forward = True
            else:
                self.log.write('\n%s nope, now tweaking twice reverse...' % time.ctime())
                print('%s nope, now tweaking twice reverse...' % time.ctime())
                caput("PIE665:Piezo1.TWR", 1)
                time.sleep(1)
                caput("PIE665:Piezo1.TWR", 1)
                time.sleep(wait_time)

                new_signal = float(self.window.pco1600_mean.text())

                if new_signal > act_signal:
                    self.log.write('\n%s yes, tweaking reverse is the right direction!' % time.ctime())
                    print('%s yes, tweaking reverse is the right direction!' % time.ctime())
                    tweak_back = True
                else:
                    caput("PIE665:Piezo1.TWF", 1)
                    time.sleep(2)
                    self.log.write('\n%s nope, signal is not rising... i do nothing' % time.ctime())
                    print('%s nope, signal is not rising... i do nothing' % time.ctime())
                    # put the original user-tweak-value
                    caput("PIE665:Piezo1.TWV", tweak_value)

            if tweak_forward:
                while new_signal > act_signal:
                    act_signal = float(self.window.pco1600_mean.text())
                    self.log.write('\n%s the normed signal is: %.2f' % (time.ctime(), act_signal))
                    self.log.write('\ntweaking forward')
                    print('%s PCO1600-mean is: %.2f' % (time.ctime(), act_signal))
                    print('tweaking forward')
                    caput("PIE665:Piezo1.TWF", 1)
                    time.sleep(wait_time)
                    new_signal = float(self.window.pco1600_mean.text())
                    self.log.write('\n%s the new PCO1600-mean is: %.2f' % (time.ctime(), new_signal))
                    print('%s the new PCO1600-mean is: %.2f' % (time.ctime(), new_signal))

                self.log.write('\ntweaking once back...')
                print('tweaking once back...')
                caput("PIE665:Piezo1.TWR", 1)
                time.sleep(wait_time)

            if tweak_back:
                while new_signal > act_signal:
                    act_signal = float(self.window.pco1600_mean.text())
                    self.log.write('\n%s the normed signal is: %.2f' % (time.ctime(), act_signal))
                    self.log.write('\ntweaking reverse')
                    print('%s PCO1600-mean is: %.2f' % (time.ctime(), act_signal))
                    print('tweaking reverse')
                    caput("PIE665:Piezo1.TWR", 1)
                    time.sleep(wait_time)
                    new_signal = float(self.window.pco1600_mean.text())
                    self.log.write('\n%s the new PCO1600-mean is: %.5e' % (time.ctime(), new_signal))
                    print('%s the new PCO1600-mean is: %.5e' % (time.ctime(), new_signal))

                self.log.write('\ntweaking once forward...')
                print('tweaking once forward...')
                caput("PIE665:Piezo1.TWF", 1)
                time.sleep(wait_time)

            if tweak_forward or tweak_back:
                act_signal = self.window.pco1600_mean.text()
                self.log.write('\n%s the new PCO1600-mean to hold is: %s' % (time.ctime(), act_signal))
                print('%s the new PCO1600-mean to hold is: %s' % (time.ctime(), act_signal))
                self.window.mean_hold.setText(self.window.pco1600_mean.text())
                time.sleep(1)

                # put the original user-tweak-value
                #caput("PIE665:Piezo1.TWV", tweak_value)
        else:
            self.log.write('\n%s no adjustment, if-condition not met' % time.ctime())
            print('%s no adjustment, if-condition not met' % time.ctime())

        # stop the camera and put it back to the state it was before
        caput("PCO1600:cam1:Acquire", 0)
        time.sleep(0.2)
        caput("PCO1600:cam1:ImageMode", image_mode_ini)
        caput("PCO1600:TIFF1:AutoSave", auto_save_ini)

        # set back the trigger PV
        caput("TOPO:Controller", "Idle")

    def show(self):
        self.window.show()

    # clean Exit of the App, see also:
    # https://stackoverflow.com/questions/53097415/pyside2-connect-close-by-window-x-to-custom-exit-method

    def eventFilter(self, obj, event):
        if obj is self.window and event.type() == QtCore.QEvent.Close:
            self.quit_app()
            event.ignore()
            return True
        return super(DCM, self).eventFilter(obj, event)

    @QtCore.Slot()
    def quit_app(self):
        # some actions to perform before actually quitting:
        if self.control:
            self.control.cancel()
            self.log.write('\n%s DCM - controller terminated' % time.ctime())
            print('%s DCM - controller terminated' % time.ctime())
        self.log.write('\n%s DCM - controller exited' % time.ctime())
        print('%s DCM - controller exited' % time.ctime())
        self.log.close()
        self.window.removeEventFilter(self)
        app.quit()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main = DCM()
    main.show()
    sys.exit(app.exec_())
