# Copyright (c) Acconeer AB, 2023
# All rights reserved

from __future__ import annotations

import numpy as np

# Added here to force pyqtgraph to choose PySide
import PySide6  # noqa: F401

import pyqtgraph as pg

import acconeer.exptool as et
from acconeer.exptool import a121
from acconeer.exptool.a121.algo._utils import estimate_frame_rate
from acconeer.exptool.a121.algo.speed import (
    Detector,
    DetectorConfig,
    DetectorMetadata,
    DetectorResult,
)
from acconeer.exptool.app.new.ui.stream_tab.plugin_widget import PluginPlotArea
import socket
import struct


serverSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
serverSocket.bind(("127.0.0.1",39040))
serverSocket.setblocking(0)


def main():
    args = a121.ExampleArgumentParser().parse_args()
    et.utils.config_logging(args)

    client = a121.Client.open(**a121.get_client_args(args))

    # Detector config set to default values for clarity.
    detector_config = DetectorConfig(
        start_point=200,
        num_points=1,
        step_length=None,
        profile=None,
        frame_rate=None,
        sweep_rate=None,
        hwaas=None,
        num_bins=50,
        max_speed=10.0,
        threshold=100.0,
    )

    SENSOR_ID = 1

    detector = Detector(client=client, sensor_id=SENSOR_ID, detector_config=detector_config)
    sensor_config = detector._get_sensor_config(detector_config)
    session_config = a121.SessionConfig(
        {SENSOR_ID: sensor_config},
        extended=False,
    )

    estimated_frame_rate = estimate_frame_rate(client, session_config)

    detector.start()

    pg_updater = PGUpdater(
        detector_config,
        sensor_config,
        estimated_frame_rate,
    )

    pg_updater.getThreshold()

    pg_process = et.PGProcess(pg_updater)
    pg_process.start()

    interrupt_handler = et.utils.ExampleInterruptHandler()
    print("Press Ctrl-C to end session")

    while not interrupt_handler.got_signal:
        detector_result = detector.get_next()
        try:
            pg_process.put_data(detector_result)
        except et.PGProccessDiedException:
            break

    detector.stop()

    print("Disconnecting...")
    client.close()


class PGUpdater:
    def __init__(
        self,
        detector_config: DetectorConfig,
        detector_metadata: DetectorMetadata,
        estimated_frame_rate: float,
    ):
        self.detector_config = detector_config

        self.n_depths = detector_metadata.num_points

        max_update_rate = PluginPlotArea._FPS

        if estimated_frame_rate > max_update_rate:
            plugin_frame_rate = float(max_update_rate)
        else:
            plugin_frame_rate = estimated_frame_rate

        self.history_length_s = 10.0
        self.history_length = int(self.history_length_s * plugin_frame_rate)

        self.time_window_length_s = 3.0
        self.time_window_length_n = int(self.time_window_length_s * plugin_frame_rate)

        self.speed_history = np.zeros(self.history_length)
        self.speed_history_xs = np.array([i for i in range(-self.history_length, 0)])

        n_ticks_to_display = 10
        x_labels = np.linspace(-self.history_length_s, 0, self.history_length)
        all_ticks = [
            (t, "{:.0f}".format(label)) for t, label in zip(self.speed_history_xs, x_labels)
        ]
        subsample_step = self.history_length // n_ticks_to_display
        self.display_ticks = [all_ticks[::subsample_step]]

        self.header_value = None
        self.speed_threshold = None
        self.socket_timer = 0

        self.setup_is_done = False


    def setup(self, win):

        win.setWindowTitle("Acconeer speed detection example")

        self.raw_fft_plot = win.addPlot(row=1, col=0)
        self.raw_fft_plot.setTitle("Frequency data")
        self.raw_fft_plot.setLabel(axis="left", text="Amplitude")
        self.raw_fft_plot.setLabel(axis="bottom", text="Frequency", units="Hz")
        self.raw_fft_plot.addLegend(labelTextSize="10pt")
        self.raw_fft_limits = et.utils.SmoothLimits()
        self.raw_fft_plot.setMenuEnabled(False)
        self.raw_fft_plot.setMouseEnabled(x=False, y=False)
        self.raw_fft_plot.hideButtons()
        self.raw_fft_plot.showGrid(x=True, y=True)
        self.raw_fft_plot.setLogMode(x=False, y=True)
        self.raw_fft_curves = []
        self.raw_fft_smooth_max = et.utils.SmoothMax(self.detector_config.frame_rate)
        self.raw_thresholds_curves = []

        for i in range(self.n_depths):
            raw_fft_curve = self.raw_fft_plot.plot(pen=et.utils.pg_pen_cycler(i), name="Fft")
            threshold_curve = self.raw_fft_plot.plot(
                pen=et.utils.pg_pen_cycler(i), name="Threshold"
            )
            self.raw_fft_curves.append(raw_fft_curve)
            self.raw_thresholds_curves.append(threshold_curve)

        self.speed_history_plot = win.addPlot(row=0, col=0)
        self.speed_history_plot.setTitle("Speed history")
        self.speed_history_plot.setLabel(axis="left", text="Speed", units="m/s")
        self.speed_history_plot.setLabel(axis="bottom", text="Time", units="Seconds")
        self.speed_history_plot.addLegend(labelTextSize="10pt")
        self.speed_history_curve = self.speed_history_plot.plot(
            pen=None,
            name="speed",
            symbol="o",
            symbolsize=3,
        )

        if self.detector_config.sweep_rate is not None:
            actual_max_speed = DetectorConfig._get_max_speed(self.detector_config.sweep_rate)
            self.speed_history_plot.setYRange(-actual_max_speed, actual_max_speed)
        else:
            self.speed_history_plot.setYRange(
                -self.detector_config.max_speed, self.detector_config.max_speed
            )
        self.speed_history_plot.setXRange(-self.history_length, 0)
        ay = self.speed_history_plot.getAxis("bottom")
        ay.setTicks(self.display_ticks)

        self.speed_html_format = (
            '<div style="text-align: center">'
            '<span style="color: #FFFFFF;font-size:15pt;">'
            "{}</span></div>"
        )

        self.speed_text_item = pg.TextItem(
            fill=pg.mkColor(0xFF, 0x7F, 0x0E, 200),
            anchor=(0.5, 0.5),
        )

        self.speed_text_item.setPos(-self.history_length / 2, -self.detector_config.max_speed / 2)
        brush = et.utils.pg_brush_cycler(1)
        self.speed_history_peak_plot_item = pg.PlotDataItem(
            pen=None, symbol="o", symbolSize=8, symbolBrush=brush, symbolPen="k"
        )
        self.speed_history_plot.addItem(self.speed_history_peak_plot_item)
        self.speed_history_plot.addItem(self.speed_text_item)

        self.speed_text_item.hide()

        self.setup_is_done = True

    def getThreshold(self):

        print("Threshold Call")
        '''self.socket_timer = self.socket_timer+1
        
        if(self.socket_timer > 100):
            msg = serverSocket.recv(1024)
            if(len(msg)==16):
                speed_struct = struct.unpack('Qd', msg)
                self.header_value = speed_struct[0]
                self.speed_threshold = speed_struct[1]
                print("Received Speed:  ", self.speed_threshold)
            self.socket_timer = 0
        TBD
        host = '127.0.0.1'
        port = 39040
        
        try:
            udp_socket.bind((host, port))

        except Exception as E:
            print(E)

        udp_socket.setblocking(0)

        while self.header_value == None and self.speed_threshold == None: 
            try:
                message, client_address = udp_socket.recvfrom(16)
                speed_struct = struct.unpack('Hd', message)

                self.header_value = speed_struct[0]
                self.speed_threshold = speed_struct[1]
                print("Received Speed:  ", self.speed_threshold)

               
            except Exception as E:
                pass


        udp_socket.close()'''





    def update(self, data: DetectorResult) -> None:


        self.socket_timer = self.socket_timer+1
        
        if(self.socket_timer > 100):
            msg = serverSocket.recv(1024)
            if(len(msg)==16):
                speed_struct = struct.unpack('Qd', msg)
                self.header_value = speed_struct[0]
                self.speed_threshold = 0.4 #speed_struct[1]
                print("Received Speed:  ", self.speed_threshold)
            self.socket_timer = 0
            
        speed_filter = 3

        psd = data.extra_result.psd
        speed_guess = data.max_speed
        
        x_speeds = data.extra_result.velocities
        
        #psd_x_speeds = list(zip(psd,x_speeds))

        #filtered_psd_x_speeds = [data for data in psd_x_speeds if data[1] >= speed_filter]

        #psd = [data[0] for data in filtered_psd_x_speeds]
        #psd = np.array(psd)
        #x_speeds = [data[1] for data in filtered_psd_x_speeds]
        #x_speeds = np.array(x_speeds)

        thresholds = data.extra_result.actual_thresholds
        #print(self.speed_history)
        self.speed_history = np.roll(self.speed_history, -1)

        #self.speed_history = [speed for speed in self.speed_history if speed >= speed_filter]

        self.speed_history[-1] = speed_guess

        #self.speed_history = [speed for speed in self.speed_history if speed >= speed_filter]

        if self.time_window_length_n > 0:
            pos_speed = np.max(self.speed_history[-self.time_window_length_n :])
            pos_ind = int(np.argmax(self.speed_history[-self.time_window_length_n :]))
            neg_speed = np.min(self.speed_history[-self.time_window_length_n :])
            neg_ind = int(np.argmin(self.speed_history[-self.time_window_length_n :]))

            if abs(neg_speed) > abs(pos_speed):
                max_display_speed = neg_speed
                max_display_ind = neg_ind
            else:
                max_display_speed = pos_speed
                max_display_ind = pos_ind
        else:
            max_display_speed = self.speed_history[-1]
            max_display_ind = -1

        if abs(max_display_speed) >3.0:
            speed_text = "Max speed estimate {:.4f} m/s".format(max_display_speed)
            speed_html = self.speed_html_format.format(speed_text)

            self.speed_text_item.setHtml(speed_html)
            self.speed_text_item.show()

            sub_xs = self.speed_history_xs[-self.time_window_length_n :]
            self.speed_history_peak_plot_item.setData(
                [sub_xs[max_display_ind]], [max_display_speed]
            )
        else:
            self.speed_history_peak_plot_item.clear()
            self.speed_text_item.hide()

        display_inds = np.array([i for i, x in enumerate(self.speed_history) if x != 0.0])
        if len(display_inds) > 0:
            display_xs = self.speed_history_xs[display_inds]
            display_data = self.speed_history[display_inds]
        else:
            display_xs = []
            display_data = []


        #print("before: ", display_xs, display_data)

        #display_data_zipped = list(zip(display_xs, display_data))
        #print(display_data_zipped)
        #filtered_display_data = [data for data in display_data_zipped if np.abs(data[1]) > 3]

        #display_xs = [data[0] for data in filtered_display_data]
        #diplay_data = [data[1] for data in filtered_display_data]

        #print("after: ", display_xs, display_data)
        #print(type(display_data))
        filtered_display_data = []
        filtered_xs = []
        for i in range(0, len(display_data)):
            if abs(display_data[i]) >= self.speed_threshold:
                filtered_display_data.append(display_data[i])
                filtered_xs.append(display_xs[i])
                #print(display_xs[i], display_data[i], display_data[i] >= 3)

            
        self.speed_history_curve.setData(filtered_xs, filtered_display_data)

        assert psd is not None
        assert thresholds is not None

        top_max = max(np.max(psd), np.max(thresholds))

        smooth_max_val = np.log10(self.raw_fft_smooth_max.update(top_max))
        self.raw_fft_plot.setYRange(-2, smooth_max_val)
        for i in range(psd.shape[1]):
            self.raw_fft_curves[i].setData(x_speeds, psd[:, i])

            threshold_line = np.full(x_speeds.shape[0], thresholds[i])
            self.raw_thresholds_curves[i].setData(x_speeds, threshold_line)


if __name__ == "__main__":
    main()
