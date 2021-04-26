#!/usr/bin/env python3

import collections
import socket

import psutil
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater


class CpuTask(DiagnosticTask):
    def __init__(self, warning_percentage, window=1):
        DiagnosticTask.__init__(self, "CPU Information")
        self._warning_percentage = int(warning_percentage)
        self._readings = collections.deque(maxlen=window)

    def _get_average_reading(self):
        def avg(lst):
            return float(sum(lst)) / len(lst) if lst else float('nan')

        return [avg(cpu_percentages) for cpu_percentages in zip(*self._readings)]

    def run(self, stat):
        self._readings.append(psutil.cpu_percent(percpu=True))
        cpu_percentages = self._get_average_reading()
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("CPU Load Average", "{:.2f}".format(cpu_average))

        warn = False
        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{:.2f}".format(val))
            if val > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         "At least one CPU exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "CPU Average {:.2f} percent".format(cpu_average))

        return stat


def main():
    rospy.init_node('system_monitor')

    updater = Updater()
    updater.setHardwareID(hostname)
    updater.add(CpuTask(rospy.get_param("~warning_percentage", 90), rospy.get_param("~window", 1)))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        pass
