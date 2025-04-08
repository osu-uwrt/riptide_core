#!/usr/bin/env python3

import rclpy
import socket
import psutil
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import subprocess
import yaml
import re
import sys


class CpuTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "CPU Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        cpu_percentages = (psutil.cpu_percent(percpu=True))
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("CPU Load Average", "{:.2f}".format(cpu_average))

        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{:.2f}".format(val))

        if cpu_average > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         "Average CPU usage exceeds {:d}%".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Average CPU utilization {:.2f}%".format(cpu_average))

        return stat


class MemoryTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Memory Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        memory_info = psutil.virtual_memory()

        stat.add("Total Memory", "{:.2f} GB".format(memory_info.total / (1024.0 ** 3)))
        stat.add("Used Memory", "{:.2f} GB".format(memory_info.used / (1024.0 ** 3)))
        stat.add("Percent Used", "{:.2f}%".format(memory_info.percent))

        if memory_info.percent > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         "Memory usage exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Memory usage {:.2f}%".format(memory_info.percent))

        return stat


class DiskTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Disk Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        all_partitions = psutil.disk_partitions()

        warn = False
        for partition in all_partitions:
            if "snap" in partition.mountpoint:
                continue
            if "loop" in partition.device:
                continue

            disk_usage = psutil.disk_usage(partition.mountpoint)

            used = disk_usage.used / (1024.0 ** 3)
            total = disk_usage.total / (1024.0 ** 3)
            stat.add("\"{:s}\" Usage".format(partition.mountpoint), "{:.2f} GB / {:.2f} GB ({:.2f}%)".format(used, total, disk_usage.percent))

            if disk_usage.percent > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         "Disk usage exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Disk usage is OK")

        return stat


# The tegrastats provider that runs tegrastats only once
class TegrastatsProvider:
    _output = None
    _has_hardware = None
    
    @classmethod
    def get_output(cls):
        if cls._output is None:
            try:
                # Run tegrastats using the original method
                p = subprocess.Popen('tegrastats', stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, shell=True)
                # Read a single line
                o = p.stdout.readline()
                cls._output = o.decode('utf-8')
            except Exception:
                cls._output = ""
        return cls._output
    
    @classmethod
    def reset(cls):
        cls._output = None
    
    @classmethod
    def has_hardware(cls):
        if cls._has_hardware is None:
            output = cls.get_output()
            cls._has_hardware = any(x in output.lower() for x in 
                                  ['cpu@', 'soc', 'gr3d_freq', 'vdd_gpu_soc'])
        return cls._has_hardware


class CoreTempTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage, error_temp):
        diagnostic_updater.DiagnosticTask.__init__(self, "Core Temperature")
        self._error_temp = error_temp
        self._warning_temp = error_temp * (int(warning_percentage) / 100.0)
        # Reset the provider to force a fresh tegrastats run
        TegrastatsProvider.reset()

    def run(self, stat):
        try:
            # Get the shared tegrastats output
            output = TegrastatsProvider.get_output()
            if not output:
                stat.summary(DiagnosticStatus.ERROR, "Failed to get tegrastats data")
                return stat

            # Parse CPU temperatures from tegrastats output
            # Example format: cpu@58.406C
            cpu_temp_match = re.search(r'cpu@([\d\.]+)C', output, re.IGNORECASE)
            
            temps = []
            if cpu_temp_match:
                cpu_temp = float(cpu_temp_match.group(1))
                temps.append(cpu_temp)
                stat.add("CPU Temperature", "{:.2f} C".format(cpu_temp))
            
            # Also look for other temperature readings
            soc_temps = re.findall(r'soc\d+@([\d\.]+)C', output)
            for i, temp in enumerate(soc_temps):
                temps.append(float(temp))
                stat.add("SOC {} Temperature".format(i), "{:.2f} C".format(float(temp)))
            
            # Check for TJ temperature (junction temperature)
            tj_match = re.search(r'tj@([\d\.]+)C', output)
            if tj_match:
                tj_temp = float(tj_match.group(1))
                temps.append(tj_temp)
                stat.add("Junction Temperature", "{:.2f} C".format(tj_temp))
            
            if not temps:
                stat.summary(DiagnosticStatus.ERROR, "Failed to read temperature data")
                return stat
                
            max_temp = max(temps)
            
            if max_temp >= self._error_temp:
                stat.summary(DiagnosticStatus.ERROR,
                            "Core temp exceeds {:.2f} C @ {:.2f} C".format(self._error_temp, max_temp))
            elif max_temp >= self._warning_temp:
                stat.summary(DiagnosticStatus.WARN,
                            "Core temp exceeds {:.2f} C @ {:.2f} C".format(self._warning_temp, max_temp))
            else:
                stat.summary(DiagnosticStatus.OK, "Max core temp {:.2f} C".format(max_temp))
                
        except Exception as e:
            stat.summary(DiagnosticStatus.ERROR, "Failed to read temperature: {}".format(str(e)))
            
        return stat

    @staticmethod
    def has_hardware():
        output = TegrastatsProvider.get_output()
        if not output:
            return False
        return 'cpu@' in output.lower() or 'soc' in output.lower()


class ComputerTempTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage, error_temp):
        diagnostic_updater.DiagnosticTask.__init__(self, "Computer Temperature")
        self._warning_percentage = int(warning_percentage)
        self._error_temp = error_temp
        self._warning_temp = error_temp * (int(warning_percentage) / 100.0)

    def run(self, stat):
        try:
            # Get the shared tegrastats output
            output = TegrastatsProvider.get_output()
            if not output:
                stat.summary(DiagnosticStatus.ERROR, "Failed to get tegrastats data")
                return stat
            
            # Parse all thermal data for system temperature
            # First try to get the SOC temps as they're the most system-representative
            soc_temps = []
            soc_matches = re.findall(r'soc\d+@([\d\.]+)C', output)
            for temp in soc_matches:
                soc_temps.append(float(temp))
            
            if soc_temps:
                # Average of SOC temperatures for overall system temp
                computer_temp = sum(soc_temps) / len(soc_temps)
                stat.add("SOC Average Temperature", "{:.2f} C".format(computer_temp))
            else:
                # If no SOC temps, try CPU temp
                cpu_match = re.search(r'cpu@([\d\.]+)C', output, re.IGNORECASE)
                if cpu_match:
                    computer_temp = float(cpu_match.group(1))
                    stat.add("CPU Temperature", "{:.2f} C".format(computer_temp))
                else:
                    stat.summary(DiagnosticStatus.ERROR, "Failed to read system temperature")
                    return stat

            if computer_temp >= self._error_temp:
                stat.summary(DiagnosticStatus.ERROR,
                             "System temp exceeds {:.2f} C @ {:.2f} C".format(self._error_temp, computer_temp))
            elif computer_temp >= self._warning_temp:
                stat.summary(DiagnosticStatus.WARN,
                             "System temp exceeds {:.2f} C @ {:.2f} C".format(self._warning_temp, computer_temp))
            else:
                stat.summary(DiagnosticStatus.OK, "System temp {:.2f} C".format(computer_temp))

        except Exception as e:
            stat.summary(DiagnosticStatus.ERROR, "Failed to read system temperature: {}".format(str(e)))
            
        return stat

    @staticmethod
    def has_hardware():
        output = TegrastatsProvider.get_output()
        if not output:
            return False
        return 'soc' in output.lower() or 'cpu@' in output.lower()


class GPUTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "GPU Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        try:
            # Get the shared tegrastats output
            output = TegrastatsProvider.get_output()
            if not output:
                stat.summary(DiagnosticStatus.ERROR, "Failed to get tegrastats data")
                return stat
            
            # Parse GPU information from tegrastats output
            # Example: GR3D_FREQ 0% cpu@58.406C
            
            # Look for GPU utilization percentage
            gpu_util_match = re.search(r'GR3D_FREQ\s+(\d+)%', output)
            if gpu_util_match:
                gpu_util = int(gpu_util_match.group(1))
                stat.add("GPU Utilization", "{}%".format(gpu_util))
                
                if gpu_util > self._warning_percentage:
                    stat.summary(DiagnosticStatus.WARN,
                               "GPU utilization exceeds {:d}%".format(self._warning_percentage))
                else:
                    stat.summary(DiagnosticStatus.OK, "GPU utilization {}%".format(gpu_util))
            else:
                # Try to extract GPU power data as fallback info
                gpu_power_match = re.search(r'VDD_GPU_SOC\s+([\d\.]+)mW/([\d\.]+)mW', output)
                if gpu_power_match:
                    gpu_power_current = float(gpu_power_match.group(1))
                    gpu_power_max = float(gpu_power_match.group(2))
                    power_percent = (gpu_power_current / gpu_power_max) * 100 if gpu_power_max > 0 else 0
                    
                    stat.add("GPU Power", "{:.2f} mW / {:.2f} mW ({:.2f}%)".format(
                        gpu_power_current, gpu_power_max, power_percent))
                    
                    if power_percent > self._warning_percentage:
                        stat.summary(DiagnosticStatus.WARN,
                                   "GPU power usage exceeds {:d}%".format(self._warning_percentage))
                    else:
                        stat.summary(DiagnosticStatus.OK, "GPU power usage {:.2f}%".format(power_percent))
                else:
                    stat.summary(DiagnosticStatus.ERROR, "Failed to read GPU data")
                    
        except Exception as e:
            stat.summary(DiagnosticStatus.ERROR, "Failed to read GPU information: {}".format(str(e)))
            
        return stat

    @staticmethod
    def has_hardware():
        output = TegrastatsProvider.get_output()
        if not output:
            return False
        return 'GR3D_FREQ' in output or 'VDD_GPU_SOC' in output


def main():
    try:
        hostname = socket.gethostname()
        rclpy.init()
        node = rclpy.create_node('computer_monitor')
        node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)

        # Load config file
        with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
            thresholds_file = yaml.safe_load(stream)
        thresholds = thresholds_file["computer_monitor_thresholds"]

        # Create diagnostics updater
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID(hostname)

        # Always add the basic tasks
        updater.add(CpuTask(thresholds["cpu_warning_percentage"]))
        updater.add(MemoryTask(thresholds["memory_warning_percentage"]))
        updater.add(DiskTask(thresholds["disk_warning_percentage"]))
        
        # Check if tegrastats is available
        tegrastat_available = TegrastatsProvider.has_hardware()
        
        if tegrastat_available:
            # Add diagnostic tasks in the original order
            updater.add(ComputerTempTask(thresholds["temp_warning_percentage"], thresholds["computer_error_temp_c"]))
            updater.add(CoreTempTask(thresholds["temp_warning_percentage"], thresholds["computer_error_temp_c"]))
            updater.add(GPUTask(thresholds["gpu_warning_percentage"]))

        updater.force_update()
        rclpy.spin(node, None)
    except Exception as e:
        print(f"Error in computer_monitor: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()