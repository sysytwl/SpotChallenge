import argparse
import curses
import logging
import os
import signal
import threading
import time

from serial_control import serial_control
from aruco_marker import ArucoMarkerDetector
import pwm



class ArmWasdInterface(object):

    def __init__(self, port):
        self.sc = serial_control(port)
        self._lock = threading.Lock()

        self._command_dictionary = {
            27: self._stop,  # ESC key
            ord(' '): self._toggle_estop,
            ord('q'): self._move_out,
            ord('e'): self._move_in,
            ord('a'): self._rotate_ccw,
            ord('d'): self._rotate_cw,
            ord('w'): self._move_up,
            ord('s'): self._move_down,
            #ord('i'): self._rotate_plus_ry,
            #ord('k'): self._rotate_minus_ry,
            #ord('u'): self._rotate_plus_rx,
            #ord('o'): self._rotate_minus_rx,
            #ord('j'): self._rotate_plus_rz,
            #ord('l'): self._rotate_minus_rz,
            ord('n'): self._toggle_gripper_open,
            ord('m'): self._toggle_gripper_closed,
            ord('c'): self._connection_start,
            ord('v'): self._connection_stop,
        }

        # coordinates
        self.coordinateX, self.coordinateY, self.coordinateZ = self.sc.forward_kinematics()

        # target from camera
        self.tvecX = []
        self.tvecY = []
        self.tvecZ = []

        self._estop = False
        self._exit_check = False
        self._locked_messages = ['', '', '', '']  # string: displayed message for user
        self._connecting = True

    def aruco_marker_detector_loop(self):
        while self._connecting:
            tvecs = self.marker_detector.detect_marker()
            if tvecs == None:
                pass
            else:
                tvec = tvecs[0][0][0]   #TODO: check
                with self.lock:
                    self.tvecX.append(tvec[0])
                    self.tvecY.append(tvec[1])
                    self.tvecZ.append(tvec[2])

    def calculate_average(self):
        with self._lock:
            if self.tvecX and self.tvecY and self.tvecZ:
                print('Hi')
                self.coordinateX = sum(self.tvecX) / len(self.tvecX)
                self.tvecX = []

                self.coordinateY = sum(self.tvecY) / len(self.tvecY)
                self.tvecY = []

                self.coordinateZ = sum(self.tvecZ) / len(self.tvecZ)
                self.tvecZ = []
                return True
            else:
                return False

    def _toggle_estop(self):
        self._estop = True
        self.sc.emergencystop()

    def drive(self, stdscr):
        while not self._exit_check:
            self._drive_draw(stdscr)
            cmd = stdscr.getch()
            self._drive_cmd(cmd)

            if self.calculate_average():
                self.add_message()
                self.sc.kinematics_top(self.coordinateX, self.coordinateY, self.coordinateZ)
                time.sleep(2)
                self.sc.stop()
            else:
                self.add_message(f'No mark detected') #TODO: check
                #self.sc.kinematics_top(self.coordinateX, self.coordinateY, self.coordinateZ)
                #time.sleep(10)
                #pwm.connector_open() #TODO: check

            time.sleep(0.5)

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        with self._lock:
          self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx):
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    def _drive_draw(self, stdscr):
        """Draw the interface screen at each update."""
        stdscr.clear()  # clear screen
        stdscr.resize(26, 96)
        #stdscr.addstr(0, 0, f'{self._robot_id.nickname:20s} {self._robot_id.serial_number}')
        #stdscr.addstr(1, 0, self._lease_str(lease_keep_alive))
        #stdscr.addstr(2, 0, "Connection Status:" + str(self._con))
        stdscr.addstr(3, 0, "Estop Status: " + str(self._estop))
        stdscr.addstr(4, 0, self.sc.read())
        #stdscr.addstr(5, 0, self._time_sync_str())
        for i in range(4):
            stdscr.addstr(6 + i, 2, self.message(i))
        stdscr.addstr(10, 0, 'Commands:                                           ')
        stdscr.addstr(11, 0, '          [SPACE]: Estop                            ')
        stdscr.addstr(12, 0, '          [c]: connection, [v]: disconnect          ')
        #stdscr.addstr(13, 0, '          [y]: Unstow arm, [h]: Stow arm            ')
        stdscr.addstr(14, 0, '          [wasd]: Radial/Azimuthal control          ')
        stdscr.addstr(15, 0, '          [qe]: Up/Down control                     ')
        stdscr.addstr(16, 0, '          [uo]: X-axis rotation control             ')
        stdscr.addstr(17, 0, '          [ik]: Y-axis rotation control             ')
        stdscr.addstr(18, 0, '          [jl]: Z-axis rotation control             ')
        stdscr.addstr(19, 0, '          [nm]: Open/Close gripper                  ')
        stdscr.addstr(20, 0, '          [ESC]: Stop                               ')
        stdscr.addstr(21, 0, '')

        stdscr.refresh()

    def _drive_cmd(self, key):
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f'Unrecognized keyboard command: \'{chr(key)}\'')

    def _stop(self):
        self._exit_check = True

    def shutdown(self):
        self._connection_stop()
        self.sc.close()

    def _move_out(self):
        # TODO: X++ Y++
        self.sc.kinematics(self.coordinateX, self.coordinateY, self.coordinateZ)

    def _move_in(self):
        # TODO: X-- Y--
        self.sc.kinematics(self.coordinateX, self.coordinateY, self.coordinateZ)

    def _rotate_ccw(self):
        self.sc.go(-1, 0 , 0, 0)

    def _rotate_cw(self):
        self.sc.go(1, 0 , 0, 0)

    def _move_up(self):
        self.coordinateZ += 1
        self.sc.kinematics(self.coordinateX, self.coordinateY, self.coordinateZ)

    def _move_down(self):
        self.coordinateZ -= 1
        self.sc.kinematics(self.coordinateX, self.coordinateY, self.coordinateZ)

    def _rotate_plus_rx(self):
        self._arm_angular_velocity_cmd_helper('rotate_plus_rx', v_rx=VELOCITY_ANGULAR_HAND)

    def _rotate_minus_rx(self):
        self._arm_angular_velocity_cmd_helper('rotate_minus_rx', v_rx=-VELOCITY_ANGULAR_HAND)

    def _rotate_plus_ry(self):
        self._arm_angular_velocity_cmd_helper('rotate_plus_ry', v_ry=VELOCITY_ANGULAR_HAND)

    def _rotate_minus_ry(self):
        self._arm_angular_velocity_cmd_helper('rotate_minus_ry', v_ry=-VELOCITY_ANGULAR_HAND)

    def _rotate_plus_rz(self):
        self._arm_angular_velocity_cmd_helper('rotate_plus_rz', v_rz=VELOCITY_ANGULAR_HAND)

    def _rotate_minus_rz(self):
        self._arm_angular_velocity_cmd_helper('rotate_minus_rz', v_rz=-VELOCITY_ANGULAR_HAND)

    def _toggle_gripper_open(self):
        self._start_robot_command('open_gripper', RobotCommandBuilder.claw_gripper_open_command())

    def _toggle_gripper_closed(self):
        self._start_robot_command('close_gripper', RobotCommandBuilder.claw_gripper_close_command())

    def _connection_start(self):
        # camera init
        self.marker_detector = ArucoMarkerDetector()
        self.marker_detector.picam2.start()
        self.opencv_thread = threading.Thread(target=self.aruco_marker_detector_loop)
        self.opencv_thread.start()

    def _connection_stop(self):
        self._connecting = False
        time.sleep(2)
        self.marker_detector.out.release()
        self.marker_detector.picam2.stop()
        self.opencv_thread.join()

def main():
    # take argument from comand line
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="port name or path")
    args = parser.parse_args()

    # Serial connection
    try:
        arm_wasd_interface = ArmWasdInterface(str(args.port))
    except Exception as e:
        print("Error: ", e)
        return False

    curses.wrapper(arm_wasd_interface.drive)
    
    # STOP
    arm_wasd_interface.shutdown()

    return True



if __name__ == '__main__':
    if not main():
        os._exit(1)
    os._exit(0) 
