#!/usr/bin/env python3
import rospy
import tkinter
from tkinter import messagebox, Label
import json
import math
import tf2_ros
import tf2_geometry_msgs
from lidar_auto_docking.msg import Initdock
import threading

class GUI(object):
    def __init__(self):
        self.top_ = tkinter.Tk()
        self.top_.geometry("600x500")
        self.top_.title("Dock Saver")
        
        # Store label reference to prevent memory leak
        self.distance_label = None

    def create_button(self, posx, posy, button_name, callback_function, button_colour):
        self.button_ = tkinter.Button(
            self.top_, text=button_name, command=callback_function, bg=button_colour
        )
        self.button_.place(x=posx, y=posy)

    def update_tk(self):
        """Safe update - call from main thread only"""
        try:
            self.top_.update_idletasks()
            self.top_.update()
        except tkinter.TclError:
            pass  # Window closed

    def show_message(self, msg):
        messagebox.showinfo(title="Save Status", message=msg)

    def show_variable(self, msg, posx, posy):
        """Thread-safe label update - destroys old label first"""
        # Destroy existing label to prevent accumulation
        if self.distance_label is not None:
            try:
                self.distance_label.destroy()
            except tkinter.TclError:
                pass
        
        self.distance_label = Label(self.top_, text=str(msg))
        self.distance_label.place(x=posx, y=posy)


class DockSaver:
    def __init__(self):
        rospy.init_node('dock_saver')

        self.dock_file_path = rospy.get_param('~load_file_path', '/tmp/dock_pose.json')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.w_pos = 1.0
        self.bot_x = 0.0
        self.bot_y = 0.0
        self.bot_z = 0.0
        self.bot_w = 1.0

        self.sub = rospy.Subscriber('init_dock', Initdock, self.dock_cb)

        self.gui = GUI()
        self.gui.create_button(
            posx=250, posy=300,
            button_name="save dock and bot pose",
            button_colour="green",
            callback_function=self.save_dock_callback
        )

        # Queue for thread-safe GUI updates
        self.gui_update_queue = []
        self.queue_lock = threading.Lock()

        # Timer callback (runs in separate thread)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        
        rospy.loginfo("Dock Saver initialized. Waiting for /init_dock...")

    def timer_cb(self, event):
        """Timer callback - runs in separate thread, queue GUI updates"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(1.0)
            )
            self.bot_x = transform.transform.translation.x
            self.bot_y = transform.transform.translation.y
            self.bot_z = transform.transform.rotation.z
            self.bot_w = transform.transform.rotation.w

            dx = abs(self.bot_x - self.x_pos)
            dy = abs(self.bot_y - self.y_pos)
            dist = math.sqrt(dx * dx + dy * dy)
            
            # Queue the GUI update instead of calling directly
            with self.queue_lock:
                self.gui_update_queue.append(("distance", round(dist, 3), 250, 250))
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr_throttle(5.0, "Failed to get transform: %s", str(e))

    def process_gui_queue(self):
        """Process queued GUI updates - call from main thread"""
        with self.queue_lock:
            updates = self.gui_update_queue[:]
            self.gui_update_queue.clear()
        
        for update in updates:
            if update[0] == "distance":
                self.gui.show_variable(update[1], update[2], update[3])

    def dock_cb(self, msg):
        self.x_pos = msg.x
        self.y_pos = msg.y
        self.z_pos = msg.z
        self.w_pos = msg.w
        rospy.loginfo_throttle(5.0, f"Dock pose received: x={msg.x:.3f}, y={msg.y:.3f}")

    def save_dock_callback(self):
        """Validate before saving"""
        # Check if dock pose was actually received
        if abs(self.x_pos) < 0.01 and abs(self.y_pos) < 0.01:
            self.gui.show_message("ERROR: No dock pose received!\nMake sure /init_dock is publishing.")
            rospy.logwarn("Attempted to save with default (0,0) dock pose")
            return
        
        output_dict = {
            "x": self.x_pos,
            "y": self.y_pos,
            "z": self.z_pos,
            "w": self.w_pos,
            "bx": self.bot_x,
            "by": self.bot_y,
            "bz": self.bot_z,
            "bw": self.bot_w
        }
        
        try:
            with open(self.dock_file_path, "w") as f:
                json.dump(output_dict, f, indent=2)
            rospy.loginfo("Saved dock pose to %s", self.dock_file_path)
            self.gui.show_message("Dock and robot Coordinates have been saved!")
        except IOError as e:
            rospy.logerr("Failed to save file: %s", str(e))
            self.gui.show_message(f"ERROR: Could not save file!\n{str(e)}")

    def spin(self):
        """Main loop - process GUI updates from main thread"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Process queued GUI updates (thread-safe)
            self.process_gui_queue()
            self.gui.update_tk()
            rate.sleep()
        
        # Cleanup
        try:
            self.gui.top_.destroy()
        except tkinter.TclError:
            pass


def main():
    node = DockSaver()
    node.spin()


if __name__ == "__main__":
    main()