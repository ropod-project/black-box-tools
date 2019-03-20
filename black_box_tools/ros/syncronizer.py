import threading
import time

class Syncronizer(object):

    """Syncronize all the topic_utils threads

    @param start_timestamp -- float (the smallest time stamp from all the collections)
    @param locks -- list of lock objects (each used by a topic_utils thread)
    @param queues -- list of Queue objects (each for sending current_time to topic_utils process)
    @param conn -- Connection object (for communicating with rosbag_play.py)
    @param timestep -- the simulation will increment with this duration each time
    @param sleep_duration -- float (duration to sleep after aquiring all locks)

    @author Dharmin B.
    """

    def __init__(self, start_timestamp=0.0, locks=[], queues=[], \
            pause_conn=None, time_step=1.0, sleep_duration=0.05):
        self.current_time = start_timestamp
        self.locks = locks
        self.queues = queues
        self.pause_connection = pause_conn
        self.time_step = time_step
        self.sleep_duration = sleep_duration
        self.syncing = True
        self.is_paused = True

    def increment_time(self):
        """increment the simulation time with timestep and wait for all 
        topic_utils threads to finish publishing. Then sleep for sleep_duration
        amount of time and release all the lock allowing all topic_utils threads
        to publish again.

        :returns: None

        """
        try:
            while self.syncing:
                for lock in self.locks :
                    lock.acquire()
                self.current_time += self.time_step
                for q in self.queues :
                    q.put(self.current_time)
                self.update_state_bool()
                while self.is_paused and self.syncing:
                    self.update_state_bool()
                    time.sleep(self.sleep_duration)
                for lock in self.locks :
                    lock.release()
                time.sleep(self.sleep_duration)
                self.update_state_bool()
        except (KeyboardInterrupt, SystemExit):
            pass
        for lock in self.locks:
            try:
                lock.release()
            except ValueError:
                pass
        # send signal to each topic_utils process to stop publishing and exit
        for q in self.queues :
            q.put(-1)

    def update_state_bool(self):
        """update self.syncing and self.is_paused by reading from connection
        is possible.
        :returns: None

        """
        if self.pause_connection.poll():
            data = self.pause_connection.recv()
            self.syncing, self.is_paused = data
