import threading
import time

class Syncronizer(object):

    """Syncronize all the topic_utils threads

    @param start_timestamp -- float (the smallest time stamp from all the collections)
    @param locks -- list of lock objects (each used by a topic_utils thread)
    @param timestep -- the simulation will increment with this duration each time
    @param sleep_duration -- float (duration to sleep after aquiring all locks)

    @author Dharmin B.
    """

    def __init__(self, start_timestamp=0.0, locks=[], time_step=1.0, sleep_duration=0.05):
        self.current_time = start_timestamp
        self.locks = locks
        self.time_step = time_step
        self.sleep_duration = sleep_duration

    def increment_time(self):
        """increment the simulation time with timestep and wait for all 
        topic_utils threads to finish publishing. Then sleep for sleep_duration
        amount of time and release all the lock allowing all topic_utils threads
        to publish again.

        :returns: None

        """
        while True:
            print("Current time", self.current_time)
            for lock in self.locks :
                lock.acquire()
            self.current_time += self.time_step
            time.sleep(self.sleep_duration)
            for lock in self.locks :
                lock.release()


    def get_current_time(self):
        """return the current simulation time
        :returns: float

        """
        # print(self.current_time)
        return self.current_time

