from .pluto import plutoDrone

class plutoSwarm:
    def __init__(self) -> None:
        '''
        A swarm instance for Pluto Drones
        '''
        self.drones = {}

    def add(self, NAME, IP_ADDRESS : str, PORT : int) -> None:
        '''
        Add a drone to the Swarm.

        NAME : Identifier for Drone.
        IP_ADDRESS : str, IP Address of Drone
        PORT : int, Port of Drone
        '''
        if NAME in self.drones:
            print('[WARNING] Identifier already in use')
        else:
            drone = plutoDrone(IP_ADDRESS, PORT = PORT)
            self.drones[NAME] = [drone, False] # False -> Current Drone Status, Not Running

    def __getitem__(self, key) -> plutoDrone:
        '''
        Returns plutoDrone instance corresponding to the ID
        '''
        if key not in self.drones:
            return self.__missing__(key)
        else:
            return self.drones[key][0]
    
    def __missing__(self, key) -> None:
        print("[WARNING] No drone with ID :", key)
        return None

    def remove(self, NAME) -> None:
        '''
        Remove a drone with NAME from swarm

        NAME : Identifier of Drone
        '''
        if NAME not in self.drones:
            print('[WARNING] No drone with ID :', NAME)
        else:
            if self.drones[NAME][1]:
                self[NAME].disconnect()
            del self.drones[NAME]

    def start(self, *DRONE_IDS) -> None:
        '''
        Start Sockets, Threads corresponding to the DRONE_IDS

        If no arguments passed, all drones in swarm are started.
        '''
        if DRONE_IDS:
            for tid in DRONE_IDS:
                if tid not in self.drones:
                    return self.__missing__(tid)
                if self[tid]._threadsRunning == False:
                    #self[tid].start()
                    pass
                print(f'Drone "{tid}" Connected!')
                self.drones[tid][1] = True

        else:
            for tid in self.drones:
                if self[tid]._threadsRunning == False:
                    #self[tid].start()
                    pass
                print(f'Drone "{tid}" Connected!')
                self.drones[tid][1] = True

    def stop(self, *DRONE_IDS) -> None:
        '''
        Stop Sockets, Threads corresponding to the DRONE_IDS

        If no arguments passed, all drones in swarm are stopped.
        '''
        if DRONE_IDS:
            for tid in DRONE_IDS:
                if tid not in self.drones:
                    return self.__missing__(tid)
                if self[tid]._threadsRunning == True:
                    #self[tid].disconnect()
                    pass
                print(f'Drone "{tid}" Stopped!')
                self.drones[tid][1] = False

        else:
            for tid in self.drones:
                if self[tid]._threadsRunning == True:
                    #self[tid].disconnect()
                    pass
                print(f'Drone "{tid}" Stopped!')
                self.drones[tid][1] = False

    def status(self, DRONE_ID) -> bool:
        '''
        Get running status of Drone with DRONE_ID

        Returns bool (True if Drone is connected, else False)
        '''
        if DRONE_ID not in self.drones:
            return self.__missing__(DRONE_ID)
        return bool(self.drones[DRONE_ID][1])

    def list_names(self) -> list:
        '''
        Get all IDs of drones in swarm

        Returns list
        '''
        return list(self.drones.keys())