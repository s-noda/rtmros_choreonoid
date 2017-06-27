#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *
from hrpsys_ros_bridge_tutorials.ml1_hrpsys_config import *

class ML1_HrpsysConfigurator(ChoreonoidHrpsysConfigurator, ML1HrpsysConfigurator):
    def __init__ (self):
        URATAHrpsysConfigurator.__init__(self, "ML1")

    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            #['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            #['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['co', "CollisionDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def startABSTIMP (self):
        self.rh_svc.setServoErrorLimit('all', 0);
        ##self.startAutoBalancer()
        ##self.startStabilizer()

if __name__ == '__main__':
    hcf = ML1_HrpsysConfigurator()
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
