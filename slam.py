# slam.py
# 
# provides Slam class
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from breezyslam.algorithms import RMHC_SLAM

def normalizeAngle(angle, (lower, upper)):
    while angle <= lower:
        angle += 360
    while angle >= upper:
        angle -= 360
    return angle

float2int = lambda x: int(0.5 + x)

class Slam(RMHC_SLAM):

    # NEEDS AN OBJECT OF LASER CLASS DECLARED. SEE LASER.PY
    def __init__(self, laser, MAP_SIZE_M=4.0, MAP_RES_PIX_PER_M=100, MAP_DEPTH=5, HOLE_WIDTH_MM = 200, RANDOM_SEED = 0xabcd,  **unused):
        MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
        RMHC_SLAM.__init__(self, \
                       laser, \
                       MAP_SIZE_PIXELS, \
                       MAP_SIZE_M, \
                       MAP_DEPTH, \
                       HOLE_WIDTH_MM, \
                       RANDOM_SEED)

        self.scanSize = laser.SCAN_SIZE
        self.breezyMap = bytearray(MAP_SIZE_PIXELS**2) # initialize map array for BreezySLAM's internal mapping

    def getBreezyMap(self):
        ''' returns the SLAM map '''
        return self.breezyMap

    def updateSlam(self, points, velocities):
        ''' updates slam map, needs new laser scan [range,angle]
            and the current velocities tuple (dxy, dtheta, dt) from odometry
            units should be [MM] [DEGREES] [S]
            returns current position from slam ''' 
        distVec = [0 for i in range(self.scanSize)]
        
        # create breezySLAM-compatible data from raw scan data
        for point in points: 
            # range measurement
            dist = float2int(point[0])
            # integer angle
            index = float2int(point[1])
            if not 0 <= index < self.scanSize: continue
            distVec[index] = dist
        
        self.update(distVec, velocities)
        x, y, theta = self.getpos()
        theta = normalizeAngle(theta, (-180.0,180.0))

        self.getmap(self.breezyMap) # write internal map to breezyMap
        return (y, x, theta)
