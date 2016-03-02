# laser.py
#
# provodes RPLidar class
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

from breezyslam.components import Laser

class RPLidar(Laser):
  def __init__(self, DIST_MIN, DIST_MAX):
    self.DIST_MIN = DIST_MIN
    self.DIST_MAX = DIST_MAX

    self.SCAN_SIZE = 360 # number of points per scan
    self.SCAN_RATE_HZ = 8 # [Hz]
    self.POINTS_PER_SEC = self.SCAN_SIZE * self.SCAN_RATE_HZ # [points/sec] 
    SCAN_DETECTION_ANGLE = 360
    SCAN_DISTANCE_NO_DETECTION_MM = self.DIST_MAX
    SCAN_DETECTION_MARGIN = 0
    LASER_OFFSET_MM = 0 # this value is distance behind center of robot 
                        # update() actually returns LIDAR unit position
    Laser.__init__(self, \
                   self.SCAN_SIZE, \
                   self.SCAN_RATE_HZ, \
                   SCAN_DETECTION_ANGLE, \
                   SCAN_DISTANCE_NO_DETECTION_MM, \
                   SCAN_DETECTION_MARGIN, \
                   LASER_OFFSET_MM)
