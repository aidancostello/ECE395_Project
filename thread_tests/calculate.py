import math

EARTH_RADIUS_M = 6378137

def to_radians(val):
    return val*math.pi/180

def to_degrees(val):
    return val*180/math.pi

def compute_angles(self_gps, target_gps):
    self_lat = to_radians(self_gps[0])
    self_lon = to_radians(self_gps[1])
    self_alt = self_gps[2]
    target_lat = to_radians(target_gps[0])
    target_lon = to_radians(target_gps[1])
    target_alt = target_gps[2]

    delta_lon = target_lon-self_lon
    delta_lat = target_lat-self_lat
    delta_alt = target_alt-self_alt

    rotation_angle = to_degrees(math.atan2(math.sin(delta_lon)*math.cos(target_lat), math.cos(self_lat)*math.sin(target_lat) - math.sin(self_lat)*math.cos(target_lat)*math.cos(delta_lon)))

    a = math.sin(delta_lat/2)*math.sin(delta_lat/2) + math.sin(delta_lon/2)*math.sin(delta_lon/2)*math.cos(self_lat)*math.cos(target_lat)
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    elevation_angle = to_degrees(math.atan2(delta_alt, EARTH_RADIUS_M*c))

    if rotation_angle < 0:
        rotation_angle += 360

    return rotation_angle, elevation_angle

def main():
    self_gps = (40.115, -88.227778, 228.6)
    target_gps = (40.115, -88.227778, 1000)

    rotation, elevation = compute_angles(self_gps, target_gps)
    print(f"rotation angle: {rotation}  elevation angle: {elevation}")

if __name__ == "__main__":
    main()