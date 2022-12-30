import time
import sys
import csv
from granturismo.intake import Listener
from granturismo.model import Wheels

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

class TrackBounds:
	def __init__(self, **kwargs):
		for key, value in kwargs.items():
			# Convert the value to the appropriate data type
			if key in ['TRACK']:
				value = int(value)
			elif key in ['DIRECTION']:
				value = str(value)
			else:
				value = float(value)

			# Set the attribute on the instance
			setattr(self, key, value)

	def __str__(self):
		# Create a list of strings for the properties
		prop_strings = []
		for key, value in self.__dict__.items():
			prop_strings.append(f'{key}: {value} ({type(value).__name__})')
	
		# Join the strings with newlines
		return '\n'.join(prop_strings)


def load_track_bounds(filename):
	# Open the CSV file
	with open(filename, 'r') as f:
		# Read the rows from the CSV file
		rows = list(csv.DictReader(f))

	# Create a list of TrackBounds instances
	track_bounds = []
	for row in rows:
		track_bounds.append(TrackBounds(**row))

	return track_bounds

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

class Packet:
	def __init__(self, **kwargs):
		for key, value in kwargs.items():
			# Convert the value to the appropriate data type
			if key in ['POSX', 'POSY', 'POSZ', 'FUEL', 'SPEED', 'TEMPFL', 'TEMPFR', 'TEMPRL', 'TEMPRR', 'CLUTCH']:
				value = float(value)
			else:
				value = int(value)

			# Set the attribute on the instance
			setattr(self, key, value)

	def __str__(self):
		# Create a list of strings for the properties
		prop_strings = []
		for key, value in self.__dict__.items():
			prop_strings.append(f'{key}: {value} ({type(value).__name__})')
	
		# Join the strings with newlines
		return '\n'.join(prop_strings)

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

def line_intersects(p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y):

	s1_x = p1_x - p0_x
	s1_y = p1_y - p0_y
	s2_x = p3_x - p2_x
	s2_y = p3_y - p2_y

	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y)
	t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y)

	d = '--'
	if s2_x > 0:
		# Second set of coordinates has a positive x direction
		d = 'PX'
	elif s2_x < 0:
		# Second set of coordinates has a negative x direction
		d = 'NX'
	elif s2_y > 0:
		# Second set of coordinates has a positive y direction
		d = 'PY'
	elif s2_y < 0:
		# Second set of coordinates has a negative y direction
		d = 'NY'
	else:
		# Second set of coordinates has no discernible direction
		d = '??'

	if s >= 0 and s <= 1 and t >= 0 and t <= 1:
		# Collision detected
		return (1, d)
	return (0, d) # No collision

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

def get_bounding_box(x1, y1, x2, y2):
	return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)

def get_bounding_box_area(box):
	return (box[2] - box[0]) * (box[3] - box[1])

def get_bounding_box_intersection(box1, box2):
	left = max(box1[0], box2[0])
	right = min(box1[2], box2[2])
	top = max(box1[1], box2[1])
	bottom = min(box1[3], box2[3])
	if left > right or top > bottom:
		# The bounding boxes do not overlap
		return None
	return left, top, right, bottom

def calculate_iou(outer_bounding_box, inner_bounding_box):
	# Calculate the area of the intersection of the bounding boxes
	intersection = get_bounding_box_intersection(outer_bounding_box, inner_bounding_box)
	if intersection is None:
		return 0
	intersection_area = get_bounding_box_area(intersection)
	outer_area = get_bounding_box_area(outer_bounding_box)
	inner_area = get_bounding_box_area(inner_bounding_box)
	iou = intersection_area / (outer_area + inner_area - intersection_area)
	return iou

def find_matching_track(L1X, L1Y, L2X, L2Y, MinX, MinY, MaxX, MaxY, track_bounds, max_matches=3, min_iou=0.02):
	# Calculate the outer bounding box for the line defined by L1X, L1Y, L2X and L2Y
	outer_bounding_box = get_bounding_box(MinX, MinY, MaxX, MaxY)

	# Find the elements with the highest IoUs
	matches = []
	for element in track_bounds:
		# Calculate the inner bounding box for the line defined by P1X, P1Y, P2X and P2Y
		inner_bounding_box = get_bounding_box(element.MINX, element.MINY, element.MAXX, element.MAXY)

		# Check if the lines intersect
		intersects, direction = line_intersects(element.P1X, element.P1Y, element.P2X, element.P2Y, L1X, L1Y, L2X, L2Y)
		if intersects == 0:
			# The lines do not intersect, so skip this element
			continue

		# Check if the direction of the element matches the direction of the intersection point
		if element.DIRECTION != direction:
			# The direction does not match, so skip this element
			continue

		# Calculate the IoU
		iou = calculate_iou(outer_bounding_box, inner_bounding_box)

		# The lines intersect, so add the element and its direction to the matches list
		matches.append((iou, element.TRACK))

	# Sort the matches list in descending order of IoU
	matches.sort(key=lambda x: x[0], reverse=True)

	# Return the top max_matches elements in the matches list
	if not matches:
		return None

	# Get the best match
	best_match = matches[0]

	# Filter out matches that are not within 2-3% of the best match
	filtered_matches = [match for match in matches if match[0] >= best_match[0] * (1 - min_iou)]
	if len(filtered_matches) > max_matches:
		filtered_matches = filtered_matches[:max_matches]

	return filtered_matches

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––


if __name__ == "__main__":

	# Load the track bounds from the CSV file
	track_bounds = load_track_bounds('gt7trackdetect.csv')

	prevLap = -1
	maxX = -999999.9
	maxY = -999999.9
	minX = 999999.9
	minY = 999999.9
	gotTrack = -1

	try:

		ip_address = sys.argv[1]

		# To use the Listener session without a `with` clause, you'll need to call the `.start()` function.
		listener = Listener(ip_address)
		listener.start()

		while True:
			packet = listener.get()
			if packet.flags.loading_or_processing:
				continue
			if packet.flags.paused:
				continue
			if packet.lap_count is None:
				continue

			try:
				if packet.flags.car_on_track and gotTrack == -1:

					newXYZ = [packet.position.x, packet.position.z]

					if packet.lap_count > 0:
						if packet.position.x > maxX:
							maxX = packet.position.x
						if packet.position.x < minX:
							minX = packet.position.x
						if packet.position.z > maxY:
							maxY = packet.position.z
						if packet.position.z < minY:
							minY = packet.position.z

					if packet.lap_count > prevLap:
						prevLap = packet.lap_count
						if prevLap > 1:
							matches = find_matching_track(oldXYZ[0], oldXYZ[1], newXYZ[0], newXYZ[1], minX, minY, maxX, maxY, track_bounds)
							if matches:
								if len(matches) == 1:
									if matches[0][0] > 0.96:
										print(f"Got a +96% match: {matches[0][1]} ({round(matches[0][0] * 100, 1)}%)")
										gotTrack = matches[0][1]
									else:
										print(f"Got a possible match: {matches[0][1]} ({round(matches[0][0] * 100, 1)}%)")
								else:
									print(f"Got {len(matches)} track matches")

					oldXYZ = newXYZ

				if packet.lap_count < prevLap:
					print("Resetting track capture")
					prevLap = -1
					maxX = -999999.9
					maxY = -999999.9
					minX = 999999.9
					minY = 999999.9
					gotTrack = -1

			# Check for Ctrl-C
			except KeyboardInterrupt:
				# Exit the loop
				break

	finally:
		listener.close()
