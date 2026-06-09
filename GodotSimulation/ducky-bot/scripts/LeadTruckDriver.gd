extends Node3D

@export var speed: float = 0.025
@export var reach_distance: float = 0.05
@export var rotation_offset_degrees: float = 180.0

var _points: Array[Vector3] = []
var _current_index: int = 0
var _finished: bool = false


func _ready() -> void:
	# Safe straight same-lane path.
	# Lead truck moves forward and then stops.
	# It does NOT loop back.
	_points = [
	Vector3(4.17, 0.08, 4.05),
	Vector3(4.17, 0.08, 4.35),
	Vector3(4.17, 0.08, 4.65),
	Vector3(4.17, 0.08, 4.95),
	Vector3(4.18, 0.08, 5.15),
	Vector3(4.25, 0.08, 5.30),
	Vector3(4.35, 0.08, 5.42),
	Vector3(4.48, 0.08, 5.50),
	Vector3(4.62, 0.08, 5.55),
	Vector3(4.78, 0.08, 5.56),
]

	global_position = _points[0]
	_current_index = 1
	_finished = false

	_face_next_point()

	print("[LeadTruckDriver] Lead truck started")


func _physics_process(delta: float) -> void:
	if _finished:
		return

	if _current_index >= _points.size():
		_finished = true
		print("[LeadTruckDriver] Lead truck reached end and stopped")
		return

	var target := _points[_current_index]
	var to_target := target - global_position
	var distance := to_target.length()

	if distance <= reach_distance:
		_current_index += 1

		if _current_index >= _points.size():
			_finished = true
			print("[LeadTruckDriver] Lead truck reached end and stopped")
			return

		_face_next_point()
		return

	var direction := to_target.normalized()
	global_position += direction * speed * delta

	look_at(global_position + direction, Vector3.UP)
	rotate_y(deg_to_rad(rotation_offset_degrees))


func _face_next_point() -> void:
	if _current_index >= _points.size():
		return

	var direction := (_points[_current_index] - global_position).normalized()

	if direction.length() > 0.0:
		look_at(global_position + direction, Vector3.UP)
		rotate_y(deg_to_rad(rotation_offset_degrees))