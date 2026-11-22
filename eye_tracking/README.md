# eye_tracking

Owned by the eye-tracking teammate. Logitech camera -> gaze pointer on the screen -> dwell selection.

Contract with the rest of the system (agree and keep stable): send events to the web UI / arm service as JSON,
rectangle coordinates `u, v` in 0..1 (origin at tag 0, u along the first edge, v along the second):

    {"type": "select", "u": 0.42, "v": 0.71}   # dwell completed on a point
    {"type": "reset"}                          # reset button / gesture

Live gaze (for drawing the pointer) is optional: {"type": "gaze", "u": ..., "v": ...}.
