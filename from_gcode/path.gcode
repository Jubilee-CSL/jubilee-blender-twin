; visualization seed added by RecordingTransport
G90
M82
G92 X0 Y0 Z0 E0
G1 X0.10 Y0.00 E0.10 F600
; === tool change: T0 ===
; tpre0.g
G90                   ; Ensure the machine is in absolute mode before issuing movements.
G0 X350.5 Y270 F20000 ; Rapid to the approach position without any current tool.
G60 S0                ; Save this position as the reference point from which to later apply new tool offsets.
; tpost0.g
G90                        ; Ensure the machine is in absolute mode before issuing movements.
G53 G1 X290.5 F6000           ; Move to the pickup position with tool-1.
G53 G1 Y338.5 F6000
; M98 P"/macros/tool_lock.g" ; Lock the tool
G91                   ; relative mode
G1 U10 F5000 H0        ; back off slightly
M569 P1.2 S0 D0        ; ensure SpreadCycle for max torque
M201 U100              ; slow acceleration
G1 U80 F1500 H1      ; torque-limit move
M201 U250              ; restore normal acceleration
G90                   ; back to absolute mode
G1 R2 Z0                   ; Restore prior Z position before tool change was initiated.
G1 R0 Y0                   ; Retract tool by restoring Y position next now accounting for new tool offset.
G1 R0 X0                   ; Restore X position now accounting for new tool offset.
M106 R2                    ; restore print cooling fan speed
; T0
