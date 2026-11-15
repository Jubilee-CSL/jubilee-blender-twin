; visualization seed added by RecordingTransport
G90
M82
G92 X0 Y0 Z0 E0
G1 X0.10 Y0.00 E0.10 F600
; === tool change: T0 ===
; tpre0.g  (local copy)
G90                   ; Ensure the machine is in absolute mode before issuing movements.
G0 X277.0 Y235.0 F20000 ; Rapid to the approach position without any current tool.
G60 S0                ; Save this position as the reference point from which to later apply new tool offsets.
; tpost0.g  (local copy)
G90                        ; Ensure the machine is in absolute mode before issuing movements.
G53 G1 X277.0 F6000           ; Move to the pickup position with tool-1.
G53 G1 Y335.0 F6000
; M98 P"/macros/tool_lock.g" ; Lock the tool
G91                 ; Set relative mode
G1 U10 F6000 H0     ; Back off the limit switch with a small move
G1 U200 F6000 H1    ; Perform up to one rotation looking for the torque limit switch
G90                 ; Set absolute mode
G1 R2 Z0                   ; Restore prior Z position before tool change was initiated.
G1 R0 Y0                   ; Retract tool by restoring Y position next now accounting for new tool offset.
G1 R0 X0                   ; Restore X position now accounting for new tool offset.
M106 R2                    ; restore print cooling fan speed
; T0
G21
G90
G0 Z10.0 F600
G0 X178.670 Y42.170 F4000
G1 Z0.0 F800
G1 X263.670 Y42.170
G1 X263.670 Y127.170
G1 X178.670 Y127.170
G1 X178.670 Y42.170
G1 Z10.0 F600
G0 X26.430 Y164.410 F4000
G1 Z0.0 F800
G1 X154.190 Y164.410
G1 X154.190 Y249.880
G1 X26.430 Y249.880
G1 X26.430 Y164.410
G1 Z10.0 F600
; === park tool: T-1 ===
; tfree0.g  (local copy)
G91                          ; Relative Mode.
G1 Z2                        ; Pop Z up slightly so we don't crash while traveling over the usable bed region.
G90                          ; Absolute Mode.
G53 G0 X277.0 Y235.0 F12000    ; Rapid to the back of the post. Stay away from the tool rack so we don't collide with tools.
G53 G1 Y335.0 F6000            ; Controlled move to the park position with tool-1. (park_x, park_y)
; M98 P"/macros/tool_unlock.g" ; Unlock the tool
G91                 ; Set relative movements
G1 U-4 F6000 H2     ; Back off the limit switch with a small move
G1 U-360 F6000 H1   ; Perform up to one rotation looking for the home limit switch
G90                 ; Restore absolute movements
G53 G1 Y305 F6000            ; Retract the pin.
; T-1
