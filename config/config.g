; Jubilee CoreXY ToolChanging Printer - Config File
; Duet 3 Mini + 3HC Expansion Board

;-------------------------------------------------------------------------------
; Name and Network
;-------------------------------------------------------------------------------
M550 P"Jubilee"               ; Machine name (accessible via http://Jubilee.local)
M552 P192.168.10.2 S1          ; Ethernet with static IP
M553 P255.255.255.0           ; Netmask

;-------------------------------------------------------------------------------
; General Setup
;-------------------------------------------------------------------------------
M111 S0                       ; Debug off 
M929 P"eventlog.txt" S1       ; Log to file

;-------------------------------------------------------------------------------
; General Preferences
;-------------------------------------------------------------------------------
M555 P2                       ; Marlin-style output
G21                           ; Metric units
G90                           ; Absolute positioning
M83                           ; Relative extrusion

;-------------------------------------------------------------------------------
; Motor (Drive) to Axis Mapping
;-------------------------------------------------------------------------------
M584 X1.0 Y1.1                ; CoreXY axes (3HC)
M584 U1.2                     ; Toolchanger lock
M584 Z0.2:0.3:0.4             ; Triple Z (mainboard)

M584 B0.1 C0.0               ; Aux motors (lighting)

;-------------------------------------------------------------------------------
; Motor Directions
;-------------------------------------------------------------------------------

; CoreXY - Use SpreadCycle for performance (no StealthChop)
M569 P1.0 S1                 ; X motor: normal direction
M569 P1.1 S1                 ; Y motor: normal direction

; Tool lock motor - StealthChop for silence (test if torque is sufficient)
M569 P1.2 S0 D0              ; U motor: reversed direction

; Z axis motors - StealthChop for silent holding (test if Z moves reliably)
M569 P0.2 S0 D3              ; Z1 motor: normal direction + StealthChop
M569 P0.3 S0 D3              ; Z2 motor: normal direction + StealthChop
M569 P0.4 S0 D3              ; Z3 motor: normal direction + StealthChop

; Auxiliary motors (B, C) - Enable StealthChop to eliminate high-pitched noise
M569 P0.1 S1 D3              ; B motor: normal direction + StealthChop
M569 P0.0 S1 D3              ; C motor: normal direction + StealthChop



;-------------------------------------------------------------------------------
; Motor Currents [mA]
;-------------------------------------------------------------------------------
M906 X{0.8*sqrt(2)*2500} I30 ;sets the idle current to 30% of the run current
M906 Y{0.8*sqrt(2)*2500}
M906 Z{0.6*sqrt(2)*1680}
M906 U{0.6*sqrt(2)*670} I100
M906 B{1*sqrt(2)*670}
M906 C{0.6*sqrt(2)*670}


;-------------------------------------------------------------------------------
; Kinematics
;-------------------------------------------------------------------------------
M669 K1                       ; CoreXY mode

; Kinematic bed ball locations.
; Locations are extracted from CAD model assuming lower left build plate corner
; is (0, 0) on a 305x305mm plate.

M671 X297.5:2.5:150 Y313.5:313.5:-16.5 S10
; Z lift point positions (from CAD, bed center = 0,0)

;-------------------------------------------------------------------------------
; Steps Per Unit Configuration
;-------------------------------------------------------------------------------
; Disable microstepping to define base steps/unit
M350 X1 Y1 Z1 U1 V1 A1

M92 X{1/(1.8*16/180)}         ; CoreXY: pulley 20T, 1.8° stepper
M92 Y{1/(1.8*16/180)}
M92 Z{360/0.9/4}              ; Z: 0.9° stepper, 4mm lead screw
M92 U{13.73/1.8}              ; Tool lock (gear ratio / step angle)
M92 B{5.18/1.8}              ; B
M92 C{5.18/1.8}              ; C

; Enable microstepping with interpolation
M350 X16 Y16 Z16 I1          ; CoreXY and Z
M350 U16 B16 C16 I1             ; U, B, C axes

;-------------------------------------------------------------------------------
; Speed and Acceleration Settings
;-------------------------------------------------------------------------------
M201 X2000 Y2000 Z100 U250 B200 C200         ; Accélérations [mm/s² ou deg/s²]
M203 X20000 Y20000 Z1500 U8000 B1200 C1200   ; Vitesse max [mm/min ou deg/min]
M566 X500 Y500 Z10 U50 B10 C10              ; Jerk [mm/min ou deg/min]


;-------------------------------------------------------------------------------
; Endstops and Probe
;-------------------------------------------------------------------------------
M574 X1 S1 P"^1.io0.in"       ; X min endstop
M574 Y1 S1 P"^1.io1.in"       ; Y min endstop
M574 U1 S1 P"^0.io2.in"       ; U min endstop (tool lock)

M574 B1 S1 P"^1.io2.in"      ; B min endstop
M574 C1 S1 P"^0.io3.in"      ; C min endstop TEMPORAIRE A CHANGER

M574 Z0                      ; Z uses probe

M558 P8 C"io0.in" H3 F360 T6000  ; Z probe (switch mode)
G31 K0 X0 Y0 Z-2             ; Probe offset (control point)

; Axis limits
M208 X-13.75:313.75 Y-44:345 Z0:320
M208 U0:200                   ; U max rotation angle
M208 B0:100                   ; B max rotation angle
M208 C0:100                   ; C max rotation angle

;-------------------------------------------------------------------------------
; Bed Heater and Sensor
;-------------------------------------------------------------------------------
M308 S0 P"temp0" Y"thermistor" T100000 B3950 A"Bed"  ; Thermistor
M950 H0 C"out0" T0           ; Heater H0 on out0, sensor 0
M143 H0 S130                 ; Max temp 130°C
M140 H0                      ; Assign bed heater

;-------------------------------------------------------------------------------
; Tool Offsets and Overrides
;-------------------------------------------------------------------------------
M98 P"/sys/toffsets.g"       ; Tool offsets
M501                         ; Load config-override.g

;-------------------------------------------------------------------------------
; Tool Definitions
;-------------------------------------------------------------------------------
M563 P0 S"Camera_top"
M563 P1 S"Light_sensor"
M563 P2 S"Outil2"
M563 P3 S"Outil3"
