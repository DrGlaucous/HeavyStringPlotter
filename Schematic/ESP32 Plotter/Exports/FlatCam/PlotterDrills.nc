(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: ESP32 Plotter-PTH.drl_edit_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Sunday, 19 February 2023 at 19:03)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 0.8)
(Tool: 2 -> Dia: 1.0)
(Tool: 3 -> Dia: 1.02)
(Tool: 4 -> Dia: 1.1)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 300)
(Tool: 2 -> Feedrate: 300)
(Tool: 3 -> Feedrate: 300)
(Tool: 4 -> Feedrate: 300)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)
(Tool: 2 -> Feedrate Rapids: 1500)
(Tool: 3 -> Feedrate Rapids: 1500)
(Tool: 4 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -1.7)
(Tool: 2 -> Z_Cut: -1.7)
(Tool: 3 -> Z_Cut: -1.7)
(Tool: 4 -> Z_Cut: -1.7)

(Tools Offset: )
(Tool: 1 -> Offset Z: 0.0)
(Tool: 2 -> Offset Z: 0.0)
(Tool: 3 -> Offset Z: 0.0)
(Tool: 4 -> Offset Z: 0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 2)
(Tool: 2 -> Z_Move: 2)
(Tool: 3 -> Z_Move: 2)
(Tool: 4 -> Z_Move: 2)

(Z Toolchange: 15 mm)
(X,Y Toolchange: 0.0000, 0.0000 mm)
(Z Start: None mm)
(Z End: 0.5 mm)
(X,Y End: None mm)
(Steps per circle: 64)
(Preprocessor Excellon: default)

(X range:    1.5000 ...   67.3000  mm)
(Y range:    1.5000 ...   47.0000  mm)

(Spindle Speed: 0 RPM)
G21
G90
G94

G01 F300.00

M5
G00 Z15.0000
T1
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 0.8000 ||| Total drills for tool T1 = 116)
M0
G00 Z15.0000

G01 F300.00
M03
G00 X56.6400 Y45.2350
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X54.1090 Y45.2350
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y42.6950
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y40.1550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y37.6150
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y35.0750
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y32.5350
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y29.9950
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y27.4550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X54.1090 Y27.4550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X54.1090 Y21.1050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y21.1050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y18.5650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y13.4850
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y16.0250
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y18.5650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y21.1050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y24.9000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y27.4400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y27.4550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y29.9950
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X49.0300 Y29.9800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X51.5700 Y29.9800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X51.5700 Y32.5200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X49.0300 Y32.5200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X49.0300 Y35.0600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X51.5700 Y35.0600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y45.2350
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y42.6950
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y40.1550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y37.6150
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y35.0750
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y32.5350
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y29.9800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y32.5200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y35.0600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y37.6000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y40.1400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X35.0620 Y45.2200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y40.1400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X10.9300 Y42.6800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X2.0000 Y46.5000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X7.1200 Y40.1400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X7.1200 Y37.6000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X7.1200 Y35.0600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X7.1200 Y32.5200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X10.9300 Y37.5850
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X10.9300 Y40.1400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y37.6000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y35.0600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y32.5200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y29.9800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X10.9300 Y27.4400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X7.1200 Y29.9800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X7.1200 Y27.4400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X5.8520 Y12.2000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X2.0400 Y12.2300
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X2.0400 Y9.6900
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X2.0400 Y7.1500
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X2.0000 Y2.0000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X5.8370 Y2.0400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X8.3770 Y2.0400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X10.9170 Y2.0400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X8.3920 Y4.5800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X5.8520 Y7.1200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X5.8520 Y9.6600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X8.3770 Y9.6750
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X12.2000 Y12.2150
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X12.2000 Y9.6750
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y9.6600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y12.2000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y14.7400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y17.2800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y19.8200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y22.3600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y24.9000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X16.0100 Y27.4400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y22.3600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y19.8200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y17.2800
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y14.7400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y12.2000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X38.8700 Y9.6600
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y10.9450
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y8.4050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y5.8650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X43.9400 Y3.3250
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X49.0300 Y5.8650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X49.0300 Y8.4050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X49.0300 Y10.9450
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X51.5700 Y10.9450
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X51.5700 Y8.4050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X51.5700 Y5.8650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y3.3250
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y5.8650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y8.4050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y10.9450
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y13.4850
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X56.6400 Y16.0250
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y16.0150
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y13.4750
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y10.9350
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y8.3950
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X66.8000 Y2.0000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X65.5400 Y16.0250
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X62.9990 Y22.3750
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X62.9990 Y24.9150
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X62.9990 Y27.4550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X59.1900 Y27.4550
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y32.5250
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y35.0650
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y37.6050
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X60.4600 Y40.1450
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X63.0020 Y40.1400
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X66.8000 Y46.5000
G01 Z-1.7000
G01 Z0
G00 Z2.0000
G00 X59.1920 Y45.2200
G01 Z-1.7000
G01 Z0
G00 Z2.0000
M05
G00 Z0.50

