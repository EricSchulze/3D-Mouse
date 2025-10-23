# Fusion360 Controller

## Overview

```
+---------------------------------+                                                                  
|                                 |                                                                  
|                                 |                                                                  
|                                 |                                                                  
|                                 |                                                                  
|            Fusion360            |                                                                  
|                                 |                                                                  
|                                 |                                                                  
|                                 |                                                                  
|                                 |                                                                  
|                                 |                                                                  
|        +---------------+        |                                                                  
|        |               |        |                                         +-----------------------+
|        |     AddIn     |        |                                         |                       |
|        |               |        |                                         |                       |
|        +---+-------+---+        |                                         |        move.py        |
|            | :8080 |            |                                         |                       |
|            +----^--+            |                                         |                       |
+-----------------+---------------+                                         +-----------+-----------+
                  |                                                                     |            
                  |                                                                     |            
                  |                         send command to                             |            
                  +---------------------------------------------------------------------+            
                                           http://localhost:8080                                     
```


## Fusion360 AddIn
The directory Fusion360ControllerAddIn is a Fusion360 AddIn and it can be added as an AddIn in Fusion360 directly.
Create a AddIn called Fusion360ControllerAddIn and it will locate in the following path by default:
```Bash
%HOME%\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\AddIns\Fusion360ControllerAddIn
```
Then copy 3DMouseControllerAddIn.py in this repo to the directory above(Replace the exisitng file).

## Client
The client is a python script that sends commands in json format to the AddIn. There are two commands: rotate and translate.  
Note: Rotation and translation are calculated in camera's local coordinate system.

### Rotate
```JSON
{
    "command": "rotate",
    "rx": rx,
    "ry": ry,
    "rz": rz
}
```
where
* rx: float, rotate camera on pitch-asix by _rx_ degree.
* ry: float, rotate camera on roll-asix by _ry_ degree.
* rz: float, rotate camera on yaw-asix by _rz_ degree.


### Translate
```JSON
{
    "command": "translate",
    "tx": tx,
    "ty": ty,
    "tz": tz
}
```
where
* tx: float, move camera along x-asix by _tx_ unit.
* ty: float, move camera along y-asix by _ty_ unit.
* tz: float, move camera along z-asix by _tz_ unit.

In fusion360, the unit is in cm by default.
